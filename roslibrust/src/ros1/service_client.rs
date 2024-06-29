use crate::{
    ros1::{
        names::Name,
        tcpros::{establish_connection, ConnectionHeader},
    },
    RosLibRustError, RosLibRustResult,
};
use abort_on_drop::ChildTask;
use roslibrust_codegen::RosServiceType;
use std::marker::PhantomData;
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::TcpStream,
    sync::{
        mpsc::{self, UnboundedReceiver},
        oneshot,
    },
};

pub type CallServiceRequest = (Vec<u8>, oneshot::Sender<CallServiceResponse>);
pub type CallServiceResponse = RosLibRustResult<Vec<u8>>;

pub struct ServiceClient<T: RosServiceType> {
    service_name: Name,
    sender: mpsc::UnboundedSender<CallServiceRequest>,
    _phantom: PhantomData<T>,
}

impl<T: RosServiceType> ServiceClient<T> {
    pub fn new(
        service_name: &Name,
        sender: mpsc::UnboundedSender<CallServiceRequest>,
    ) -> ServiceClient<T> {
        Self {
            service_name: service_name.to_owned(),
            sender,
            _phantom: PhantomData,
        }
    }

    pub fn service_name(&self) -> &Name {
        &self.service_name
    }

    pub async fn call(&self, request: &T::Request) -> RosLibRustResult<T::Response> {
        let request_payload = serde_rosmsg::to_vec(request)
            .map_err(|err| RosLibRustError::SerializationError(err.to_string()))?;
        let (response_tx, response_rx) = oneshot::channel();

        self.sender
            .send((request_payload, response_tx))
            .map_err(|_err| RosLibRustError::Disconnected)?;

        match response_rx.await {
            Ok(Ok(result_payload)) => {
                log::trace!(
                    "Service client for {} got response: {:?}",
                    self.service_name,
                    result_payload
                );

                // Okay the 1.. is funky and needs to be addressed
                // This is a little buried in the ROS documentation by the first byte is the "success" byte
                // if it is 1 then the rest of the payload is the response
                // Otherwise ros silently swaps the payload out for an error message
                // We need to parse that error message and display somewhere
                let response: T::Response = serde_rosmsg::from_slice(&result_payload[1..])
                    .map_err(|err| RosLibRustError::SerializationError(err.to_string()))?;
                return Ok(response);
            }
            Ok(Err(err)) => {
                return Err(err);
            }
            Err(_err) => {
                return Err(RosLibRustError::Disconnected);
            }
        }
    }
}

pub struct ServiceClientLink {
    service_type: String,
    call_sender: mpsc::UnboundedSender<CallServiceRequest>,
    _actor_task: ChildTask<()>,
}

impl ServiceClientLink {
    pub async fn new(
        node_name: &Name,
        service_name: &str,
        service_type: &str,
        service_uri: &str,
        srv_definition: &str,
        md5sum: &str,
    ) -> RosLibRustResult<Self> {
        let header = ConnectionHeader {
            caller_id: node_name.to_string(),
            latching: false,
            msg_definition: srv_definition.to_owned(),
            md5sum: md5sum.to_owned(),
            // Note: using "topic" indicates a subscription
            // using "service" indicates a service client
            topic: None,
            service: Some(service_name.to_owned()),
            topic_type: service_type.to_owned(),
            tcp_nodelay: false,
        };

        let (call_tx, call_rx) = mpsc::unbounded_channel::<CallServiceRequest>();

        let stream = establish_connection(&node_name, &service_name, &service_uri, header).await.map_err(|err| {
            log::error!("Failed to establish connection to service URI {service_uri} for service {service_name}: {err}");
            RosLibRustError::from(err)
        })?;

        let actor_context = Self::actor_context(stream, service_name.to_owned(), call_rx);

        let handle = tokio::spawn(actor_context);

        Ok(Self {
            service_type: service_type.to_owned(),
            call_sender: call_tx,
            _actor_task: handle.into(),
        })
    }

    pub fn get_sender(&self) -> mpsc::UnboundedSender<CallServiceRequest> {
        self.call_sender.clone()
    }

    pub fn service_type(&self) -> &str {
        &self.service_type
    }

    async fn actor_context(
        mut stream: TcpStream,
        service_name: String,
        mut call_rx: UnboundedReceiver<CallServiceRequest>,
    ) {
        // Listen on a receiver for calls to forward to the service
        loop {
            match call_rx.recv().await {
                Some(request) => {
                    Self::handle_service_call(&mut stream, &service_name, request).await
                }
                None => {
                    // Channel closed
                    break;
                }
            }
        }
    }

    /// Infallible version of handle_service_call that regardless of what occurs
    /// Sends the response back on the response channel, delegates work to handle_service_call_fallible
    async fn handle_service_call(
        stream: &mut TcpStream,
        service_name: &str,
        (request, response_sender): CallServiceRequest,
    ) {
        let response = Self::handle_service_call_fallible(stream, request).await;
        let response = response.map_err(|err| {
            log::error!(
                "Failed to send and receive service call for service {service_name}: {err:?}"
            );
            RosLibRustError::from(err)
        });
        let send_result = response_sender.send(response);
        if let Err(_err) = send_result {
            log::error!("Failed to send service call result back to handle for service {service_name}, channel closed");
        }
    }

    /// Helper function for calling a service
    /// Send the raw bytes of the request out
    /// Receives the full raw bytes of the response and returns them if nothing goes wrong
    async fn handle_service_call_fallible(
        stream: &mut TcpStream,
        request: Vec<u8>,
    ) -> Result<Vec<u8>, std::io::Error> {
        // Send the bytes of the request to the service
        stream.write_all(&request).await?;

        // Service calls magically have an extra byte in the TCPROS spec that indicates success/failure
        let mut success_byte = [0u8; 1];
        let _success_byte_read = stream.read_exact(&mut success_byte).await?;
        if success_byte[0] != 1 && success_byte[0] != 0 {
            log::error!(
                "Invalid service call success byte {}, value should be 1 or 0",
                success_byte[0]
            );
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "Invalid service call success byte",
            ));
        }
        let success = success_byte[0] == 1;

        if success {
            // Parse length of the payload body
            let mut body_len_bytes = [0u8; 4];
            let _body_bytes_read = stream.read_exact(&mut body_len_bytes).await?;
            let body_len = u32::from_le_bytes(body_len_bytes) as usize;

            let mut body = vec![0u8; body_len];
            stream.read_exact(&mut body).await?;

            // Dumb mangling here, our implementation expects the length and success at the front
            // got to be a better way than this
            let full_body = [success_byte.to_vec(), body_len_bytes.to_vec(), body].concat();

            Ok(full_body)
        } else {
            // MAJOR TODO: need to parse error from stream here!
            Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "Failure response from service:",
            ))
        }
    }
}

#[cfg(feature = "ros1_test")]
#[cfg(test)]
mod test {
    use log::info;

    use crate::{
        ros1::{NodeError, NodeHandle},
        RosLibRustError,
    };

    roslibrust_codegen_macro::find_and_generate_ros_messages!(
        "assets/ros1_test_msgs",
        "assets/ros1_common_interfaces"
    );

    // Some logic in the service client specifically for handling large payloads
    // trying to intentionally exercise that path
    // TODO this could probably be moved to integration tests or own file
    #[test_log::test(tokio::test)]
    async fn test_large_service_payload_client() {
        let nh = NodeHandle::new(
            "http://localhost:11311",
            "test_large_service_payload_client",
        )
        .await
        .unwrap();

        // Advertise a service that just echo's the bytes back
        let _handle = nh
            .advertise_service::<test_msgs::RoundTripArray, _>("large_service_payload", |request| {
                Ok(test_msgs::RoundTripArrayResponse {
                    bytes: request.bytes,
                })
            })
            .await
            .unwrap();

        // Picking random value that should be larger than MTU
        // Making sure the ROS message gets split over multiple TCP transactions
        // and that we correctly re-assemble it on the other end
        let bytes = vec![0; 10_000];

        info!("Starting service call");
        let response = nh
            .service_client::<test_msgs::RoundTripArray>("large_service_payload")
            .await
            .unwrap()
            .call(&test_msgs::RoundTripArrayRequest {
                bytes: bytes.clone(),
            })
            .await
            .unwrap();
        info!("Service call complete");

        assert_eq!(response.bytes, bytes);
    }

    #[test_log::test(tokio::test)]
    async fn error_on_unprovided_service() {
        let nh = NodeHandle::new("http://localhost:11311", "error_on_unprovided_service")
            .await
            .unwrap();

        let client = nh
            .service_client::<test_msgs::RoundTripArray>("unprovided_service")
            .await;
        assert!(client.is_err());
        // Note / TODO: this currently returns an IoError(Kind(ConnectionAborted))
        // which is better than hanging, but not a good error type to return
        if !matches!(client, Err(NodeError::IoError(_))) {
            panic!("Unexpected error type");
        }
    }

    #[test_log::test(tokio::test)]
    async fn persistent_client_can_be_called_multiple_times() {
        let nh = NodeHandle::new(
            "http://localhost:11311",
            "/persistent_client_can_be_called_multiple_times",
        )
        .await
        .unwrap();

        let server_fn = |request: test_msgs::AddTwoIntsRequest| {
            Ok(test_msgs::AddTwoIntsResponse {
                sum: request.a + request.b,
            })
        };

        let _handle = nh
            .advertise_service::<test_msgs::AddTwoInts, _>(
                "/persistent_client_can_be_called_multiple_times/add_two",
                server_fn,
            )
            .await
            .unwrap();

        let client = nh
            .service_client::<test_msgs::AddTwoInts>(
                "/persistent_client_can_be_called_multiple_times/add_two",
            )
            .await
            .unwrap();

        for i in 0..10 {
            let call: test_msgs::AddTwoIntsResponse = client
                .call(&test_msgs::AddTwoIntsRequest { a: 1, b: i })
                .await
                .unwrap();

            assert_eq!(call.sum, 1 + i);
        }
    }
}
