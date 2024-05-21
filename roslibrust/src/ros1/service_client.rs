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
                log::debug!(
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

pub struct ServiceServerLink {
    service_type: String,
    call_sender: mpsc::UnboundedSender<CallServiceRequest>,
    _actor_task: ChildTask<()>,
}

impl ServiceServerLink {
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

    async fn handle_service_call(
        stream: &mut TcpStream,
        service_name: &str,
        (request, response_sender): CallServiceRequest,
    ) {
        match stream.write_all(&request).await {
            Ok(()) => {
                // Wait for the result
                let mut read_buffer = vec![];
                let read_result = match stream.read_buf(&mut read_buffer).await {
                    // TODO: We may not get the full payload back in the call and need to check the length bytes
                    Ok(_nbytes) => Ok(read_buffer),
                    Err(err) => Err(RosLibRustError::from(err)),
                };
                if response_sender.send(read_result).is_err() {
                    log::warn!(
                        "Failed to send service call result back to handle for service {}",
                        &service_name
                    );
                }
            }
            Err(err) => {
                log::error!("Failed to send service call request: {err:?}");
                if response_sender
                    .send(Err(RosLibRustError::from(err)))
                    .is_err()
                {
                    log::error!("Service client requesting call hung up");
                }
            }
        }
    }
}
