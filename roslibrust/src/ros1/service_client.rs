use crate::{
    ros1::{
        names::Name,
        tcpros::{establish_connection, ConnectionHeader},
    },
    RosLibRustError, RosLibRustResult,
};
use abort_on_drop::ChildTask;
use roslibrust_codegen::RosServiceType;
use std::{marker::PhantomData, sync::Arc};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::TcpStream,
    sync::{
        mpsc::{self, UnboundedReceiver},
        oneshot,
    },
};

use super::tcpros;

pub type CallServiceRequest = (Vec<u8>, oneshot::Sender<CallServiceResponse>);
pub type CallServiceResponse = RosLibRustResult<Vec<u8>>;

// Note: ServiceClient is clone, and this is expressly different behavior than calling .service_client() twice on NodeHandle
// clonning a ServiceClient does not create a new connection to the service, but instead creates a second handle to the
// same underlying service client.
#[derive(Clone)]
pub struct ServiceClient<T: RosServiceType> {
    service_name: Name,
    sender: mpsc::UnboundedSender<CallServiceRequest>,
    _phantom: PhantomData<T>,
    // A given copy of a service client is actually just a handle to an underlying actor
    // When the last ServiceClient is dropped this will shut down the underlying actor and TCP connection
    _link: Arc<ServiceClientLink>,
}

impl<T: RosServiceType> ServiceClient<T> {
    pub(crate) fn new(
        service_name: &Name,
        sender: mpsc::UnboundedSender<CallServiceRequest>,
        link: ServiceClientLink,
    ) -> ServiceClient<T> {
        Self {
            service_name: service_name.to_owned(),
            sender,
            _phantom: PhantomData,
            _link: Arc::new(link),
        }
    }

    pub fn service_name(&self) -> &Name {
        &self.service_name
    }

    pub async fn call(&self, request: &T::Request) -> RosLibRustResult<T::Response> {
        let request_payload = roslibrust_serde_rosmsg::to_vec(request)
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
                let response: T::Response = roslibrust_serde_rosmsg::from_slice(&result_payload)
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
            md5sum: Some(md5sum.to_owned()),
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
            call_sender: call_tx,
            _actor_task: handle.into(),
        })
    }

    pub fn get_sender(&self) -> mpsc::UnboundedSender<CallServiceRequest> {
        self.call_sender.clone()
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
            let body = tcpros::receive_body(stream).await?;
            Ok(body)
        } else {
            // Parse an error message as the body
            let error_body = tcpros::receive_body(stream).await?;
            let err_msg: String =
                roslibrust_serde_rosmsg::from_slice(&error_body).map_err(|err| {
                    log::error!("Failed to parse service call error message: {err}");
                    std::io::Error::new(
                        std::io::ErrorKind::InvalidData,
                        "Failed to parse service call error message",
                    )
                })?;
            // TODO probably specific error type for this
            Err(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("Failure response from service server: {err_msg}"),
            ))
        }
    }
}
