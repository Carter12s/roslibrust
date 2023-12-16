use super::tcpros::{establish_connection, ConnectionHeader};
use crate::RosLibRustError;
use abort_on_drop::ChildTask;
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::TcpStream,
    sync::{
        mpsc::{self, UnboundedReceiver},
        oneshot,
    },
};

type CallServiceRequest = (Vec<u8>, oneshot::Sender<Result<Vec<u8>, RosLibRustError>>);

pub struct ServiceServerLink {
    service_name: String,
    call_sender: mpsc::UnboundedSender<CallServiceRequest>,
    _actor_task: ChildTask<()>,
}

impl ServiceServerLink {
    pub async fn new(
        node_name: &str,
        service_name: &str,
        service_type: &str,
        service_uri: &str,
        srv_definition: &str,
        md5sum: &str,
    ) -> Self {
        let header = ConnectionHeader {
            caller_id: node_name.to_owned(),
            latching: false,
            msg_definition: srv_definition.to_owned(),
            md5sum: md5sum.to_owned(),
            topic: service_name.to_owned(),
            topic_type: service_type.to_owned(),
            tcp_nodelay: false,
        };

        let (call_tx, call_rx) = mpsc::unbounded_channel::<CallServiceRequest>();

        let actor_context = Self::actor_context(
            node_name.to_owned(),
            service_name.to_owned(),
            service_uri.to_owned(),
            header,
            call_rx,
        );

        let handle = tokio::spawn(actor_context);

        Self {
            service_name: service_name.to_owned(),
            call_sender: call_tx,
            _actor_task: handle.into(),
        }
    }

    async fn actor_context(
        node_name: String,
        service_name: String,
        service_uri: String,
        header: ConnectionHeader,
        mut call_rx: UnboundedReceiver<CallServiceRequest>,
    ) {
        if let Ok(mut stream) =
            establish_connection(&node_name, &service_name, &service_uri, header).await
        {
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
        } else {
            log::error!(
                "Failed to establish connection to {service_uri} for service {service_name}"
            );
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
