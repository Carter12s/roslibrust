use std::{
    net::{Ipv4Addr, SocketAddr},
    sync::Arc,
};

use abort_on_drop::ChildTask;
use log::*;
use tokio::io::AsyncWriteExt;

use crate::tcpros::{self, ConnectionHeader};

use super::{names::Name, NodeHandle, TypeErasedCallback};

/// ServiceServer is simply a lifetime control
/// The underlying ServiceServer is kept alive while object is kept alive.
/// Dropping this object, un-advertises the underlying service with rosmaster
// TODO: is this actually the API we want to provide?
// Maybe we should just let people manually call an unadvertise_service method?
pub struct ServiceServer {
    service_name: Name,
    node_handle: NodeHandle,
}

impl ServiceServer {
    pub fn new(service_name: Name, node_handle: NodeHandle) -> Self {
        Self {
            service_name,
            node_handle,
        }
    }
}

impl Drop for ServiceServer {
    fn drop(&mut self) {
        debug!("Dropping service server: {:?}", self.service_name);
        let _ = self
            .node_handle
            .unadvertise_service_server(&self.service_name.to_string());
    }
}

/// Internal type held by the NodeServer to keep track of a given service server
pub(crate) struct ServiceServerLink {
    // Handle to internal task that is accepting and processing new requests
    // Task is automatically aborted when link is dropped
    _child_task: ChildTask<()>,
    port: u16,
    service_name: String,
}

impl Drop for ServiceServerLink {
    fn drop(&mut self) {
        log::debug!("Dropping service server link: {:?}", self.service_name);
    }
}

impl ServiceServerLink {
    pub(crate) async fn new(
        method: Box<TypeErasedCallback>,
        host_addr: Ipv4Addr,
        service_name: Name,
        node_name: Name,
        service_type: String, // name of the message type e.g. "std_srvs/Trigger"
        md5sum: String,       // md5sum of the service message type
        srv_definition: String, // Full text of the service message type definition
    ) -> Result<Self, std::io::Error> {
        // TODO A lot of this is duplicated with publisher
        // We could probably move chunks into tcpros.rs and re-use

        // Setup a socket for receiving service requests on:
        let host_addr = SocketAddr::from((host_addr, 0));
        let tcp_listener = tokio::net::TcpListener::bind(host_addr).await?;
        let port = tcp_listener
            .local_addr()
            .expect("Bound tcp address did not have local address")
            .port();
        let service_name_copy = service_name.to_string();

        let task = tokio::spawn(Self::actor(
            tcp_listener,
            service_name,
            node_name,
            method,
            service_type,
            md5sum,
            srv_definition,
        ));

        Ok(Self {
            _child_task: task.into(),
            port,
            service_name: service_name_copy,
        })
    }

    /// Need to provide the port the server is listening on to the rest of the application
    /// so we can inform rosmaster of the full URI of where this service is located
    pub(crate) fn port(&self) -> u16 {
        self.port
    }

    /// Internal static function that actually operates the service server
    /// When new() is called as task is spawned that runs this function
    async fn actor(
        listener: tokio::net::TcpListener,
        service_name: Name, // Service path of the this service
        node_name: Name,    // Name of node we're running on
        method: Box<TypeErasedCallback>,
        service_type: String,
        md5sum: String,
        srv_definition: String,
    ) {
        // We have to move our callback into an Arc so the separately spawned tasks for each service connection
        // can access it in parrallel and not worry about the lifetime.
        // TODO: it may be better to Arc it upfront?
        let arc_method = Arc::new(method);
        // Tasks list here is needed to ensure that dropping this future drops child futures
        let mut tasks: Vec<ChildTask<()>> = vec![];
        loop {
            // Accept new TCP connections
            match listener.accept().await {
                Ok((stream, peer_addr)) => {
                    let task = tokio::spawn(Self::handle_tcp_connection(
                        stream,
                        peer_addr,
                        service_name.clone(),
                        node_name.clone(),
                        arc_method.clone(),
                        service_type.clone(),
                        md5sum.clone(),
                        srv_definition.clone(),
                    ));
                    // Add spawned task to child task list to ensure dropping shuts down server
                    tasks.push(task.into());
                }
                Err(e) => {
                    // Not entirely sure what circumstances can cause this?
                    // Loss of networking functionality on the host?
                    warn!("Error accepting TCP connection for service {service_name}: {e:?}");
                }
            };
        }
    }

    /// Each TCP connection made to the service server is processed in a separate task
    /// This function handles a single TCP connection
    async fn handle_tcp_connection(
        mut stream: tokio::net::TcpStream,
        peer_addr: SocketAddr,
        service_name: Name,
        node_name: Name,
        method: Arc<Box<TypeErasedCallback>>,
        service_type: String,
        md5sum: String,
        srv_definition: String,
    ) {
        // TODO for a bunch of the error branches in this handling
        // it is unclear whether we should respond over the socket
        // with an error or not?
        // Probably it is better to try to send an error back?
        debug!("Received service_request connection from {peer_addr} for {service_name}");

        let connection_header = match tcpros::receive_header(&mut stream).await {
            Ok(header) => {
                debug!("Received service request for {service_name} with header {header:?}");
                header
            }
            Err(e) => {
                warn!("Communication error while handling service request connection for {service_name}, could not parse header: {e:?}");
                // TODO returning here simply closes the socket? Should we respond with an error instead?
                return;
            }
        };
        trace!("Got connection header: {connection_header:#?} for service {service_name}");

        // Respond with our header
        let response_header = ConnectionHeader {
            caller_id: node_name.to_string(),
            latching: false,
            msg_definition: srv_definition,
            md5sum: Some(md5sum),
            service: Some(service_name.to_string()),
            topic: None,
            topic_type: service_type.to_string(),
            tcp_nodelay: false,
            persistent: None,
        };
        let bytes = response_header.to_bytes(false).unwrap();
        if let Err(e) = stream.write_all(&bytes).await {
            warn!("Communication error while handling service request connection for {service_name}, could not write response header: {e:?}");
            // TODO returning here simply closes the socket? Should we respond with an error instead?
            return;
        }

        // Each loop is one body:
        loop {
            let full_body = match tcpros::receive_body(&mut stream).await {
                Ok(body) => body,
                Err(e) => {
                    // Note this was degraded to debug! from warn! as every single use client produces this message
                    debug!("Communication error while handling service request connection for {service_name}, could not read body: {e:?}");
                    // Returning here closes the socket
                    return;
                }
            };

            let response = (method)(full_body);

            match response {
                Ok(response) => {
                    // MAJOR TODO: handle error here

                    // Another funky thing here
                    // services have to respond with one extra byte at the front
                    // to indicate success
                    let full_response = [vec![1u8], response].concat();

                    stream.write_all(&full_response).await.unwrap();
                    debug!("Wrote full service response for {service_name}");
                }
                Err(e) => {
                    warn!("Error from user service method for {service_name}: {e:?}");

                    let error_string = format!("{:?}", e);
                    let error_bytes = roslibrust_serde_rosmsg::to_vec(&error_string).unwrap();
                    let full_response = [vec![0u8], error_bytes].concat();

                    stream.write_all(&full_response).await.unwrap();
                }
            }

            // If a persistent service connection was requested keep requesting bodies
            if let Some(true) = connection_header.persistent {
                continue;
            } else {
                // This will result in the task shutting down, dropping the TCP socket and clean shutdown
                debug!("Service request connection for {service_name} is not persistent, shutting down");
                break;
            }
        }
    }
}
