use std::net::{Ipv4Addr, SocketAddr};

use abort_on_drop::ChildTask;
use log::*;
use tokio::io::{AsyncReadExt, AsyncWriteExt};

use crate::ros1::tcpros::ConnectionHeader;

use super::{names::Name, NodeHandle};

// TODO: someday I'd like to define a trait alias here for a ServerFunction
// Currently unstable:
// https://doc.rust-lang.org/beta/unstable-book/language-features/trait-alias.html
// trait ServerFunction<T> = Fn(T::Request) -> Err(T::Response, Box<dyn std::error::Error + Send + Sync>) + Send + Sync + 'static;

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
        // MAJOR TODO
        // unimplemented!()
        // Need to eventually define and call this:
        // self.node_handle
        //     .unadvertise_service(&self.service_name.to_string());
    }
}

/// Internal type held by the NodeServer to keep track of a given service server
pub(crate) struct ServiceServerLink {
    // Handle to internal task that is accepting and processing new requests
    // Task is automatically aborted when link is dropped
    _child_task: ChildTask<()>,
    port: u16,
}

impl ServiceServerLink {
    pub(crate) async fn new(
        method: Box<
            dyn Fn(Vec<u8>) -> Result<Vec<u8>, Box<dyn std::error::Error + Send + Sync>>
                + Send
                + Sync
                + 'static,
        >,
        host_addr: Ipv4Addr,
        service_name: Name,
        node_name: Name,
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

        let task = tokio::spawn(Self::actor(tcp_listener, service_name, node_name, method));

        Ok(Self {
            _child_task: task.into(),
            port,
        })
    }

    pub(crate) fn port(&self) -> u16 {
        self.port
    }

    /// Internal static function that actually operates the service server
    /// When new() is called as task is spawned that runs this function
    async fn actor(
        listener: tokio::net::TcpListener,
        service_name: Name, // Service path of the this service
        node_name: Name,    // Name of node we're running on
        method: Box<
            dyn Fn(Vec<u8>) -> Result<Vec<u8>, Box<dyn std::error::Error + Send + Sync>>
                + Send
                + Sync
                + 'static,
        >,
    ) {
        loop {
            // Accept new TCP connections
            match listener.accept().await {
                Ok((mut stream, peer_addr)) => {
                    // TODO for a bunch of the error branches in this handling
                    // it is unclear whether we should respond over the socket
                    // with an error or not?
                    // Probably it is better to try to send an error back?
                    debug!(
                        "Received service_request connection from {peer_addr} for {service_name}"
                    );

                    // Get the header from the stream:
                    let mut header_len_bytes = [0u8; 4];
                    if let Err(e) = stream.read_exact(&mut header_len_bytes).await {
                        warn!("Communication error while handling service request connection for {service_name}, could not get header length: {e:?}");
                        continue;
                    }
                    let header_len = u32::from_le_bytes(header_len_bytes) as usize;

                    let mut connection_header = vec![0u8; header_len];
                    if let Err(e) = stream.read_exact(&mut connection_header).await {
                        warn!("Communication error while handling service request connection for {service_name}, could not get header body: {e:?}");
                        continue;
                    }
                    let connection_header = match ConnectionHeader::from_bytes(&connection_header) {
                        Ok(header) => header,
                        Err(e) => {
                            warn!("Communication error while handling service request connection for {service_name}, could not parse header: {e:?}");
                            continue;
                        }
                    };
                    debug!(
                        "Got connection header: {connection_header:#?} for service {service_name}"
                    );

                    // Respond with our header
                    // TODO this is pretty cursed, may want a better version
                    let response_header = ConnectionHeader {
                        caller_id: node_name.to_string(),
                        latching: false,
                        msg_definition: "".to_string(),
                        md5sum: "".to_string(),
                        service: None,
                        topic: None,
                        topic_type: "".to_string(),
                        tcp_nodelay: false,
                    };
                    let bytes = response_header.to_bytes(false).unwrap();
                    if let Err(e) = stream.write_all(&bytes).await {
                        warn!("Communication error while handling service request connection for {service_name}, could not write response header: {e:?}");
                        continue;
                    }

                    let mut body_len_bytes = [0u8; 4];
                    if let Err(e) = stream.read_exact(&mut body_len_bytes).await {
                        warn!("Communication error while handling service request connection for {service_name}, could not get body length: {e:?}");
                        continue;
                    }
                    let body_len = u32::from_le_bytes(body_len_bytes) as usize;
                    debug!("Got body length {body_len} for service {service_name}");

                    let mut body = vec![0u8; body_len];
                    if let Err(e) = stream.read_exact(&mut body).await {
                        warn!("Communication error while handling service request connection for {service_name}, could not get body: {e:?}");
                        continue;
                    }
                    debug!("Got body for service {service_name}: {body:#?}");

                    // Okay this is funky and I should be able to do better here
                    // serde_rosmsg expects the length at the front
                    let full_body = [body_len_bytes.to_vec(), body].concat();

                    let response = (method)(full_body);

                    match response {
                        Ok(response) => {
                            // MAJOR TODO: handle error here

                            // Another funky thing here
                            // services have to respond with one extra byte at the front
                            // to indicate success
                            let full_response = [vec![1u8], response].concat();

                            stream.write_all(&full_response).await.unwrap();
                        }
                        Err(e) => {
                            warn!("Error from user service method for {service_name}: {e:?}");
                            // MAJOR TODO: respond with error
                        }
                    }
                }
                Err(e) => {
                    warn!("Error accepting TCP connection for service {service_name}: {e:?}");
                }
            };
        }
    }
}

#[cfg(feature = "ros1_test")]
#[cfg(test)]
mod test {
    use log::*;
    use tokio::time::timeout;

    use crate::ros1::NodeHandle;

    // TODO we're regenerating the messages in a lot of places
    // makes the tests slower to compile and run
    // we should probably generate messages for tests once in a central place?
    roslibrust_codegen_macro::find_and_generate_ros_messages!(
        "assets/ros1_common_interfaces",
        "assets/ros1_test_msgs"
    );

    #[test_log::test(tokio::test)]
    async fn basic_service_server() {
        const TIMEOUT: std::time::Duration = std::time::Duration::from_secs(1);
        debug!("Getting node handle");
        let nh = NodeHandle::new("http://localhost:11311", "basic_service_server")
            .await
            .unwrap();

        let server_fn = |request: test_msgs::AddTwoIntsRequest| {
            info!("Got request: {request:?}");
            return Ok(test_msgs::AddTwoIntsResponse {
                sum: request.a + request.b,
            });
        };

        // Create the server
        debug!("Creating server");
        let _handle = nh
            .advertise_service::<test_msgs::AddTwoInts, _>("basic_service_server", server_fn)
            .await
            .unwrap();

        // Make the request
        debug!("Calling service");
        let call: test_msgs::AddTwoIntsResponse = timeout(
            TIMEOUT,
            timeout(
                TIMEOUT,
                nh.service_client::<test_msgs::AddTwoInts>("basic_service_server"),
            )
            .await
            .unwrap()
            .unwrap()
            .call(&test_msgs::AddTwoIntsRequest { a: 1, b: 2 }),
        )
        .await
        .unwrap()
        .unwrap();

        assert_eq!(call.sum, 3);
        debug!("Got 3");
    }
}
