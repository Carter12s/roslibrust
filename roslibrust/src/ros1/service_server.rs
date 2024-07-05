use std::{
    net::{Ipv4Addr, SocketAddr},
    sync::Arc,
};

use abort_on_drop::ChildTask;
use log::*;
use tokio::io::{AsyncReadExt, AsyncWriteExt};

use crate::ros1::tcpros::{self, ConnectionHeader};

use super::{names::Name, NodeHandle};

// TODO: someday I'd like to define a trait alias here for a ServerFunction
// Currently unstable:
// https://doc.rust-lang.org/beta/unstable-book/language-features/trait-alias.html
// trait ServerFunction<T> = Fn(T::Request) -> Err(T::Response, Box<dyn std::error::Error + Send + Sync>) + Send + Sync + 'static;

type TypeErasedCallback = dyn Fn(Vec<u8>) -> Result<Vec<u8>, Box<dyn std::error::Error + Send + Sync>>
    + Send
    + Sync
    + 'static;

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

        let connection_header = match tcpros::recieve_header(&mut stream).await {
            Ok(header) => header,
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
        };
        let bytes = response_header.to_bytes(false).unwrap();
        if let Err(e) = stream.write_all(&bytes).await {
            warn!("Communication error while handling service request connection for {service_name}, could not write response header: {e:?}");
            // TODO returning here simply closes the socket? Should we respond with an error instead?
            return;
        }

        // TODO we're not currently reading the persistent flag out of the connection header and treating
        // all connections as persistent
        // That means we expect one header exchange, and then multiple body exchanges
        // Each loop is one body:
        loop {
            let mut body_len_bytes = [0u8; 4];
            if let Err(e) = stream.read_exact(&mut body_len_bytes).await {
                // Note: this was lowered to debug! from warn! because this is intentionally done by tools like `rosservice` to discover service type
                debug!("Communication error while handling service request connection for {service_name}, could not get body length: {e:?}");
                // TODO returning here simply closes the socket? Should we respond with an error instead?
                return;
            }
            let body_len = u32::from_le_bytes(body_len_bytes) as usize;
            trace!("Got body length {body_len} for service {service_name}");

            let mut body = vec![0u8; body_len];
            if let Err(e) = stream.read_exact(&mut body).await {
                warn!("Communication error while handling service request connection for {service_name}, could not get body: {e:?}");
                // TODO returning here simply closes the socket? Should we respond with an error instead?
                return;
            }
            trace!("Got body for service {service_name}: {body:#?}");

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

                    let error_string = format!("{:?}", e);
                    let error_bytes = serde_rosmsg::to_vec(&error_string).unwrap();
                    let full_response = [vec![0u8], error_bytes].concat();

                    stream.write_all(&full_response).await.unwrap();
                }
            }
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
        let nh = NodeHandle::new("http://localhost:11311", "/basic_service_server")
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
            .advertise_service::<test_msgs::AddTwoInts, _>(
                "/basic_service_server/add_two",
                server_fn,
            )
            .await
            .unwrap();

        // Make the request
        debug!("Calling service");
        let call: test_msgs::AddTwoIntsResponse = timeout(
            TIMEOUT,
            timeout(
                TIMEOUT,
                nh.service_client::<test_msgs::AddTwoInts>("/basic_service_server/add_two"),
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

    #[test_log::test(tokio::test)]
    async fn dropping_service_server_kill_correctly() {
        debug!("Getting node handle");
        let nh = NodeHandle::new("http://localhost:11311", "/dropping_service_node")
            .await
            .unwrap();

        let server_fn = |request: test_msgs::AddTwoIntsRequest| {
            info!("Got request: {request:?}");
            return Ok(test_msgs::AddTwoIntsResponse {
                sum: request.a + request.b,
            });
        };

        // Create the server
        let handle = nh
            .advertise_service::<test_msgs::AddTwoInts, _>("~/add_two", server_fn)
            .await
            .unwrap();

        // Make the request (should succeed)
        let client = nh
            .service_client::<test_msgs::AddTwoInts>("~/add_two")
            .await
            .unwrap();
        let _call: test_msgs::AddTwoIntsResponse = client
            .call(&test_msgs::AddTwoIntsRequest { a: 1, b: 2 })
            .await
            .unwrap();

        // Shut down the server
        std::mem::drop(handle);
        // Wait a little bit for server shut down to process
        tokio::time::sleep(std::time::Duration::from_millis(250)).await;

        // Make the request again (should fail)
        let call_2 = client
            .call(&test_msgs::AddTwoIntsRequest { a: 1, b: 2 })
            .await;
        debug!("Got call_2: {call_2:?}");
        assert!(
            call_2.is_err(),
            "Shouldn't be able to call after server is shut down"
        );

        // Create a new clinet
        let client = nh
            .service_client::<test_msgs::AddTwoInts>("~/add_two")
            .await;
        // Client should fail to create as there should be no provider of the service
        assert!(
            client.is_err(),
            "Shouldn't be able to connect again (no provider of service)"
        );

        // Confirm ros master no longer reports our service as provided (via rosapi for fun)
        let rosapi_client = nh
            .service_client::<rosapi::Services>("/rosapi/services")
            .await
            .unwrap();
        let service_list: rosapi::ServicesResponse = rosapi_client
            .call(&rosapi::ServicesRequest {})
            .await
            .unwrap();
        assert!(!service_list
            .services
            .contains(&"/dropping_service_node/add_two".to_string()));
    }

    #[test_log::test(tokio::test)]
    async fn service_error_behavior() {
        debug!("Getting node handle");
        let nh = NodeHandle::new("http://localhost:11311", "/service_error_behavior")
            .await
            .unwrap();

        let server_fn = |request: test_msgs::AddTwoIntsRequest| -> Result<
            test_msgs::AddTwoIntsResponse,
            Box<dyn std::error::Error + Send + Sync>,
        > {
            info!("Got request: {request:?}");
            return Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                "test message",
            )));
        };

        // Create the server
        let _handle = nh
            .advertise_service::<test_msgs::AddTwoInts, _>("~/add_two", server_fn)
            .await
            .unwrap();

        // Make the request (should fail)
        let client = nh
            .service_client::<test_msgs::AddTwoInts>("~/add_two")
            .await
            .unwrap();
        let call = client
            .call(&test_msgs::AddTwoIntsRequest { a: 1, b: 2 })
            .await;
        // Okay so this is logging the error message correctly, but the contents currently suck:
        // "Got call: Err(IoError(Custom { kind: Other, error: "Failure response from service server: Custom { kind: NotFound, error: \"test message\" }" }))"
        // We should someday clean up error types here, but frankly errors throughout the entire crate need an overhaul
        debug!("Got call: {call:?}");
        assert!(call.is_err());
    }
}
