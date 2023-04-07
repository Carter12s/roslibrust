use std::{net::{Ipv4Addr, SocketAddr}, convert::Infallible};

use hyper::{Body, Response, StatusCode};
use log::*;

use crate::NodeHandle;

/// Hosts an xmlrpc API that rosmaster will call to notify of new subscribers / publishers etc.
/// Is also called by other ros nodes to initiate point to point connections.
/// Note: ROS uses "master/slave" terminology here. We continue to refer to the ROS central server as the ROS master,
/// but are intentionally using "NodeServer" in place of where ROS says "Slave API"
pub(crate) struct NodeServer {}

impl NodeServer {
    // Creates a new node server and starts it running
    // Returns a join handle used to indicate if the server stops running, and the port the server bound to
    // TODO this will need to take in some kind of async handle back to nodes data
    pub(crate) fn new(handle: NodeHandle, host_addr: Ipv4Addr) -> u16 {
        let host_addr = SocketAddr::from((host_addr, 0));

        let make_svc = hyper::service::make_service_fn(move |connection| {
            debug!("New node xmlrpc connection {connection:?}");
            let handle =  handle.clone(); // Make a unique handle for each connection
            async move {
                Ok::<_, Infallible>(hyper::service::service_fn(move |req| {
                    let handle = handle.clone(); // Each call gets their own copy of the handle too
                    NodeServer::respond(handle, req)
                }))
            }
        });

        let server = hyper::server::Server::bind(&host_addr.into());
        let server = server.serve(make_svc);
        let addr = server.local_addr();

        tokio::spawn(async move {
            let x = server.await;
            error!("Node's xmlrpc server was stopped unexpectedly: {x:?}");
        });
        addr.port()
    }

    async fn respond(
        handle: NodeHandle,
        body: hyper::Request<Body>,
    ) -> Result<Response<Body>, Infallible> {
        let body = match hyper::body::to_bytes(body).await {
            Ok(bytes) => bytes,
            Err(e) => {
                let error_str = format!(
                    "Failed to get bytes from http request on xmlrpc server, ignoring: {e:?}"
                );
                warn!("{error_str}");
                return Ok(Response::builder()
                    .status(StatusCode::BAD_REQUEST)
                    .body(Body::from(error_str))
                    .unwrap());
            }
        };
        let body = match String::from_utf8(body.to_vec()) {
            Ok(s) => s,
            Err(e) => {
                let error_str = format!("Failed to parse http body as valid utf8 string: {e:?}");
                warn!("{error_str}");
                return Ok(Response::builder()
                    .status(StatusCode::BAD_REQUEST)
                    .body(Body::from(error_str))
                    .unwrap());
            }
        };
        let (method_name, args) = match serde_xmlrpc::request_from_str(&body) {
            Ok(x) => x,
            Err(e) => {
                let error_str =
                    format!("Failed to parse valid xmlrpc method request out of body: {e:?}");
                warn!("{error_str}");
                return Ok(Response::builder()
                    .status(StatusCode::BAD_REQUEST)
                    .body(Body::from(error_str))
                    .unwrap());
            }
        };
        match method_name.as_str() {
            "getMasterUri" => {
                debug!("getMasterUri called");
                let lock = handle.inner.read().await;
                let uri = lock.get_master_uri();
                serde_xmlrpc::re
                return Ok(Response::builder().status(StatusCode::OK).body(Body::from()).unwrap());
            }
            "getPid" => {
                debug!("getPid called");
            }
            "getSubscriptions" => {
                debug!("getSubscriptions called")
            }
            "getPublications" => {
                debug!("getPublications called");
            }
            "paramUpdate" => {
                debug!("paramUpdate called");
            }
            "publisherUpdate" => {
                debug!("publisherUpdate called");
            }
            "requestTopic" => {
                debug!("requestTopic called");
            }
            // getBusStats, getBusInfo <= have decided not to impl these
            _ => {
                let error_str = format!("Client attempted call function {method_name} which is not implemented by the Node's xmlrpc server.");
                warn!("{error_str}");
                return Ok(Response::builder()
                    .status(StatusCode::NOT_IMPLEMENTED)
                    .body(Body::from(error_str))
                    .unwrap());
            }
        };
        info!("GOT REQUEST: {body:?}");
        Ok(Response::new("Hello World".into()))
    }
}