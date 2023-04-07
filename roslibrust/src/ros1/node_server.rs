use std::{
    convert::Infallible,
    net::{Ipv4Addr, SocketAddr},
};

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
            let handle = handle.clone(); // Make a unique handle for each connection
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
                debug!("getMasterUri called by {args:?}");
                let lock = handle.inner.read().await;
                let uri = lock.get_master_uri();
                // Ros response is (int, str, str) (Status Code, Error Message, Master URI)
                // Double vec is because ROS is stupid
                let response = serde_xmlrpc::Value::Array(vec![0.into(), "".into(), uri.into()]);
                let body = match serde_xmlrpc::response_to_string(vec![response].into_iter()) {
                    Ok(b) => {
                        debug!("Responding: {b}");
                        b
                    }
                    Err(e) => {
                        let error_str =
                            format!("Failed to serialize result to valid xmlrpc: {e:?}");
                        warn!("{error_str}");
                        // TODO may be better to return an xmlrpc fault instead of an http error here?
                        return Ok(Response::builder()
                            .status(StatusCode::INTERNAL_SERVER_ERROR)
                            .body(Body::from(error_str))
                            .unwrap());
                    }
                };
                return Ok(Response::builder()
                    .status(StatusCode::OK)
                    .body(Body::from(body))
                    .unwrap());
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

#[cfg(test)]
mod test {
    use crate::{MasterClient, NodeHandle};

    #[tokio::test]
    async fn test_get_master_uri() {
        let nh = NodeHandle::new("http://localhost:11311", "/get_master_uri_test_node")
            .await
            .unwrap();
        let client_uri = nh.get_client_uri().await;

        // Manually call the nodes "getMasterUri" and check the response
        let client = reqwest::Client::new();
        let body =
            serde_xmlrpc::request_to_string("getMasterUri", vec!["/get_master_uri_tester".into()])
                .unwrap();
        let res = client
            .post(client_uri)
            .body(body)
            .send()
            .await
            .unwrap()
            .text()
            .await
            .unwrap();
        let (error_code, debug_str, uri): (i8, String, String) =
            serde_xmlrpc::response_from_str(&res).unwrap();

        assert_eq!(error_code, 0);
        assert_eq!(debug_str, "");
        assert_eq!(uri, "http://localhost:11311");
    }
}
