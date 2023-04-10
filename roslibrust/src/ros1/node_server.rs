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

    // Our actual service handler with our error type
    async fn respond_inner(
        handle: NodeHandle,
        body: hyper::Request<Body>,
    ) -> Result<Response<Body>, Response<Body>> {
        // Await the bytes of the body
        let body = hyper::body::to_bytes(body).await.map_err(|e| {
            Self::error_mapper(
                e,
                "Failed to get bytes from http request on xmlrpc server, request ignored",
                StatusCode::BAD_REQUEST,
            )
        })?;

        // Parse as string
        let body = String::from_utf8(body.to_vec()).map_err(|e| {
            Self::error_mapper(
                e,
                "Failed to parse http body as valid utf8 string, request ignored",
                StatusCode::BAD_REQUEST,
            )
        })?;

        // Parse as xmlrpc request
        let (method_name, args) = serde_xmlrpc::request_from_str(&body).map_err(|e| {
            Self::error_mapper(
                e,
                "Failed to parse valid xmlrpc method request out of body, request ignored",
                StatusCode::BAD_REQUEST,
            )
        })?;

        // Match on allowable functions
        match method_name.as_str() {
            "getMasterUri" => {
                debug!("getMasterUri called by {args:?}");
                let lock = handle.inner.read().await;
                let uri = lock.get_master_uri();
                Self::to_response(uri)
            }
            "getPid" => {
                debug!("getPid called by {args:?}");
                let pid = std::process::id();
                let pid: i32 = pid.try_into().map_err(|e| {
                    Self::error_mapper(e,
                         "Operation system returned a PID which does not fit into i32, and therefor cannot be sent via xmlrpc",
                         StatusCode::INTERNAL_SERVER_ERROR)})?;
                Self::to_response(pid)
            }
            "getSubscriptions" => {
                debug!("getSubscriptions called by {args:?}");
                let subs = handle.get_subscriptions().await;
                // Have to manually convert to value because of Vec specialization
                let subs = serde_xmlrpc::to_value(subs).map_err(|e| {
                    Self::error_mapper(
                        e,
                        "Subscriptions contained names which could not be validly serialized to xmlrpc",
                        StatusCode::INTERNAL_SERVER_ERROR)})?;
                Self::to_response(subs)
            }
            "getPublications" => {
                debug!("getPublications called by {args:?}");
                let pubs = handle.get_publications().await;
                let pubs = serde_xmlrpc::to_value(pubs).map_err(|e| {
                    Self::error_mapper(
                        e,
                        "Publications contained names which could not be validly serialized to xmlrpc",
                        StatusCode::INTERNAL_SERVER_ERROR)})?;
                Self::to_response(pubs)
            }
            "paramUpdate" => {
                debug!("paramUpdate called by {args:?}");
                if args.len() != 3 {

                }
                unimplemented!()
            }
            "publisherUpdate" => {
                debug!("publisherUpdate called by {args:?}");
                unimplemented!()
            }
            "requestTopic" => {
                debug!("requestTopic called by {args:?}");
                unimplemented!()
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
        }
    }

    // Helper function for converting serde_xmlrpc stuff into responses
    fn to_response(v: impl Into<serde_xmlrpc::Value>) -> Result<Response<Body>, Response<Body>> {
        serde_xmlrpc::response_to_string(
            vec![serde_xmlrpc::Value::Array(vec![
                0.into(),
                "".into(),
                v.into(),
            ])]
            .into_iter(),
        )
        .map_err(|e| {
            Self::error_mapper(
                e,
                "Failed to serialize response data into valid xml",
                StatusCode::INTERNAL_SERVER_ERROR,
            )
        })
        .map(|body| {
            Response::builder()
                .status(StatusCode::OK)
                .body(Body::from(body))
                .unwrap()
        })
    }

    // Helper function for converting error to http responses
    fn error_mapper(
        e: impl std::error::Error,
        msg: &str,
        code: hyper::http::StatusCode,
    ) -> Response<Body> {
        let error_msg = format!("{msg}: {e:?}");
        warn!("{error_msg}");
        Response::builder()
            .status(code)
            .body(Body::from(error_msg))
            .unwrap()
    }

    // Is the actual function we hand to hyper
    async fn respond(
        handle: NodeHandle,
        body: hyper::Request<Body>,
    ) -> Result<Response<Body>, Infallible> {
        // Call our inner function and unwrap error type into response
        match Self::respond_inner(handle, body).await {
            Ok(body) => Ok(body),
            Err(body) => Ok(body),
        }
    }
}

#[cfg(test)]
mod test {
    use log::debug;
    use serde::de::DeserializeOwned;
    use serde_xmlrpc::Value;

    use crate::NodeHandle;

    // Helper function for making a call to one of the node server apis
    async fn call_node_server<T: DeserializeOwned>(
        uri: &str,
        endpoint: &str,
        args: Vec<Value>,
    ) -> T {
        let client = reqwest::Client::new();
        let body = serde_xmlrpc::request_to_string(endpoint, args).unwrap();
        let res = client
            .post(uri)
            .body(body)
            .send()
            .await
            .unwrap()
            .text()
            .await
            .unwrap();
        debug!("Got response: {res:?}");
        let (error_code, debug_str, value): (i8, String, T) =
            serde_xmlrpc::response_from_str(&res).unwrap();

        assert_eq!(error_code, 0);
        assert_eq!(debug_str, "");
        value
    }

    #[tokio::test]
    async fn test_get_master_uri() {
        let nh = NodeHandle::new("http://localhost:11311", "/get_master_uri_test_node")
            .await
            .unwrap();
        let client_uri = nh.get_client_uri().await;

        let uri: String = call_node_server(
            &client_uri,
            "getMasterUri",
            vec!["/get_master_uri_tester".into()],
        )
        .await;
        assert_eq!(uri, "http://localhost:11311");
    }

    #[tokio::test]
    async fn test_get_subscriptions() {
        // Stub test until we can actually subscribe
        let nh = NodeHandle::new("http://localhost:11311", "/get_subscriptions_test_node")
            .await
            .unwrap();
        let client_uri = nh.get_client_uri().await;
        // TODO actually subscribe here

        let subs: Vec<(String, String)> = call_node_server(
            &client_uri,
            "getSubscriptions",
            vec!["/get_subscriptions_tester".into()],
        )
        .await;

        // TODO assert we get our subscription back here
    }

    #[tokio::test]
    async fn test_get_publications() {
        // Stub test until we can actually advertise
        let nh = NodeHandle::new("http://localhost:11311", "/get_subscriptions_test_node")
            .await
            .unwrap();
        let client_uri = nh.get_client_uri().await;
        // TODO actually subscribe here

        let pubs: Vec<(String, String)> = call_node_server(
            &client_uri,
            "getPublications",
            vec!["/get_publications_tester".into()],
        )
        .await;

        // TODO assert we get our subscription back here

    }
}
