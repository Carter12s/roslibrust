use super::NodeServerHandle;
use abort_on_drop::ChildTask;
use hyper::{Body, Response, StatusCode};
use log::*;
use std::{
    convert::Infallible,
    net::{Ipv4Addr, SocketAddr},
};

#[allow(unused)]
enum RosXmlStatusCode {
    Error,
    Failure,
    Success,
    Other(i32),
}

impl RosXmlStatusCode {
    pub fn code(&self) -> i32 {
        match self {
            RosXmlStatusCode::Error => -1,
            RosXmlStatusCode::Failure => 0,
            RosXmlStatusCode::Success => 1,
            RosXmlStatusCode::Other(code) => *code,
        }
    }
}

/// Hosts an xmlrpc API that rosmaster will call to notify of new subscribers / publishers etc.
/// Is also called by other ros nodes to initiate point to point connections.
/// Note: ROS uses "master/slave" terminology here. We continue to refer to the ROS central server as the ROS master,
/// but are intentionally using "XmlRpcServer" in place of where ROS says "Slave API"
pub(crate) struct XmlRpcServer {}

pub(crate) struct XmlRpcServerHandle {
    port: u16,
    _handle: ChildTask<()>,
}

impl XmlRpcServerHandle {
    pub fn port(&self) -> u16 {
        self.port
    }
}

impl XmlRpcServer {
    pub fn new(
        host_addr: Ipv4Addr,
        node_server: NodeServerHandle,
    ) -> Result<XmlRpcServerHandle, XmlRpcError> {
        let make_svc = hyper::service::make_service_fn(move |connection| {
            debug!("New node xmlrpc connection {connection:?}");
            let node_server = node_server.clone();
            async move {
                Ok::<_, Infallible>(hyper::service::service_fn(move |req| {
                    XmlRpcServer::respond(node_server.clone(), req)
                }))
            }
        });
        let host_addr = SocketAddr::from((host_addr, 0));
        let server = hyper::server::Server::try_bind(&host_addr.into())?;
        let server = server.serve(make_svc);
        let addr = server.local_addr();

        let handle = tokio::spawn(async {
            if let Err(err) = server.await {
                log::error!("xmlrpc server encountered error: {err:?}");
            }
        });

        Ok(XmlRpcServerHandle {
            port: addr.port(),
            _handle: handle.into(),
        })
    }

    // Our actual service handler with our error type
    async fn respond_inner(
        node_server: NodeServerHandle,
        body: hyper::Request<Body>,
    ) -> Result<Response<Body>, Response<Body>> {
        // Await the bytes of the body
        let body = hyper::body::to_bytes(body).await.map_err(|e| {
            Self::make_error_response(
                e,
                "Failed to get bytes from http request on xmlrpc server, request ignored",
                StatusCode::BAD_REQUEST,
            )
        })?;

        // Parse as string
        let body = String::from_utf8(body.to_vec()).map_err(|e| {
            Self::make_error_response(
                e,
                "Failed to parse http body as valid utf8 string, request ignored",
                StatusCode::BAD_REQUEST,
            )
        })?;

        // Parse as xmlrpc request
        let (method_name, args) = serde_xmlrpc::request_from_str(&body).map_err(|e| {
            Self::make_error_response(
                e,
                "Failed to parse valid xmlrpc method request out of body, request ignored",
                StatusCode::BAD_REQUEST,
            )
        })?;

        // Match on allowable functions
        match method_name.as_str() {
            "getMasterUri" => {
                debug!("getMasterUri called by {args:?}");
                match node_server.get_master_uri().await {
                    Ok(uri) => Self::to_response(uri),
                    Err(e) => Err(Self::make_error_response(
                        e,
                        "Unable to retrieve master URI",
                        StatusCode::INTERNAL_SERVER_ERROR,
                    )),
                }
            }
            "getPid" => {
                debug!("getPid called by {args:?}");
                let pid = std::process::id();
                let pid: i32 = pid.try_into().map_err(|e| {
                    Self::make_error_response(e,
                         "Operation system returned a PID which does not fit into i32, and therefor cannot be sent via xmlrpc",
                         StatusCode::INTERNAL_SERVER_ERROR)})?;
                Self::to_response(pid)
            }
            "getSubscriptions" => {
                debug!("getSubscriptions called by {args:?}");
                match node_server.get_subscriptions().await {
                    Ok(subs) => {
                        match serde_xmlrpc::to_value(subs) {
                            Ok(subs) => Self::to_response(subs),
                            Err(e) => Err(Self::make_error_response(
                                e,
                                "Subscriptions contained names which could not be validly serialized to xmlrpc",
                                StatusCode::INTERNAL_SERVER_ERROR))
                        }
                    },
                    Err(e) => Err(Self::make_error_response(e, "Unable to get subscriptions", StatusCode::INTERNAL_SERVER_ERROR))
                }
            }
            "getPublications" => {
                debug!("getPublications called by {args:?}");
                match node_server.get_publications().await {
                    Ok(pubs) => match serde_xmlrpc::to_value(pubs) {
                        Ok(pubs) => Self::to_response(pubs),
                        Err(e) => Err(Self::make_error_response(
                            e,
                            "Publications contained names which could not be validly serialized to xmlrpc",
                            StatusCode::INTERNAL_SERVER_ERROR))
                    },
                    Err(e) => Err(Self::make_error_response(e, "Unable to get publications", StatusCode::INTERNAL_SERVER_ERROR))
                }
            }
            "paramUpdate" => {
                // Not supporting params for first cut
                debug!("paramUpdate called by {args:?}");
                unimplemented!()
            }
            "publisherUpdate" => {
                debug!("publisherUpdate called by {args:?}");
                let (_caller_id, topic, publishers): (String, String, Vec<String>) =
                    serde_xmlrpc::from_values(args).map_err(|e| {
                        Self::make_error_response(
                            e,
                            "Failed to parse arguments to publisherUpdate",
                            StatusCode::BAD_REQUEST,
                        )
                    })?;
                node_server
                    .set_peer_publishers(topic, publishers)
                    .map_err(|e| {
                        Self::make_error_response(
                            e,
                            "Unable to set peer publishers",
                            StatusCode::INTERNAL_SERVER_ERROR,
                        )
                    })?;

                // ROS's API is for us to still return an int, but the value is literally named "ignore"...
                Self::to_response(0)
            }
            "requestTopic" => {
                debug!("requestTopic called by {args:?}");
                let (caller_id, topic, protocols): (String, String, Vec<Vec<String>>) =
                    serde_xmlrpc::from_values(args).map_err(|e| {
                        Self::make_error_response(
                            e,
                            "Failed to parse arguments to requestTopic",
                            StatusCode::BAD_REQUEST,
                        )
                    })?;
                let protocols = protocols.iter().flatten().cloned().collect::<Vec<_>>();
                debug!("Request for topic {topic} from {caller_id} via protocols {protocols:?}");
                let params = node_server
                    .request_topic(&caller_id, &topic, &protocols)
                    .await
                    .map_err(|e| {
                        Self::make_error_response(
                            e,
                            "Unable to get parameters for requested topic",
                            StatusCode::INTERNAL_SERVER_ERROR,
                        )
                    })?;

                let response = Self::make_success_response(
                    RosXmlStatusCode::Success,
                    format!("ready on {}:{}", params.hostname.clone(), params.port).as_str(),
                    serde_xmlrpc::to_value((params.protocol, params.hostname, params.port))
                        .unwrap(),
                );

                log::debug!("Sending response for requested topic {response:?}");
                Ok(response)
            }
            "shutdown" => {
                debug!("shutdown called by {args:?}");
                let (caller_id, msg): (String, String) =
                    serde_xmlrpc::from_values(args).map_err(|e| {
                        Self::make_error_response(
                            e,
                            "Failed to parse arguments",
                            StatusCode::BAD_REQUEST,
                        )
                    })?;
                debug!("Received request for shutdown from {caller_id}: {msg}");
                node_server.shutdown().map_err(|e| {
                    Self::make_error_response(
                        e,
                        "Unable to shutdown",
                        StatusCode::INTERNAL_SERVER_ERROR,
                    )
                })?;

                Self::to_response(0)
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

    fn make_success_response(
        status_code: RosXmlStatusCode,
        status_msg: &str,
        value: serde_xmlrpc::Value,
    ) -> Response<Body> {
        match serde_xmlrpc::response_to_string(
            vec![serde_xmlrpc::Value::Array(vec![
                status_code.code().into(),
                status_msg.into(),
                value,
            ])]
            .into_iter(),
        ) {
            Ok(body) => Response::builder()
                .status(StatusCode::OK)
                .body(Body::from(body))
                .unwrap(),
            Err(err) => Self::make_error_response(
                err,
                "Failed to serialize response data into valid xml",
                StatusCode::INTERNAL_SERVER_ERROR,
            ),
        }
    }

    // Helper function for converting serde_xmlrpc stuff into responses
    fn to_response(v: impl Into<serde_xmlrpc::Value>) -> Result<Response<Body>, Response<Body>> {
        serde_xmlrpc::response_to_string(
            vec![serde_xmlrpc::Value::Array(vec![
                1.into(),
                "".into(),
                v.into(),
            ])]
            .into_iter(),
        )
        .map_err(|e| {
            Self::make_error_response(
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
    fn make_error_response(
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
        node_server: NodeServerHandle,
        body: hyper::Request<Body>,
    ) -> Result<Response<Body>, Infallible> {
        // Call our inner function and unwrap error type into response
        match Self::respond_inner(node_server, body).await {
            Ok(body) => Ok(body),
            Err(body) => Ok(body),
        }
    }
}

#[derive(thiserror::Error, Debug)]
pub enum XmlRpcError {
    #[error(transparent)]
    HyperError(#[from] hyper::Error),
}
