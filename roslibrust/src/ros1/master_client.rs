//! This module is concerned with direct communication over xmlprc between the master

use std::net::TcpListener;

use log::*;
use tokio::net::ToSocketAddrs;

#[derive(thiserror::Error, Debug)]
pub enum RosMasterError {
    #[error("Failed to understand xmlrpc message: {0}")]
    InvalidXmlRpcMessage(#[from] serde_xmlrpc::Error),
    #[error("Failed to communicate with server: {0}")]
    ServerCommunicationFailure(#[from] reqwest::Error),
    #[error("Ros Master Reported an Internal Error: {0}")]
    MasterError(String),
    #[error("Failure creating node xmlrpc server: {0}")]
    HostIoError(#[from] std::io::Error),
}

/// A client that exposes the API hosted by the [rosmaster](http://wiki.ros.org/ROS/Master_API)
pub struct MasterClient {
    client: reqwest::Client,
    // Address at which the rosmaster should be found
    master_uri: String,
    // TODO this should become auto-generated via ROS_IP / ROS_HOSTNAME and having OS give us a port
    // Address at which this node should be reached
    client_uri: String,
    // An id for this node
    id: String,
    // Handle for our node server's task
    server_handle: tokio::task::JoinHandle<Result<(),RosMasterError>>,
}

// TODO: This is failing to decode system state...
/// These types are really just aliases to make the getSystemState's return type workable
// #[derive(serde::Deserialize, Debug)]
// pub struct ServiceList(Vec<(String, Vec<String>)>);
// #[derive(serde::Deserialize, Debug)]
// pub struct SubscriberList(Vec<(String, Vec<String>)>);
// #[derive(serde::Deserialize, Debug)]
// pub struct PublisherList(Vec<(String, Vec<String>)>);
// #[derive(serde::Deserialize, Debug)]
// pub struct SystemState((PublisherList, SubscriberList, ServiceList));

impl MasterClient {
    /// Constructs a new client for communicating with a ros master
    /// - master_uri: Expects a fully resolved uri for the master e.g. "http://localhost:11311"
    /// - client_uri: Expects a fully resolved name where this client should host its xmlrpc at
    /// - id: A client_id to use when communicating with the master, expected to be a valid ros name e.g. "/my_node"
    pub async fn new(
        master_uri: impl Into<String>,
        client_uri: impl Into<String>,
        id: impl Into<String>,
    ) -> Result<MasterClient, RosMasterError> {
        let client_uri: String = client_uri.into();
        let client_uri_copy = client_uri.clone();
        let server_handle = tokio::spawn(async {
           NodeServer::run(client_uri).await
        });

        // Create a client, but we want to verify a valid connection before handing control back,
        // so we make an initial request and confirm with works before returning
        let client = MasterClient {
            client: reqwest::Client::new(),
            master_uri: master_uri.into(),
            client_uri: client_uri_copy,
            id: id.into(),
            server_handle,
        };

        match client.get_uri().await {
            Ok(_) => Ok(client),
            Err(e) => Err(e),
        }
    }

    async fn post<T: serde::de::DeserializeOwned + std::fmt::Debug>(
        &self,
        request: String,
    ) -> Result<T, RosMasterError> {
        trace!("Sending master: {request}");
        let response = self
            .client
            .post(&self.master_uri)
            .body(request)
            .send()
            .await?
            .text()
            .await?;
        trace!("Got response: {response}");
        let (status_code, msg, data) =
            serde_xmlrpc::response_from_str::<(i8, String, T)>(&response)?;
        match status_code {
            1 => {
                trace!("Parsed from rosmaster: {msg:?} {data:?}");
            }
            _ => {
                return Err(RosMasterError::MasterError(msg));
            }
        };
        Ok(data)
    }

    /// Hits the master's xmlrpc endpoint "getUri" and provides the response
    pub async fn get_uri(&self) -> Result<String, RosMasterError> {
        let body = serde_xmlrpc::request_to_string("getUri", vec![self.id.clone().into()])?;
        self.post(body).await
    }

    /// Hits the master's xmlrpc endpoint "getTopicTypes" and provides the response as
    /// Vec<(topicName, topicType)>
    pub async fn get_topic_types(&self) -> Result<Vec<(String, String)>, RosMasterError> {
        let body = serde_xmlrpc::request_to_string("getTopicTypes", vec![self.id.clone().into()])?;
        self.post(body).await
    }

    /// Hits the master's xmlrpc endpoint "registerService", does not actually begin hosting the service uri.
    pub async fn register_service(
        &self,
        service: impl Into<String>,
        service_uri: impl Into<String>,
    ) -> Result<(), RosMasterError> {
        let body = serde_xmlrpc::request_to_string(
            "registerService",
            vec![
                self.id.clone().into(),
                service.into().into(),
                service_uri.into().into(),
                self.client_uri.clone().into(),
            ],
        )?;
        // Little conversion here to ignore third response parameter
        let _: u8 = self.post(body).await?;
        Ok(())
    }

    /// Hits the master's xmlrpc endpoint "unregisterService", returns true if the service was un-registered
    /// and false if the master reports that the service was not registered as a provider
    pub async fn unregister_service(
        &self,
        service: impl Into<String>,
        service_uri: impl Into<String>,
    ) -> Result<bool, RosMasterError> {
        let body = serde_xmlrpc::request_to_string(
            "unregisterService",
            vec![
                self.id.clone().into(),
                service.into().into(),
                service_uri.into().into(),
            ],
        )?;
        let x: u8 = self.post(body).await?;
        Ok(x.eq(&1))
    }

    /// Hits the master's xmlrpc endpoint "registerSubscriber", returns the list of publishers to that topic
    pub async fn register_subscriber(
        &self,
        topic: impl Into<String>,
        topic_type: impl Into<String>,
    ) -> Result<Vec<String>, RosMasterError> {
        let body = serde_xmlrpc::request_to_string(
            "registerSubscriber",
            vec![
                self.id.clone().into(),
                topic.into().into(),
                topic_type.into().into(),
                self.client_uri.clone().into(),
            ],
        )?;
        self.post(body).await
    }

    /// Hits the master's xmlrpc endpoint "unreisterSubscriber", returns true if the subscriber was registered
    /// for the topic and false if the server reported this operation as a no-op.
    pub async fn unregister_subscriber(
        &self,
        topic: impl Into<String>,
    ) -> Result<bool, RosMasterError> {
        let body = serde_xmlrpc::request_to_string(
            "unregisterSubscriber",
            vec![
                self.id.clone().into(),
                topic.into().into(),
                self.client_uri.clone().into(),
            ],
        )?;
        // little hack to convert response to bool
        let x: u8 = self.post(body).await?;
        Ok(x.eq(&1))
    }

    /// Hits the master's xmlrpc endpoint "registerPublisher", returns the list of current subscribers to the topic
    pub async fn register_publisher(
        &self,
        topic: impl Into<String>,
        topic_type: impl Into<String>,
    ) -> Result<Vec<String>, RosMasterError> {
        let body = serde_xmlrpc::request_to_string(
            "registerPublisher",
            vec![
                self.id.clone().into(),
                topic.into().into(),
                topic_type.into().into(),
                self.client_uri.clone().into(),
            ],
        )?;
        self.post(body).await
    }

    /// Hits the master's xmlrpc endpoint "unreisterPublisher", returns true if the subscriber was registered
    /// for the topic and false if the server reported this operation as a no-op.
    pub async fn unregister_publisher(
        &self,
        topic: impl Into<String>,
    ) -> Result<bool, RosMasterError> {
        let body = serde_xmlrpc::request_to_string(
            "unregisterPublisher",
            vec![
                self.id.clone().into(),
                topic.into().into(),
                self.client_uri.clone().into(),
            ],
        )?;
        // little hack to convert response to bool
        let x: u8 = self.post(body).await?;
        Ok(x.eq(&1))
    }

    /// Hits the master's xmlrpc endpoint "lookupNode" and returns the uri associated with the given node name
    pub async fn lookup_node(
        &self,
        node_name: impl Into<String>,
    ) -> Result<String, RosMasterError> {
        let body = serde_xmlrpc::request_to_string(
            "lookupNode",
            vec![self.id.clone().into(), node_name.into().into()],
        )?;
        self.post(body).await
    }

    /// Hits the master's xmlrpc endpoint "lookup_service" and returns the uri associated with the service
    pub async fn lookup_service(
        &self,
        service_name: impl Into<String>,
    ) -> Result<String, RosMasterError> {
        let body = serde_xmlrpc::request_to_string(
            "lookupService",
            vec![self.id.clone().into(), service_name.into().into()],
        )?;
        self.post(body).await
    }

    /// Hits the master's xmlrpc endpoint "getPublishedTopics" and returns the corresponding list.
    /// This does not include topics which have been subscribed to, but have no publisher according to ROS's
    /// documentation.
    /// - subgraph: Restrict topic names to match within the specified subgraph.
    ///             Subgraph namespace is resolved relative to the caller's namespace.
    ///             Use empty string to specify all names.
    /// Returns a Vec of tuples of (topic name, topic type)
    pub async fn get_published_topics(
        &self,
        subgraph: impl Into<String>,
    ) -> Result<Vec<(String, String)>, RosMasterError> {
        let body = serde_xmlrpc::request_to_string(
            "getPublishedTopics",
            vec![self.id.clone().into(), subgraph.into().into()],
        )?;
        self.post(body).await
    }

    // TODO not working
    // /// Hits the master's xmlrpc endpoint "getSystemState" and returns the response
    // pub async fn get_system_state(&self) -> Result<SystemState, RosMasterError> {
    //     let body = serde_xmlrpc::request_to_string("getSystemState", vec![self.id.clone().into()])?;
    //     self.post(body).await
    // }
}

/// Hosts an xmlrpc API that rosmaster will call to notify of new subscribers / publishers etc.
/// Is also called by other ros nodes to initiate point to point connections.
/// Note: ROS uses "master/slave" terminology here. We continue to refer to the ROS central server as the ROS master,
/// but are intentionally using "NodeServer" in place of where ROS says "Slave API"
struct NodeServer {
    socket: tokio::net::TcpListener,
}

impl NodeServer {
    // Simultaneously creates the server and starts receiving
    // This task returning indicates the server has stopped running
    async fn run(host_addr: impl ToSocketAddrs) -> Result<(), RosMasterError> {
        // TODO massaging around here with env vars and ports..
        let socket = tokio::net::TcpListener::bind(host_addr).await?;
        loop {
            // Do server stuff
        }
    }
}

#[cfg(feature = "ros1_test")]
#[cfg(test)]
mod test {

    use crate::{MasterClient, RosMasterError};

    // TODO may be a bug here in testing due to overlapping clients...
    async fn test_client() -> Result<MasterClient, RosMasterError> {
        MasterClient::new(
            "http://localhost:11311",
            "http://localhost:11312",
            "/native_ros1_test",
        )
        .await
    }

    // Testing MasterClient's get_uri function returns a non-empty string
    #[tokio::test]
    async fn test_get_uri() -> Result<(), RosMasterError> {
        assert!(!test_client().await?.get_uri().await?.is_empty());
        Ok(())
    }

    #[tokio::test]
    async fn test_get_topic_types() {
        let topic_types = test_client()
            .await
            .unwrap()
            .get_topic_types()
            .await
            .unwrap();
        assert!(!topic_types.is_empty());
    }

    #[tokio::test]
    async fn test_register_and_unregister_service() {
        let client = test_client().await.unwrap();
        let service = "/my_service";
        let service_uri = "http://localhost:11312";
        // Register
        client.register_service(service, service_uri).await.unwrap();

        // Confirm it exists
        assert_eq!(
            client.lookup_service("/my_service").await.unwrap(),
            service_uri
        );

        // Unregister service
        assert!(client
            .unregister_service(service, service_uri)
            .await
            .unwrap());
    }

    #[tokio::test]
    async fn test_register_and_unregister_subscriber() {
        let client = test_client().await.unwrap();

        // Register subscriber
        let topic = "/my_topic";
        let topic_type = "std_msgs/String";
        client.register_subscriber(topic, topic_type).await.unwrap();

        // Unregister subscriber
        client.unregister_subscriber(topic).await.unwrap();
    }

    #[tokio::test]
    async fn test_register_and_unregister_publisher() {
        let client = test_client().await.unwrap();

        // Register publisher
        let topic = "/my_topic";
        let topic_type = "std_msgs/String";
        client.register_publisher(topic, topic_type).await.unwrap();

        // Unregister publisher
        assert!(client.unregister_publisher(topic).await.unwrap());
    }

    #[tokio::test]
    async fn test_lookup_node() {
        let client = test_client().await.unwrap();
        let node_name = "/rosout";
        let node_uri = client.lookup_node(node_name).await.unwrap();
        assert!(!node_uri.is_empty());
    }

    #[tokio::test]
    async fn test_get_published_topics() {
        let client = test_client().await.unwrap();
        let subgraph = "";
        let topics = client.get_published_topics(subgraph).await.unwrap();
        assert!(!topics.is_empty());
    }
}
