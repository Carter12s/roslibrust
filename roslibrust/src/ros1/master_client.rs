//! This module is concerned with direct communication over xmlprc between the master

use log::*;

#[derive(thiserror::Error, Debug)]
pub enum RosMasterError {
    #[error("Failed to understand xmlrpc message: {0}")]
    InvalidXmlRpcMessage(#[from] serde_xmlrpc::Error),
    #[error("Failed to communicate with server: {0}")]
    ServerCommunicationFailure(#[from] reqwest::Error),
    #[error("Ros Master Reported an Internal Error: {0}")]
    MasterError(String),
    #[error("Failure running xmlrpc server: {0}")]
    HostIoError(#[from] hyper::Error),
    #[error("Failed to determine a valid ip address on which to host the nodes xmlrpc server, check that one of ROS_IP, ROS_HOSTNAME or the computer's hostname resolve to a valid Ipv4 address {0}")]
    HostIpResolutionFailure(String),
}

/// A client that exposes the API hosted by the [rosmaster](http://wiki.ros.org/ROS/Master_API)
// TODO consider exposing this type publicly
pub(crate) struct MasterClient {
    client: reqwest::Client,
    // Address at which the rosmaster should be found
    master_uri: String,
    // TODO this should become auto-generated via ROS_IP / ROS_HOSTNAME and having OS give us a port
    // Address at which this node should be reached
    client_uri: String,
    // An id for this node
    id: String,
}

/// Format of data returned by rosmaster's getSystemState
#[derive(Debug)]
struct StateEntry {
    topic: String,
    nodes: Vec<String>,
}

/// The complete list of publishers, subscribers, and service hosts know to the master
#[derive(Debug)]
pub struct SystemState {
    publishers: Vec<StateEntry>,
    subscribers: Vec<StateEntry>,
    service_providers: Vec<StateEntry>,
}

impl SystemState {
    /// Helper function for checking if a node is registered as a publisher of a given topic.
    /// Returns true iff the node is a publisher of that topic
    pub fn is_publishing(&self, topic: &str, node: &str) -> bool {
        let Some(entry) = self.publishers.iter().find(|entry| entry.topic.eq(topic)) else {
            return false;
        };
        entry
            .nodes
            .iter()
            .find(|name| name.as_str().eq(node))
            .is_some()
    }

    /// Helper function for checking if a node is registered as a subscriber of a given topic.
    /// Returns true iff the node is a subscriber of that topic.
    pub fn is_subscribed(&self, topic: &str, node: &str) -> bool {
        let Some(entry) = self.subscribers.iter().find(|entry| entry.topic.eq(topic)) else {
            return false;
        };
        entry
            .nodes
            .iter()
            .find(|name| name.as_str().eq(node))
            .is_some()
    }

    pub fn is_service_provider(&self, topic: &str, node: &str) -> bool {
        let Some(entry) = self
            .service_providers
            .iter()
            .find(|entry| entry.topic.eq(topic))
        else {
            return false;
        };
        entry
            .nodes
            .iter()
            .find(|name| name.as_str().eq(node))
            .is_some()
    }
}

impl MasterClient {
    /// Constructs a new client for communicating with a ros master
    /// - master_uri: Expects a fully resolved uri for the master e.g. "http://localhost:11311"
    /// - client_uri: The URI that should be told to other Nodes / Master to reach this nodes xmlrpc server
    /// - id: A client_id to use when communicating with the master, expected to be a valid ros name e.g. "/my_node"
    pub async fn new(
        master_uri: impl Into<String>,
        client_uri: impl Into<String>,
        id: impl Into<String>,
    ) -> Result<MasterClient, RosMasterError> {
        // Create a client, but we want to verify a valid connection before handing control back,
        // so we make an initial request and confirm with works before returning
        let client = MasterClient {
            client: reqwest::Client::new(),
            master_uri: master_uri.into(),
            client_uri: client_uri.into(),
            id: id.into(),
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

    /// Returns the master uri this client is configured to reach
    pub fn get_master_uri(&self) -> &str {
        &self.master_uri
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

    /// Hits the master's xmlrpc endpoint "registerService", does not actually begin hosting the
    /// service uri.
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

    /// Hits the master's xmlrpc endpoint "unregisterService", returns true if the service was
    /// un-registered and false if the master reports that the service was not registered as a
    /// provider
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

    /// Hits the master's xmlrpc endpoint "registerSubscriber", returns the list of publisher node
    /// URIs for that topic
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

    /// Hits the master's xmlrpc endpoint "unregisterSubscriber", returns true if the subscriber
    /// was registered for the topic and false if the server reported this operation as a no-op.
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

    /// Hits the master's xmlrpc endpoint "registerPublisher", returns the list of current
    /// subscriber node URIs to the topic
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

    /// Hits the master's xmlrpc endpoint "unregisterPublisher", returns true if the subscriber was
    /// registered for the topic and false if the server reported this operation as a no-op.
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

    /// Hits the master's xmlrpc endpoint "lookupNode" and returns the uri associated with the
    /// given node name
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

    /// Hits the master's xmlrpc endpoint "lookup_service" and returns the uri associated with the
    /// service
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
    /// This does not include topics which have been subscribed to, but have no publisher according
    /// to ROS's documentation.
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

    /// Returns where this client believes its own node's xmlrpc server is hosted at.
    /// This is simply a getter for the client_uri passed in while constructing this client.
    pub fn client_uri(&self) -> &str {
        &self.client_uri
    }

    /// Hits the master's xmlrpc endpoint "getSystemState" and returns the response
    pub async fn get_system_state(&self) -> Result<SystemState, RosMasterError> {
        // Comes in order of Publishers, Subscribers, Services
        // Each entry contains first the topic name then the list of nodes that publish/subscriber/provide that topic/service
        let body = serde_xmlrpc::request_to_string("getSystemState", vec![self.id.clone().into()])?;
        debug!("System State Body: {body}");
        let res: Vec<Vec<(String, Vec<String>)>> = self.post(body).await?;
        if res.len() != 3 {
            return Err(RosMasterError::InvalidXmlRpcMessage(
                serde_xmlrpc::Error::DecodeError(format!(
                    "Incorrect number of fields returned by getSystemState: {res:?}"
                )),
            ));
        }
        let mut res: Vec<Vec<StateEntry>> = res
            .into_iter()
            .map(|e| {
                e.into_iter()
                    .map(|(topic, nodes)| StateEntry { topic, nodes })
                    .collect()
            })
            .collect();

        // WARNING: order matters here:
        Ok(SystemState {
            service_providers: res.pop().unwrap(),
            subscribers: res.pop().unwrap(),
            publishers: res.pop().unwrap(),
        })
    }
}

#[cfg(feature = "ros1_test")]
#[cfg(test)]
mod test {

    use crate::{MasterClient, RosMasterError};

    const TEST_NODE_ID: &str = "/native_ros1_test";

    async fn test_client() -> Result<MasterClient, RosMasterError> {
        MasterClient::new(
            "http://localhost:11311",
            "http://localhost:11312",
            TEST_NODE_ID,
        )
        .await
    }

    #[test_log::test(tokio::test)]
    async fn get_system_state() -> Result<(), RosMasterError> {
        let _state = test_client().await?.get_system_state().await?;
        Ok(())
    }

    // Testing MasterClient's get_uri function returns a non-empty string
    #[test_log::test(tokio::test)]
    async fn test_get_uri() -> Result<(), RosMasterError> {
        assert!(!test_client().await?.get_uri().await?.is_empty());
        Ok(())
    }

    #[test_log::test(tokio::test)]
    async fn test_get_topic_types() {
        let topic_types = test_client()
            .await
            .unwrap()
            .get_topic_types()
            .await
            .unwrap();
        assert!(!topic_types.is_empty());
    }

    #[test_log::test(tokio::test)]
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

    #[test_log::test(tokio::test)]
    async fn test_register_and_unregister_subscriber() {
        let client = test_client().await.unwrap();

        // Register subscriber
        let topic = "/my_topic";
        let topic_type = "std_msgs/String";
        client.register_subscriber(topic, topic_type).await.unwrap();

        // Use system state to verify we are registered as subscribed
        let state = client.get_system_state().await.unwrap();
        assert!(state.is_subscribed(topic, TEST_NODE_ID));

        // Unregister subscriber
        assert!(client.unregister_subscriber(topic).await.unwrap());

        // Use system state to verify we are no longer registered
        let state = client.get_system_state().await.unwrap();
        assert!(!state.is_subscribed(topic, TEST_NODE_ID));
    }

    #[test_log::test(tokio::test)]
    async fn test_register_and_unregister_publisher() {
        let client = test_client().await.unwrap();

        // Register publisher
        let topic = "/my_topic";
        let topic_type = "std_msgs/String";
        client.register_publisher(topic, topic_type).await.unwrap();

        // Use system state to verify we are registered as a publisher
        let state = client.get_system_state().await.unwrap();
        assert!(state.is_publishing(topic, TEST_NODE_ID));

        // Unregister publisher
        assert!(client.unregister_publisher(topic).await.unwrap());

        // Use system state to verify we are no longer registered as a publisher
        let state = client.get_system_state().await.unwrap();
        assert!(!state.is_publishing(topic, TEST_NODE_ID));
    }

    #[test_log::test(tokio::test)]
    async fn test_lookup_node() {
        let client = test_client().await.unwrap();
        let node_name = "/rosout";
        let node_uri = client.lookup_node(node_name).await.unwrap();
        assert!(!node_uri.is_empty());
    }

    #[test_log::test(tokio::test)]
    async fn test_get_published_topics() {
        let client = test_client().await.unwrap();
        let subgraph = "";
        let topics = client.get_published_topics(subgraph).await.unwrap();
        assert!(!topics.is_empty());
    }
}
