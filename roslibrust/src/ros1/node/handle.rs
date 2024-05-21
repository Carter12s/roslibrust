use super::actor::{Node, NodeServerHandle};
use crate::{
    ros1::{
        names::Name, publisher::Publisher, service_client::ServiceClient, subscriber::Subscriber,
        NodeError,
    },
    RosLibRustResult,
};

/// Represents a handle to an underlying [Node]. NodeHandle's can be freely cloned, moved, copied, etc.
/// This class provides the user facing API for interacting with ROS.
#[derive(Clone)]
pub struct NodeHandle {
    inner: NodeServerHandle,
}

impl NodeHandle {
    // TODO builder, result, better error type
    /// Creates a new node connect and returns a handle to it
    /// It is idiomatic to call this once per process and treat the created node as singleton.
    /// The returned handle can be freely clone'd to create additional handles without creating additional connections.
    pub async fn new(master_uri: &str, name: &str) -> Result<NodeHandle, NodeError> {
        let name = Name::new(name)?;
        // Follow ROS rules and determine our IP and hostname
        let (addr, hostname) = super::determine_addr().await?;

        let node = Node::new(master_uri, &hostname, &name, addr).await?;
        let nh = NodeHandle { inner: node };

        Ok(nh)
    }

    pub fn is_ok(&self) -> bool {
        !self.inner.node_server_sender.is_closed()
    }

    pub async fn get_client_uri(&self) -> Result<String, NodeError> {
        self.inner.get_client_uri().await
    }

    pub async fn advertise<T: roslibrust_codegen::RosMessageType>(
        &self,
        topic_name: &str,
        queue_size: usize,
    ) -> Result<Publisher<T>, NodeError> {
        let sender = self
            .inner
            .register_publisher::<T>(topic_name, queue_size)
            .await?;
        Ok(Publisher::new(topic_name, sender))
    }

    pub async fn subscribe<T: roslibrust_codegen::RosMessageType>(
        &self,
        topic_name: &str,
        queue_size: usize,
    ) -> Result<Subscriber<T>, NodeError> {
        let receiver = self
            .inner
            .register_subscriber::<T>(topic_name, queue_size)
            .await?;
        Ok(Subscriber::new(receiver))
    }

    pub async fn service_client<T: roslibrust_codegen::RosServiceType>(
        &self,
        service_name: &str,
    ) -> RosLibRustResult<ServiceClient<T>> {
        let service_name = Name::new(service_name)?;
        let sender = self
            .inner
            .register_service_client::<T>(&service_name)
            .await?;
        Ok(ServiceClient::new(&service_name, sender))
    }
}
