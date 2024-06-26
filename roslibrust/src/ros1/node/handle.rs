use super::actor::{Node, NodeServerHandle};
use crate::{
    ros1::{
        names::Name, publisher::Publisher, service_client::ServiceClient, subscriber::Subscriber,
        NodeError, ServiceServer,
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
    /// Creates a new node, connects, and returns a handle to it
    /// It is idiomatic to call this once per process and treat the created node as singleton.
    /// The returned handle can be freely clone'd to create additional handles without creating additional connections.
    ///   - master_uri: Expects a fully resolved http uri for the master e.g. "http://my_host_name:11311"
    ///   - name: The name of the node, expected to be a valid ros name, all names are interpreted as 'global' in
    ///     ROS's namespace system. e.g. "my_node" -> "/my_node". "~my_node" is not supported
    pub async fn new(master_uri: &str, name: &str) -> Result<NodeHandle, NodeError> {
        let name = if name.starts_with("/") {
            Name::new(name)?
        } else {
            Name::new(&format!("/{}", name))?
        };

        // Extra safety check that our name resolves now
        let _ = Name::new("test").unwrap().resolve_to_global(&name);

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
    ) -> Result<ServiceClient<T>, NodeError> {
        let service_name = Name::new(service_name)?;
        let sender = self
            .inner
            .register_service_client::<T>(&service_name)
            .await?;
        Ok(ServiceClient::new(&service_name, sender))
    }

    pub async fn advertise_service<T, F>(
        &self,
        service_name: &str,
        server: F,
    ) -> Result<ServiceServer, NodeError>
    where
        T: roslibrust_codegen::RosServiceType,
        F: Fn(T::Request) -> Result<T::Response, Box<dyn std::error::Error + Send + Sync>>
            + Send
            + Sync
            + 'static,
    {
        let service_name = Name::new(service_name)?;
        let _response = self
            .inner
            .register_service_server::<T, F>(&service_name, server)
            .await?;
        Ok(ServiceServer::new(service_name, self.clone()))
    }
}
