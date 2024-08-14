use super::actor::{Node, NodeServerHandle};
use crate::{
    ros1::{
        names::Name, publisher::Publisher, service_client::ServiceClient, subscriber::Subscriber,
        subscriber::SubscriberAny,
        NodeError, ServiceServer,
    },
    ServiceFn,
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

    /// This function may be removed...
    /// All node handles connect to a backend node server that actually handles the communication with ROS
    /// If this function returns false, the backend node server has shut down and this handle is invalid.
    /// This state should be unreachable by normal usage of the library.
    pub fn is_ok(&self) -> bool {
        !self.inner.node_server_sender.is_closed()
    }

    /// Returns the network uri of XMLRPC server for the underlying node.
    /// This is address where ROS master communicates with the node.
    pub async fn get_client_uri(&self) -> Result<String, NodeError> {
        self.inner.get_client_uri().await
    }

    /// Create a new publisher for the given type.
    ///
    /// This function can be called multiple times to create multiple publishers for the same topic,
    /// however the FIRST call will establish the queue size and latching behavior for the topic.
    /// Subsequent calls will simply be given additional handles to the underlying publication.
    /// This behavior was chosen to mirror ROS1's API, however it is reccomended to .clone() the returend publisher
    /// instead of calling this function multiple times.
    pub async fn advertise<T: roslibrust_codegen::RosMessageType>(
        &self,
        topic_name: &str,
        queue_size: usize,
        latching: bool,
    ) -> Result<Publisher<T>, NodeError> {
        let sender = self
            .inner
            .register_publisher::<T>(topic_name, queue_size, latching)
            .await?;
        Ok(Publisher::new(topic_name, sender))
    }

    pub async fn subscribe_any(
        &self,
        topic_name: &str,
        queue_size: usize,
    ) -> Result<SubscriberAny, NodeError> {
        let receiver = self
            .inner
            .register_subscriber::<roslibrust_codegen::ShapeShifter>(topic_name, queue_size)
            .await?;
        Ok(SubscriberAny::new(receiver))
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
        Ok(sender)
    }

    pub async fn advertise_service<T, F>(
        &self,
        service_name: &str,
        server: F,
    ) -> Result<ServiceServer, NodeError>
    where
        T: roslibrust_codegen::RosServiceType,
        F: ServiceFn<T>,
    {
        let service_name = Name::new(service_name)?;
        let _response = self
            .inner
            .register_service_server::<T, F>(&service_name, server)
            .await?;
        Ok(ServiceServer::new(service_name, self.clone()))
    }

    // TODO Major: This should probably be moved to NodeServerHandle?
    /// Not intended to be called manually
    /// Stops hosting the specified server.
    /// This is automatically called when dropping the ServiceServer returned by [advertise_service]
    pub(crate) fn unadvertise_service_server(&self, service_name: &str) -> Result<(), NodeError> {
        // TODO should we be using Name as the type of service_name here?
        // I don't love Name's API at the moment
        // This function is intended to be called in a "Drop impl" which is non-async
        // so we're wrapping in a task here.
        // This should be fine due to the "cmd dispatch" that is the current communication mechanism with NodeServer
        let copy = self.clone();
        let name_copy = service_name.to_string();
        tokio::spawn(async move {
            let result = copy.inner.unadvertise_service(&name_copy).await;
            if let Err(e) = result {
                log::error!("Failed to undvertise service: {e:?}");
            }
        });
        Ok(())
    }
}
