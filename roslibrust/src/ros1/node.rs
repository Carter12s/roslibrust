//! This module contains the top level Node and NodeHandle classes.
//! These wrap the lower level management of a ROS Node connection into a higher level and thread safe API.

use super::{
    names::Name,
    publisher::{Publication, Publisher},
    subscriber::{Subscriber, Subscription},
};
use crate::{MasterClient, RosMasterError, ServiceCallback, XmlRpcServer, XmlRpcServerHandle};
use abort_on_drop::ChildTask;
use dashmap::DashMap;
use roslibrust_codegen::RosMessageType;
use std::{
    net::{IpAddr, Ipv4Addr, ToSocketAddrs},
    sync::Arc,
};
use tokio::sync::{broadcast, mpsc, oneshot};

#[derive(Debug)]
pub struct ProtocolParams {
    pub hostname: String,
    pub protocol: String,
    pub port: u16,
}

#[derive(Debug)]
pub enum NodeMsg {
    GetMasterUri {
        reply: oneshot::Sender<String>,
    },
    GetClientUri {
        reply: oneshot::Sender<String>,
    },
    GetSubscriptions {
        reply: oneshot::Sender<Vec<(String, String)>>,
    },
    GetPublications {
        reply: oneshot::Sender<Vec<(String, String)>>,
    },
    SetPeerPublishers {
        topic: String,
        publishers: Vec<String>,
    },
    Shutdown,
    RegisterPublisher {
        reply: oneshot::Sender<Result<mpsc::Sender<Vec<u8>>, String>>,
        topic: String,
        topic_type: String,
        queue_size: usize,
        msg_definition: String,
        md5sum: String,
    },
    RegisterSubscriber {
        reply: oneshot::Sender<Result<broadcast::Receiver<Vec<u8>>, String>>,
        topic: String,
        topic_type: String,
        queue_size: usize,
        msg_definition: String,
        md5sum: String,
    },
    RequestTopic {
        reply: oneshot::Sender<Result<ProtocolParams, String>>,
        caller_id: String,
        topic: String,
        protocols: Vec<String>,
    },
}

#[derive(Clone)]
pub(crate) struct NodeServerHandle {
    node_server_sender: mpsc::UnboundedSender<NodeMsg>,
    // If this handle should keep the underlying node task alive it will hold an
    // Arc to the underlying node task. This is an option because internal handles
    // within the node shouldn't keep it alive (e.g. what we hand to xml server)
    _node_task: Option<Arc<ChildTask<()>>>,
}

impl NodeServerHandle {
    /// Get the URI of the master node.
    pub async fn get_master_uri(&self) -> Result<String, Box<dyn std::error::Error>> {
        let (sender, receiver) = oneshot::channel();
        match self
            .node_server_sender
            .send(NodeMsg::GetMasterUri { reply: sender })
        {
            Ok(()) => Ok(receiver.await.map_err(|err| Box::new(err))?),
            Err(e) => Err(Box::new(e)),
        }
    }

    pub async fn get_client_uri(&self) -> Result<String, Box<dyn std::error::Error + Send + Sync>> {
        let (sender, receiver) = oneshot::channel();
        match self
            .node_server_sender
            .send(NodeMsg::GetClientUri { reply: sender })
        {
            Ok(()) => Ok(receiver.await.map_err(|err| Box::new(err))?),
            Err(e) => Err(Box::new(e)),
        }
    }

    /// Gets the list of topics the node is currently subscribed to.
    /// Returns a tuple of (Topic Name, Topic Type) e.g. ("/rosout", "rosgraph_msgs/Log").
    pub async fn get_subscriptions(
        &self,
    ) -> Result<Vec<(String, String)>, Box<dyn std::error::Error>> {
        let (sender, receiver) = oneshot::channel();
        match self
            .node_server_sender
            .send(NodeMsg::GetSubscriptions { reply: sender })
        {
            Ok(()) => Ok(receiver.await.map_err(|err| Box::new(err))?),
            Err(e) => Err(Box::new(e)),
        }
    }

    /// Gets the list of topic the node is currently publishing to.
    /// Returns a tuple of (Topic Name, Topic Type) e.g. ("/rosout", "rosgraph_msgs/Log").
    pub async fn get_publications(
        &self,
    ) -> Result<Vec<(String, String)>, Box<dyn std::error::Error>> {
        log::debug!("get_publications reached");
        let (sender, receiver) = oneshot::channel();
        match self
            .node_server_sender
            .send(NodeMsg::GetPublications { reply: sender })
        {
            Ok(()) => {
                log::debug!("Awaiting receiver");
                Ok(receiver.await.map_err(|err| Box::new(err))?)
            }
            Err(e) => Err(Box::new(e)),
        }
    }

    /// Updates the list of know publishers for a given topic
    /// This is used to know who to reach out to for updates
    pub fn set_peer_publishers(
        &self,
        topic: String,
        publishers: Vec<String>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        Ok(self
            .node_server_sender
            .send(NodeMsg::SetPeerPublishers { topic, publishers })
            .map_err(|err| Box::new(err))?)
    }

    pub fn shutdown(&self) -> Result<(), Box<dyn std::error::Error>> {
        self.node_server_sender
            .send(NodeMsg::Shutdown)
            .map_err(|err| Box::new(err))?;
        Ok(())
    }

    pub async fn register_publisher<T: RosMessageType>(
        &self,
        topic: &str,
        topic_type: &str,
        queue_size: usize,
    ) -> Result<mpsc::Sender<Vec<u8>>, Box<dyn std::error::Error + Send + Sync>> {
        let (sender, receiver) = oneshot::channel();
        match self.node_server_sender.send(NodeMsg::RegisterPublisher {
            reply: sender,
            topic: topic.to_owned(),
            topic_type: topic_type.to_owned(),
            queue_size,
            msg_definition: T::DEFINITION.to_owned(),
            md5sum: T::MD5SUM.to_owned(),
        }) {
            Ok(()) => {
                log::trace!("Recieved okay from node_server_send.");
                let received = receiver.await.map_err(|err| Box::new(err))?;
                log::trace!("Awaiting receiver produced: {received:?}");
                Ok(received.map_err(|err| {
                    log::error!("Failed to register publisher: {err}");
                    Box::new(std::io::Error::from(std::io::ErrorKind::ConnectionAborted))
                })?)
            }
            Err(err) => Err(Box::new(err)),
        }
    }

    pub async fn register_subscriber<T: RosMessageType>(
        &self,
        topic: &str,
        queue_size: usize,
    ) -> Result<broadcast::Receiver<Vec<u8>>, Box<dyn std::error::Error + Send + Sync>> {
        let (sender, receiver) = oneshot::channel();
        match self.node_server_sender.send(NodeMsg::RegisterSubscriber {
            reply: sender,
            topic: topic.to_owned(),
            topic_type: T::ROS_TYPE_NAME.to_owned(),
            queue_size,
            msg_definition: T::DEFINITION.to_owned(),
            md5sum: T::MD5SUM.to_owned(),
        }) {
            Ok(()) => {
                let received = receiver.await.map_err(|err| Box::new(err))?;
                Ok(received.map_err(|err| {
                    log::error!("Failed to register subscriber: {err}");
                    Box::new(std::io::Error::from(std::io::ErrorKind::ConnectionAborted))
                })?)
            }
            Err(err) => Err(Box::new(err)),
        }
    }

    pub async fn request_topic(
        &self,
        caller_id: &str,
        topic: &str,
        protocols: &[String],
    ) -> Result<ProtocolParams, Box<dyn std::error::Error>> {
        let (sender, receiver) = oneshot::channel();
        match self.node_server_sender.send(NodeMsg::RequestTopic {
            caller_id: caller_id.to_owned(),
            topic: topic.to_owned(),
            protocols: protocols.into(),
            reply: sender,
        }) {
            Ok(()) => {
                let received = receiver.await.map_err(|err| Box::new(err))?;
                Ok(received.map_err(|err| {
                    log::error!(
                        "Fail to coordinate channel between publisher and subscriber: {err}"
                    );
                    Box::new(std::io::Error::from(std::io::ErrorKind::AddrNotAvailable))
                })?)
            }
            Err(e) => Err(Box::new(e)),
        }
    }
}

/// Represents a single "real" node, typically only one of these is expected per process
/// but nothing should specifically prevent that.
pub struct Node {
    // The xmlrpc client this node uses to make requests to master
    client: MasterClient,
    // Server which handles updates from the rosmaster and other ROS nodes
    _xmlrpc_server: XmlRpcServerHandle,
    // Receiver for requests to the Node actor
    node_msg_rx: mpsc::UnboundedReceiver<NodeMsg>,
    // Map of topic names to the publishing channels associated with the topic
    publishers: tokio::sync::RwLock<std::collections::HashMap<String, Publication>>,
    // Record of subscriptions this node has
    subscriptions: DashMap<String, Subscription>,
    // Record of what services this node is serving
    services: DashMap<String, ServiceCallback>,
    // TODO need signal to shutdown xmlrpc server when node is dropped
    host_addr: Ipv4Addr,
    hostname: String,
    node_name: String,
}

impl Node {
    async fn new(
        master_uri: &str,
        hostname: &str,
        node_name: &str,
        addr: Ipv4Addr,
    ) -> Result<NodeServerHandle, Box<dyn std::error::Error + Send + Sync>> {
        let (node_sender, node_receiver) = mpsc::unbounded_channel();
        let xml_server_handle = NodeServerHandle {
            node_server_sender: node_sender.clone(),
            // None here because this handle should not keep task alive
            _node_task: None,
        };
        // Create our xmlrpc server and bind our socket so we know our port and can determine our local URI
        let xmlrpc_server = XmlRpcServer::new(addr, xml_server_handle)?;
        let client_uri = format!("http://{hostname}:{}", xmlrpc_server.port());

        if let None = Name::new(node_name) {
            log::error!("Node name {node_name} is not valid");
            return Err(Box::new(std::io::Error::from(
                std::io::ErrorKind::InvalidInput,
            )));
        }

        let rosmaster_client = MasterClient::new(master_uri, client_uri, node_name).await?;
        let mut node = Self {
            client: rosmaster_client,
            _xmlrpc_server: xmlrpc_server,
            node_msg_rx: node_receiver,
            publishers: tokio::sync::RwLock::new(std::collections::HashMap::new()),
            subscriptions: DashMap::new(),
            services: DashMap::new(),
            host_addr: addr,
            hostname: hostname.to_owned(),
            node_name: node_name.to_owned(),
        };

        let t = Arc::new(
            tokio::spawn(async move {
                loop {
                    match node.node_msg_rx.recv().await {
                        Some(NodeMsg::Shutdown) => {
                            log::info!("Shutdown requested, shutting down node");
                            break;
                        }
                        Some(node_msg) => {
                            node.handle_msg(node_msg).await;
                        }
                        None => {
                            break;
                        }
                    }
                }
            })
            .into(),
        );

        let node_server_handle = NodeServerHandle {
            node_server_sender: node_sender,
            _node_task: Some(t),
        };
        Ok(node_server_handle)
    }

    async fn handle_msg(&mut self, msg: NodeMsg) {
        match msg {
            NodeMsg::GetMasterUri { reply } => {
                let _ = reply.send(self.client.get_master_uri().to_owned());
            }
            NodeMsg::GetClientUri { reply } => {
                let _ = reply.send(self.client.client_uri().to_owned());
            }
            NodeMsg::GetSubscriptions { reply } => {
                let _ = reply.send(
                    self.subscriptions
                        .iter()
                        .map(|entry| (entry.key().clone(), entry.topic_type().to_owned()))
                        .collect(),
                );
            }
            NodeMsg::GetPublications { reply } => {
                let _ = reply.send(
                    self.publishers
                        .read()
                        .await
                        .iter()
                        .map(|(key, entry)| (key.clone(), entry.topic_type().to_owned()))
                        .collect(),
                );
            }
            NodeMsg::SetPeerPublishers { topic, publishers } => {
                if let Some(mut subscription) = self.subscriptions.get_mut(&topic) {
                    for publisher_uri in publishers {
                        if let Err(err) = subscription.add_publisher_source(&publisher_uri).await {
                            log::error!(
                                "Unable to create subscribe stream for topic {topic}: {err}"
                            );
                        }
                    }
                } else {
                    log::warn!(
                        "Got peer publisher update for topic we weren't subscribed to, ignoring"
                    );
                }
            }
            NodeMsg::RegisterPublisher {
                reply,
                topic,
                topic_type,
                queue_size,
                msg_definition,
                md5sum,
            } => {
                log::trace!("Got register publisher command from mpsc for {topic:?}");
                let res = self
                    .register_publisher(topic, &topic_type, queue_size, msg_definition, md5sum)
                    .await;
                log::trace!("Result of register_publisher: {res:?}");
                match res {
                    Ok(handle) => reply.send(Ok(handle)),
                    Err(err) => reply.send(Err(err.to_string())),
                }
                .expect("Failed to reply on oneshot");
            }
            NodeMsg::RegisterSubscriber {
                reply,
                topic,
                topic_type,
                queue_size,
                msg_definition,
                md5sum,
            } => {
                let _ = reply.send(
                    self.register_subscriber(
                        &topic,
                        &topic_type,
                        queue_size,
                        &msg_definition,
                        &md5sum,
                    )
                    .await
                    .map_err(|err| err.to_string()),
                );
            }
            NodeMsg::RequestTopic {
                reply,
                topic,
                protocols,
                ..
            } => {
                // TODO: Should move the actual implementation similar to RegisterPublisher
                if protocols
                    .iter()
                    .find(|proto| proto.as_str() == "TCPROS")
                    .is_some()
                {
                    if let Some((_key, publishing_channel)) = self
                        .publishers
                        .read()
                        .await
                        .iter()
                        .find(|(key, _pub)| *key == &topic)
                    {
                        let protocol_params = ProtocolParams {
                            hostname: self.hostname.clone(),
                            protocol: String::from("TCPROS"), // Hardcoded as the only option for now
                            port: publishing_channel.port(),
                        };
                        let _ = reply.send(Ok(protocol_params));
                    } else {
                        let err_str = format!("Got request for topic {topic} from subscriber which this node does not publish");
                        log::warn!("{err_str}");
                        let _ = reply.send(Err(err_str));
                    }
                } else {
                    let err_str = format!(
                        "No supported protocols in the request from the subscriber: {protocols:?}"
                    );
                    log::error!("{err_str}");
                    let _ = reply.send(Err(err_str));
                }
            }
            NodeMsg::Shutdown => {
                unreachable!("This node msg is handled in the wrapping handling code");
            }
        }
    }

    async fn register_subscriber(
        &mut self,
        topic: &str,
        topic_type: &str,
        queue_size: usize,
        msg_definition: &str,
        md5sum: &str,
    ) -> Result<broadcast::Receiver<Vec<u8>>, Box<dyn std::error::Error>> {
        match self
            .subscriptions
            .iter()
            .find(|subscription| subscription.key() == topic)
        {
            Some(subscription) => Ok(subscription.get_receiver()),
            None => {
                let mut subscription = Subscription::new(
                    &self.node_name,
                    &topic,
                    &topic_type,
                    queue_size,
                    msg_definition.to_owned(),
                    md5sum.to_owned(),
                );
                let current_publishers = self.client.register_subscriber(topic, topic_type).await?;
                for publisher in current_publishers {
                    if let Err(err) = subscription.add_publisher_source(&publisher).await {
                        log::error!("Unable to create subscriber connection to {publisher} for {topic}: {err}");
                    }
                }
                let receiver = subscription.get_receiver();
                self.subscriptions.insert(topic.to_owned(), subscription);
                Ok(receiver)
            }
        }
    }

    async fn register_publisher(
        &mut self,
        topic: String,
        topic_type: &str,
        queue_size: usize,
        msg_definition: String,
        md5sum: String,
    ) -> Result<mpsc::Sender<Vec<u8>>, Box<dyn std::error::Error>> {
        log::trace!("Registering publisher for: {topic:?}");

        let existing_entry = {
            let read_lock = self.publishers.read().await;
            read_lock.iter().find_map(|(key, value)| {
                log::trace!("Found existing entry for: {topic:?}");
                if key.as_str() == &topic {
                    if value.topic_type() == topic_type {
                        Some(Ok(value.get_sender()))
                    } else {
                        Some(Err(Box::new(std::io::Error::from(
                            std::io::ErrorKind::AddrInUse,
                        ))))
                    }
                } else {
                    None
                }
            })
        };

        if let Some(handle) = existing_entry {
            Ok(handle?)
        } else {
            log::trace!("Creating new entry for {topic:?}");
            let channel = Publication::new(
                &self.node_name,
                false,
                &topic,
                self.host_addr,
                queue_size,
                &msg_definition,
                &md5sum,
                topic_type,
            )
            .await
            .map_err(|err| {
                log::error!("Failed to create publishing channel: {err:?}");
                err
            })?;
            log::trace!("Created new publication for {topic:?}");
            let handle = channel.get_sender();
            {
                let mut write_lock = self.publishers.write().await;
                write_lock.insert(topic.clone(), channel);
            }
            log::trace!("Inserted new publsiher into dashmap");
            let _current_subscribers = self.client.register_publisher(&topic, topic_type).await?;
            log::trace!("Registered new publication for {topic:?}");
            Ok(handle)
        }
    }
}

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
    pub async fn new(
        master_uri: &str,
        name: &str,
    ) -> Result<NodeHandle, Box<dyn std::error::Error + Send + Sync>> {
        // Follow ROS rules and determine our IP and hostname
        let (addr, hostname) = determine_addr().await?;

        let node = Node::new(master_uri, &hostname, name, addr).await?;
        let nh = NodeHandle { inner: node };

        Ok(nh)
    }

    pub fn is_ok(&self) -> bool {
        !self.inner.node_server_sender.is_closed()
    }

    pub async fn get_client_uri(&self) -> Result<String, Box<dyn std::error::Error + Send + Sync>> {
        self.inner.get_client_uri().await
    }

    pub async fn advertise<T: roslibrust_codegen::RosMessageType>(
        &self,
        topic_name: &str,
        queue_size: usize,
    ) -> Result<Publisher<T>, Box<dyn std::error::Error + Send + Sync>> {
        log::trace!("Advertising: {topic_name:?}");
        let sender = self
            .inner
            .register_publisher::<T>(topic_name, T::ROS_TYPE_NAME, queue_size)
            .await?;
        log::trace!("Advertised: {topic_name:?}");
        Ok(Publisher::new(topic_name, sender))
    }

    pub async fn subscribe<T: roslibrust_codegen::RosMessageType>(
        &self,
        topic_name: &str,
        queue_size: usize,
    ) -> Result<Subscriber<T>, Box<dyn std::error::Error + Send + Sync>> {
        let receiver = self
            .inner
            .register_subscriber::<T>(topic_name, queue_size)
            .await?;
        Ok(Subscriber::new(receiver))
    }
}

// TODO at the end of the day I'd like to offer a builder pattern for configuration that allow manual setting of this or "ros idiomatic" behavior - Carter
/// Following ROS's idiomatic address rules uses ROS_HOSTNAME and ROS_IP to determine the address that server should be hosted at.
/// Returns both the resolved IpAddress of the host (used for actually opening the socket), and the String "hostname" which should
/// be used in the URI.
async fn determine_addr() -> Result<(Ipv4Addr, String), RosMasterError> {
    // If ROS_IP is set that trumps anything else
    if let Ok(ip_str) = std::env::var("ROS_IP") {
        let ip = ip_str.parse().map_err(|e| {
            RosMasterError::HostIpResolutionFailure(format!(
                "ROS_IP environment variable did not parse to a valid IpAddr::V4: {e:?}"
            ))
        })?;
        return Ok((ip, ip_str));
    }
    // If ROS_HOSTNAME is set that is next highest precedent
    if let Ok(name) = std::env::var("ROS_HOSTNAME") {
        let ip = hostname_to_ipv4(&name).await?;
        return Ok((ip, name));
    }
    // If neither env var is set, use the computers "hostname"
    let name = gethostname::gethostname();
    let name = name.into_string().map_err(|e| {
            RosMasterError::HostIpResolutionFailure(format!("This host's hostname is a string that cannot be validly converted into a Rust type, and therefore we cannot convert it into an IpAddrv4: {e:?}"))
        })?;
    let ip = hostname_to_ipv4(&name).await?;
    return Ok((ip, name));
}

/// Given a the name of a host use's std::net::ToSocketAddrs to perform a DNS lookup and return the resulting IP address.
/// This function is intended to be used to determine the correct IP host the socket for the xmlrpc server on.
async fn hostname_to_ipv4(name: &str) -> Result<Ipv4Addr, RosMasterError> {
    let name_with_port = &format!("{name}:0");
    let mut i = tokio::net::lookup_host(name_with_port).await.map_err(|e| {
        RosMasterError::HostIpResolutionFailure(format!(
            "Failure while attempting to lookup ROS_HOSTNAME: {e:?}"
        ))
    })?;
    if let Some(addr) = i.next() {
        match addr.ip() {
                IpAddr::V4(ip) => Ok(ip),
                IpAddr::V6(ip) => {
                    Err(RosMasterError::HostIpResolutionFailure(format!("ROS_HOSTNAME resolved to an IPv6 address which is not support by ROS/roslibrust: {ip:?}")))
                }
            }
    } else {
        Err(RosMasterError::HostIpResolutionFailure(format!(
            "ROS_HOSTNAME did not resolve any address: {name:?}"
        )))
    }
}
