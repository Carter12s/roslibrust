//! This module contains the top level Node and NodeHandle classes.
//! These wrap the lower level management of a ROS Node connection into a higher level and thread safe API.

use std::{
    net::{IpAddr, Ipv4Addr, ToSocketAddrs},
    sync::Arc,
};

use dashmap::DashMap;

use crate::{
    MasterClient, NodeServer, PublisherHandle, RosMasterError, ServiceCallback, Subscription,
};

/// Represents a single "real" node, typically only one of these is expected per process
/// but nothing should specifically prevent that.
pub struct Node {
    client: Option<MasterClient>,
    publishers: DashMap<String, PublisherHandle>,
    subscriptions: DashMap<String, Subscription>,
    services: DashMap<String, ServiceCallback>,
}

impl Node {
    // TODO better error
    async fn new() -> Result<Node, Box<dyn std::error::Error + Send + Sync>> {
        // We have an initialization loop, because we don't have all the information we
        // need to create our client until we spawn our server and actually bind our
        // local TCP port, but our server needs a NodeHandle...
        // So I'm breaking RAII and assigning client after construction.
        Ok(Node {
            client: None,
            publishers: DashMap::new(),
            subscriptions: DashMap::new(),
            services: DashMap::new(),
        })
    }

    fn set_client(&mut self, client: MasterClient) {
        self.client = Some(client);
    }

    pub(crate) fn get_master_uri(&self) -> String {
        // Client is option due to connection loop issue, but always valid to unwrap
        self.client.as_ref().unwrap().get_master_uri().to_string()
    }
}

/// Represents a handle to an underlying [Node]. NodeHandle's can be freely cloned, moved, copied, etc.
/// This class provides the user facing API for interacting with ROS.
#[derive(Clone)]
pub struct NodeHandle {
    pub(crate) inner: Arc<tokio::sync::RwLock<Node>>, // TODO we may be able to get away with no RwLock here?
}

impl NodeHandle {
    // TODO builder, async, result, better error type
    /// Creates a new node connect and returns a handle to it
    /// It is idiomatic to call this once per process and treat the created node as singleton.
    /// The returned handle can be freely clone'd to create additional handles without creating additional connections.
    pub async fn new(
        master_uri: &str,
        name: &str,
    ) -> Result<NodeHandle, Box<dyn std::error::Error + Send + Sync>> {
        // Create the central "Node" data record
        let nh = NodeHandle {
            inner: Arc::new(tokio::sync::RwLock::new(Node::new().await?)),
        };

        // Follow ROS rules and determine our IP and hostname
        let (addr, hostname) = Self::determine_addr()?;

        // Create our xmlrpc server and bind our socket so we know our port and can determine our local URI
        let port = NodeServer::new(nh.clone(), addr);

        let client_uri = format!("http://{hostname}:{port}");
        let client = MasterClient::new(master_uri, client_uri, name).await?;
        nh.inner.write().await.set_client(client);

        // TODO spawn our TcpManager here for TCPROS

        Ok(nh)
    }

    // TODO at the end of the day I'd like to offer a builder pattern for configuration that allow manual setting of this or "ros idiomatic" behavior - Carter
    /// Following ROS's idiomatic address rules uses ROS_HOSTNAME and ROS_IP to determine the address that server should be hosted at.
    /// Returns both the resolved IpAddress of the host (used for actually opening the socket), and the String "hostname" which should
    /// be used in the URI.
    fn determine_addr() -> Result<(Ipv4Addr, String), RosMasterError> {
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
            let ip = Self::hostname_to_ipv4(&name)?;
            return Ok((ip, name));
        }
        // If neither env var is set, use the computers "hostname"
        let name = gethostname::gethostname();
        let name = name.into_string().map_err(|e| {
            RosMasterError::HostIpResolutionFailure(format!("This host's hostname is a string that cannot be validly converted into a Rust type, and therefore we cannot convert it into an IpAddrv4: {e:?}"))
        })?;
        let ip = Self::hostname_to_ipv4(&name)?;
        return Ok((ip, name));
    }

    /// Given a the name of a host use's std::net::ToSocketAddrs to perform a DNS lookup and return the resulting IP address.
    /// This function is intended to be used to determine the correct IP host the socket for the xmlrpc server on.
    fn hostname_to_ipv4(name: &str) -> Result<Ipv4Addr, RosMasterError> {
        let mut i = (name, 0).to_socket_addrs().map_err(|e| {
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

    // TODO pub vs. pub(crate)
    /// Gets the URI at which this node is hosting its xmlrpc server
    pub async fn get_client_uri(&self) -> String {
        self.inner
            .read()
            .await
            .client
            .as_ref()
            .unwrap()
            .client_uri()
            .to_string()
    }

    /// Gets the list of topics the node is currently subscribed to.
    /// Returns a tuple of (Topic Name, Topic Type) e.g. ("/rosout", "rosgraph_msgs/Log").
    pub async fn get_subscriptions(&self) -> Vec<(String, String)> {
        self.inner
            .read()
            .await
            .subscriptions
            .iter()
            .map(|entry| (entry.key().clone(), entry.topic_type.clone()))
            .collect()
    }

    /// Gets the list of topic the node is currently publishing to.
    /// Returns a tuple of (Topic Name, Topic Type) e.g. ("/rosout", "rosgraph_msgs/Log").
    pub async fn get_publications(&self) -> Vec<(String, String)> {
        self.inner
            .read()
            .await
            .publishers
            .iter()
            .map(|entry| (entry.key().clone(), entry.topic_type.clone()))
            .collect()
    }
}
