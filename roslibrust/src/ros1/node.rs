//! This module contains the top level Node and NodeHandle classes.
//! These wrap the lower level management of a ROS Node connection into a higher level and thread safe API.

use std::sync::{Arc, RwLock};

use dashmap::DashMap;

use crate::{MasterClient, PublisherHandle, ServiceCallback, Subscription};

/// Represents a single "real" node, typically only one of these is expected per process
/// but nothing should specifically prevent that.
pub struct Node {
    client: MasterClient, // Used for calling into ROS, and manages our xmlrpc server
    publishers: DashMap<String, PublisherHandle>,
    subscriptions: DashMap<String, Subscription>,
    services: DashMap<String, ServiceCallback>,
}

impl Node {
    async fn new() -> Result<Node, Box<dyn std::error::Error>> {
        let client = MasterClient::new("http://localhost:11311", "/name").await?;
        Ok(Node {
            client: client,
            publishers: DashMap::new(),
            subscriptions: DashMap::new(),
            services: DashMap::new(),
        })
    }
}

/// Represents a handle to an underlying [Node]. NodeHandle's can be freely cloned, moved, copied, etc.
/// This class provides the user facing API for interacting with ROS.
pub struct NodeHandle {
    pub(crate) inner: Arc<RwLock<Node>>,
}

impl NodeHandle {
    // TODO builder, async, result
    pub async fn new() -> Result<NodeHandle, Box<dyn std::error::Error>> {
        let inner = Arc::new(RwLock::new(Node::new().await?));
        Ok(NodeHandle { inner })
    }
}
