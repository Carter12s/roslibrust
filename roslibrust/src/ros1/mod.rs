//! This module holds all content for directly working with ROS1 natively

/// [master_client] module contains code for calling xmlrpc functions on the master
mod master_client;
pub use master_client::*;

mod names;

/// [node] module contains the central Node and NodeHandle APIs
mod node;
pub use node::*;

mod publisher;
pub use publisher::Publisher;
mod service_client;
pub use service_client::ServiceClient;
mod subscriber;
pub use subscriber::Subscriber;
mod service_server;
pub use service_server::ServiceServer;
mod tcpros;

/// Provides a common type alias for type erased service server functions.
/// Internally we use this type to store collections of server functions.
pub(crate) type TypeErasedCallback = dyn Fn(Vec<u8>) -> Result<Vec<u8>, Box<dyn std::error::Error + Send + Sync>>
    + Send
    + Sync
    + 'static;
