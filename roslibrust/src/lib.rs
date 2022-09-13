//! A crate for interfacing to ROS (the Robot Operating System) via the [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)
//!
//! # Introduction
//! This crate is designed to provide a convenient API for interfacing between Rust and ROS.
//! This crate prioritizes ergonomics and ease-of-use over performance, while leveraging Rust's
//! exceptional type system and memory guarantees to ensure correctness.
//!
//! ## Why Use This Crate?
//! Do you have a fleet of systems distributed around a factory, city, nation, or solar system? Do you want to write a system
//! for managing that fleet? Problems this crate solves for you:
//!   - Rust is better than C++, Easier to test than Python, and well suited for cloud services
//!   - Native ROS communication is PITA for firewalls / VPNs, rosbridge is much easier to deal with when remote
//!   - Don't write your own REST API or leave your comfortable ROS style abstractions. Listen to the same messages, and call the same
//!     services you would if you were local
//!   - Deal with multiple versions of a message definition by utilizing Rust enums, optional fields, and other serde directives
//!   - Run a single process on a single server that can slurp data from 100's of your systems efficiently and without bottlenecks
//!  
//! ### Why not use rosrust?
//! rosrust is a completely valid alternative to this crate! The reasons to use this crate are the same reasons why people use rosbridge
//! in general:
//!   - rosbridge is served on a single port meaning it can more easily be exposed through docker or firewalls
//!   - rosbirdge's JSON encoding makes it possible to work with multiple versions of a ROS message at the same time. A custom written
//!     message type that utilizes enums / optional fields allows a rust client to deal with version differences amongst a fleet of
//!     systems
//!   - async. rosrust was written before async/.await was standardized in rust an is fundamentally a synchronous paradigm. This makes
//!     it much more difficult to leverage Rust's "fearless concurrency" than with `roslibrust`
//!
//! ### Is rosbridge slower?
//! Yes! Your mileage may vary. This crate is much more designed for low bandwidth communication with a large fleet of systems than
//! writing high performance nodes designed to run locally.
//!
//! ## How Does It Work?
//! When you create a new client via ClientHandle::new() or ClientHandle::new_with_options() a new connection to rosbridge is created.
//! This new connection is literally opening a new Websocket. A specific "stubborn spin" tokio task is created which handles reading
//! from the websocket, and automatically connecting / reconnecting if communication is interrupted.
//!
//! This central stubborn_spin task has the only access to the -read- half of the websocket and is constantly .await'ing any new messages
//! on the websocket connection. When a new message is received from rosbridge it does its best to dispatch it to the relevant entities
//! that are registered with the Client.
//!
//! It is completely valid and recommended to clone() ClientHandle to pass access around your application, in fact many of the types
//! returned by roslibrust include clones of the client handle within them to enable functionality like automatically de-registering
//! when dropped. It is not recommended, although completely valid, to create multiple new clients to the same rosbridge server as
//! the additional websockets created add overhead to both roslibrust and rosbridge.
//!
//! ### How Subscribers Work
//! Each time subscribe is called, a new queue for that subscriber is created. When a `publish` message is received from rosbridge,
//! the message is duplicated and inserted into the queue for *each* subscriber. This means if you call subscribe multiple times
//! on the same topic, each of the returned subscribers will receive a copy of every message. Currently, the size of the queue for
//! every publisher is hardcoded to 1_000 messages control of this will be provided in a future version.
//!
//! If the queue for a subscriber is full and a new message arrives the central spin task will not block but simply drop that message
//! for that subscriber (warnings will be logged).
//!
//! Internally roslibrust type-erases the type that `subscribe` is called with an stores a callback where the deserialization
//! is embedded within the callback. Roslibrust does not check that the type that subscribe is called with matches the type of the topic.
//! If an incorrect type is used, each time a message is received on the topic it will fail to de-serialize and an error will be emitted
//! by the subscriber. This can be useful when building client designed to work with multiple different versions of a message definition.
//!
//! When the subscriber returned from the subscribe call is dropped it removes is queue from the client. When the last subscriber
//! on a given topic dropped the client will automatically unsubscribe from the topic with rosbridge.
//!
//! ### How Publishers Work
//! When advertise is called a publisher an advertise message is sent to rosbridge_server and a publisher returned.
//! Dropping the publisher will automatically unadvertise the topic.
//! roslibrust currently does not support multiple publishers / multiple advertises for a single topic.
//!
//! ### How Service Servers Work
//! When advertise service is called you must pass into it a callback conforming to the libraries requirements.
//! Specifically, roslibrust attempts to follow "good" ros error handling convention and be as compatible as possible
//! with various error types; however, due to the async nature of the crate `Box<dyn Error + Send + Sync>` is needed.
//!
//!

// Contains definition of custom ROS types that codegen emits
pub mod integral_types;

#[cfg(feature = "client")]
mod rosbridge;
pub use rosbridge::*;

#[cfg(feature = "codegen")]
mod codegen;
pub use codegen::*;

/// Utilities functions primarily for working with ros env vars and package structures
#[cfg(feature = "utils")]
pub mod utils;

use log::*;
use serde::de::DeserializeOwned;
use serde::Serialize;
use std::fmt::Debug;

/// For now starting with a central error type, may break this up more in future
#[derive(thiserror::Error, Debug)]
pub enum RosLibRustError {
    #[error("Not currently connected to ros master / bridge")]
    Disconnected,
    #[error("Websocket communication error: {0}")]
    CommFailure(tokio_tungstenite::tungstenite::Error),
    #[error("Operation timed out: {0}")]
    Timeout(#[from] tokio::time::error::Elapsed),
    #[error("Failed to parse message from JSON: {0}")]
    InvalidMessage(#[from] serde_json::Error),
    // Generic catch-all error type for not-yet-handled errors
    // TODO ultimately this type will be removed from API of library
    #[error(transparent)]
    Unexpected(#[from] anyhow::Error),
}

impl From<tokio_tungstenite::tungstenite::Error> for RosLibRustError {
    fn from(e: tokio_tungstenite::tungstenite::Error) -> Self {
        // TODO we probably want to expand this type and do some matching here
        RosLibRustError::CommFailure(e)
    }
}

type RosLibRustResult<T> = Result<T, RosLibRustError>;

/// Fundamental traits for message types this crate works with
/// This trait will be satisfied for any types generated with this crate's message_gen functionality
pub trait RosMessageType:
    'static + DeserializeOwned + Default + Send + Serialize + Sync + Clone + Debug
{
    /// Expected to be the combination pkg_name/type_name string describing the type to ros
    /// Example: std_msgs/Header
    const ROS_TYPE_NAME: &'static str;
}

// This special impl allows for services with no args / returns
impl RosMessageType for () {
    const ROS_TYPE_NAME: &'static str = "";
}

/// Fundamental traits for service types this crate works with
/// This trait will be satisfied for any services definitions generated with this crate's message_gen functionality
pub trait RosServiceType {
    /// Name of the ros service e.g. `rospy_tutorials/AddTwoInts`
    const ROS_SERVICE_NAME: &'static str;
    /// The type of data being sent in the request
    type Request: RosMessageType;
    /// The type of the data
    type Response: RosMessageType;
}
