//! A crate for interfacing to ROS (the Robot Operating System) via the [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)

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

// Tests are fully private module
#[cfg(test)]
mod integration_tests;

use log::*;
use serde::de::DeserializeOwned;
use serde::Serialize;
use std::fmt::Debug;

/// For now starting with a central error type, may break this up more in future
#[derive(thiserror::Error, Debug)]
pub enum RosLibRustError {
    // TODO would like to add support for this error, but for now you'll just get CommFailures
    // #[error("Not currently connected to ros master / bridge")]
    // Disconnected,
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
