//! # roslibrust_common
//! This crate provides common types and traits used throughout the roslibrust ecosystem.

/// For now starting with a central error type, may break this up more in future
#[derive(thiserror::Error, Debug)]
pub enum RosLibRustError {
    #[error("Not currently connected to ros master / bridge")]
    Disconnected,
    // TODO we probably want to eliminate tungstenite from this and hide our
    // underlying websocket implementation from the API
    // currently we "technically" break the API when we change tungstenite verisons
    #[error("Websocket communication error: {0}")]
    CommFailure(#[from] tokio_tungstenite::tungstenite::Error),
    #[error("Operation timed out: {0}")]
    Timeout(#[from] tokio::time::error::Elapsed),
    #[error("Failed to parse message from JSON: {0}")]
    InvalidMessage(#[from] serde_json::Error),
    #[error("TCPROS serialization error: {0}")]
    SerializationError(String),
    #[error("Rosbridge server reported an error: {0}")]
    ServerError(String),
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),
    #[error("Name does not meet ROS requirements: {0}")]
    InvalidName(String),
    // Generic catch-all error type for not-yet-handled errors
    // TODO ultimately this type will be removed from API of library
    #[error(transparent)]
    Unexpected(#[from] anyhow::Error),
}

/// Generic result type used as standard throughout library.
/// Note: many functions which currently return this will be updated to provide specific error
/// types in the future instead of the generic error here.
pub type RosLibRustResult<T> = Result<T, RosLibRustError>;

/// Fundamental traits for message types this crate works with
/// This trait will be satisfied for any types generated with this crate's message_gen functionality
pub trait RosMessageType:
    'static + serde::de::DeserializeOwned + Send + serde::Serialize + Sync + Clone + std::fmt::Debug
{
    /// Expected to be the combination pkg_name/type_name string describing the type to ros
    /// Example: std_msgs/Header
    const ROS_TYPE_NAME: &'static str;

    /// The computed md5sum of the message file and its dependencies
    /// This field is optional, and only needed when using ros1 native communication
    const MD5SUM: &'static str = "";

    /// The definition from the msg, srv, or action file
    /// This field is optional, and only needed when using ros1 native communication
    const DEFINITION: &'static str = "";
}

// This special impl allows for services with no args / returns
impl RosMessageType for () {
    const ROS_TYPE_NAME: &'static str = "";
    const MD5SUM: &'static str = "";
    const DEFINITION: &'static str = "";
}

/// Fundamental traits for service types this crate works with
/// This trait will be satisfied for any services definitions generated with this crate's message_gen functionality
pub trait RosServiceType: 'static + Send + Sync {
    /// Name of the ros service e.g. `rospy_tutorials/AddTwoInts`
    const ROS_SERVICE_NAME: &'static str;
    /// The computed md5sum of the message file and its dependencies
    const MD5SUM: &'static str;
    /// The type of data being sent in the request
    type Request: RosMessageType;
    /// The type of the data
    type Response: RosMessageType;
}

// Note: service Fn is currently defined here as it used by ros1 and roslibrust impls
/// This trait describes a function which can validly act as a ROS service
/// server with roslibrust. We're really just using this as a trait alias
/// as the full definition is overly verbose and trait aliases are unstable.
pub trait ServiceFn<T: RosServiceType>:
    Fn(T::Request) -> Result<T::Response, Box<dyn std::error::Error + 'static + Send + Sync>>
    + Send
    + Sync
    + 'static
{
}

/// Automatic implementation of ServiceFn for Fn
impl<T, F> ServiceFn<T> for F
where
    T: RosServiceType,
    F: Fn(T::Request) -> Result<T::Response, Box<dyn std::error::Error + 'static + Send + Sync>>
        + Send
        + Sync
        + 'static,
{
}

/// A generic message type used by some implementations to provide a generic subscriber / publisher without serialization
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct ShapeShifter(Vec<u8>);

// The equivalent of rospy AnyMsg or C++ ShapeShifter, subscribe_any() uses this type
impl RosMessageType for ShapeShifter {
    const ROS_TYPE_NAME: &'static str = "*";
    const MD5SUM: &'static str = "*";
    const DEFINITION: &'static str = "";
}

/// Contains functions for calculating md5sums of message definitions
/// These functions are needed both in roslibrust_ros1 and roslibrust_codegen so they're in this crate
pub mod md5sum;

/// Contains the generic traits represent a pubsub system and service system
/// These traits will be implemented for specific backends to provides access to "ROS Like" functionality
pub mod topic_provider;
pub use topic_provider::*; // Bring topic provider traits into root namespace
