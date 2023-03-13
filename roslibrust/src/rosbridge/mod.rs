// Subscriber is a transparent module, we directly expose internal types
// Module exists only to organize source code.
mod subscriber;
pub use subscriber::*;

// Publisher is a transparent module, we directly expose internal types
// Module exists only to organize source code.
mod publisher;
pub use publisher::*;

// Client is a transparent module, we directly expose internal types
// Module exists only to organize source code
mod client;
pub use client::*;

// Tests are fully private module
#[cfg(test)]
mod integration_tests;
// Standard return type for all tests to use
#[cfg(test)]
#[allow(dead_code)]
type TestResult = Result<(), anyhow::Error>;

// Topic provider is locked behind a feature until it is stabalized
// additionally because of its use of generic associated types, it requires rust >1.65
#[cfg(feature = "topic_provider")]
mod topic_provider;

/// Communication primitives for the rosbridge_suite protocol
mod comm;

use futures_util::stream::{SplitSink, SplitStream};
use log::*;
use std::collections::HashMap;
use tokio::net::TcpStream;
use tokio_tungstenite::*;
use tungstenite::Message;

/// For now starting with a central error type, may break this up more in future
#[derive(thiserror::Error, Debug)]
pub enum RosLibRustError {
    #[error("Not currently connected to ros master / bridge")]
    Disconnected,
    // TODO we probably want to eliminate tungstenite from this and hide our
    // underlying websocket implementation from the API
    // currently we "technically" break the API when we change tungstenite verisons
    #[error("Websocket communication error: {0}")]
    CommFailure(tokio_tungstenite::tungstenite::Error),
    #[error("Operation timed out: {0}")]
    Timeout(#[from] tokio::time::error::Elapsed),
    #[error("Failed to parse message from JSON: {0}")]
    InvalidMessage(#[from] serde_json::Error),
    #[error("Rosbridge server reported an error: {0}")]
    ServerError(String),
    // Generic catch-all error type for not-yet-handled errors
    // TODO ultimately this type will be removed from API of library
    #[error(transparent)]
    Unexpected(#[from] anyhow::Error),
}

/// Provides an implementation tranlating the underlying websocket error into our error type
impl From<tokio_tungstenite::tungstenite::Error> for RosLibRustError {
    fn from(e: tokio_tungstenite::tungstenite::Error) -> Self {
        // TODO we probably want to expand this type and do some matching here
        RosLibRustError::CommFailure(e)
    }
}

/// Generic result type used as standard throughout library.
/// Note: many functions which currently return this will be updated to provide specific error
/// types in the future instead of the generic error here.
pub type RosLibRustResult<T> = Result<T, RosLibRustError>;

/// Used for type erasure of message type so that we can store arbitrary handles
type Callback = Box<dyn Fn(&str) + Send + Sync>;

/// Type erasure of callback for a service
/// Internally this will covert the input string to the Request type
/// Send that converted type into the user's callback
/// Get the result of the user's callback and then serialize that so it can be transmitted
// TODO reconsider use of serde_json::Value here vs. tungstenite::Message vs. &str
// Not quite sure what type we want to erase to?
// I can make a good argument for &str because that should be generic even if we switch
// backends - Carter 2022-10-6
type ServiceCallback = Box<
    dyn Fn(&str) -> Result<serde_json::Value, Box<dyn std::error::Error + Send + Sync>>
        + Send
        + Sync,
>;

/// The handle returned to the caller of advertise_service this struct represents the lifetime
/// of the service, and dropping this struct automatically unadvertises and removes the service.
/// No interaction with this struct is expected beyond managing its lifetime.
pub struct ServiceHandle {
    /// Reference back to the client this service originated so we can notify it when we are dropped
    client: ClientHandle,
    /// Topic that the service is served on, this is used to uniquely identify it, only one service
    /// may exist for a given topic (per client, we can't control it on the ROS side)
    topic: String,
}

/// Service handles automatically unadvertise their service when dropped.
impl Drop for ServiceHandle {
    fn drop(&mut self) {
        self.client.unadvertise_service(&self.topic);
    }
}

/// Our underlying communication socket type (maybe move to comm?)
type Socket = tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<TcpStream>>;

/// We split our underlying socket into two halves with separate locks on read and write.
/// This is the read half.
type Reader = SplitStream<Socket>;

/// We split our underlying socket into two halves with separate locks on read and write.
/// This is the write half.
type Writer = SplitSink<Socket, Message>;

/// Topics have a fundamental queue *per subscriber* this is te queue type used for each subscriber.
type MessageQueue<T> = deadqueue::limited::Queue<T>;

// TODO queue size should be configurable for subscribers
const QUEUE_SIZE: usize = 1_000;

/// Internal tracking structure used to maintain information about each subscription our client has
/// with rosbridge.
struct Subscription {
    /// Map of "subscriber id" -> callback
    /// Subscriber ids are randomly generated
    /// There will be one callback per subscriber to the topic.
    // Note: don't need dashmap here as the subscription is already inside a dashmap
    pub handles: HashMap<uuid::Uuid, Callback>,
    /// Name of ros type (package_name/message_name), used for re-subscribes
    pub topic_type: String,
}

struct PublisherHandle {
    #[allow(dead_code)]
    topic: String,
    #[allow(dead_code)]
    msg_type: String,
}
