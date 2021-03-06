// Contains tools for generating message definitions from ROS1 .msg and .srv files
pub mod message_gen;
// Utilities functions primarily for working with ros env vars and package structures
pub mod util;
// Communication primitives for the rosbridge_suite protocol
pub mod comm;

// TODO look into restricting visibility, right now very useful for examples / integration tests
pub mod test_msgs;

// Subscriber is a transparent module, we directly expose internal types
// Module exists only to organize source code.
mod subscriber;
pub use subscriber::*;

// Publisher is a transparent module, we directly expose internal types
// Module exists only to organize source code.
mod publisher;
pub use publisher::*;

// Tests are fully private module
mod integration_tests;

use anyhow::anyhow;
use comm::RosBridgeComm;
use dashmap::DashMap;
use futures_util::stream::{SplitSink, SplitStream, StreamExt};
use log::*;
use rand::distributions::Alphanumeric;
use rand::{thread_rng, Rng};
use serde::de::DeserializeOwned;
use serde::Serialize;
use serde_json::Value;
use std::collections::HashMap;
use std::fmt::Debug;
use std::str::FromStr;
use std::sync::Arc;
use tokio::net::TcpStream;
use tokio::sync::RwLock;
use tokio::time::Duration;
use tokio_tungstenite::*;
use tungstenite::Message;

/// Used for type erasure of message type so that we can store arbitrary handles
type Callback = Box<dyn Fn(&str) -> () + Send + Sync>;

/// Our underlying comunication type
//TODO we really want to split this into seperate read/write heads that can be locked seperatly
type Socket = tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<TcpStream>>;
type Reader = SplitStream<Socket>;
type Writer = SplitSink<Socket, Message>;

type MessageQueue<T> = deadqueue::limited::Queue<T>;

// TODO queue size should be configurable for subscribers
const QUEUE_SIZE: usize = 1_000;

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

struct Subscription {
    /// Map of "subscriber id" -> callback
    /// Subscriber ids are randomly generated
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

/// Builder options for creating a client
#[derive(Clone)]
pub struct ClientOptions {
    url: String,
    timeout: Option<Duration>,
}

impl ClientOptions {
    /// Expects a fully describe websocket url, e.g. 'ws://localhost:9090'
    pub fn new<S: Into<String>>(url: S) -> ClientOptions {
        ClientOptions {
            url: url.into(),
            timeout: None,
        }
    }

    /// Configures a default timeout for all operations.
    /// Underlying communication implementations may define their own timeouts, this options does
    /// not affect those timeouts, but adds an additional on top to prempt any operations.
    pub fn timeout<T: Into<Duration>>(mut self, duration: T) -> ClientOptions {
        self.timeout = Some(duration.into());
        self
    }
}

// Fundamental structure for a client
#[derive(Clone)]
pub struct Client {
    // TODO replace Socket with trait RosBridgeComm to allow mocking
    reader: Arc<RwLock<Reader>>,
    writer: Arc<RwLock<Writer>>,
    // Stores a record of the publishers we've handed out
    publishers: Arc<DashMap<String, PublisherHandle>>,
    subscriptions: Arc<DashMap<String, Subscription>>,
    // Contains any outstanding service calls we're waiting for a response on
    // Map key will be a uniquely generated id for each call
    service_calls: Arc<DashMap<String, tokio::sync::oneshot::Sender<Value>>>,
    opts: Arc<ClientOptions>,
}

impl Client {
    // internal implementation of new
    async fn _new(opts: ClientOptions) -> RosLibRustResult<Client> {
        let (writer, reader) = Client::stubborn_connect(&opts.url).await;
        let client = Client {
            reader: Arc::new(RwLock::new(reader)),
            writer: Arc::new(RwLock::new(writer)),
            publishers: Arc::new(DashMap::new()),
            subscriptions: Arc::new(DashMap::new()),
            service_calls: Arc::new(DashMap::new()),
            opts: Arc::new(opts),
        };

        // Spawn the spin task
        // The internal stubborn spin task continues to try to reconnect on failure
        // TODO MAJOR! Dropping the returned client doesn't shut down this task right now! That's a problem!
        let client_copy = client.clone();
        let _jh = tokio::task::spawn(client_copy.stubborn_spin());

        Ok(client)
    }

    pub async fn new_with_options(opts: ClientOptions) -> RosLibRustResult<Client> {
        Client::timeout(opts.timeout, Client::_new(opts)).await
    }

    /// Connects a rosbridge instance at the given url
    /// Expects a fully describe websocket url, e.g. 'ws://localhost:9090'
    /// When awaited will not resolve until connection is completed
    // TODO better error handling
    pub async fn new<S: Into<String>>(url: S) -> RosLibRustResult<Client> {
        Client::new_with_options(ClientOptions::new(url)).await
    }

    // Internal implementation of subscribe
    async fn _subscribe<Msg>(&mut self, topic_name: &str) -> RosLibRustResult<Subscriber<Msg>>
    where
        Msg: RosMessageType,
    {
        // Lookup / create a subscription entry for tracking
        let mut cbs = self
            .subscriptions
            .entry(topic_name.to_string())
            .or_insert(Subscription {
                handles: HashMap::new(),
                topic_type: Msg::ROS_TYPE_NAME.to_string(),
            });

        // TODO Possible bug here? We send a subscribe message each time even if already subscribed
        // Send subscribe message to rosbridge to initiate it sending us messages
        let mut stream = self.writer.write().await;
        stream.subscribe(&topic_name, &Msg::ROS_TYPE_NAME).await?;

        // Create a new watch channel for this topic
        let queue = Arc::new(MessageQueue::new(QUEUE_SIZE));

        // Move the tx into a callback that takes raw string data
        // This allows us to store the callbacks generic on type, Msg conversion is embedded here
        let topic_name_copy = topic_name.to_string();
        let queue_copy = queue.clone();
        let send_cb = Box::new(move |data: &str| {
            let converted = match serde_json::from_str::<Msg>(data) {
                Err(e) => {
                    // TODO makes sense for callback to return Result<>, instead of this handling
                    // Should do better error propogation
                    error!(
                        "Failed to deserialize ros message: {:?}. Message will be skipped!",
                        e
                    );
                    return;
                }
                Ok(t) => t,
            };

            match queue_copy.try_push(converted) {
                Ok(()) => {
                    // Msg queued succesfully
                }
                Err(msg) => {
                    info!(
                        "Queue on topic {} is full attempting to drop oldest messgae",
                        &topic_name_copy
                    );
                    let _dropped = queue_copy.try_pop();
                    // Retry pushing into queue
                    match queue_copy.try_push(msg) {
                        Ok(()) => {
                            trace!("Msg was queued succesfully after dropping front");
                        }
                        Err(msg) => {
                            // We don't expect to see this, the only way this should be possible
                            // would be if due to a race condition a message was inserted into queue
                            // between the try_pop and try_push.
                            // This closure should be the only place where push occurs, so this is not
                            // expected
                            error!(
                                "Msg was dropped during recieve because queue could not be emptied: {:?}", msg
                            );
                        }
                    }
                }
            }
        });

        // Create subscriber
        let sub = Subscriber::new(self.clone(), queue, topic_name.to_string());

        // Store callback in map under the subscriber's id
        cbs.handles.insert(*sub.get_id(), send_cb);

        Ok(sub)
    }

    /// Subscribe to a given topic expecting msgs of provided type
    pub async fn subscribe<Msg>(&mut self, topic_name: &str) -> RosLibRustResult<Subscriber<Msg>>
    where
        Msg: RosMessageType,
    {
        Client::timeout(self.opts.timeout, self._subscribe(topic_name)).await
    }

    /// This function remobes the entry for a subscriber in from the client, and if it is the last
    /// subscriber for a given topic then dispatchs an unsubscribe message to the master/bridge
    fn unsubscribe(&mut self, topic_name: &str, id: &uuid::Uuid) -> RosLibRustResult<()> {
        // Identify the subscription entry for the subscriber
        let mut subscription = self.subscriptions.get_mut(topic_name).ok_or(
            anyhow::anyhow!(
                "Topic not found in subscriptions upon dropping,\
        This should be impossible and indicates a bug in the roslibrust crate. Topic: {} UUID: {:?}", &topic_name, &id
            )
        )?;

        // Attempt to remove the subscribers specific handle
        let _cb = subscription
            .value_mut()
            .handles
            .remove(&id)
            .ok_or(anyhow::anyhow!(
            "Subscriber id {:?} was not found in handles list for topic {:?} while unsubscribing",
            &id,
            &topic_name
        ))?;

        if !subscription.handles.is_empty() {
            // All good we can just drop this one subscriber
            return Ok(());
        }
        // Otherwise this is the last subscriber for that topic and we need to unsubscribe now
        // Copy so we can move into closure
        let client_copy = self.clone();
        let topic_copy = topic_name.to_string();
        // Actually send the unsubscribe message in a task so subscriber::Drop can call this function
        tokio::spawn(async move {
            let mut stream = client_copy.writer.write().await;
            match stream.unsubscribe(&topic_copy).await {
                Ok(_) => {}
                Err(e) => error!(
                    "Failed to send unsubscribe while dropping subscriber: {:?}",
                    e
                ),
            }
        });
        Ok(())
    }

    // This function is not async specifically so it can be called from drop
    // same reason why it doesn't return anything
    // Called automatically when Publisher is dropped
    fn unadvertise(&mut self, topic_name: &str) {
        let copy = self.clone();
        let topic_name_copy = topic_name.to_string();
        tokio::spawn(async move {
            // Remove publisher from our records
            copy.publishers.remove(&topic_name_copy);

            // Send unadvertise message
            {
                debug!("Unadvertise waiting for comm lock");
                let mut comm = copy.writer.write().await;
                debug!("Unadvertise got comm lock");
                if let Err(e) = comm.unadvertise(&topic_name_copy).await {
                    error!("Failed to send unadvertise in comm layer: {:?}", e);
                }
            }
        });
    }

    // TODO apparently rosbridge doesn't support this!
    // TODO get them to! and then implement!
    // /// Advertises a service and returns a handle to the service server
    // /// Serivce will be active until the handle is dropped!
    // pub async fn advertise_service(
    //     &mut self,
    //     topic: &str,
    // ) -> Result<ServiceServer, Box<dyn Error>> {
    //     unimplemented!()
    // }

    pub async fn call_service<Req: RosMessageType, Res: RosMessageType>(
        &mut self,
        service: &str,
        req: Req,
    ) -> RosLibRustResult<Res> {
        let (tx, rx) = tokio::sync::oneshot::channel();
        let rand_string: String = thread_rng()
            .sample_iter(&Alphanumeric)
            .take(30)
            .map(char::from)
            .collect();
        {
            if self.service_calls.insert(rand_string.clone(), tx).is_some() {
                error!("ID collision encounted in call_service");
            }
        }
        {
            let mut comm = self.writer.write().await;
            comm.call_service(service, &rand_string, req).await?;
        }
        // TODO timeout impl here
        let msg = match rx.await {
            Ok(msg) => msg,
            Err(e) =>
            panic!("The sender end of a service channel was dropped while rx was being awaited, this should not be possible: {}", e),
        };
        Ok(serde_json::from_value(msg)?)
    }

    /// Implementation of timeout that is a no-op if timeout is 0 or unconfigured
    /// Only works on functions that already return our result type
    // This might not be needed but reading tokio::timeout docs I couldn't confirm this
    async fn timeout<F, T>(timeout: Option<Duration>, future: F) -> RosLibRustResult<T>
    where
        F: futures::Future<Output = RosLibRustResult<T>>,
    {
        if let Some(t) = timeout {
            tokio::time::timeout(t, future).await?
        } else {
            future.await
        }
    }

    /// Core read loop, receives messages from rosbridge and dispatches them.
    // Creating a client spawns a task which calls stubborn spin, which in turn calls this funtion
    async fn spin(&mut self) -> RosLibRustResult<()> {
        debug!("Start spin");
        loop {
            let read = {
                let mut stream = self.reader.write().await;
                match stream.next().await {
                    Some(Ok(msg)) => msg,
                    Some(Err(e)) => {
                        return Err(e.into());
                    }
                    None => {
                        return Err(RosLibRustError::Unexpected(anyhow!(
                            "Wtf does none mean here?"
                        )));
                    }
                }
            };
            debug!("Got message: {:?}", read);
            match read {
                Message::Text(text) => {
                    debug!("got message: {}", text);
                    // TODO better error handling here serde_json::Error not send
                    let parsed: serde_json::Value = serde_json::from_str(text.as_str()).unwrap();
                    let parsed_object = parsed
                        .as_object()
                        .expect("Recieved non-object json response");
                    let op = parsed_object
                        .get("op")
                        .expect("Op field not present on returned object.")
                        .as_str()
                        .expect("Op field was not of string type.");
                    let op = comm::Ops::from_str(op)?;
                    match op {
                        comm::Ops::Publish => {
                            self.handle_publish(parsed).await;
                        }
                        comm::Ops::ServiceResponse => {
                            self.handle_response(parsed).await;
                        }
                        _ => {
                            warn!("Unhandled op type {}", op)
                        }
                    }
                }
                Message::Close(close) => {
                    // TODO how should we respond to this?
                    // How do we represent connection status via our API well?
                    panic!("Close requested from server: {:?}", close);
                }
                Message::Ping(ping) => {
                    debug!("Ping received: {:?}", ping);
                }
                Message::Pong(pong) => {
                    debug!("Pong received {:?}", pong);
                }
                _ => {
                    panic!("Non-text response received");
                }
            }
        }
    }

    /// Wraps spin in retry logic to handle reconnections automagically
    async fn stubborn_spin(mut self) -> RosLibRustResult<()> {
        debug!("Starting stubborn_spin");
        loop {
            {
                let res = self.spin().await;
                if res.is_err() {
                    warn!(
                        "Spin failed with error: {}, attempting to reconnect",
                        res.err().unwrap()
                    );
                } else {
                    panic!("Spin should not exit cleanly?");
                }
            }
            // Reconnect stream
            let (writer, reader) = Client::stubborn_connect(&self.opts.url).await;
            self.reader = Arc::new(RwLock::new(reader));
            self.writer = Arc::new(RwLock::new(writer));

            // TODO re-advertise!
            // Resend rosbridge our subscription requests to re-establish inflight subscriptions
            // Clone here is dumb, but required due to async
            let mut subs: Vec<(String, String)> = vec![];
            {
                for sub in self.subscriptions.iter() {
                    subs.push((sub.key().clone(), sub.value().topic_type.clone()))
                }
            }
            let mut stream = self.writer.write().await;
            for (topic, topic_type) in &subs {
                stream.subscribe(topic, topic_type).await?;
            }
        }
    }

    /// Response handler for received publish messages
    /// Converts the return message to the subscribed type and calls any callbacks
    /// Panics if publish is received for unexpected topic
    async fn handle_publish(&mut self, data: Value) {
        // TODO lots of error handling!
        let callbacks = self
            .subscriptions
            .get(data.get("topic").unwrap().as_str().unwrap());
        let callbacks = match callbacks {
            Some(callbacks) => callbacks,
            _ => panic!("Received publish message for unsubscribed topic!"),
        };
        for (_id, callback) in &callbacks.handles {
            callback(
                serde_json::to_string(data.get("msg").unwrap())
                    .unwrap()
                    .as_str(),
            )
        }
    }

    async fn handle_response(&mut self, data: Value) {
        // TODO lots of error handling!
        let id = data.get("id").unwrap().as_str().unwrap();
        let (_id, call) = self.service_calls.remove(id).unwrap();
        let res = data.get("values").unwrap();
        call.send(res.clone()).unwrap();
    }

    /// Connects to websocket at specified URL, retries indefinitely
    async fn stubborn_connect(url: &str) -> (Writer, Reader) {
        loop {
            match Client::connect(&url).await {
                Err(e) => {
                    warn!("Failed to reconnect: {:?}", e);
                    // TODO configurable rate?
                    tokio::time::sleep(tokio::time::Duration::from_millis(200)).await;
                    continue;
                }
                Ok(stream) => {
                    let (writer, reader) = stream.split();
                    return (writer, reader);
                }
            }
        }
    }

    /// Bassic connection attempt and error wrapping
    async fn connect(url: &str) -> RosLibRustResult<Socket> {
        let attempt = tokio_tungstenite::connect_async(url).await;
        match attempt {
            Ok((stream, _response)) => Ok(stream),
            Err(e) => Err(e.into()),
        }
    }

    /// Publishes a message
    pub(crate) async fn publish<T>(&mut self, topic: &str, msg: T) -> RosLibRustResult<()>
    where
        T: RosMessageType,
    {
        let mut stream = self.writer.write().await;
        debug!("Publish got write lock on comm");
        stream.publish(topic, msg).await?;
        Ok(())
    }

    pub async fn advertise<T>(&mut self, topic: &str) -> RosLibRustResult<Publisher<T>>
    where
        T: RosMessageType,
    {
        {
            match self.publishers.get(topic) {
                Some(_) => {
                    // TODO if we ever remove this restriction we should still check types match
                    return Err(RosLibRustError::Unexpected(anyhow!(
                        "Attempted to create two publisher to same topic, this is not supported"
                    )));
                }
                None => {
                    // TODO can we insert while holding a get in dashmap?
                    self.publishers.insert(
                        topic.to_string(),
                        PublisherHandle {
                            topic: topic.to_string(),
                            msg_type: T::ROS_TYPE_NAME.to_string(),
                        },
                    );
                }
            }
        }

        {
            let mut stream = self.writer.write().await;
            debug!("Advertise got lock on comm");
            stream.advertise::<T>(topic).await?;
        }
        Ok(Publisher::new(topic.to_string(), self.clone()))
    }
}
