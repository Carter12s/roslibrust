// Subscriber is a transparent module, we directly expose internal types
// Module exists only to organize source code.
mod subscriber;
pub use subscriber::*;

// Publisher is a transparent module, we directly expose internal types
// Module exists only to organize source code.
mod publisher;
pub use publisher::*;

/// Communication primitives for the rosbridge_suite protocol
mod comm;


use anyhow::anyhow;
use comm::RosBridgeComm;
use crate::{RosLibRustError, RosLibRustResult, RosMessageType, RosServiceType};
use dashmap::DashMap;
use futures_util::stream::{SplitSink, SplitStream, StreamExt};
use log::*;
use rand::distributions::Alphanumeric;
use rand::{thread_rng, Rng};
use serde_json::Value;
use std::collections::HashMap;
use std::str::FromStr;
use std::sync::Arc;
use tokio::net::TcpStream;
use tokio::sync::RwLock;
use tokio::time::Duration;
use tokio_tungstenite::*;
use tungstenite::Message;

/// Used for type erasure of message type so that we can store arbitrary handles
type Callback = Box<dyn Fn(&str) -> () + Send + Sync>;
/// Type erasure of callback for a service
/// Internally this will covert the input string to the Request type
/// Send that converted type into the user's callback
/// Get the result of the user's callback and then serialize that so it can be transmitted
// TODO need to implement drop logic on this
type ServiceCallback = Box<
    dyn Fn(&str) -> Result<serde_json::Value, Box<dyn std::error::Error + Send + Sync>>
        + Send
        + Sync,
>;

/// Our underlying communication type
//TODO we really want to split this into separate read/write heads that can be locked separately
type Socket = tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<TcpStream>>;
type Reader = SplitStream<Socket>;
type Writer = SplitSink<Socket, Message>;

type MessageQueue<T> = deadqueue::limited::Queue<T>;

// TODO queue size should be configurable for subscribers
const QUEUE_SIZE: usize = 1_000;

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

/// A client connection to the rosbridge_server that allows for publishing and subscribing to topics
#[derive(Clone)]
pub struct Client {
    // TODO replace Socket with trait RosBridgeComm to allow mocking
    reader: Arc<RwLock<Reader>>,
    writer: Arc<RwLock<Writer>>,
    // Stores a record of the publishers we've handed out
    publishers: Arc<DashMap<String, PublisherHandle>>,
    subscriptions: Arc<DashMap<String, Subscription>>,
    services: Arc<DashMap<String, ServiceCallback>>,
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
            services: Arc::new(DashMap::new()),
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

    /// This function removes the entry for a subscriber in from the client, and if it is the last
    /// subscriber for a given topic then dispatches an unsubscribe message to the master/bridge
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

    /// Advertises a service and returns a handle to the service server
    /// Service will be active until the handle is dropped!
    pub async fn advertise_service<T: RosServiceType>(
        &mut self,
        topic: &str,
        server: fn(
            T::Request,
        )
            -> Result<T::Response, Box<dyn std::error::Error + 'static + Send + Sync>>,
    ) -> RosLibRustResult<()> {
        let mut writer = self.writer.write().await;
        writer.advertise_service(topic, T::ROS_SERVICE_NAME).await?;

        // We need to do type erasure and hide the request by wrapping their closure in a generic closure
        let erased_closure = move |message: &str| -> Result<
            serde_json::Value,
            Box<dyn std::error::Error + Send + Sync>,
        > {
            // Type erase the incoming type
            let parsed_msg = serde_json::from_str(message)?;
            let response = server(parsed_msg)?;
            // Type erase the outgoing type
            let response_string = serde_json::json!(response);
            Ok(response_string)
        };

        let res = self
            .services
            .insert(topic.to_string(), Box::new(erased_closure));
        if let Some(_previous_server) = res {
            warn!("Re-registering a server for a pre-existing topic? Are you sure you want to do this");
            unimplemented!()
        }

        //TODO need to provide unadvertise and drop callback

        Ok(())
    }

    /// Calls a ros service and returns the response
    ///
    /// Service calls can fail if communication is interrupted.
    /// This method is currently unaffected by the clients Timeout configuration.
    ///
    /// Roadmap:
    ///   - Provide better error information when a service call fails
    ///   - Integrate with Client's timeout better
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
                error!("ID collision encountered in call_service");
            }
        }
        {
            let mut comm = self.writer.write().await;
            Client::timeout(
                self.opts.timeout,
                comm.call_service(service, &rand_string, req),
            )
            .await?;
        }

        // Having to do manual timeout logic here because of error types
        let recv = if let Some(timeout) = self.opts.timeout {
            tokio::time::timeout(timeout, rx).await?
        } else {
            rx.await
        };

        let msg = match recv {
            Ok(msg) => msg,
            Err(e) =>
            // TODO remove panic! here, this could result from dropping communication, need to handle disconnect better
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
    // Creating a client spawns a task which calls stubborn spin, which in turn calls this function
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
                            trace!("handling publish for {:?}", &parsed);
                            self.handle_publish(parsed).await;
                        }
                        comm::Ops::ServiceResponse => {
                            trace!("handling service response for {:?}", &parsed);
                            self.handle_response(parsed).await;
                        }
                        comm::Ops::CallService => {
                            trace!("handling call_service for {:?}", &parsed);
                            self.handle_service(parsed).await;
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
            _ => panic!("Received publish message for unsubscribed topic!"), // TODO probably shouldn't be a panic?
        };
        for (_id, callback) in &callbacks.handles {
            callback(
                // TODO possible bug here if "msg" isn't defined remove this unwrap
                serde_json::to_string(data.get("msg").unwrap())
                    .unwrap()
                    .as_str(),
            )
        }
    }

    /// Response handler for receiving a service call looks up if we have a service
    /// registered for the incoming topic and if so dispatches to the callback
    async fn handle_service(&mut self, data: Value) {
        // Unwrap is okay, field is fully required and strictly type
        let topic = data.get("service").unwrap().as_str().unwrap();
        // Unwrap is okay, field is strictly typed to string
        let id = data.get("id").map(|id| id.as_str().unwrap().to_string());

        // Lookup if we have a service for the message
        let callback = self.services.get(topic);
        let callback = match callback {
            Some(callback) => callback,
            _ => panic!("Received call_service for unadvertised service!"),
        };
        // TODO likely bugs here remove this unwrap. Unclear what we are expected to get for empty service
        let request = data.get("args").unwrap().to_string();
        let mut writer = self.writer.write().await;
        match callback(&request) {
            Ok(res) => {
                // TODO unwrap here is probably bad... Failure to write means disconnected?
                writer.service_response(topic, id, true, res).await.unwrap();
            }
            Err(e) => {
                error!("A service callback on topic {:?} failed with {:?} sending response false in service_response", data.get("service"), e);
                writer
                    .service_response(topic, id, false, serde_json::json!(format!("{e}")))
                    .await
                    .unwrap();
            }
        };

        // Now we need to send the service_response back
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

    /// Basic connection attempt and error wrapping
    async fn connect(url: &str) -> RosLibRustResult<Socket> {
        let attempt = tokio_tungstenite::connect_async(url).await;
        match attempt {
            Ok((stream, _response)) => Ok(stream),
            Err(e) => Err(e.into()),
        }
    }

    /// Publishes a message
    pub(crate) async fn publish<T>(&self, topic: &str, msg: T) -> RosLibRustResult<()>
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