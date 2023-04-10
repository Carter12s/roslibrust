use crate::rosbridge::comm;
use crate::{rosbridge::comm::RosBridgeComm, RosLibRustError};
use crate::{Publisher, ServiceHandle, Subscriber};
use anyhow::anyhow;
use dashmap::DashMap;
use futures::StreamExt;
use log::*;
use rand::distributions::Alphanumeric;
use rand::{thread_rng, Rng};
use roslibrust_codegen::{RosMessageType, RosServiceType};
use serde_json::Value;
use std::collections::HashMap;
use std::str::FromStr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use tokio::sync::RwLock;
use tokio::time::Duration;
use tokio_tungstenite::tungstenite::Message;

use super::{
    MessageQueue, PublisherHandle, Reader, RosLibRustResult, ServiceCallback, Socket, Subscription,
    Writer, QUEUE_SIZE,
};

/// Builder options for creating a client
#[derive(Clone)]
pub struct ClientHandleOptions {
    url: String,
    timeout: Option<Duration>,
}

impl ClientHandleOptions {
    /// Expects a fully describe websocket url, e.g. 'ws://localhost:9090'
    pub fn new<S: Into<String>>(url: S) -> ClientHandleOptions {
        ClientHandleOptions {
            url: url.into(),
            timeout: None,
        }
    }

    /// Configures a default timeout for all operations.
    /// Underlying communication implementations may define their own timeouts, this options does
    /// not affect those timeouts, but adds an additional on top to preempt any operations.
    pub fn timeout<T: Into<Duration>>(mut self, duration: T) -> ClientHandleOptions {
        self.timeout = Some(duration.into());
        self
    }
}

/// The ClientHandle is the fundamental object through which users of this library are expected to interact with it.
///
/// Creating a new ClientHandle will create an underlying connection to rosbridge and spawn an async connection task,
/// which is responsible for continuously managing that connection and attempts to re-establish the connection if it goes down.
///
/// ClientHandle is clone and multiple handles can be clone()'d from the original and passed throughout your application.
/// ```no_run
/// # roslibrust_codegen_macro::find_and_generate_ros_messages!(
/// #    "../assets/ros1_common_interfaces/std_msgs"
/// # );
/// # #[tokio::main]
/// # async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
///   // Create a new client
///   let mut handle = roslibrust::ClientHandle::new("ws://localhost:9090").await?;
///   // Create a copy of the handle (does not create a seperate connection)
///   let mut handle2 = handle.clone();
///   tokio::spawn(async move {
///     let subscription = handle.subscribe::<std_msgs::Header>("/topic").await.unwrap();
///   });
///   tokio::spawn(async move{
///     let subscription = handle2.subscribe::<std_msgs::Header>("/topic").await.unwrap();
///   });
///   # Ok(())
/// # }
/// // Both tasks subscribe to the same topic, but since the use the same underlying client only one subscription is made to rosbridge
/// // Both subscribers will receive a copy of each message received on the topic
/// ```
#[derive(Clone)]
pub struct ClientHandle {
    pub(crate) inner: Arc<RwLock<Client>>,
    pub(crate) is_disconnected: Arc<AtomicBool>,
}

impl ClientHandle {
    /// Creates a new client handle with configurable options.
    ///
    /// Use this method if you need more control than [ClientHandle::new] provides.
    /// Like [ClientHandle::new] this function does not resolve until the connection is established for the first time.
    /// This function respects the [ClientHandleOptions] timeout and will return with an error if a connection is not
    /// established within the timeout.
    pub async fn new_with_options(opts: ClientHandleOptions) -> RosLibRustResult<Self> {
        let inner = Arc::new(RwLock::new(timeout(opts.timeout, Client::new(opts)).await?));
        let inner_weak = Arc::downgrade(&inner);

        // We connect when we create Client
        let is_disconnected = Arc::new(AtomicBool::new(false));

        // Spawn the spin task
        // The internal stubborn spin task continues to try to reconnect on failure
        let _ = tokio::task::spawn(stubborn_spin(inner_weak, is_disconnected.clone()));

        Ok(ClientHandle {
            inner,
            is_disconnected,
        })
    }

    /// Connects a rosbridge instance at the given url
    /// Expects a fully describe websocket url, e.g. 'ws://localhost:9090'
    /// When awaited will not resolve until connection is successfully made.
    pub async fn new<S: Into<String>>(url: S) -> RosLibRustResult<Self> {
        Self::new_with_options(ClientHandleOptions::new(url)).await
    }

    fn check_for_disconnect(&self) -> RosLibRustResult<()> {
        match self.is_disconnected.load(Ordering::Relaxed) {
            false => Ok(()),
            true => Err(RosLibRustError::Disconnected),
        }
    }

    // Internal implementation of subscribe
    async fn _subscribe<Msg>(&self, topic_name: &str) -> RosLibRustResult<Subscriber<Msg>>
    where
        Msg: RosMessageType,
    {
        // Lookup / create a subscription entry for tracking
        let client = self.inner.read().await;
        let mut cbs = client
            .subscriptions
            .entry(topic_name.to_string())
            .or_insert(Subscription {
                handles: HashMap::new(),
                topic_type: Msg::ROS_TYPE_NAME.to_string(),
            });

        // TODO Possible bug here? We send a subscribe message each time even if already subscribed
        // Send subscribe message to rosbridge to initiate it sending us messages
        let mut stream = client.writer.write().await;
        stream.subscribe(topic_name, Msg::ROS_TYPE_NAME).await?;

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
                    // Msg queued successfully
                }
                Err(msg) => {
                    info!(
                        "Queue on topic {} is full attempting to drop oldest message",
                        &topic_name_copy
                    );
                    let _dropped = queue_copy.try_pop();
                    // Retry pushing into queue
                    match queue_copy.try_push(msg) {
                        Ok(()) => {
                            trace!("Msg was queued successfully after dropping front");
                        }
                        Err(msg) => {
                            // We don't expect to see this, the only way this should be possible
                            // would be if due to a race condition a message was inserted into queue
                            // between the try_pop and try_push.
                            // This closure should be the only place where push occurs, so this is not
                            // expected
                            error!(
                                "Msg was dropped during receive because queue could not be emptied: {:?}", msg
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

    /// Subscribe to a given topic expecting msgs of provided type.
    /// ```no_run
    /// # roslibrust_codegen_macro::find_and_generate_ros_messages!(
    /// #    "../assets/ros1_common_interfaces/std_msgs"
    /// # );
    /// # #[tokio::main]
    /// # async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    ///   // Create a new client
    ///   let mut handle = roslibrust::ClientHandle::new("ws://localhost:9090").await?;
    ///   // Subscribe using ::<T> style
    ///   let subscriber1 = handle.subscribe::<std_msgs::Header>("/topic").await?;
    ///   // Subscribe using explicit type style
    ///   let subscriber2: roslibrust::Subscriber<std_msgs::Header> = handle.subscribe::<std_msgs::Header>("/topic").await?;
    ///   # Ok(())
    /// # }
    /// ```
    /// This function returns after a subscribe message has been sent to rosbridge, it will
    /// return immediately with an error if call while currently disconnected.
    ///
    /// It does not error if subscribed type does not match the topic type or check this in anyway.
    /// If a type different that what is expected on the topic is published the deserialization of that message will fail,
    /// and the returned subscriber will simply not receive that message.
    /// Roslibrust will log an error which can be used to detect this situation.
    /// This can be useful to subscribe to the same topic with multiple different types and whichever
    /// types successfully deserialize the message will receive a message.
    ///
    /// ```no_run
    /// # // TODO figure out how to de-duplicate code here with this message definition...
    /// # mod ros1 {
    /// # roslibrust_codegen_macro::find_and_generate_ros_messages!(
    /// #   "../assets/ros1_common_interfaces/std_msgs"
    /// # );
    /// # }
    /// # mod ros2 {
    /// # roslibrust_codegen_macro::find_and_generate_ros_messages!(
    /// #   "../assets/ros1_common_interfaces/std_msgs"
    /// # );
    /// # }
    /// # #[tokio::main]
    /// # async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    ///   // Create a new client
    ///   let mut handle = roslibrust::ClientHandle::new("ws://localhost:9090").await?;
    ///   // Subscribe to the same topic with two different types
    ///   let ros1_subscriber = handle.subscribe::<ros1::std_msgs::Header>("/topic").await?;
    ///   let ros2_subscriber = handle.subscribe::<ros2::std_msgs::Header>("/topic").await?;
    ///   // Await both subscribers and get a result back from whichever succeeds at deserializing
    ///   let time = tokio::select!(
    ///     r1_msg = ros1_subscriber.next() => r1_msg.stamp,
    ///     r2_msg = ros2_subscriber.next() => r2_msg.stamp,
    ///   );
    /// # Ok(())
    /// # }
    /// ```
    pub async fn subscribe<Msg>(&self, topic_name: &str) -> RosLibRustResult<Subscriber<Msg>>
    where
        Msg: RosMessageType,
    {
        self.check_for_disconnect()?;
        timeout(
            self.inner.read().await.opts.timeout,
            self._subscribe(topic_name),
        )
        .await
    }

    // Publishes a message
    // Fails immediately(ish) if disconnected
    // Returns success when message is put on websocket (no confirmation of receipt)
    pub(crate) async fn publish<T>(&self, topic: &str, msg: T) -> RosLibRustResult<()>
    where
        T: RosMessageType,
    {
        self.check_for_disconnect()?;
        let client = self.inner.read().await;
        let mut stream = client.writer.write().await;
        debug!("Publish got write lock on comm");
        stream.publish(topic, msg).await?;
        Ok(())
    }

    /// Advertises a topic to be published to and returns a type specific publisher to use.
    ///
    /// Dropping the publisher will automatically un-advertise the topic. Publisher is not clone,
    /// and calling advertise multiple times targeting the same topic is not currently supported and
    /// will result in an error.
    ///
    /// This function returns with a failure if currently disconnected when called.
    ///
    /// No type checking of the advertised type is performed. If the serialization of T is not
    /// accepted by rosbridge as compatible with rosmaster's type, that information will only be
    /// available in rosbridge's logs.
    ///
    /// ```no_run
    /// # roslibrust_codegen_macro::find_and_generate_ros_messages!(
    /// #    "../assets/ros1_common_interfaces/std_msgs"
    /// # );
    /// # #[tokio::main]
    /// # async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    ///   // Create a new client
    ///   let mut handle = roslibrust::ClientHandle::new("ws://localhost:9090").await?;
    ///   // Advertise using ::<T> style
    ///   let mut publisher = handle.advertise::<std_msgs::Header>("/topic").await?;
    ///   // Advertise using explicit type
    ///   let mut publisher2: roslibrust::Publisher<std_msgs::Header> = handle.advertise("/other_topic").await?;
    ///   # Ok(())
    /// # }
    /// ```
    pub async fn advertise<T>(&self, topic: &str) -> RosLibRustResult<Publisher<T>>
    where
        T: RosMessageType,
    {
        self.check_for_disconnect()?;
        let client = self.inner.read().await;
        if client.publishers.contains_key(topic) {
            // TODO if we ever remove this restriction we should still check types match
            return Err(RosLibRustError::Unexpected(anyhow!(
                "Attempted to create two publisher to same topic, this is not supported"
            )));
        } else {
            client.publishers.insert(
                topic.to_string(),
                PublisherHandle {
                    topic_type: T::ROS_TYPE_NAME.to_string(),
                },
            );
        }

        {
            let mut stream = client.writer.write().await;
            debug!("Advertise got lock on comm");
            stream.advertise::<T>(topic).await?;
        }
        Ok(Publisher::new(topic.to_string(), self.clone()))
    }

    /// Calls a ros service and returns the response
    ///
    /// Service calls can fail if communication is interrupted.
    /// This method is currently unaffected by the clients Timeout configuration.
    ///
    /// Roadmap:
    ///   - Provide better error information when a service call fails
    ///   - Integrate with ClientHandle's timeout better
    ///
    /// ```no_run
    /// # roslibrust_codegen_macro::find_and_generate_ros_messages!(
    /// #    "../assets/ros1_common_interfaces/rosapi"
    /// # );
    /// # #[tokio::main]
    /// # async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    ///   // Create a new client
    ///   let mut handle = roslibrust::ClientHandle::new("ws://localhost:9090").await?;
    ///   // Call service using implied types
    ///   // Note: () "the empty type" has an explicit definition as a RosMessageType and can be used in place of naming an empty message
    ///   let response: rosapi::GetTimeResponse  = handle.call_service("/rosapi/get_time", ()).await?;
    ///   // Call service using explicit types
    ///   let response = handle.call_service::<rosapi::GetTimeRequest, rosapi::GetTimeResponse>("/rosapi/get_time", rosapi::GetTimeRequest{}).await?;
    /// # Ok(())
    /// # }
    /// ```
    pub async fn call_service<Req: RosMessageType, Res: RosMessageType>(
        &self,
        service: &str,
        req: Req,
    ) -> RosLibRustResult<Res> {
        self.check_for_disconnect()?;
        let (tx, rx) = tokio::sync::oneshot::channel();
        // TODO replace with uuid::v4()
        let rand_string: String = thread_rng()
            .sample_iter(&Alphanumeric)
            .take(30)
            .map(char::from)
            .collect();
        let client = self.inner.read().await;
        {
            if client
                .service_calls
                .insert(rand_string.clone(), tx)
                .is_some()
            {
                error!("ID collision encountered in call_service");
            }
        }
        {
            let mut comm = client.writer.write().await;
            timeout(
                client.opts.timeout,
                comm.call_service(service, &rand_string, req),
            )
            .await?;
        }

        // Having to do manual timeout logic here because of error types
        let recv = if let Some(timeout) = client.opts.timeout {
            tokio::time::timeout(timeout, rx).await?
        } else {
            rx.await
        };

        // Attempt to actually pull data out
        let msg = match recv {
            Ok(msg) => msg,
            Err(e) =>
            // TODO remove panic! here, this could result from dropping communication, need to handle disconnect better
            panic!("The sender end of a service channel was dropped while rx was being awaited, this should not be possible: {}", e),
        };

        // Attempt to convert data to response type
        match serde_json::from_value(msg.clone()) {
            Ok(val) => Ok(val),
            Err(e) => {
                // We failed to parse the value as an expected type, before just giving up, try to parse as string
                // if we got a string it indicates a server side error, otherwise we got the wrong datatype back
                match serde_json::from_value(msg) {
                    Ok(s) => return Err(RosLibRustError::ServerError(s)),
                    Err(_) => {
                        // Return the error from the origional parse
                        return Err(RosLibRustError::InvalidMessage(e));
                    }
                }
            }
        }
    }

    /// Advertises a service and returns a handle that manages the lifetime of the service.
    /// Service will be active until the handle is dropped!
    ///
    /// See examples/service_server.rs for usage.
    pub async fn advertise_service<T: RosServiceType>(
        &self,
        topic: &str,
        server: fn(
            T::Request,
        )
            -> Result<T::Response, Box<dyn std::error::Error + 'static + Send + Sync>>,
    ) -> RosLibRustResult<ServiceHandle> {
        self.check_for_disconnect()?;
        {
            let client = self.inner.read().await;
            let mut writer = client.writer.write().await;
            // Before proceeding check we don't already have an active service_server for this topic
            if client.services.contains_key(topic) {
                error!(
                    "Re-registering a server for the pre-existing topic: {topic} This will fail!"
                );
                return Err(RosLibRustError::Unexpected(anyhow!("roslibrust does not support re-advertising a service without first dropping the previous Service")));
            }

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

            let res = client
                .services
                .insert(topic.to_string(), Box::new(erased_closure));
            if let Some(_previous_server) = res {
                error!("This should not be possible, but somehow you managed to double advertise a service despite the guard...");
            }
            // Don't advertise the service until we've reached this point, otherwise we'll double advertise
            writer.advertise_service(topic, T::ROS_SERVICE_NAME).await?;
        } // Drop client lock here so we can clone without creating an issue

        Ok(ServiceHandle {
            client: self.clone(),
            topic: topic.to_string(),
        })
    }

    // Internal method for removing a service, this is expected to be automatically called
    // by dropping the relevant service handle. Intentionally not async as a result.
    pub(crate) fn unadvertise_service(&self, topic: &str) {
        let copy = self.inner.clone();
        let topic = topic.to_string();
        tokio::spawn(async move {
            let client = copy.read().await;
            let entry = client.services.remove(&topic);
            // Since this is called by drop we can't really propagate and error and instead simply have to log
            if entry.is_none() {
                error!(
                    "Unadvertise service was called on topic `{topic}` however no service was found.\
                This likely indicates and error with the roslibrust crate."
                );
            }

            // Regardless of whether we found an entry we should still send he unadvertise_service message to rosbridge
            let mut writer = client.writer.write().await;
            let res = writer.unadvertise_service(&topic).await;
            if let Err(e) = res {
                error!("Failed to send unadvertise_service message when service handle was dropped for `{topic}`: {e}");
            }
        });
    }

    // This function is not async specifically so it can be called from drop
    // same reason why it doesn't return anything
    // Called automatically when Publisher is dropped
    pub(crate) fn unadvertise(&self, topic_name: &str) {
        let copy = self.clone();
        let topic_name_copy = topic_name.to_string();
        tokio::spawn(async move {
            // Remove publisher from our records
            let client = copy.inner.read().await;
            client.publishers.remove(&topic_name_copy);

            // Send unadvertise message
            {
                debug!("Unadvertise waiting for comm lock");
                let mut comm = client.writer.write().await;
                debug!("Unadvertise got comm lock");
                if let Err(e) = comm.unadvertise(&topic_name_copy).await {
                    error!("Failed to send unadvertise in comm layer: {:?}", e);
                }
            }
        });
    }

    // This function removes the entry for a subscriber in from the client, and if it is the last
    // subscriber for a given topic then dispatches an unsubscribe message to the master/bridge
    pub(crate) fn unsubscribe(&self, topic_name: &str, id: &uuid::Uuid) -> RosLibRustResult<()> {
        // Copy so we can move into closure
        let client = self.clone();
        let topic_name = topic_name.to_string();
        let id = *id;
        // Actually send the unsubscribe message in a task so subscriber::Drop can call this function
        tokio::spawn(async move {
            // Identify the subscription entry for the subscriber
            let client = client.inner.read().await;
            let mut subscription = match client.subscriptions.get_mut(&topic_name) {
                Some(subscription) => subscription,
                None => {
                    error!("Topic not found in subscriptions upon dropping. This should be impossible and indicates a bug in the roslibrust crate. Topic: {topic_name} UUID: {id:?}");
                    return;
                }
            };
            if subscription.value_mut().handles.remove(&id).is_none() {
                error!("Subscriber id {id:?} was not found in handles list for topic {topic_name:?} while unsubscribing");
                return;
            }

            if subscription.handles.is_empty() {
                // This is the last subscriber for that topic and we need to unsubscribe now
                let mut stream = client.writer.write().await;
                match stream.unsubscribe(&topic_name).await {
                    Ok(_) => {}
                    Err(e) => error!(
                        "Failed to send unsubscribe while dropping subscriber: {:?}",
                        e
                    ),
                }
            }
        });
        Ok(())
    }
}

/// A client connection to the rosbridge_server that allows for publishing and subscribing to topics
pub(crate) struct Client {
    // TODO replace Socket with trait RosBridgeComm to allow mocking
    reader: RwLock<Reader>,
    writer: RwLock<Writer>,
    // Stores a record of the publishers we've handed out
    publishers: DashMap<String, PublisherHandle>,
    subscriptions: DashMap<String, Subscription>,
    services: DashMap<String, ServiceCallback>,
    // Contains any outstanding service calls we're waiting for a response on
    // Map key will be a uniquely generated id for each call
    service_calls: DashMap<String, tokio::sync::oneshot::Sender<Value>>,
    opts: ClientHandleOptions,
}

impl Client {
    // internal implementation of new
    async fn new(opts: ClientHandleOptions) -> RosLibRustResult<Self> {
        let (writer, reader) = stubborn_connect(&opts.url).await;
        let client = Self {
            reader: RwLock::new(reader),
            writer: RwLock::new(writer),
            publishers: DashMap::new(),
            services: DashMap::new(),
            subscriptions: DashMap::new(),
            service_calls: DashMap::new(),
            opts,
        };

        Ok(client)
    }

    async fn handle_message(&self, msg: Message) -> RosLibRustResult<()> {
        match msg {
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

        Ok(())
    }

    async fn handle_response(&self, data: Value) {
        // TODO lots of error handling!
        let id = data.get("id").unwrap().as_str().unwrap();
        let (_id, call) = self.service_calls.remove(id).unwrap();
        let res = data.get("values").unwrap();
        call.send(res.clone()).unwrap();
    }

    /// Response handler for receiving a service call looks up if we have a service
    /// registered for the incoming topic and if so dispatches to the callback
    async fn handle_service(&self, data: Value) {
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

    async fn spin_once(&self) -> RosLibRustResult<()> {
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
        self.handle_message(read).await
    }

    /// Response handler for received publish messages
    /// Converts the return message to the subscribed type and calls any callbacks
    /// Panics if publish is received for unexpected topic
    async fn handle_publish(&self, data: Value) {
        // TODO lots of error handling!
        let callbacks = self
            .subscriptions
            .get(data.get("topic").unwrap().as_str().unwrap());
        let callbacks = match callbacks {
            Some(callbacks) => callbacks,
            _ => panic!("Received publish message for unsubscribed topic!"), // TODO probably shouldn't be a panic?
        };
        for callback in callbacks.handles.values() {
            callback(
                // TODO possible bug here if "msg" isn't defined remove this unwrap
                serde_json::to_string(data.get("msg").unwrap())
                    .unwrap()
                    .as_str(),
            )
        }
    }

    async fn reconnect(&mut self) -> RosLibRustResult<()> {
        // Reconnect stream
        let (writer, reader) = stubborn_connect(&self.opts.url).await;
        self.reader = RwLock::new(reader);
        self.writer = RwLock::new(writer);

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

        Ok(())
    }
}

/// Wraps spin in retry logic to handle reconnection attempts automagically
async fn stubborn_spin(
    client: std::sync::Weak<RwLock<Client>>,
    is_disconnected: Arc<AtomicBool>,
) -> RosLibRustResult<()> {
    debug!("Starting stubborn_spin");
    while let Some(client) = client.upgrade() {
        const SPIN_DURATION: Duration = Duration::from_millis(10);

        match tokio::time::timeout(SPIN_DURATION, client.read().await.spin_once()).await {
            Ok(Ok(())) => {}
            Ok(Err(err)) => {
                is_disconnected.store(true, Ordering::Relaxed);
                warn!("Spin failed with error: {err}, attempting to reconnect");
                client.write().await.reconnect().await?;
                is_disconnected.store(false, Ordering::Relaxed);
            }
            Err(_) => {
                // Time out occurred, so we'll check on our weak pointer again
            }
        }
    }

    Ok(())
}

// Implementation of timeout that is a no-op if timeout is 0 or un-configured
// Only works on functions that already return our result type
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

// Connects to websocket at specified URL, retries indefinitely
async fn stubborn_connect(url: &str) -> (Writer, Reader) {
    loop {
        match connect(url).await {
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

// Basic connection attempt and error wrapping
async fn connect(url: &str) -> RosLibRustResult<Socket> {
    let attempt = tokio_tungstenite::connect_async(url).await;
    match attempt {
        Ok((stream, _response)) => Ok(stream),
        Err(e) => Err(e.into()),
    }
}
