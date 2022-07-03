// Contains tools for generating message definitions from ROS1 .msg and .srv files
pub mod message_gen;
// Utilities functions primarily for working with ros env vars and package structures
pub mod util;
// Communication primitives for the rosbridge_suite protocol
pub mod comm;
// Contains generated messages created by message_gen from the test_msgs directory
// TODO look into restricting visibility of this... #[cfg(test)] and pub(crate) not working as needed
pub mod test_msgs;

// Subscriber is a transparent module, we directly expose internal types
// Module exists only to organize source code.
mod subscriber;
pub use subscriber::*;

use anyhow::anyhow;
use comm::RosBridgeComm;
use dashmap::DashMap;
use futures::StreamExt;
use log::*;
use rand::distributions::Alphanumeric;
use rand::{thread_rng, Rng};
use serde::de::DeserializeOwned;
use serde::Serialize;
use serde_json::Value;
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
type Stream = tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<TcpStream>>;

type MessageQueue<T> = deadqueue::limited::Queue<T>;

// TODO queue size should be configurable for both subscribers and publishers
const QUEUE_SIZE: usize = 1_000;

/// For now starting with a central error type, may break this up more in future
#[derive(thiserror::Error, Debug)]
pub enum RosLibRustError {
    // TODO would like to add support for this error, but for now you'll just get CommFailures
    // #[error("Not currently connected to ros master / bridge")]
    // Disconnected,
    #[error("Websoket communication error: {0}")]
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
    pub handles: Vec<Callback>,
    /// Name of ros type (package_name/message_name), used for re-subscribes
    pub topic_type: String,
}

/// The type given to someone when they advertise a topic
// TODO do we need to if multiple publishers to same topic are created and only unadvertise
// when all are dropped?
// Instead of giving back a publisher should we give back a reference to one, and give back the
// same reference when you advertise multiple times? Would require non-mut references?
pub struct Publisher<T: RosMessageType> {
    topic: String,
    // auto incrementing sequence number increased once per publish
    // TODO have to somehow detect if message has header
    // We likely want to implement a Stamped trait and impl it automatically in message gen
    // if we detect a `Header header` in the .msg/.srv
    #[allow(dead_code)]
    seq: usize,
    // Stores a copy of the client so that we can de-register ourselves
    client: Client,
    _marker: std::marker::PhantomData<T>,
}

struct PublisherHandle {
    #[allow(dead_code)]
    topic: String,
    #[allow(dead_code)]
    msg_type: String,
}

impl<T: RosMessageType> Drop for Publisher<T> {
    fn drop(&mut self) {
        self.client.unadvertise(&self.topic);
    }
}

impl<T: RosMessageType> Publisher<T> {
    /// The "standard" publish function sends the message out, returns when publish succeeds
    pub async fn publish(&mut self, msg: T) -> RosLibRustResult<()> {
        self.client.publish(&self.topic, msg).await
    }
}

// TODO we probably want to impl futures sink or some shit?

/// Fundamental traits for message types this works with
/// This trait will be satisfied for any types generate with this crate's message_gen
pub trait RosMessageType:
    'static + DeserializeOwned + Default + Send + Serialize + Sync + Clone + Debug
{
    const ROS_TYPE_NAME: &'static str;
}

/// This special impl allows for services with no args / returns
impl RosMessageType for () {
    const ROS_TYPE_NAME: &'static str = "";
}

// TODO: Potential options to add
//  * stubborn reconnect interval
//  * Automatic header Seq / Stamp setting
#[derive(Clone)]
pub struct ClientOptions {
    url: String,
    timeout: Option<Duration>,
}

impl ClientOptions {
    /// Expects a fully describe websocket url, e.g. 'ws://localhost:9090'
    /// URL is required to be set
    pub fn new<S: Into<String>>(url: S) -> ClientOptions {
        ClientOptions {
            url: url.into(),
            timeout: None,
        }
    }

    /// Configures a default timeout for all operations
    pub fn timeout<T: Into<Duration>>(mut self, duration: T) -> ClientOptions {
        self.timeout = Some(duration.into());
        self
    }
}

// Fundamental structure for a client
#[derive(Clone)]
pub struct Client {
    // TODO split stream into read and write halves
    // TODO replace Stream with trait RosBridgeComm to allow mocking
    comm: Arc<RwLock<Stream>>,
    // Stores a record of the publishers we've handed out
    publishers: Arc<DashMap<String, PublisherHandle>>,
    subscriptions: Arc<DashMap<String, Subscription>>,
    // Contains any outstanding service calls we're waiting for a response on
    // Map key will be a uniquely generated id for each call
    service_calls: Arc<DashMap<String, tokio::sync::oneshot::Sender<Value>>>,
    opts: Arc<ClientOptions>,
}

impl Client {
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
            let mut comm = self.comm.write().await;
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

    // internal implementation of new
    async fn _new(opts: ClientOptions) -> RosLibRustResult<Client> {
        let client = Client {
            comm: Arc::new(RwLock::new(Client::stubborn_connect(&opts.url).await)),
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
                handles: vec![],
                topic_type: Msg::ROS_TYPE_NAME.to_string(),
            });

        // TODO Possible bug here? We send a subscribe message each time even if already subscribed
        // Send subscribe message to rosbridge to initiate it sending us messages
        let mut stream = self.comm.write().await;
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

        // Store callback
        cbs.handles.push(send_cb);

        // Return a subscription handle
        Ok(Subscriber::new(self.clone(), queue))
    }

    /// Subscribe to a given topic expecting msgs of provided type
    pub async fn subscribe<Msg>(&mut self, topic_name: &str) -> RosLibRustResult<Subscriber<Msg>>
    where
        Msg: RosMessageType,
    {
        Client::timeout(self.opts.timeout, self._subscribe(topic_name)).await
    }

    pub async fn unsubscribe(&mut self, topic_name: &str) -> RosLibRustResult<()> {
        self.subscriptions.remove(topic_name);
        let mut stream = self.comm.write().await;
        stream.unsubscribe(topic_name).await?;
        Ok(())
    }

    // This function is not async specifically so it can be called from drop
    // It same reason it doesn't return anything
    fn unadvertise(&mut self, topic_name: &str) {
        let copy = self.clone();
        let topic_name_copy = topic_name.to_string();
        tokio::spawn(async move {
            // Remove publisher from our records
            copy.publishers.remove(&topic_name_copy);

            // Send unadvertise message
            {
                debug!("Unadvertise waiting for comm lock");
                let mut comm = copy.comm.write().await;
                debug!("Unadvertise got comm lock");
                if let Err(e) = comm.unadvertise(&topic_name_copy).await {
                    error!("Failed to send unadvertise in comm layer: {:?}", e);
                }
            }
        });
    }

    /// Core read loop, receives messages from rosbridge and dispatches them.
    // Creating a client spawns a task which calls stubborn spin, which in turn calls this funtion
    async fn spin(&mut self) -> RosLibRustResult<()> {
        debug!("Start spin");
        loop {
            let read = {
                let mut stream = self.comm.write().await;
                // Okay this is what I want to do, but I can't because this actually block publishes
                // We have to instead settle for some periodic bullshit.
                // TODO figure out how to properly duplex the websocket...
                // We need to split the channel into seperate halves with indpendently blocking read / write halves
                // stream.next().await
                match tokio::time::timeout(tokio::time::Duration::from_millis(1), stream.next())
                    .await
                {
                    Ok(read) => read,
                    Err(_) => {
                        // Timed out, release mutex so other tasks can use stream
                        continue;
                    }
                }
            };
            let read = match read {
                Some(r) => r?,
                None => {
                    return Err(RosLibRustError::Unexpected(anyhow::anyhow!(
                        "Wtf does none mean here?"
                    )));
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
            self.comm = Arc::new(RwLock::new(Client::stubborn_connect(&self.opts.url).await));

            // TODO re-advertise!
            // Resend rosbridge our subscription requests to re-establish inflight subscriptions
            // Clone here is dumb, but required due to async
            let mut subs: Vec<(String, String)> = vec![];
            {
                for sub in self.subscriptions.iter() {
                    subs.push((sub.key().clone(), sub.value().topic_type.clone()))
                }
            }
            let mut stream = self.comm.write().await;
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
        for callback in &callbacks.handles {
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
    async fn stubborn_connect(url: &str) -> Stream {
        loop {
            match Client::connect(&url).await {
                Err(e) => {
                    warn!("Failed to reconnect: {:?}", e);
                    // TODO configurable rate?
                    tokio::time::sleep(tokio::time::Duration::from_millis(200)).await;
                    continue;
                }
                Ok(stream) => {
                    return stream;
                }
            }
        }
    }

    /// Bassic connection attempt and error wrapping
    async fn connect(url: &str) -> RosLibRustResult<Stream> {
        let attempt = tokio_tungstenite::connect_async(url).await;
        match attempt {
            Ok((stream, _response)) => Ok(stream),
            Err(e) => Err(e.into()),
        }
    }

    /// Publishes a message
    async fn publish<T>(&mut self, topic: &str, msg: T) -> RosLibRustResult<()>
    where
        T: RosMessageType,
    {
        let mut stream = self.comm.write().await;
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
            let mut stream = self.comm.write().await;
            debug!("Advertise got lock on comm");
            stream.advertise::<T>(topic).await?;
        }
        Ok(Publisher {
            topic: topic.to_string(),
            seq: 0,
            client: self.clone(),
            _marker: Default::default(),
        })
    }
}

// TODO all tests in this mod require a running rosbridge_server at localhost:9090
// How to set that up before this module runs?
// How to test against both rosbridge_1 and rosbridge_2 automagically?
#[cfg(test)]
mod general_usage {
    use crate::test_msgs::NodeInfo;
    #[allow(dead_code)]
    pub const LOCAL_WS: &str = "ws://localhost:9090";

    /// Ensures that associate constants are generated on the test_msgs correctly
    /// requires test_msgs gen_code to have been generated.
    /// Compliation is passing for this test
    //TODO may move this code somewhere else
    #[test]
    fn test_associated_contants() {
        let _ = NodeInfo::STATUS_UNINITIALIZED;
    }

    // TODO would like some test like this when we support disconnected...
    // For now this test should hang on stubborn connect
    // #[tokio::test]
    // /// Tests the correct error is returned when failing to connect
    // async fn test_disconnected_on_bad_client() {
    //     // This is an intentionally valid address point where we DO NOT expect a bridge to be running
    //     match Client::new("ws://localhost:9999").await {
    //         Ok(_) => assert!(false, "Client should not be able to connect here!"),
    //         Err(RosLibRustError::Disconnected) => {} // test passes hooray!,
    //         Err(_) => assert!(false, "Got some other error!"),
    //     }
    // }
}

#[cfg(test)]
#[cfg(feature = "running_bridge")]
mod integration_tests {
    use crate::general_usage::LOCAL_WS;
    use crate::test_msgs::Header;
    use crate::{Client, ClientOptions, Subscriber};
    use log::debug;
    use tokio::time::timeout;
    // On my laptop test was ~90% reliable at 10ms
    // Had 1 spurious github failure at 100
    const TIMEOUT: Duration = Duration::from_millis(200);
    use tokio::time::Duration;

    type TestResult = Result<(), anyhow::Error>;

    /**
    This test does a round trip publish subscribe for real
    Requires a running local rosbridge
    TODO figure out how to automate setting up the needed environment for this
    */
    #[tokio::test]
    async fn self_publish() {
        // TODO figure out better logging for tests

        let _ = simple_logger::SimpleLogger::new()
            .with_level(log::LevelFilter::Debug)
            .without_timestamps()
            .init();

        const TOPIC: &str = "self_publish";
        // 100ms allowance for connecting so tests still fails
        let mut client = timeout(TIMEOUT, Client::new(LOCAL_WS))
            .await
            .expect("Failed to create client in time")
            .unwrap();

        timeout(TIMEOUT, client.advertise::<Header>(TOPIC))
            .await
            .expect("Failed to advertise in time")
            .unwrap();
        let rx = timeout(TIMEOUT, client.subscribe::<Header>(TOPIC))
            .await
            .expect("Failed to subscribe in time")
            .unwrap();

        // Delay here to allow subscribe to complete before publishing
        // Test is flaky without it
        tokio::time::sleep(TIMEOUT).await;

        let msg_out = Header {
            seq: 666,
            stamp: Default::default(),
            frame_id: "self_publish".to_string(),
        };

        timeout(TIMEOUT, client.publish(TOPIC, msg_out.clone()))
            .await
            .expect("Failed to publish in time")
            .unwrap();

        let msg_in = timeout(TIMEOUT, rx.next())
            .await
            .expect("Failed to receive in time");

        assert_eq!(msg_in, msg_out);
    }

    #[tokio::test]
    /// Designed to test behavior when receiving a message of unexpected type on a topic
    // TODO this test is good, but actually shows how bad the ergonomics are and how we want to improve them!
    // We want a failed message parse / type mismatch to come through to the subscriber
    async fn bad_message_recv() -> TestResult {
        let mut client = Client::new_with_options(
            ClientOptions::new(crate::general_usage::LOCAL_WS).timeout(TIMEOUT),
        )
        .await?;

        let mut publisher = client
            .advertise::<crate::test_msgs::TimeI>("/bad_message_recv/topic")
            .await?;

        let sub: Subscriber<crate::test_msgs::Header> =
            client.subscribe("/bad_message_recv/topic").await?;

        publisher
            .publish(crate::test_msgs::TimeI { secs: 0, nsecs: 0 })
            .await?;

        match timeout(TIMEOUT, sub.next()).await {
            Err(_elapsed) => {
                // Test passed! it should timeout
                // Not actually behavior we want, error of some kind should come through subscription
            }
            _ => {
                assert!(false, "Bad message made it throught");
            }
        }
        Ok(())
    }

    #[tokio::test]
    async fn timeouts_new() {
        // Intentionally a port where there won't be a server at
        let opts = ClientOptions::new("ws://localhost:666").timeout(TIMEOUT);
        assert!(Client::new_with_options(opts).await.is_err());
        // Impossibly short to actually work
        let opts = ClientOptions::new(LOCAL_WS).timeout(Duration::from_nanos(1));
        assert!(Client::new_with_options(opts).await.is_err());
        // Doesn't timeout if given enough time
        let opts = ClientOptions::new(LOCAL_WS).timeout(TIMEOUT);
        assert!(Client::new_with_options(opts).await.is_ok());
    }

    /// This test doesn't actually do much, but instead confirms the internal structure of the lib is multi-threaded correclty
    /// The whole goal here is to catch send / sync complier errors
    #[tokio::test]
    async fn parrallel_construnction() {
        let mut client = Client::new(LOCAL_WS)
            .await
            .expect("Failed to construct client");

        let mut client_1 = client.clone();
        tokio::task::spawn(async move {
            let _ = client_1
                .advertise::<Header>("parrallel_1")
                .await
                .expect("Failed to advertise _1");
        });

        tokio::task::spawn(async move {
            let _ = client
                .subscribe::<Header>("parrallel_1")
                .await
                .expect("Failed to subscribe _1");
        });
    }

    /// Tests that dropping a publisher correctly unadvertises
    #[tokio::test]
    // This test is currently broken, it seems that rosbridge still sends the message regardless
    // of advertise / unadvertise status. Unclear how to confirm whether advertise was sent or not
    #[ignore]
    async fn unadvertise() -> TestResult {
        let _ = simple_logger::SimpleLogger::new()
            .with_level(log::LevelFilter::Debug)
            .without_timestamps()
            .init();

        // Flow:
        //  1. Create a publisher and subscriber
        //  2. Send a message and confirm connection works (topic was advertised)
        //  3. Drop the publisher, unadvertise should be sent
        //  4. Manually send a message without a publisher it should fail since topic was unadvertised
        const TOPIC: &str = "/unadvertise";
        debug!("Start unadvertise test");

        let opt = ClientOptions::new(LOCAL_WS).timeout(TIMEOUT);

        let mut client = Client::new_with_options(opt).await?;
        let mut publisher = client.advertise(TOPIC).await?;
        debug!("Got publisher");

        let sub = client.subscribe::<Header>(TOPIC).await?;
        debug!("Got subscriber");

        let msg = Header::default();
        publisher.publish(msg).await?;
        timeout(TIMEOUT, sub.next()).await?;

        debug!("Dropping publisher");
        // drop subscriber so it doesn't keep topic open
        std::mem::drop(sub);
        // unadvertise should happen here
        std::mem::drop(publisher);

        // Wait for drop to complete
        tokio::time::sleep(TIMEOUT).await;

        let sub = client.subscribe::<Header>(TOPIC).await?;
        // manually publishing using private api
        let msg = Header::default();
        client.publish(TOPIC, msg).await?;

        match timeout(TIMEOUT, sub.next()).await {
            Ok(_msg) => {
                anyhow::bail!("Received message after unadvertised!");
            }
            Err(_e) => {
                // All good! Timeout should expire
            }
        }
        Ok(())
    }
}
