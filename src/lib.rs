pub mod message_gen;
pub mod util;

use std::collections::HashMap;
use std::error::Error;
use tokio::sync::watch;

use futures::future::BoxFuture;
use futures::{SinkExt, StreamExt};
use log::*;
use serde::de::DeserializeOwned;
use serde_json::{json, Value};
use tokio::net::TcpStream;
use tokio::task::JoinHandle;
use tokio_tungstenite::tungstenite::client::AutoStream;
use tokio_tungstenite::*;
use tungstenite::Message;

type Callback = Box<dyn Fn(&str) -> () + Send>;
type Stream = Box<tokio_tungstenite::WebSocketStream<TcpStream>>;

struct Subscription {
    pub handles: Vec<Callback>,
    // Name of ros type (package_name/message_name), used for re-subscribes
    pub topic_type: String,
}

/// Fundamental traits for message types this works with
/// This trait will be satisfied for any types generate with this crate's message_gen
pub trait RosMessageType: 'static + DeserializeOwned + Default + Send + Sync + Clone {
    const ROS_TYPE_NAME: &'static str;
}

/// Holds a single websocket connection, requests will be processed by a spin_once or spin() call.
pub struct Client {
    stream: Stream,
    subscriptions: HashMap<String, Subscription>,
    url: String,
}

impl Client {
    /// Connects a rosbridge instance at the given url
    /// Expects a fully describe websocket url, e.g. 'ws://localhost:9090'
    /// When awaited will not resolve until connection is completed
    // TODO better error handling
    // TODO spawn spin task here?
    pub async fn new(url: String) -> Result<Client, Box<dyn Error>> {
        Ok(Client {
            stream: Client::stubborn_connect(url.to_string()).await,
            subscriptions: HashMap::new(),
            url,
        })
    }

    /// Subscribe to a given topic expecting msgs of provided type
    pub async fn subscribe<Msg>(&mut self, topic_name: &str) -> tokio::sync::watch::Receiver<Msg>
    where
        Msg: RosMessageType,
    {
        let cbs = self
            .subscriptions
            .entry(topic_name.to_string())
            .or_insert(Subscription {
                handles: vec![],
                topic_type: Msg::ROS_TYPE_NAME.to_string(),
            });

        // TODO single place to define rosbridge operations
        // Send subscribe message to rosbridge to initiate it sending us messages
        Client::send_subscribe_request(
            &mut self.stream,
            &topic_name.to_string(),
            &Msg::ROS_TYPE_NAME.to_string(),
        )
        .await;

        // TODO we have spsc, so watch is kinda overkill? Buffer of 1 is nice...
        // Note: Type erasure with callback is forcing generating a new channel per subscription
        // instead of allowing us to re-use the subscription.
        // I can't figure out how to store rx's with different types...
        // Trait objects can't
        // Create a new watch channel for this topic
        let (tx, mut rx) = tokio::sync::watch::channel(Msg::default());

        // Do a dummy  recieve to purge the default... watch feels like wrong choice again...
        let x = rx.recv().await;

        // Move the tx into a callback that takes raw string data
        // This allows us to store the callbacks generic on type, Msg conversion is embedded here
        let send_cb = Box::new(move |data: &str| {
            let converted: Msg = serde_json::from_str(data).unwrap();
            tx.broadcast(converted);
        });

        // Store callback
        cbs.handles.push(send_cb);

        // Return a subscription handle
        rx
    }

    pub async fn unsubscribe(&mut self, topic_name: &str) {
        self.subscriptions.remove(topic_name);
        let msg = json!(
        {
        "op": "unsubscribe",
        "topic": topic_name,
        }
        );
        let msg = tungstenite::Message::Text(msg.to_string());
        self.stream.send(msg).await;
    }

    /// Core read loop, receives messages from rosbridge and dispatches them.
    pub async fn spin(&mut self) -> Result<(), Box<dyn Error>> {
        debug!("Start spin");
        loop {
            let read = self.stream.next().await;
            let read = match read {
                Some(r) => r?,
                None => {
                    return Err(Box::new(simple_error::SimpleError::new(
                        "WTF does none mean here?",
                    )))
                }
            };
            match read {
                Message::Text(text) => {
                    debug!("got message: {}", text);
                    let parsed: serde_json::Value = serde_json::from_str(text.as_str())?;
                    let parsed_object = parsed
                        .as_object()
                        .expect("Recieved non-object json response");
                    let op = parsed_object
                        .get("op")
                        .expect("Op field not present on returned object.")
                        .as_str()
                        .expect("Op field was not of string type.");
                    match op {
                        "publish" => {
                            self.handle_publish(parsed);
                        }
                        _ => {
                            warn!("Unhandled op type {}", op)
                        }
                    }
                }
                Message::Close(close) => {
                    panic!("Close requested from server");
                }
                Message::Ping(ping) => {
                    debug!("Ping received");
                }
                Message::Pong(pong) => {
                    debug!("Pong received");
                }
                _ => {
                    panic!("Non-text response received");
                }
            }
        }
    }

    /// Wraps spin in retry logic to handle reconnections automagically
    pub async fn stubborn_spin(&mut self) -> Result<(), Box<dyn Error>> {
        debug!("Starting stubborn_spin");
        loop {
            let res = self.spin().await;
            if res.is_err() {
                warn!(
                    "Spin failed with error: {}, attempting to reconnect",
                    res.err().unwrap()
                );
            } else {
                panic!("Spin should not exit cleanly?");
            }
            // Reconnect stream
            self.stream = Client::stubborn_connect(self.url.to_string()).await;
            // Resend rosbridge our subscription requests to re-establish inflight subscriptions
            for (topic, sub) in &self.subscriptions {
                Client::send_subscribe_request(&mut self.stream, &topic, &sub.topic_type).await;
            }
        }
    }

    /// Response handler for received publish messages
    /// Converts the return message to the subscribed type and calls any callbacks
    /// Panics if publish is received for unexpected topic
    fn handle_publish(&mut self, data: Value) {
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

    /// Connects to websocket at specified URL, retries indefinitely
    async fn stubborn_connect(url: String) -> Stream {
        loop {
            let connection_attempt = tokio_tungstenite::connect_async(&url).await;
            if connection_attempt.is_err() {
                warn!("Failed to reconnect: {}", connection_attempt.err().unwrap());
                // TODO configurable rate?
                tokio::time::delay_for(tokio::time::Duration::from_millis(200)).await;
                continue;
            }
            let (stream, _) = connection_attempt.unwrap();
            return Box::new(stream);
        }
    }

    /// Sends a subscribe request packet to rosbridge
    // Note: doesn't borrow self only stream so we can iterate subscriptions and call simultaneously
    async fn send_subscribe_request(
        stream: &mut Stream,
        topic: &String,
        msg_type: &String,
    ) -> Result<(), Box<dyn Error>> {
        let msg = json!(
        {
        "op": "subscribe",
        "topic": topic,
        "type": msg_type,
        }
        );
        let msg = tungstenite::Message::Text(msg.to_string());
        stream.send(msg).await?;
        Ok(())
    }

    /*
    TODO:
        * Unsubscribe
        * Service call, is this going to need to return a future?
        * Type generation from messages: A) static B) with cargo...
        * Testing?
        Low priority:
            * advertise
            * un-advertise
     */
}
