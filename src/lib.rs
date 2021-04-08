pub mod message_gen;
pub mod util;

use std::collections::HashMap;
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

/// Holds a single websocket connection, requests will be processed by a spin_once or spin() call.
pub struct Client {
    stream: Box<tokio_tungstenite::WebSocketStream<TcpStream>>,
    subscriptions: HashMap<String, Vec<Callback>>,
}

impl Client {
    /// Connects a rosbridge instance at the given url
    /// Expects a fully describe websocket url, e.g. 'ws://localhost:9090'
    /// Panics if connection fails
    // TODO better error handling
    // TODO spawn spin task here?
    pub async fn new(url: &str) -> Client {
        // TODO we should be able to async connect in the future
        info!("About to tungstentite");
        let (stream, _) = tokio_tungstenite::connect_async(url)
            .await
            .expect("Failed to connect client");
        info!("Connect async");
        Client {
            stream: Box::new(stream),
            subscriptions: HashMap::new(),
        }
    }

    /// Subscribe to a given topic expecting msgs of provided type
    pub async fn subscribe<Msg>(&mut self, topic_name: &str) -> tokio::sync::watch::Receiver<Msg>
    where
        Msg: 'static + DeserializeOwned + Default + Send + Sync + Clone,
    {
        let cbs = self
            .subscriptions
            .entry(topic_name.to_string())
            .or_insert(vec![]);

        // TODO single place to define rosbridge operations
        // Send subscribe message to rosbridge to initiate it sending us messages
        let msg = json!(
        {
        "op": "subscribe",
        "topic": topic_name,
        }
        );
        let msg = tungstenite::Message::Text(msg.to_string());
        // TODO async here
        self.stream.send(msg).await.unwrap();

        // TODO we have spsc watch is kinda overkill? Buffer of 1 is nice...
        // Note: Type erasure with callback is forcing generating a new channel per subscription
        // instead of allowing us to re-use the subscription.
        // I can't figure out how to store rx's with different types...
        // Trait objects can't
        // Create a new watch channel for this topic
        let (tx, rx) = tokio::sync::watch::channel(Msg::default());

        // Move the tx into a callback that takes raw string data
        // This allows us to store the callbacks generic on type, Msg conversion is embedded here
        let send_cb = Box::new(move |data: &str| {
            let converted: Msg = serde_json::from_str(data).unwrap();
            tx.broadcast(converted);
            // TODO can we detect that all rx's have been dropped and auto clean-up here?
        });

        // Store callback
        cbs.push(send_cb);

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
    /// TODO async version?
    /// TODO how to integrate with other event loops?
    /// TODO how will borrowing work with these callbacks
    /// TODO should we use something like 'bus' crate instead of callbacks?
    /// TODO spin_once version?
    pub async fn spin(&mut self) {
        while let read = self.stream.next().await.unwrap().unwrap() {
            match read {
                Message::Text(text) => {
                    debug!("got message: {}", text);
                    let parsed: serde_json::Value = serde_json::from_str(text.as_str())
                        .expect("Invalid json received from server.");

                    let parsed_object = parsed
                        .as_object()
                        .expect("Server sent json that was not an object");

                    let op = parsed_object
                        .get("op")
                        .expect("Server sent json without an 'op' key")
                        .as_str()
                        .unwrap();
                    match op {
                        "publish" => {
                            self.handle_publish(parsed);
                        }
                        _ => {
                            warn!("Unhandled op type {}", op)
                        }
                    }
                }
                _ => {
                    panic!("Non-text response received");
                }
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
        for callback in callbacks {
            callback(
                serde_json::to_string(data.get("msg").unwrap())
                    .unwrap()
                    .as_str(),
            )
        }
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
