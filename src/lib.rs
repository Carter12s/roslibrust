pub mod message_gen;
pub mod gen_msgs;
pub mod util;

use std::collections::HashMap;
use std::net::TcpStream;

use log::*;
use serde_json::{json, Value};
use tokio_tungstenite::*;
use tungstenite::Message;
use serde::de::DeserializeOwned;

type Callback = Box<dyn Fn(&str) -> ()>;

/// Basic "syncish" client representation.
/// Holds a single websocket connection, requests will be processed by a spin_once or spin() call.
pub struct Client {
    pub stream: tungstenite::WebSocket<TcpStream>,
    subscriptions: HashMap<String, Vec<Callback>>,
}

impl Client {
    /// Connects a rosbridge instance at the given url
    /// Expects a fully describe websocket url, e.g. 'ws://localhost:9090'
    /// Panics if connection fails
    // TODO better error handling
    pub fn new(url: &str) -> Client {
        let (stream, _) = tungstenite::connect(url).expect("Failed to connect client");
        Client {
            stream,
            subscriptions: HashMap::new(),
        }
    }

    /// Subscribe to a given topic expecting msgs of provided type
    pub fn subscribe<Msg: 'static>(&mut self, topic_name: &str, callback: fn(Msg))
        where
            Msg: DeserializeOwned,
    {
        // Register callback in callback map
        let topic = self
            .subscriptions
            .entry(topic_name.to_string())
            .or_insert(vec![]);
        topic.push(Box::new(move |data: &str| {
            let msg: Msg =
                serde_json::from_str(data).expect("Could not interpret data as message type.");
            callback(msg)
        }));
        // TODO single place to define rosbridge operations
        let msg = json!(
        {
        "op": "subscribe",
        "topic": topic_name,
        }
        );
        let msg = tungstenite::Message::Text(msg.to_string());
        self.stream
            .write_message(msg)
            .expect("Failed to write subscription message!");
    }

    /// Core read loop, recivies messages from rosbridge and dispatches them.
    /// TODO async version?
    /// TODO how to integrate with other event loops?
    /// TODO how will borrowing work with these callbacks
    /// TODO should we use something like 'bus' crate instead of callbacks?
    /// TODO spin_once version?
    pub fn spin(&mut self) {
        loop {
            let read = self.stream.read_message().unwrap();
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
        let callbacks = self.subscriptions.get(data.get("topic").unwrap().as_str().unwrap());
        let callbacks = match callbacks {
            Some(callbacks) => callbacks,
            _ => panic!("Received publish message for unsubscribed topic!"),
        };
        for i in callbacks {
            i(serde_json::to_string(data.get("msg").unwrap()).unwrap().as_str())
        }
    }

    /*
    TODO:
        * Unsubscribe
        * Service call, is this going to need to return a future?
        * Type generation from messages: A) static B) with cargo...
        Low priority:
            * advertise
            * un-advertise
     */
}
