use std::collections::HashMap;
use std::net::TcpStream;

use serde_json::json;
use tokio_tungstenite::*;
use tungstenite::Message;

type Callback = fn(&str) -> ();

pub struct Client {
    pub stream: tungstenite::WebSocket<TcpStream>,
    subscriptions: HashMap<String, Vec<Callback>>,
}

impl Client {
    pub fn new(url: &str) -> Client {
        let (stream, _) = tungstenite::connect(url).expect("Failed to connect client");
        Client {
            stream,
            subscriptions: HashMap::new(),
        }
    }

    pub fn subscribe(&mut self, topic_name: &str, callback: Callback) {
        // Register callback in callback map
        let topic = self
            .subscriptions
            .entry(topic_name.to_string())
            .or_insert(vec![]);
        topic.push(callback);
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

    pub fn spin(&mut self) {
        loop {
            let read = self.stream.read_message().unwrap();
            match read {
                Message::Text(text) => {
                    print!("got message: {:?}\n", text);
                }
                _ => {
                    panic!("Non-text response received");
                }
            }
        }
    }
}
