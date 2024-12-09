use crate::ros1::{names::Name, tcpros::ConnectionHeader};
use abort_on_drop::ChildTask;
use log::*;
use roslibrust_codegen::{RosMessageType, ShapeShifter};
use std::{marker::PhantomData, sync::Arc};
use tokio::{
    io::AsyncWriteExt,
    net::TcpStream,
    sync::{
        broadcast::{self, error::RecvError},
        RwLock,
    },
};

use super::tcpros;

pub struct Subscriber<T> {
    receiver: broadcast::Receiver<Vec<u8>>,
    _phantom: PhantomData<T>,
}

impl<T: RosMessageType> Subscriber<T> {
    pub(crate) fn new(receiver: broadcast::Receiver<Vec<u8>>) -> Self {
        Self {
            receiver,
            _phantom: PhantomData,
        }
    }

    pub async fn next(&mut self) -> Option<Result<T, SubscriberError>> {
        trace!("Subscriber of type {:?} awaiting recv()", T::ROS_TYPE_NAME);
        let data = match self.receiver.recv().await {
            Ok(v) => {
                trace!("Subscriber of type {:?} received data", T::ROS_TYPE_NAME);
                v
            }
            Err(RecvError::Closed) => return None,
            Err(RecvError::Lagged(n)) => return Some(Err(SubscriberError::Lagged(n))),
        };
        trace!(
            "Subscriber of type {:?} deserializing data",
            T::ROS_TYPE_NAME
        );
        let tick = tokio::time::Instant::now();
        match roslibrust_serde_rosmsg::from_slice::<T>(&data[..]) {
            Ok(p) => {
                let duration = tick.elapsed();
                trace!(
                    "Subscriber of type {:?} deserialized data in {duration:?}",
                    T::ROS_TYPE_NAME
                );
                Some(Ok(p))
            }
            Err(e) => Some(Err(e.into())),
        }
    }
}

pub struct SubscriberAny {
    receiver: broadcast::Receiver<Vec<u8>>,
    _phantom: PhantomData<ShapeShifter>,
}

impl SubscriberAny {
    pub(crate) fn new(receiver: broadcast::Receiver<Vec<u8>>) -> Self {
        Self {
            receiver,
            _phantom: PhantomData,
        }
    }

    // pub async fn next(&mut self) -> Option<Result<ShapeShifter, SubscriberError>> {
    pub async fn next(&mut self) -> Option<Result<Vec<u8>, SubscriberError>> {
        let data = match self.receiver.recv().await {
            Ok(v) => v,
            Err(RecvError::Closed) => return None,
            Err(RecvError::Lagged(n)) => return Some(Err(SubscriberError::Lagged(n))),
        };
        Some(Ok(data))
    }
}

pub struct Subscription {
    subscription_tasks: Vec<ChildTask<()>>,
    _msg_receiver: broadcast::Receiver<Vec<u8>>,
    msg_sender: broadcast::Sender<Vec<u8>>,
    connection_header: ConnectionHeader,
    known_publishers: Arc<RwLock<Vec<String>>>,
}

impl Subscription {
    pub fn new(
        node_name: &Name,
        topic_name: &str,
        topic_type: &str,
        queue_size: usize,
        msg_definition: String,
        md5sum: String,
    ) -> Self {
        let (sender, receiver) = broadcast::channel(queue_size);
        let connection_header = ConnectionHeader {
            caller_id: node_name.to_string(),
            latching: false,
            msg_definition,
            md5sum: Some(md5sum),
            topic: Some(topic_name.to_owned()),
            topic_type: topic_type.to_owned(),
            tcp_nodelay: false,
            service: None,
            persistent: None,
        };

        Self {
            subscription_tasks: vec![],
            _msg_receiver: receiver,
            msg_sender: sender,
            connection_header,
            known_publishers: Arc::new(RwLock::new(vec![])),
        }
    }

    pub fn topic_type(&self) -> &str {
        self.connection_header.topic_type.as_str()
    }

    pub fn get_receiver(&self) -> broadcast::Receiver<Vec<u8>> {
        self.msg_sender.subscribe()
    }

    pub async fn add_publisher_source(
        &mut self,
        publisher_uri: &str,
    ) -> Result<(), std::io::Error> {
        let is_new_connection = {
            self.known_publishers
                .read()
                .await
                .iter()
                .find(|publisher| publisher.as_str() == publisher_uri)
                .is_none()
        };

        if is_new_connection {
            let node_name = self.connection_header.caller_id.clone();
            let topic_name = self.connection_header.topic.as_ref().unwrap().clone();
            let connection_header = self.connection_header.clone();
            let sender = self.msg_sender.clone();
            let publisher_list = self.known_publishers.clone();
            let publisher_uri = publisher_uri.to_owned();
            trace!("Creating new subscription connection for {publisher_uri} on {topic_name}");
            let handle = tokio::spawn(async move {
                if let Ok(mut stream) = establish_publisher_connection(
                    &node_name,
                    &topic_name,
                    &publisher_uri,
                    connection_header,
                )
                .await
                {
                    publisher_list.write().await.push(publisher_uri.to_owned());
                    // Repeatedly read from the stream until its dry
                    loop {
                        trace!(
                            "Subscription to {} receiving from {} is awaiting next body",
                            topic_name,
                            publisher_uri
                        );
                        match tcpros::receive_body(&mut stream).await {
                            Ok(body) => {
                                trace!(
                                    "Subscription to {} receiving from {} received body",
                                    topic_name,
                                    publisher_uri
                                );
                                let send_result = sender.send(body);
                                if let Err(err) = send_result {
                                    log::error!("Unable to send message data due to dropped channel, closing connection: {err}");
                                    break;
                                }
                            }
                            Err(e) => {
                                log::debug!("Failed to read body from publisher connection: {e}, closing connection");
                                break;
                            }
                        }
                    }
                }
            });
            self.subscription_tasks.push(handle.into());
        }

        Ok(())
    }
}

async fn establish_publisher_connection(
    node_name: &str,
    topic_name: &str,
    publisher_uri: &str,
    conn_header: ConnectionHeader,
) -> Result<TcpStream, std::io::Error> {
    let publisher_channel_uri = send_topic_request(node_name, topic_name, publisher_uri).await?;
    let mut stream = TcpStream::connect(publisher_channel_uri).await?;

    let conn_header_bytes = conn_header.to_bytes(true)?;
    stream.write_all(&conn_header_bytes[..]).await?;

    if let Ok(responded_header) = tcpros::receive_header(&mut stream).await {
        if conn_header.md5sum == Some("*".to_string())
            || responded_header.md5sum == Some("*".to_string())
            || conn_header.md5sum == responded_header.md5sum
        {
            log::debug!(
                "Established connection with publisher for {:?}",
                conn_header.topic
            );
            Ok(stream)
        } else {
            log::error!(
                "Tried to subscribe to {}, but md5sums do not match. Expected {:?}, received {:?}",
                topic_name,
                conn_header.md5sum,
                responded_header.md5sum
            );
            Err(std::io::ErrorKind::InvalidData)
        }
    } else {
        log::error!("Could not parse connection header data sent by publisher");
        Err(std::io::ErrorKind::InvalidData)
    }
    .map_err(std::io::Error::from)
}

async fn send_topic_request(
    node_name: &str,
    topic_name: &str,
    publisher_uri: &str,
) -> Result<String, std::io::Error> {
    let xmlrpc_client = reqwest::Client::new();
    let body = serde_xmlrpc::request_to_string(
        "requestTopic",
        vec![
            node_name.into(),
            topic_name.into(),
            serde_xmlrpc::Value::Array(vec![serde_xmlrpc::Value::Array(vec!["TCPROS".into()])]),
        ],
    )
    .unwrap();

    let response = xmlrpc_client
        .post(publisher_uri)
        .body(body)
        .send()
        .await
        .map_err(|err| {
            log::error!("Unable to send subscribe request to publisher: {err}");
            std::io::ErrorKind::ConnectionAborted
        })?;
    if response.status().is_success() {
        if let Ok(response_data) = response.text().await {
            if let Ok((_code, _description, (protocol, hostname, port))) =
                serde_xmlrpc::response_from_str::<(i8, String, (String, String, u16))>(
                    &response_data,
                )
            {
                if protocol == "TCPROS" {
                    let tcpros_endpoint = format!("{hostname}:{port}");
                    log::debug!("Got a TCPROS publisher endpoint at {tcpros_endpoint}");
                    Ok(tcpros_endpoint)
                } else {
                    log::error!("Got unsupported protocol {protocol}");
                    Err(std::io::ErrorKind::Unsupported.into())
                }
            } else {
                log::error!("Failed to deserialize requestTopic response {response_data}");
                Err(std::io::ErrorKind::InvalidData.into())
            }
        } else {
            log::error!("No data received with the response");
            Err(std::io::ErrorKind::InvalidData.into())
        }
    } else {
        log::error!(
            "Failed to request topic data from the publisher's XMLRPC server for {publisher_uri}: {response:#?}"
        );
        Err(std::io::ErrorKind::ConnectionRefused.into())
    }
}

#[derive(thiserror::Error, Debug)]
pub enum SubscriberError {
    /// Deserialize Error from `serde_rosmsg::Error` (stored as String because of dyn Error)
    #[error("serde_rosmsg Error: {0}")]
    DeserializeError(String),
    #[error("you are too slow, {0} messages were skipped")]
    Lagged(u64),
}

impl From<roslibrust_serde_rosmsg::Error> for SubscriberError {
    fn from(value: roslibrust_serde_rosmsg::Error) -> Self {
        Self::DeserializeError(value.to_string())
    }
}
