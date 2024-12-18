use crate::{
    names::Name,
    tcpros::{self, ConnectionHeader},
};
use abort_on_drop::ChildTask;
use log::*;
use roslibrust_common::RosMessageType;
use std::{
    marker::PhantomData,
    net::{Ipv4Addr, SocketAddr},
};
use tokio::{
    io::AsyncWriteExt,
    sync::broadcast::{self, error::RecvError},
};

use super::actor::NodeServerHandle;

/// The regular Publisher representation returned by calling advertise on a [crate::ros1::NodeHandle].
pub struct Publisher<T> {
    // Name of the topic this publisher is publishing on
    topic_name: String,
    // Actual channel on which messages are sent to be published
    sender: broadcast::Sender<Vec<u8>>,
    // When the last publisher for a given topic is dropped, this channel is used to signal to cleanup
    // for the underlying publication
    _shutdown_channel: tokio::sync::mpsc::Sender<()>,
    // Phantom data to ensure that the type is known at compile time
    phantom: PhantomData<T>,
}

impl<T: RosMessageType> Publisher<T> {
    pub(crate) fn new(
        topic_name: &str,
        sender: broadcast::Sender<Vec<u8>>,
        shutdown_channel: tokio::sync::mpsc::Sender<()>,
    ) -> Self {
        Self {
            topic_name: topic_name.to_owned(),
            sender,
            _shutdown_channel: shutdown_channel,
            phantom: PhantomData,
        }
    }

    /// Queues a message to be sent on the related topic.
    // TODO Major this no longer needs to be (or should be) async
    pub async fn publish(&self, data: &T) -> Result<(), PublisherError> {
        let data = roslibrust_serde_rosmsg::to_vec(&data)?;
        // TODO this is a pretty dumb...
        // because of the internal channel used for re-direction this future doesn't
        // actually complete when the data is sent, but merely when it is queued to be sent
        // This function could probably be non-async
        // Or we should do some significant re-work to have it only yield when the data is sent.
        self.sender
            .send(data)
            .map_err(|_| PublisherError::StreamClosed)?;
        debug!("Publishing data on topic {}", self.topic_name);
        Ok(())
    }
}

/// A specialty publisher used when message type is not known at compile time.
///
/// Relies on user to provide serialized data. Typically used with playback from bag files.
pub struct PublisherAny {
    topic_name: String,
    sender: broadcast::Sender<Vec<u8>>,
    // When the last publisher for a given topic is dropped, this channel is used to signal to cleanup
    // Don't need to send a message, simply dropping the last handle lets to node know to clean up
    // Note: this has to be used because tokio::sync::broadcast doesn't have a WeakSender
    _shutdown: tokio::sync::mpsc::Sender<()>,
    phantom: PhantomData<Vec<u8>>,
}

impl PublisherAny {
    pub(crate) fn new(
        topic_name: &str,
        sender: broadcast::Sender<Vec<u8>>,
        shutdown: tokio::sync::mpsc::Sender<()>,
    ) -> Self {
        Self {
            topic_name: topic_name.to_owned(),
            sender,
            _shutdown: shutdown,
            phantom: PhantomData,
        }
    }

    /// Queues a message to be sent on the related topic.
    ///
    /// This expects the data to be the raw bytes of the message body as they would appear going over the wire.
    /// See ros1_publish_any.rs example for more details.
    /// Body length should be included as first four bytes.
    // TODO this no longer needs to be (or should be) async
    pub async fn publish(&self, data: &Vec<u8>) -> Result<(), PublisherError> {
        // TODO this is a pretty dumb...
        // because of the internal channel used for re-direction this future doesn't
        // actually complete when the data is sent, but merely when it is queued to be sent
        // This function could probably be non-async
        // Or we should do some significant re-work to have it only yield when the data is sent.
        self.sender
            .send(data.to_vec())
            .map_err(|_| PublisherError::StreamClosed)?;
        debug!("Publishing data on topic {}", self.topic_name);
        Ok(())
    }
}

pub(crate) struct Publication {
    topic_type: String,
    listener_port: u16,
    _tcp_accept_task: ChildTask<()>,
    publish_sender: broadcast::Sender<Vec<u8>>,
    // We store a weak handle to the shutdown channel
    // This allows us to create new Publisher with a shutdown sender, but doesn't keep the shutdown channel alive
    // Had to add this because broadcast doesn't have a weak sender equivalent
    weak_shutdown_channel: tokio::sync::mpsc::WeakSender<()>,
}

impl Publication {
    /// Spawns a new publication and sets up all tasks to run it
    /// Returns a handle to the publication and a mpsc::Sender to send messages to be published
    /// Dropping the Sender will (eventually) result in the publication being dropped and all tasks being canceled
    pub(crate) async fn new(
        node_name: &Name,
        latching: bool,
        topic_name: &str,
        host_addr: Ipv4Addr,
        queue_size: usize,
        msg_definition: &str,
        md5sum: &str,
        topic_type: &str,
        node_handle: NodeServerHandle,
    ) -> Result<
        (
            Self,
            broadcast::Sender<Vec<u8>>,
            tokio::sync::mpsc::Sender<()>,
        ),
        std::io::Error,
    > {
        // Get a socket for receiving connections on
        let host_addr = SocketAddr::from((host_addr, 0));
        let tcp_listener = tokio::net::TcpListener::bind(host_addr).await?;
        let listener_port = tcp_listener.local_addr().unwrap().port();

        // Setup the channel will will receive messages to be published on
        let (sender, receiver) = broadcast::channel::<Vec<u8>>(queue_size);

        // Setup the ROS connection header that we'll respond to all incoming connections with
        let responding_conn_header = ConnectionHeader {
            caller_id: node_name.to_string(),
            latching,
            msg_definition: msg_definition.to_owned(),
            md5sum: Some(md5sum.to_owned()),
            topic: Some(topic_name.to_owned()),
            topic_type: topic_type.to_owned(),
            tcp_nodelay: false,
            service: None,
            persistent: None,
        };
        trace!("Publisher connection header: {responding_conn_header:?}");

        // Setup a channel to signal to the publication to clean itself up
        let (shutdown_tx, shutdown_rx) = tokio::sync::mpsc::channel(1);
        let weak_shutdown_channel = shutdown_tx.downgrade();

        // Create the task that will accept new TCP connections
        let topic_name_copy = topic_name.to_owned();
        let tcp_accept_handle = tokio::spawn(async move {
            Self::tcp_accept_task(
                tcp_listener,
                topic_name_copy,
                responding_conn_header,
                receiver,
                shutdown_rx,
                node_handle,
            )
            .await
        });

        let sender_copy = sender.clone();
        Ok((
            Self {
                topic_type: topic_type.to_owned(),
                _tcp_accept_task: tcp_accept_handle.into(),
                listener_port,
                publish_sender: sender,
                weak_shutdown_channel,
            },
            sender_copy,
            shutdown_tx,
        ))
    }

    pub(crate) fn get_senders(
        &self,
    ) -> (
        broadcast::Sender<Vec<u8>>,
        tokio::sync::mpsc::WeakSender<()>,
    ) {
        (
            self.publish_sender.clone(),
            self.weak_shutdown_channel.clone(),
        )
    }

    pub(crate) fn port(&self) -> u16 {
        self.listener_port
    }

    pub(crate) fn topic_type(&self) -> &str {
        &self.topic_type
    }

    /// Wraps the functionality that the publish task will perform
    /// this task is spawned by new, and canceled when the Publication is dropped
    /// This task constantly pulls new messages from the main publish buffer and
    /// sends them to all of the TCP Streams that are connected to the topic.
    async fn publish_task(
        mut rx: broadcast::Receiver<Vec<u8>>, // Receives messages to publish from the main buffer of messages
        mut stream: tokio::net::TcpStream,
        topic: String,
        last_message: Option<Vec<u8>>, // If we're latching will contain a message to send right away
    ) {
        let peer = stream.peer_addr();
        debug!("Publish task has started for publication: {topic} connection to {peer:?}");

        if let Some(last_message) = last_message {
            let res = stream.write_all(&last_message).await;
            match res {
                Ok(_) => {}
                Err(e) => {
                    error!("Failed to send latch message to subscriber: {e:?}");
                }
            }
        }

        loop {
            match rx.recv().await {
                Ok(msg_to_publish) => {
                    trace!("Publish task got message to publish for topic: {topic}");
                    let send_result = stream.write_all(&msg_to_publish[..]).await;
                    match send_result {
                        Ok(_) => {
                            trace!("Publish task sent message to topic: {topic}");
                        }
                        Err(err) => {
                            // Shut down this TCP connection if we can't write a whole message
                            debug!("Failed to send data to subscriber: {err}, removing");
                            break;
                        }
                    }
                }
                Err(RecvError::Lagged(num)) => {
                    debug!("TCP for peer {peer:?} is lagging behind, {num} messages were skipped");
                    continue;
                }
                Err(RecvError::Closed) => {
                    debug!("No more senders for the publisher channel, ending task");
                    break;
                }
            }
        }
        debug!("Publish task has exited for publication: {topic} connection to {peer:?}");
    }

    /// Wraps the functionality that the tcp_accept task will perform
    /// This task is spawned by new, and canceled when the Publication is dropped
    /// This task constantly accepts new TCP connections and adds them to the list of streams to send data to.
    async fn tcp_accept_task(
        tcp_listener: tokio::net::TcpListener, // The TCP listener to accept connections on
        topic_name: String,                    // Only used for logging
        responding_conn_header: ConnectionHeader, // Header we respond with
        mut rx: broadcast::Receiver<Vec<u8>>, // Receives messages to publish from the main buffer of messages
        mut shutdown_rx: tokio::sync::mpsc::Receiver<()>, // Channel to signal to the publication to clean itself up
        nh: NodeServerHandle,
    ) {
        debug!("TCP accept task has started for publication: {topic_name}");
        let mut last_message = None;
        loop {
            let result = tokio::select! {
                shutdown = shutdown_rx.recv() => {
                    match shutdown {
                        Some(_) => error!("Message should never be sent on this channel"),
                        None => debug!("TCP accept task has received shutdown signal for publication: {topic_name}"),
                    }
                    // Notify our Node that we're shutting down
                    nh.unregister_publisher(&topic_name).await.unwrap();
                    // Exit our loop and shutdown this task
                    break;
                }
                result = tcp_listener.accept() => {
                    // Process the new TCP connection
                    result
                },
                // TODO this can be optimized
                // We shouldn't even call recv() if we're not latching
                msg = rx.recv() => {
                    match msg {
                        Ok(msg) =>{
                            // If we're latching save the message
                            if responding_conn_header.latching {
                              last_message = Some(msg);
                            }
                        },
                        Err(RecvError::Lagged(num)) => {
                            debug!("TCP accept task for {topic_name} is lagging behind, {num} messages were skipped");
                            continue;
                        }
                        Err(RecvError::Closed) => {
                            debug!("No more senders for the publisher channel, ending task");
                            break;
                        }
                    }
                    continue;
                }
            };

            let (mut stream, peer_addr) = match result {
                Ok(result) => result,
                Err(e) => {
                    error!("Error accepting TCP connection for topic {topic_name}: {e:?}");
                    continue;
                }
            };

            info!("Received connection from subscriber at {peer_addr} for topic {topic_name}");
            // Read the connection header:
            let connection_header = match tcpros::receive_header(&mut stream).await {
                Ok(header) => header,
                Err(e) => {
                    error!("Failed to read connection header: {e:?}");
                    stream
                        .shutdown()
                        .await
                        .expect("Unable to shutdown tcpstream");
                    continue;
                }
            };

            debug!(
                "Received subscribe request for {:?} with md5sum {:?}",
                connection_header.topic, connection_header.md5sum
            );
            // I can't find documentation for this anywhere, but when using
            // `rostopic hz` with one of our publishers I discovered that the rospy code sent "*" as the md5sum
            // To indicate a "generic subscription"...
            // I also discovered that `rostopic echo` does not send a md5sum (even thou ros documentation says its required)
            if let Some(connection_md5sum) = connection_header.md5sum {
                if connection_md5sum != "*" {
                    if let Some(local_md5sum) = &responding_conn_header.md5sum {
                        // TODO(lucasw) is it ok to match any with "*"?
                        // if local_md5sum != "*" && connection_md5sum != *local_md5sum {
                        if connection_md5sum != *local_md5sum {
                            warn!(
                                    "Got subscribe request for {}, but md5sums do not match. Expected {:?}, received {:?}",
                                    topic_name,
                                    local_md5sum,
                                    connection_md5sum,
                                    );
                            // Close the TCP connection
                            stream
                                .shutdown()
                                .await
                                .expect("Unable to shutdown tcpstream");
                            continue;
                        }
                    }
                }
            }
            // Write our own connection header in response
            let response_header_bytes = responding_conn_header
                .to_bytes(false)
                .expect("Couldn't serialize connection header");
            stream
                .write_all(&response_header_bytes[..])
                .await
                .expect("Unable to respond on tcpstream");

            // Create a new task to handle writing to the TCP stream
            // Note: we continue to hold on to a root "rx" in this accept task that means that we
            // always keep the channel open from the receive side.
            let rx_copy = rx.resubscribe();
            let topic_name_copy = topic_name.clone();
            let last_message_copy = last_message.clone();
            tokio::spawn(async move {
                Self::publish_task(rx_copy, stream, topic_name_copy, last_message_copy).await;
            });

            debug!(
                "Added stream for topic {:?} to subscriber {}",
                connection_header.topic, peer_addr
            );
        }
    }
}

impl Drop for Publication {
    fn drop(&mut self) {
        debug!("Dropping publication for topic {}", self.topic_type);
    }
}

#[derive(thiserror::Error, Debug)]
pub enum PublisherError {
    /// Serialize Error from `serde_rosmsg::Error` (stored as String because of dyn Error)
    #[error("serde_rosmsg Error: {0}")]
    SerializingError(String),
    #[error("connection closed, no further messages can be sent")]
    StreamClosed,
}

impl From<roslibrust_serde_rosmsg::Error> for PublisherError {
    fn from(value: roslibrust_serde_rosmsg::Error) -> Self {
        Self::SerializingError(value.to_string())
    }
}
