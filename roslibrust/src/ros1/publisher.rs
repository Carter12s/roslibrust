use crate::ros1::{
    names::Name,
    node::actor::NodeServerHandle,
    tcpros::{self, ConnectionHeader},
};
use abort_on_drop::ChildTask;
use log::*;
use roslibrust_codegen::RosMessageType;
use std::{
    marker::PhantomData,
    net::{Ipv4Addr, SocketAddr},
    sync::Arc,
};
use tokio::{
    io::AsyncWriteExt,
    sync::{
        broadcast::{self, error::RecvError},
        RwLock,
    },
};

/// The regular Publisher representation returned by calling advertise on a [crate::ros1::NodeHandle].
pub struct Publisher<T> {
    topic_name: String,
    sender: broadcast::Sender<Vec<u8>>,
    phantom: PhantomData<T>,
}

impl<T: RosMessageType> Publisher<T> {
    pub(crate) fn new(topic_name: &str, sender: broadcast::Sender<Vec<u8>>) -> Self {
        Self {
            topic_name: topic_name.to_owned(),
            sender,
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
    phantom: PhantomData<Vec<u8>>,
}

impl PublisherAny {
    pub(crate) fn new(topic_name: &str, sender: broadcast::Sender<Vec<u8>>) -> Self {
        Self {
            topic_name: topic_name.to_owned(),
            sender,
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
    // TODO: Need to make sure this isn't keeping things alive that it shouldn't
    publish_sender: broadcast::Sender<Vec<u8>>,
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
    ) -> Result<(Self, broadcast::Sender<Vec<u8>>), std::io::Error> {
        // Get a socket for receiving connections on
        let host_addr = SocketAddr::from((host_addr, 0));
        let tcp_listener = tokio::net::TcpListener::bind(host_addr).await?;
        let listener_port = tcp_listener.local_addr().unwrap().port();

        // Setup the channel will will receive messages to be published on
        let (sender, receiver) = broadcast::channel::<Vec<u8>>(queue_size);

        // Setup the ROS connection header that we'll respond to all incomming connections with
        let responding_conn_header = ConnectionHeader {
            caller_id: node_name.to_string(),
            latching,
            msg_definition: msg_definition.to_owned(),
            md5sum: Some(md5sum.to_owned()),
            topic: Some(topic_name.to_owned()),
            topic_type: topic_type.to_owned(),
            tcp_nodelay: false,
            service: None,
        };
        trace!("Publisher connection header: {responding_conn_header:?}");

        // Setup storage for internal list of TCP streams
        let subscriber_streams = Arc::new(RwLock::new(Vec::new()));

        // Setup storage for the last message published (used for latching)
        let last_message = Arc::new(RwLock::new(None));

        // Create the task that will accept new TCP connections
        let subscriber_streams_copy = subscriber_streams.clone();
        let last_message_copy = last_message.clone();
        let topic_name_copy = topic_name.to_owned();
        let tcp_accept_handle = tokio::spawn(async move {
            Self::tcp_accept_task(
                tcp_listener,
                subscriber_streams_copy,
                topic_name_copy,
                responding_conn_header,
                last_message_copy,
                receiver,
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
            },
            sender_copy,
        ))
    }

    pub(crate) fn get_sender(&self) -> broadcast::Sender<Vec<u8>> {
        self.publish_sender.clone()
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
    ) {
        let peer = stream.peer_addr();
        debug!("Publish task has started for publication: {topic} connection to {peer:?}");
        loop {
            match rx.recv().await {
                Ok(msg_to_publish) => {
                    trace!("Publish task got message to publish for topic: {topic}");
                    // Proxy the message to the watch channel
                    if let Err(err) = stream.write_all(&msg_to_publish[..]).await {
                        // TODO: A single failure between nodes that cross host boundaries is probably normal, should make this more robust perhaps
                        debug!("Failed to send data to subscriber: {err}, removing");
                    }
                }
                Err(RecvError::Lagged(num)) => {
                    debug!("TCP for peer {peer:?} is lagging behind, {num} messages were skipped");
                    continue;
                }
                Err(RecvError::Closed) => {
                    debug!(
                        "No more senders for the publisher channel, triggering publication cleanup"
                    );
                    // TODO SHIT SHOW HERE
                    // broadcast stuffs breaks our cleanup plan

                    // All senders dropped, so we want to cleanup the Publication off of the node
                    // Tell the node server to dispose of this publication and unadvertise it
                    // Note: we need to do this in a spawned task or a drop-loop race condition will occur
                    // Dropping publication results in this task being dropped, which can end up canceling the future that is doing the dropping
                    // if we simply .await here
                    // TODO: This allows publisher to clean themselves up iff node remains running after publisher is dropped...
                    // NodeHandle clean-up is not resulting in a good state clean-up currently..
                    // let nh_copy = node_handle.clone();
                    // let topic = topic.clone();
                    // tokio::spawn(async move {
                    //     let _ = nh_copy.unregister_publisher(&topic).await;
                    // });
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
        subscriber_streams: Arc<RwLock<Vec<tokio::net::TcpStream>>>, // Where accepted streams are stored
        topic_name: String,                                          // Only used for logging
        responding_conn_header: ConnectionHeader,                    // Header we respond with
        last_message: Arc<RwLock<Option<Vec<u8>>>>, // Last message published (used for latching)
        rx: broadcast::Receiver<Vec<u8>>, // Receives messages to publish from the main buffer of messages
    ) {
        debug!("TCP accept task has started for publication: {topic_name}");
        loop {
            if let Ok((mut stream, peer_addr)) = tcp_listener.accept().await {
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

                // If we're configured to latch, send the last message to the new subscriber
                if responding_conn_header.latching {
                    if let Some(last_message) = last_message.read().await.as_ref() {
                        debug!(
                            "Publication configured to be latching and has last_message, sending"
                        );
                        // TODO likely a bug here... but pretty subtle
                        // If we disconnect here, out accept task could get blocked trying to write this message out
                        // until the TCP Socket errors. Resulting in this publisher being unable to connect to new
                        // subscribers for some number of seconds.
                        // This write_all should be moved into a separate task
                        let res = stream.write_all(last_message).await;
                        match res {
                            Ok(_) => {}
                            Err(e) => {
                                error!("Failed to send latch message to subscriber: {e:?}");
                                // Note doing any handling here, TCP stream will be cleaned up during
                                // next regular publish in the publish task
                            }
                        }
                    }
                }

                // let mut wlock = subscriber_streams.write().await;
                // wlock.push(stream);

                // Create a new task to handle writing to the TCP stream
                let rx_copy = rx.resubscribe();
                let topic_name_copy = topic_name.clone();
                tokio::spawn(async move {
                    Self::publish_task(rx_copy, stream, topic_name_copy).await;
                });

                debug!(
                    "Added stream for topic {:?} to subscriber {}",
                    connection_header.topic, peer_addr
                );
            }
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
