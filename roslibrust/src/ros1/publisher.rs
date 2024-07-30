use crate::ros1::{
    names::Name,
    tcpros::{self, ConnectionHeader},
};
use abort_on_drop::ChildTask;
use roslibrust_codegen::RosMessageType;
use std::{
    marker::PhantomData,
    net::{Ipv4Addr, SocketAddr},
    sync::Arc,
};
use tokio::{
    io::AsyncWriteExt,
    sync::{mpsc, RwLock},
};

pub struct Publisher<T> {
    topic_name: String,
    sender: mpsc::Sender<Vec<u8>>,
    phantom: PhantomData<T>,
}

impl<T: RosMessageType> Publisher<T> {
    pub(crate) fn new(topic_name: &str, sender: mpsc::Sender<Vec<u8>>) -> Self {
        Self {
            topic_name: topic_name.to_owned(),
            sender,
            phantom: PhantomData,
        }
    }

    /// Queues a message to be send on the related topic.
    /// Returns when the data has been queued not when data is actually sent.
    pub async fn publish(&self, data: &T) -> Result<(), PublisherError> {
        let data = serde_rosmsg::to_vec(&data)?;
        // TODO this is a pretty dumb...
        // because of the internal channel used for re-direction this future doesn't
        // actually complete when the data is sent, but merely when it is queued to be sent
        // This function could probably be non-async
        // Or we should do some significant re-work to have it only yield when the data is sent.
        self.sender
            .send(data)
            .await
            .map_err(|_| PublisherError::StreamClosed)?;
        log::debug!("Publishing data on topic {}", self.topic_name);
        Ok(())
    }
}

pub struct Publication {
    topic_type: String,
    listener_port: u16,
    _channel_task: ChildTask<()>,
    _publish_task: ChildTask<()>,
    publish_sender: mpsc::Sender<Vec<u8>>,
}

impl Publication {
    pub async fn new(
        node_name: &Name,
        latching: bool,
        topic_name: &str,
        host_addr: Ipv4Addr,
        queue_size: usize,
        msg_definition: &str,
        md5sum: &str,
        topic_type: &str,
    ) -> Result<Self, std::io::Error> {
        let host_addr = SocketAddr::from((host_addr, 0));
        let tcp_listener = tokio::net::TcpListener::bind(host_addr).await?;
        let listener_port = tcp_listener.local_addr().unwrap().port();

        let (sender, mut receiver) = mpsc::channel::<Vec<u8>>(queue_size);

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
        log::trace!("Publisher connection header: {responding_conn_header:?}");

        let subscriber_streams = Arc::new(RwLock::new(Vec::new()));

        let subscriber_streams_copy = subscriber_streams.clone();
        let topic_name = topic_name.to_owned();
        let listener_handle = tokio::spawn(async move {
            let subscriber_streams = subscriber_streams_copy;
            loop {
                if let Ok((mut stream, peer_addr)) = tcp_listener.accept().await {
                    log::info!(
                        "Received connection from subscriber at {peer_addr} for topic {topic_name}"
                    );

                    // Read the connection header:
                    let connection_header = match tcpros::receive_header(&mut stream).await {
                        Ok(header) => header,
                        Err(e) => {
                            log::error!("Failed to read connection header: {e:?}");
                            stream
                                .shutdown()
                                .await
                                .expect("Unable to shutdown tcpstream");
                            continue;
                        }
                    };

                    log::debug!(
                        "Received subscribe request for {:?} with md5sum {:?}",
                        connection_header.topic,
                        connection_header.md5sum
                    );
                    // I can't find documentation for this anywhere, but when using
                    // `rostopic hz` with one of our publishers I discovered that the rospy code sent "*" as the md5sum
                    // To indicate a "generic subscription"...
                    // I also discovered that `rostopic echo` does not send a md5sum (even thou ros documentation says its required)
                    if let Some(connection_md5sum) = connection_header.md5sum {
                        if connection_md5sum != "*" {
                            if let Some(local_md5sum) = &responding_conn_header.md5sum {
                                if connection_md5sum != *local_md5sum {
                                    log::warn!(
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
                        .write(&response_header_bytes[..])
                        .await
                        .expect("Unable to respond on tcpstream");
                    let mut wlock = subscriber_streams.write().await;
                    wlock.push(stream);
                    log::debug!(
                        "Added stream for topic {:?} to subscriber {}",
                        connection_header.topic,
                        peer_addr
                    );
                }
            }
        });

        let publish_task = tokio::spawn(async move {
            loop {
                match receiver.recv().await {
                    Some(msg_to_publish) => {
                        let mut streams = subscriber_streams.write().await;
                        let mut streams_to_remove = vec![];
                        for (stream_idx, stream) in streams.iter_mut().enumerate() {
                            if let Err(err) = stream.write(&msg_to_publish[..]).await {
                                // TODO: A single failure between nodes that cross host boundaries is probably normal, should make this more robust perhaps
                                log::debug!("Failed to send data to subscriber: {err}, removing");
                                streams_to_remove.push(stream_idx);
                            }
                        }
                        // Subtract the removed count to account for shifting indices after each
                        // remove, only works if they're sorted which should be the case given how
                        // it's being populated (forward enumeration)
                        streams_to_remove.into_iter().enumerate().for_each(
                            |(removed_cnt, stream_idx)| {
                                streams.remove(stream_idx - removed_cnt);
                            },
                        );
                    }
                    None => {
                        log::debug!("No more senders for the publisher channel, exiting...");
                        break;
                    }
                }
            }
        });

        Ok(Self {
            topic_type: topic_type.to_owned(),
            _channel_task: listener_handle.into(),
            listener_port,
            publish_sender: sender,
            _publish_task: publish_task.into(),
        })
    }

    pub fn get_sender(&self) -> mpsc::Sender<Vec<u8>> {
        self.publish_sender.clone()
    }

    pub fn port(&self) -> u16 {
        self.listener_port
    }

    pub fn topic_type(&self) -> &str {
        &self.topic_type
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

impl From<serde_rosmsg::Error> for PublisherError {
    fn from(value: serde_rosmsg::Error) -> Self {
        Self::SerializingError(value.to_string())
    }
}
