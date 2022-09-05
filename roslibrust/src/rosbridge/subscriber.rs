// This file / module covers the return type from a subscribe call
// The subscriber manages the lifetime of the subscription and provides an
// API which allows us to hide our underlying queue/channel type

use log::error;
use std::sync::Arc;

use crate::{rosbridge::MessageQueue, Client, RosMessageType};

/// Represents a single instance of listening to a topic, and provides the ability to extract messages
///
/// A single topic can be subscribed to multiple times and each subscriber will get a unique message queue.
/// All subscribers will receive a copy of the incoming message anytime one is received.
/// When the last subscriber is dropped the topic is automatically un-subscribed to.
/// The internal message queue is currently fixed sized at 1_000 item maximum.
///
/// The internal message queue is internally mutex'ed meaning const access to this class is sufficient for use.
///
/// Roadmap:
///  - Expose queue size control
///  - Provide unlimited queue (maybe?)
///  - Provide automatic alerting mechanism on queue growth / fullness
pub struct Subscriber<T: RosMessageType> {
    // Randomly generated unique id of the subscriber used to track its lifetime with the client
    id: uuid::Uuid,
    // ROS topic name this is subscribed to, currently only used in Drop impl to help client
    topic: String,
    // Holds an internal copy of client to reference back to when being drop'ed
    client: Client,
    queue: Arc<MessageQueue<T>>,
}

impl<T: RosMessageType> Subscriber<T> {
    // External API is accessed through Client::subscribe
    // This function is just a convenience wrapper for our internal API
    pub(crate) fn new(client: Client, queue: Arc<MessageQueue<T>>, topic: String) -> Self {
        Subscriber {
            id: uuid::Uuid::new_v4(),
            topic,
            client,
            queue,
        }
    }

    /// Returns the number of messages currently queued in the subscriber
    pub fn len(&self) -> usize {
        self.queue.len()
    }

    /// aka pop(). Returns the oldest message in the internal message queue.
    ///
    /// Blocks if queue is empty
    ///
    /// Warning: failing to call next() fast enough can cause messages to accumulate in the internal queue,
    /// this can cause latency to build-up and may not be desirable.
    pub async fn next(&self) -> T {
        self.queue.pop().await
    }

    /// Returns the most recently received message, flushing all older message from the queue.
    /// Will ".await" if no message is available. This is the recommended method of subscribing and
    /// prevents the queue overfilling if processing messages too slowly.
    pub async fn most_recent(&self) -> T {
        while self.queue.len() > 1 {
            let _ = self.queue.try_pop();
        }
        self.queue.pop().await
    }

    // Used internally to track subscribers within the Client
    pub(crate) fn get_id(&self) -> &uuid::Uuid {
        &self.id
    }
}

/// Informs the client that the subscriber is being dropped so that
/// the client can track when the last subscriber for a topic is dropped
impl<T: RosMessageType> Drop for Subscriber<T> {
    fn drop(&mut self) {
        match self.client.unsubscribe(&self.topic, &self.id) {
            Ok(_) => {}
            Err(e) => {
                error!("Failed to unsubscribe while dropping subscriber: topic={:?}, id={:?}, err={:?}", &self.topic, &self.id, e);
            }
        }
    }
}
