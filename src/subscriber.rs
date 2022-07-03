// This file / module covers the return type from a subscribe call
// The subscriber manages the lifetime of the subscription and provides an
// API which allows us to hide our underlying queue/channel type

use std::sync::Arc;

use crate::{Client, MessageQueue, RosMessageType};

pub struct Subscriber<T: RosMessageType> {
    // Holds an internal copy of client to reference back to for dropping
    // TODO needs to use to auto-unsubscriber
    #[allow(dead_code)]
    client: Client,
    queue: Arc<MessageQueue<T>>,
}

impl<T: RosMessageType> Subscriber<T> {
    pub fn new(client: Client, queue: Arc<MessageQueue<T>>) -> Self {
        Subscriber { client, queue }
    }

    /// Returns the number of messages currently queued in the subscriber
    pub fn len(&self) -> usize {
        self.queue.len()
    }

    // aka pop(). Returns the oldest message in the internal message queue.
    // Blocks if queue is empty
    // Warning: failing to call next() fast enough can messages to accumulate in the internal queue,
    // this can cause latency to build-up and may not be desirable.
    pub async fn next(&self) -> T {
        self.queue.pop().await
    }

    // Returns the most recently recieved message, flushing all older message from the queue.
    // Blocks if queue is empty
    pub async fn most_recent(&self) -> T {
        while self.queue.len() > 1 {
            let _ = self.queue.try_pop();
        }
        self.queue.pop().await
    }
}
