use crate::{Client, RosLibRustResult, RosMessageType};

/// A handle given to the caller when they advertise a topic
///
/// Publisher's have a single core function [publish](Publisher::publish) which provides the ability
/// to send message on the associated topic. Publishers automatically un-advertise the topic when
/// they are dropped. Currently, only one publisher is allowed to be created per topic, and
/// subsequent calls to advertise() on the same topic will fail.
///
/// Roadmap for Publisher:
///   - Support clone() / multiple advertise
///   - Ability for publish to by const
// Instead of giving back a publisher should we give back a reference to one, and give back the
// same reference when you advertise multiple times? Would require non-mut references?
#[derive(Clone)]
pub struct Publisher<T: RosMessageType> {
    topic: String,
    // auto incrementing sequence number increased once per publish
    // TODO have to somehow detect if message has header
    // We likely want to implement a Stamped trait and impl it automatically in message gen
    // if we detect a `Header header` in the .msg/.srv
    // For now, leaving seq off of Publisher until it is fully supported
    // #[allow(dead_code)]
    // seq: usize,
    // Stores a copy of the client so that we can de-register ourselves
    client: Client,
    _marker: std::marker::PhantomData<T>,
}

/// Publisher will un-advertise its topic automatically on drop
impl<T: RosMessageType> Drop for Publisher<T> {
    fn drop(&mut self) {
        self.client.unadvertise(&self.topic);
    }
}

impl<T: RosMessageType> Publisher<T> {
    pub(crate) fn new(topic: String, client: Client) -> Self {
        Publisher {
            topic,
            client,
            _marker: Default::default(),
        }
    }
    /// The "standard" publish function sends the message out, returns when publish succeeds
    ///
    /// The publish will be abandoned if the connection to the server is lost while in flight.
    /// Successful sending of the message does not guarantee successful receipt or re-transmission by
    /// rosbridge_server, rosbridge_server will fail to re-transmit if the type of the message does not
    /// match the topic's definition on roscore.
    pub async fn publish(&self, msg: T) -> RosLibRustResult<()> {
        self.client.publish(&self.topic, msg).await
    }
}
