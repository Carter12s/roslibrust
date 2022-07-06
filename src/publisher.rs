use crate::{Client, RosLibRustResult, RosMessageType};

/// The type given to someone when they advertise a topic
// TODO do we need to if multiple publishers to same topic are created and only unadvertise
// when all are dropped?
// Instead of giving back a publisher should we give back a reference to one, and give back the
// same reference when you advertise multiple times? Would require non-mut references?
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

/// Publisher unadervtises its topic automatically on drop
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
    pub async fn publish(&mut self, msg: T) -> RosLibRustResult<()> {
        self.client.publish(&self.topic, msg).await
    }
}
