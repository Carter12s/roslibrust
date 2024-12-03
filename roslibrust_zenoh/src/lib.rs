//! A crate for interfacing to ROS1 via the [zenoh-ros1-plugin / zenoh-ros1-bridge](https://github.com/eclipse-zenoh/zenoh-plugin-ros1).

use roslibrust::topic_provider::{Publish, Service, ServiceProvider, Subscribe, TopicProvider};
use roslibrust::{RosLibRustError, RosLibRustResult};
use roslibrust_codegen::{RosMessageType, RosServiceType};

use log::*;

pub struct ZenohClient {
    session: zenoh::Session,
}

impl ZenohClient {
    /// Creates a new client wrapped around a Zenoh session
    pub fn new(session: zenoh::Session) -> Self {
        Self { session }
    }
}

pub struct ZenohPublisher<T> {
    publisher: zenoh::pubsub::Publisher<'static>,
    _marker: std::marker::PhantomData<T>,
}

impl<T: RosMessageType> Publish<T> for ZenohPublisher<T> {
    async fn publish(&self, data: &T) -> RosLibRustResult<()> {
        let bytes = roslibrust_serde_rosmsg::to_vec(data).map_err(|e| {
            RosLibRustError::SerializationError(format!("Failed to serialize message: {e:?}"))
        })?;

        // Note: serde_rosmsg places the length of the message as the first four bytes
        // Which zenoh ros1 bridge does not expect, so we need to strip it off.
        match self.publisher.put(&bytes[4..]).await {
            Ok(()) => Ok(()),
            Err(e) => Err(RosLibRustError::Unexpected(anyhow::anyhow!(
                "Failed to publish message to zenoh: {e:?}"
            ))),
        }
    }
}

// Using type alias here, I have no idea why zenoh has this type so deep
type ZenohSubInner =
    zenoh::pubsub::Subscriber<zenoh::handlers::FifoChannelHandler<zenoh::sample::Sample>>;
pub struct ZenohSubscriber<T> {
    subscriber: ZenohSubInner,
    _marker: std::marker::PhantomData<T>,
}

impl<T: RosMessageType> Subscribe<T> for ZenohSubscriber<T> {
    async fn next(&mut self) -> RosLibRustResult<T> {
        let next = self.subscriber.recv_async().await;

        let sample = match next {
            Ok(sample) => sample,
            Err(e) => {
                // TODO errors still suck with this API
                return Err(RosLibRustError::Unexpected(anyhow::anyhow!(
                    "Failed to receive next sample: {e:?}"
                )));
            }
        };

        let bytes = sample.payload().to_bytes();

        // This is messy roslibrust expects the starting bytes to be total message size
        // which Zenoh or the bridge is stripping somewhere, so I'm just manually sticking them back for now
        // This is very inefficient, but it works for now.
        let starting_bytes = (bytes.len() as u32).to_le_bytes();
        let bytes = [&starting_bytes, &bytes[..]].concat();

        let msg = roslibrust_serde_rosmsg::from_slice(&bytes).map_err(|e| {
            RosLibRustError::SerializationError(format!("Failed to deserialize sample: {e:?}"))
        })?;
        Ok(msg)
    }
}

impl TopicProvider for ZenohClient {
    type Publisher<T: RosMessageType> = ZenohPublisher<T>;

    type Subscriber<T: RosMessageType> = ZenohSubscriber<T>;

    async fn advertise<T: RosMessageType>(
        &self,
        topic: &str,
    ) -> RosLibRustResult<Self::Publisher<T>> {
        let mangled_topic = mangle_topic(topic, T::ROS_TYPE_NAME, T::MD5SUM);
        let publisher = match self.session.declare_publisher(mangled_topic).await {
            Ok(publisher) => publisher,
            Err(e) => {
                // TODO errors still suck with this API...
                return Err(RosLibRustError::Unexpected(anyhow::anyhow!(
                    "Failed to declare publisher: {e:?}"
                )));
            }
        };

        Ok(ZenohPublisher {
            publisher,
            _marker: std::marker::PhantomData,
        })
    }

    async fn subscribe<T: RosMessageType>(
        &self,
        topic: &str,
    ) -> RosLibRustResult<Self::Subscriber<T>> {
        let mangled_topic = mangle_topic(topic, T::ROS_TYPE_NAME, T::MD5SUM);
        let sub = match self.session.declare_subscriber(mangled_topic).await {
            Ok(sub) => sub,
            Err(e) => {
                // TODO errors still suck with this API...
                return Err(RosLibRustError::Unexpected(anyhow::anyhow!(
                    "Failed to declare subscriber: {e:?}"
                )));
            }
        };
        Ok(ZenohSubscriber {
            subscriber: sub,
            _marker: std::marker::PhantomData,
        })
    }
}

/// Takes in a regular ros topic and type and returns a zenoh topic mangled in the way the zenoh-ros1-plugin does
fn mangle_topic(topic: &str, type_str: &str, md5sum: &str) -> String {
    // Name mangling stuff!
    // See: https://github.com/eclipse-zenoh/zenoh-plugin-ros1/issues/131
    // Explicit implementation at: https://github.com/eclipse-zenoh/zenoh-plugin-ros1/blob/main/zenoh-plugin-ros1/src/ros_to_zenoh_bridge/topic_utilities.rs
    // Note: the implementation inside of the bridge uses unstable zenoh, duplicating implementation here with stable zenoh instead.

    // Remove leading and trailing slashes in the topic
    let topic = topic.trim_start_matches('/').trim_end_matches("/");
    // Encode the type as hex
    let type_str = hex::encode(type_str.as_bytes());
    format!("{type_str}/{md5sum}/{topic}")
}

pub struct ZenohServiceClient<T: RosServiceType> {
    session: zenoh::Session,
    zenoh_query: String,
    _marker: std::marker::PhantomData<T>,
}

impl<T: RosServiceType> Service<T> for ZenohServiceClient<T> {
    async fn call(&self, request: &T::Request) -> RosLibRustResult<T::Response> {
        let request_bytes = roslibrust_serde_rosmsg::to_vec(request).map_err(|e| {
            RosLibRustError::SerializationError(format!("Failed to serialize message: {e:?}"))
        })?;
        debug!("request bytes: {request_bytes:?}");

        let query = match self
            .session
            .get(&self.zenoh_query)
            .payload(&request_bytes[4..])
            // .timeout(tokio::time::Duration::from_secs(1))
            .await
        {
            Ok(query) => query,
            Err(e) => {
                // TODO errors still suck with this API...
                return Err(RosLibRustError::Unexpected(anyhow::anyhow!(
                    "Failed to create query for service: {e:?}"
                )));
            }
        };

        let response = match query.recv_async().await {
            Ok(data) => data,
            Err(e) => {
                return Err(RosLibRustError::Unexpected(anyhow::anyhow!(
                    "Failed to receive response from service: {e:?}"
                )));
            }
        };

        // TODO unclear why this is double failable in the API
        let sample = match response.into_result() {
            Ok(bytes) => bytes,
            Err(e) => {
                return Err(RosLibRustError::Unexpected(anyhow::anyhow!(
                    "Failed to receive sample from service: {e:?}"
                )));
            }
        };

        let bytes = sample.payload().to_bytes();

        // This is messy roslibrust expects the starting bytes to be total message size
        // which Zenoh or the bridge is stripping somewhere, so I'm just manually sticking them back for now
        // This is very inefficient, but it works for now.
        let starting_bytes = (bytes.len() as u32).to_le_bytes();
        let bytes = [&starting_bytes, &bytes[..]].concat();

        let msg = roslibrust_serde_rosmsg::from_slice(&bytes).map_err(|e| {
            RosLibRustError::SerializationError(format!("Failed to deserialize sample: {e:?}"))
        })?;
        Ok(msg)
    }
}

impl ServiceProvider for ZenohClient {
    type ServiceClient<T: RosServiceType> = ZenohServiceClient<T>;
    type ServiceServer = ();

    async fn service_client<T: roslibrust_codegen::RosServiceType + 'static>(
        &self,
        topic: &str,
    ) -> RosLibRustResult<Self::ServiceClient<T>> {
        let mangled_topic = mangle_topic(topic, T::ROS_SERVICE_NAME, T::MD5SUM);

        Ok(ZenohServiceClient {
            session: self.session.clone(),
            zenoh_query: mangled_topic,
            _marker: std::marker::PhantomData,
        })
    }

    async fn advertise_service<
        T: roslibrust_codegen::RosServiceType + 'static,
        F: roslibrust::ServiceFn<T>,
    >(
        &self,
        topic: &str,
        server: F,
    ) -> RosLibRustResult<Self::ServiceServer> {
        todo!()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mangle_topic() {
        assert_eq!(
            mangle_topic(
                "/chatter",
                "std_msgs/String",
                "992ce8a1687cec8c8bd883ec73ca41d1"
            ),
            "7374645f6d7367732f537472696e67/992ce8a1687cec8c8bd883ec73ca41d1/chatter"
        );
    }

    #[test]
    fn test_mangle_service() {
        assert_eq!(
            mangle_topic(
                "/service_server_rs/my_set_bool",
                "std_srvs/SetBool",
                "09fb03525b03e7ea1fd3992bafd87e16"
            ),
            "7374645f737276732f536574426f6f6c/09fb03525b03e7ea1fd3992bafd87e16/service_server_rs/my_set_bool");
    }
}
