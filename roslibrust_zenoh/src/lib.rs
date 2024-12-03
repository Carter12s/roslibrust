//! A crate for interfacing to ROS1 via the [zenoh-ros1-plugin / zenoh-ros1-bridge](https://github.com/eclipse-zenoh/zenoh-plugin-ros1).

use roslibrust::topic_provider::{Publish, Service, ServiceProvider, Subscribe, TopicProvider};
use roslibrust::{RosLibRustError, RosLibRustResult};
use roslibrust_codegen::{RosMessageType, RosServiceType};

use log::*;
use tokio::select;
use zenoh::bytes::ZBytes;

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

/// Identical to mangle_topic, but for services we want to separate the datatype stuff from the service name
fn mangle_service(service: &str, type_str: &str, md5sum: &str) -> (String, String) {
    let service = service.trim_start_matches('/').trim_end_matches("/");

    let type_str = hex::encode(type_str.as_bytes());
    (
        format!("{type_str}/{md5sum}"),
        format!("{service}").to_string(),
    )
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

/// The "holder type" returned when advertising a service
/// Dropping this will stop the service server
pub struct ZenohServiceServer {
    // Dropping this will stop zenoh's declaration of the queryable
    _queryable: zenoh::query::Queryable<()>,
    // Dropping this will stop the advertising of the service
    _shutdown_channel: tokio::sync::oneshot::Sender<()>,
}

impl ServiceProvider for ZenohClient {
    type ServiceClient<T: RosServiceType> = ZenohServiceClient<T>;
    type ServiceServer = ZenohServiceServer;

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
        let mangled_topic = mangle_topic(topic, T::ROS_SERVICE_NAME, T::MD5SUM);

        let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
        let (shutdown_tx, mut shutdown_rx) = tokio::sync::oneshot::channel();

        let x = self
            .session
            .declare_queryable(mangled_topic)
            .callback(move |query| {
                let _ = tx.send(query).map_err(|e| {
                    error!("Failed to send query: {e:?}");
                });
            })
            .await
            .map_err(|e| {
                RosLibRustError::Unexpected(anyhow::anyhow!("Failed to declare queryable: {e:?}"))
            })?;

        // Spawn a task to handle the queries
        // This task will shut down when queryable is dropped
        tokio::spawn(async move {
            while let Some(query) = rx.recv().await {
                debug!("Got query: {query:?}");
                let Some(payload) = query.payload() else {
                    error!("Received a query with no payload for a ros0 service {query:?}");
                    continue;
                };
                let bytes = payload.to_bytes();
                debug!("Got bytes: {bytes:?}");
                // TODO MAJOR HACK HERE STILL
                // Our deserialization still expects the first four bytes to be the total message size
                // So we're just going to manually add the bytes back in
                let starting_bytes = (bytes.len() as u32).to_le_bytes();
                let bytes = [&starting_bytes, &bytes[..]].concat();

                let Ok(request) = roslibrust_serde_rosmsg::from_slice(&bytes).map_err(|e| {
                    error!("Failed to deserialize request: {e:?}");
                }) else {
                    continue;
                };

                let Ok(response) = server(request).map_err(|e| {
                    error!("Failed to handle request: {e:?}");
                }) else {
                    continue;
                };

                let Ok(response_bytes) = roslibrust_serde_rosmsg::to_vec(&response).map_err(|e| {
                    error!("Failed to serialize response: {e:?}");
                }) else {
                    continue;
                };

                // TODO HACK HERE STILL
                // Zenoh doesn't want the first four bytes that are the overall message size:
                let response_bytes = &response_bytes[4..];

                let _ = query
                    .reply(query.key_expr(), response_bytes)
                    .await
                    .map_err(|e| {
                        error!("Failed to reply to query: {e:?}");
                    });
            }
        });
        // zenoh-ros1-bridge won't serve our service without us publishing info on 'ros1_discovery_info'
        // We don't have to worry about this for publishers, because zenoh will initiate the bridge whenever someone subscribes on the ros side.
        // For service, zenoh-ros1-bridge has to create the service before anyone can call it so it has to know that it needs to do that.
        // This is a bit of a brittle implementation as it relies on internal implementation details for zenoh-ros1-bridge, but it works for now.
        // See: https://github.com/eclipse-zenoh/zenoh-plugin-ros1/blob/main/zenoh-plugin-ros1/src/ros_to_zenoh_bridge/discovery.rs

        // Note: I'm uncertain about "discovery_namespace" and just using * for now
        // Note: I'm uncertain about "bridge_namespace" and just using * for now
        let (type_mangle, service_name) = mangle_service(topic, T::ROS_SERVICE_NAME, T::MD5SUM);
        let zenoh_info_topic = format!("ros1_discovery_info/*/srv/{type_mangle}/*/{service_name}");

        let q2 = self
            .session
            .declare_publisher(zenoh_info_topic)
            .await
            .map_err(|e| {
                RosLibRustError::Unexpected(anyhow::anyhow!(
                    "Failed to declare queryable for service discovery: {e:?}"
                ))
            })?;
        tokio::spawn(async move {
            let mut interval = tokio::time::interval(std::time::Duration::from_secs(1));
            loop {
                let shutdown = shutdown_rx.try_recv();
                match shutdown {
                    Ok(_) => {
                        break;
                    }
                    Err(tokio::sync::oneshot::error::TryRecvError::Empty) => {
                        // Continue no shutdown yet
                    }
                    Err(tokio::sync::oneshot::error::TryRecvError::Closed) => {
                        break;
                    }
                }
                // Send an empty message to the discovery topic
                let res = q2.put(ZBytes::default()).await;
                if let Err(e) = res {
                    error!("Failed to publish service discovery info: {e:?}");
                }
                interval.tick().await;
            }
        });

        Ok(ZenohServiceServer {
            _queryable: x,
            _shutdown_channel: shutdown_tx,
        })
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
