//! This module holds all content for directly working with ROS1 natively

use roslibrust_common::*;

/// [master_client] module contains code for calling xmlrpc functions on the master
mod master_client;
pub use master_client::*;

mod names;

/// [node] module contains the central Node and NodeHandle APIs
mod node;
pub use node::*;

mod publisher;
pub use publisher::Publisher;
pub use publisher::PublisherAny;
mod service_client;
pub use service_client::ServiceClient;
mod subscriber;
pub use subscriber::Subscriber;
mod service_server;
pub use service_server::ServiceServer;
mod tcpros;

/// Provides a common type alias for type erased service server functions.
/// Internally we use this type to store collections of server functions.
pub(crate) type TypeErasedCallback = dyn Fn(Vec<u8>) -> Result<Vec<u8>, Box<dyn std::error::Error + Send + Sync>>
    + Send
    + Sync
    + 'static;

// Implement the generic roslibrust trait
impl TopicProvider for NodeHandle {
    type Publisher<T: RosMessageType> = crate::Publisher<T>;
    type Subscriber<T: RosMessageType> = crate::Subscriber<T>;

    async fn advertise<T: RosMessageType>(
        &self,
        topic: &str,
    ) -> RosLibRustResult<Self::Publisher<T>> {
        // TODO MAJOR: consider promoting queue size, making unlimited default
        self.advertise::<T>(topic.as_ref(), 10, false)
            .await
            .map_err(|e| e.into())
    }

    async fn subscribe<T: RosMessageType>(
        &self,
        topic: &str,
    ) -> RosLibRustResult<Self::Subscriber<T>> {
        // TODO MAJOR: consider promoting queue size, making unlimited default
        self.subscribe(topic, 10).await.map_err(|e| e.into())
    }
}

impl<T: RosServiceType> Service<T> for ServiceClient<T> {
    async fn call(&self, request: &T::Request) -> RosLibRustResult<T::Response> {
        self.call(request).await
    }
}

impl ServiceProvider for crate::NodeHandle {
    type ServiceClient<T: RosServiceType> = crate::ServiceClient<T>;
    type ServiceServer = crate::ServiceServer;

    async fn service_client<T: RosServiceType + 'static>(
        &self,
        topic: &str,
    ) -> RosLibRustResult<Self::ServiceClient<T>> {
        // TODO bad error mapping here...
        self.service_client::<T>(topic).await.map_err(|e| e.into())
    }

    async fn advertise_service<T: RosServiceType + 'static, F>(
        &self,
        topic: &str,
        server: F,
    ) -> RosLibRustResult<Self::ServiceServer>
    where
        F: ServiceFn<T>,
    {
        self.advertise_service::<T, F>(topic, server)
            .await
            .map_err(|e| e.into())
    }
}

impl<T: RosMessageType> Subscribe<T> for crate::Subscriber<T> {
    async fn next(&mut self) -> RosLibRustResult<T> {
        let res = crate::Subscriber::next(self).await;
        match res {
            Some(Ok(msg)) => Ok(msg),
            Some(Err(e)) => {
                log::error!("Subscriber got error: {e:?}");
                // TODO gotta do better error conversion / error types here
                Err(crate::RosLibRustError::Unexpected(anyhow::anyhow!(
                    "Subscriber got error: {e:?}"
                )))
            }
            None => {
                log::error!("Subscriber hit dropped channel");
                Err(crate::RosLibRustError::Unexpected(anyhow::anyhow!(
                    "Channel closed, something was dropped?"
                )))
            }
        }
    }
}

// Provide an implementation of publish for ros1 backend
impl<T: RosMessageType> Publish<T> for Publisher<T> {
    async fn publish(&self, data: &T) -> RosLibRustResult<()> {
        // TODO error type conversion here is terrible and we need to standardize error stuff badly
        self.publish(data)
            .await
            .map_err(|e| RosLibRustError::SerializationError(e.to_string()))
    }
}

#[cfg(test)]
mod test {
    use roslibrust_common::*;

    // Prove that we've implemented the topic provider trait fully for NodeHandle
    #[test]
    #[should_panic]
    fn topic_provider_can_be_used_with_ros1() {
        struct MyClient<T: TopicProvider> {
            _client: T,
        }

        // Kinda a hack way to make the compiler prove it could construct a MyClient<NodeHandle> with out actually
        // constructing one at runtime
        let new_mock: Result<crate::NodeHandle, _> = Err(anyhow::anyhow!("Expected error"));

        let _x = MyClient {
            _client: new_mock.unwrap(), // panic
        };
    }
}
