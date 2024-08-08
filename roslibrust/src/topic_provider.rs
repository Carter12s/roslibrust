use roslibrust_codegen::{RosMessageType, RosServiceType};

use crate::{RosLibRustResult, ServiceFn};

// Indicates that something is a publisher and has our expected publish
// Implementors of this trait are expected to auto-cleanup the publisher when dropped
pub trait Publish<T: RosMessageType> {
    // Note: this is really just syntactic de-sugared `async fn`
    // However see: https://blog.rust-lang.org/2023/12/21/async-fn-rpit-in-traits.html
    // This generates a warning is rust as of writing due to ambiguity around the "Send-ness" of the return type
    // We only plan to work with multi-threaded work stealing executors (e.g. tokio) so we're manually specifying Send
    fn publish(&self, data: &T) -> impl futures::Future<Output = RosLibRustResult<()>> + Send;
}

impl<T: RosMessageType> Publish<T> for crate::Publisher<T> {
    async fn publish(&self, data: &T) -> RosLibRustResult<()> {
        self.publish(data).await
    }
}

impl<T: RosMessageType> Publish<T> for crate::ros1::Publisher<T> {
    async fn publish(&self, data: &T) -> RosLibRustResult<()> {
        // TODO error type conversion here is terrible and we need to standardize error stuff badly
        self.publish(data)
            .await
            .map_err(|e| crate::RosLibRustError::SerializationError(e.to_string()))
    }
}

/// This trait generically describes the capability of something to act as an async interface to a set of topics
///
/// This trait is largely based on ROS concepts, but could be extended to other protocols / concepts.
/// Fundamentally, it assumes that topics are uniquely identified by a string name (likely an ASCII assumption is buried in here...).
/// It assumes topics only carry one data type, but is not expected to enforce that.
/// It assumes that all actions can fail due to a variety of causes, and by network interruption specifically.
// #[async_trait]
pub trait TopicProvider {
    // These associated types makeup the other half of the API
    // They are expected to be "self-deregistering", where dropping them results in unadvertise or unsubscribe operations as appropriate
    type Publisher<T: RosMessageType>: Publish<T>;
    type Subscriber<T: RosMessageType>;
    type ServiceHandle;

    /// Advertises a topic to be published to and returns a type specific publisher to use.
    ///
    /// The returned publisher is expected to be "self-deregistering", where dropping the publisher results in the appropriate unadvertise operation.
    fn advertise<T: RosMessageType>(
        &self,
        topic: &str,
    ) -> impl futures::Future<Output = RosLibRustResult<Self::Publisher<T>>> + Send;

    /// Subscribes to a topic and returns a type specific subscriber to use.
    ///
    /// The returned subscriber is expected to be "self-deregistering", where dropping the subscriber results in the appropriate unsubscribe operation.
    fn subscribe<T: RosMessageType>(
        &self,
        topic: &str,
    ) -> impl futures::Future<Output = RosLibRustResult<Self::Subscriber<T>>> + Send;

    fn call_service<S: RosServiceType>(
        &self,
        topic: &str,
        request: S::Request,
    ) -> impl std::future::Future<Output = RosLibRustResult<S::Response>> + Send;

    fn advertise_service<T: RosServiceType, F>(
        &self,
        topic: &str,
        server: F,
    ) -> impl futures::Future<Output = RosLibRustResult<Self::ServiceHandle>> + Send
    where
        F: ServiceFn<T>;
}

// Implementation of TopicProvider trait for rosbridge client
impl TopicProvider for crate::ClientHandle {
    type Publisher<T: RosMessageType> = crate::Publisher<T>;
    type Subscriber<T: RosMessageType> = crate::Subscriber<T>;
    type ServiceHandle = crate::ServiceHandle;

    async fn advertise<T: RosMessageType>(
        &self,
        topic: &str,
    ) -> RosLibRustResult<Self::Publisher<T>> {
        self.advertise::<T>(topic.as_ref()).await
    }

    async fn subscribe<T: RosMessageType>(
        &self,
        topic: &str,
    ) -> RosLibRustResult<Self::Subscriber<T>> {
        self.subscribe(topic).await
    }

    async fn call_service<S: RosServiceType>(
        &self,
        topic: &str,
        request: S::Request,
    ) -> RosLibRustResult<S::Response> {
        self.call_service::<S>(topic, request).await
    }

    async fn advertise_service<T: RosServiceType, F>(
        &self,
        topic: &str,
        server: F,
    ) -> RosLibRustResult<Self::ServiceHandle>
    where
        F: ServiceFn<T>,
    {
        self.advertise_service::<T, F>(topic, server).await
    }
}

#[cfg(feature = "ros1")]
impl TopicProvider for crate::ros1::NodeHandle {
    type Publisher<T: RosMessageType> = crate::ros1::Publisher<T>;
    type Subscriber<T: RosMessageType> = crate::ros1::Subscriber<T>;
    type ServiceHandle = crate::ros1::ServiceServer;

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

    async fn call_service<S: RosServiceType + Send>(
        &self,
        topic: &str,
        request: S::Request,
    ) -> RosLibRustResult<S::Response> {
        let client = self.service_client::<S>(topic).await?;
        client.call(&request).await
    }

    async fn advertise_service<T: RosServiceType, F>(
        &self,
        topic: &str,
        server: F,
    ) -> RosLibRustResult<Self::ServiceHandle>
    where
        F: ServiceFn<T>,
    {
        self.advertise_service::<T, F>(topic, server)
            .await
            .map_err(|e| e.into())
    }
}

#[cfg(test)]
mod test {
    use super::TopicProvider;
    use crate::{ros1::NodeHandle, ClientHandle};

    // This test specifically fails because TopicProvider is not object safe
    // Traits that have methods with generic parameters cannot be object safe in rust (currently)
    // #[test]
    // fn topic_provider_can_be_constructed() -> TestResult {
    //     let x: Box<dyn TopicProvider> = Box::new(ClientHandle::new(""));
    //     Ok(())
    // }

    // This tests proves that you could use topic provider in a compile time api, but not object safe...
    #[test_log::test]
    #[should_panic]
    fn topic_provider_can_be_used_at_compile_time() {
        struct MyClient<T: TopicProvider> {
            _client: T,
        }

        // Kinda a hack way to make the compiler prove it could construct a MyClient<ClientHandle> with out actually
        // constructing one at runtime
        let new_mock: Result<ClientHandle, _> = Err(anyhow::anyhow!("Expected error"));

        let _x = MyClient {
            _client: new_mock.unwrap(), // panic
        };
    }

    #[test_log::test]
    #[should_panic]
    fn topic_provider_can_be_used_with_ros1() {
        struct MyClient<T: TopicProvider> {
            _client: T,
        }

        // Kinda a hack way to make the compiler prove it could construct a MyClient<ClientHandle> with out actually
        // constructing one at runtime
        let new_mock: Result<NodeHandle, _> = Err(anyhow::anyhow!("Expected error"));

        let _x = MyClient {
            _client: new_mock.unwrap(), // panic
        };
    }
}
