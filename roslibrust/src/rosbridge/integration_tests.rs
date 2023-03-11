/// Tests in this module require a running rosbridge server supporting websocket connections on port 9090
/// run `docker-compose -f ros1_bridge_compose.yaml up` in the docker folder of this repo to start a bridge
/// and then run `cargo test --features running bridge` to execute these tests
/// NOTE: these aren't stored in "tests" dir as many tests need access to private functions like publish()
#[cfg(test)]
#[cfg(feature = "running_bridge")]
mod integration_tests {
    // Note sure exactly where this check should live, but here works
    #[cfg(all(feature = "ros1_test", feature = "ros2_test"))]
    compile_error!("Cannot build with both ros1_test and ros2_test enabled at the same time");

    use std::sync::Arc;

    use crate::{
        rosbridge::TestResult, ClientHandle, ClientHandleOptions, RosLibRustError, Subscriber,
    };
    use log::debug;
    use tokio::time::{timeout, Duration};
    // On my laptop test was ~90% reliable at 10ms
    // Had 1 spurious github failure at 100
    const TIMEOUT: Duration = Duration::from_millis(200);
    const LOCAL_WS: &str = "ws://localhost:9090";

    #[cfg(feature = "ros1_test")]
    roslibrust_codegen_macro::find_and_generate_ros_messages!(
        "assets/ros1_common_interfaces/ros_comm_msgs",
        "assets/ros1_common_interfaces/std_msgs",
        // "assets/test_msgs" // Note: we can't use these message in integration tests since they aren't installed inside our docker image (yet!)
    );

    #[cfg(feature = "ros2_test")]
    roslibrust_codegen_macro::find_and_generate_ros_messages!(
        "assets/ros2_common_interfaces/std_msgs",
        "assets/ros2_common_interfaces/std_srvs"
    );
    // This replaces the fact that Time.msg is no longer in std_msgs in ROS2
    #[cfg(feature = "ros2_test")]
    use roslibrust_codegen::integral_types::Time;

    use std_msgs::*;
    // Warning is here because a test is being skipped in ros2_test
    // this can be removed when we get self_service_call working
    #[allow(unused_imports)]
    use std_srvs::*;

    /**
    This test does a round trip publish subscribe for real
    Requires a running local rosbridge
    TODO figure out how to automate setting up the needed environment for this
    */
    #[tokio::test]
    async fn self_publish() {
        // TODO figure out better logging for tests
        let _ = simple_logger::SimpleLogger::new()
            .with_level(log::LevelFilter::Debug)
            .without_timestamps()
            .init();

        const TOPIC: &str = "self_publish";
        // 100ms allowance for connecting so tests still fails
        let mut client = timeout(TIMEOUT, ClientHandle::new(LOCAL_WS))
            .await
            .expect("Failed to create client in time")
            .unwrap();

        timeout(TIMEOUT, client.advertise::<Header>(TOPIC))
            .await
            .expect("Failed to advertise in time")
            .unwrap();
        let rx = timeout(TIMEOUT, client.subscribe::<Header>(TOPIC))
            .await
            .expect("Failed to subscribe in time")
            .unwrap();

        // Delay here to allow subscribe to complete before publishing
        // Test is flaky without it
        tokio::time::sleep(TIMEOUT).await;

        #[cfg(feature = "ros1_test")]
        let msg_out = Header {
            seq: 666,
            stamp: Default::default(),
            frame_id: "self_publish".to_string(),
        };

        #[cfg(feature = "ros2_test")]
        let msg_out = Header {
            stamp: Default::default(),
            frame_id: "self_publish".to_string(),
        };

        timeout(TIMEOUT, client.publish(TOPIC, msg_out.clone()))
            .await
            .expect("Failed to publish in time")
            .unwrap();

        let msg_in = timeout(TIMEOUT, rx.next())
            .await
            .expect("Failed to receive in time");

        assert_eq!(msg_in, msg_out);
    }

    #[tokio::test]
    /// Designed to test behavior when receiving a message of unexpected type on a topic
    // TODO this test is good, but actually shows how bad the ergonomics are and how we want to improve them!
    // We want a failed message parse / type mismatch to come through to the subscriber
    async fn bad_message_recv() -> TestResult {
        let mut client =
            ClientHandle::new_with_options(ClientHandleOptions::new(LOCAL_WS).timeout(TIMEOUT))
                .await?;

        let publisher = client.advertise::<Time>("/bad_message_recv/topic").await?;

        let sub: Subscriber<Header> = client.subscribe("/bad_message_recv/topic").await?;

        #[cfg(feature = "ros1_test")]
        publisher
            .publish(Time {
                data: roslibrust_codegen::Time { secs: 0, nsecs: 0 },
            })
            .await?;

        #[cfg(feature = "ros2_test")]
        publisher.publish(Time { secs: 0, nsecs: 0 }).await?;

        match timeout(TIMEOUT, sub.next()).await {
            Err(_elapsed) => {
                // Test passed! it should timeout
                // Not actually behavior we want, error of some kind should come through subscription
            }
            _ => {
                assert!(false, "Bad message made it through");
            }
        }
        Ok(())
    }

    #[tokio::test]
    async fn timeouts_new() {
        // Intentionally a port where there won't be a server at
        let opts = ClientHandleOptions::new("ws://localhost:9091").timeout(TIMEOUT);
        assert!(ClientHandle::new_with_options(opts).await.is_err());
        // Impossibly short to actually work
        let opts = ClientHandleOptions::new(LOCAL_WS).timeout(Duration::from_nanos(1));
        assert!(ClientHandle::new_with_options(opts).await.is_err());
        // Doesn't timeout if given enough time
        let opts = ClientHandleOptions::new(LOCAL_WS).timeout(TIMEOUT);
        assert!(ClientHandle::new_with_options(opts).await.is_ok());
    }

    /// This test doesn't actually do much, but instead confirms the internal structure of the lib is multi-threaded correctly
    /// The whole goal here is to catch send / sync complier errors
    #[tokio::test]
    async fn parallel_construction() {
        let mut client = ClientHandle::new(LOCAL_WS)
            .await
            .expect("Failed to construct client");

        let client_1 = client.clone();
        tokio::task::spawn(async move {
            let _ = client_1
                .advertise::<Header>("parrallel_1")
                .await
                .expect("Failed to advertise _1");
        });

        tokio::task::spawn(async move {
            let _ = client
                .subscribe::<Header>("parrallel_1")
                .await
                .expect("Failed to subscribe _1");
        });
    }

    /// Tests that dropping a publisher correctly unadvertises
    #[tokio::test]
    // This test is currently broken, it seems that rosbridge still sends the message regardless
    // of advertise / unadvertise status. Unclear how to confirm whether unadvertise was sent or not
    #[ignore]
    async fn unadvertise() -> TestResult {
        let _ = simple_logger::SimpleLogger::new()
            .with_level(log::LevelFilter::Debug)
            .without_timestamps()
            .init();

        // Flow:
        //  1. Create a publisher and subscriber
        //  2. Send a message and confirm connection works (topic was advertised)
        //  3. Drop the publisher, unadvertise should be sent
        //  4. Manually send a message without a publisher it should fail since topic was unadvertised
        const TOPIC: &str = "/unadvertise";
        debug!("Start unadvertise test");

        let opt = ClientHandleOptions::new(LOCAL_WS).timeout(TIMEOUT);

        let mut client = ClientHandle::new_with_options(opt).await?;
        let publisher = client.advertise(TOPIC).await?;
        debug!("Got publisher");

        let sub = client.subscribe::<Header>(TOPIC).await?;
        debug!("Got subscriber");

        let msg = Header::default();
        publisher.publish(msg).await?;
        timeout(TIMEOUT, sub.next()).await?;

        debug!("Dropping publisher");
        // drop subscriber so it doesn't keep topic open
        std::mem::drop(sub);
        // unadvertise should happen here
        std::mem::drop(publisher);

        // Wait for drop to complete
        tokio::time::sleep(TIMEOUT).await;

        let sub = client.subscribe::<Header>(TOPIC).await?;
        // manually publishing using private api
        let msg = Header::default();
        client.publish(TOPIC, msg).await?;

        match timeout(TIMEOUT, sub.next()).await {
            Ok(_msg) => {
                anyhow::bail!("Received message after unadvertised!");
            }
            Err(_e) => {
                // All good! Timeout should expire
            }
        }
        Ok(())
    }

    // This test currently doesn't work for ROS2, however all other service functionalities appear fine
    // It may be that ros2 prevents a "service_loop" where a node calls a service on itself?
    // unclear...
    #[cfg(feature = "ros1_test")]
    #[tokio::test]
    async fn self_service_call() -> TestResult {
        let _ = simple_logger::SimpleLogger::new()
            .with_level(log::LevelFilter::Debug)
            .without_timestamps()
            .init();

        let opt = ClientHandleOptions::new(LOCAL_WS).timeout(TIMEOUT);
        let mut client = ClientHandle::new_with_options(opt).await?;

        let cb =
            |_req: SetBoolRequest| -> Result<SetBoolResponse, Box<dyn std::error::Error + Send + Sync>> {
                Ok(SetBoolResponse {
                    success: true,
                    message: "call_success".to_string(),
                })
            };

        let topic = "/self_service_call";

        let handle = client
            .advertise_service::<SetBool>(topic, cb)
            .await
            .expect("Failed to advertise service");

        // Make sure service advertise makes it through
        tokio::time::sleep(TIMEOUT).await;

        let response: SetBoolResponse = client
            .call_service(topic, SetBoolRequest { data: true })
            .await
            .expect("Failed to call service");
        assert_eq!(response.message, "call_success");

        // ros2 freaks out if we unadvertise the service while still in flight so pause here
        tokio::time::sleep(TIMEOUT).await;

        // Intentionally drop handle to unadvertise the service
        std::mem::drop(handle);

        // Should now fail to get a response after the handle is dropped
        let response = client
            .call_service::<SetBoolRequest, SetBoolResponse>(topic, SetBoolRequest { data: true })
            .await;
        assert!(response.is_err());

        Ok(())
    }

    #[tokio::test]
    async fn test_strong_and_weak_client_counts() -> TestResult {
        let opt = ClientHandleOptions::new(LOCAL_WS).timeout(TIMEOUT);
        let client = ClientHandle::new_with_options(opt).await?;
        // Can't be certain what state the spin loop is in (it could be upgraded from WeakPtr) so we sum the two
        assert_eq!(
            Arc::strong_count(&client.inner) + Arc::weak_count(&client.inner),
            2
        );

        {
            let client_2 = client.clone();
            assert_eq!(
                Arc::strong_count(&client.inner) + Arc::weak_count(&client.inner),
                3
            );
            assert_eq!(
                Arc::strong_count(&client_2.inner) + Arc::weak_count(&client_2.inner),
                3
            );
        }

        assert_eq!(
            Arc::strong_count(&client.inner) + Arc::weak_count(&client.inner),
            2
        );

        Ok(())
    }

    #[tokio::test]
    async fn test_disconnect_returns_error() -> TestResult {
        let client =
            ClientHandle::new_with_options(ClientHandleOptions::new(LOCAL_WS).timeout(TIMEOUT))
                .await?;
        client
            .is_disconnected
            .store(true, std::sync::atomic::Ordering::Relaxed);

        let res = client.advertise::<Time>("/bad_message_recv/topic").await;
        assert!(matches!(res, Err(RosLibRustError::Disconnected)));

        Ok(())
    }

    #[tokio::test]
    async fn working_with_char() -> TestResult {
        let mut client =
            ClientHandle::new_with_options(ClientHandleOptions::new(LOCAL_WS).timeout(TIMEOUT))
                .await?;

        let subscriber = client.subscribe::<std_msgs::Char>("/char_topic").await?;
        tokio::time::sleep(TIMEOUT).await;
        let publisher = client.advertise("/char_topic").await?;
        tokio::time::sleep(TIMEOUT).await;

        // Note because C++ char != rust char some care has to be taken when converting
        let x = std_msgs::Char {
            data: 'x'.try_into().unwrap(),
        };
        publisher.publish(x.clone()).await?;

        let y = timeout(TIMEOUT, subscriber.next())
            .await
            .expect("Failed to receive char message");

        assert_eq!(&x, &y);

        assert_eq!(y.data as char, 'x');

        Ok(())
    }

}
