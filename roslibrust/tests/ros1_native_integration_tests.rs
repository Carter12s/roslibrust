//! This test file is intended to contain all integration tests of ROS1 native fuctionality.
//! Any test which interacts with actual running ros nodes should be in this file.

#[cfg(feature = "ros1_test")]
mod tests {
    use log::*;
    use roslibrust::ros1::{NodeError, NodeHandle};
    use tokio::time::timeout;

    roslibrust_codegen_macro::find_and_generate_ros_messages!(
        "assets/ros1_test_msgs",
        "assets/ros1_common_interfaces"
    );

    #[test_log::test(tokio::test)]
    async fn test_latching() {
        let nh = NodeHandle::new("http://localhost:11311", "test_latching")
            .await
            .unwrap();

        // Create a publisher that is latching
        let publisher = nh
            .advertise::<std_msgs::String>("/test_latching", 1, true)
            .await
            .unwrap();

        // Publish message to no one
        publisher
            .publish(&std_msgs::String {
                data: "test".to_owned(),
            })
            .await
            .unwrap();

        // Create a subscriber that will connect to the publisher
        let mut subscriber = nh
            .subscribe::<std_msgs::String>("/test_latching", 1)
            .await
            .unwrap();

        // Try to get message from subscriber
        let msg = subscriber.next().await.unwrap().unwrap();

        // Confirm we got the message we published
        assert_eq!(msg.data, "test");
    }

    #[test_log::test(tokio::test)]
    async fn test_not_latching() {
        // Opposite of test_latching, confirms no message appears when latching is false
        let nh = NodeHandle::new("http://localhost:11311", "test_not_latching")
            .await
            .unwrap();

        let publisher = nh
            .advertise::<std_msgs::String>("/test_not_latching", 1, false)
            .await
            .unwrap();

        publisher
            .publish(&std_msgs::String {
                data: "test".to_owned(),
            })
            .await
            .unwrap();

        let mut subscriber = nh
            .subscribe::<std_msgs::String>("/test_not_latching", 1)
            .await
            .unwrap();

        let res =
            tokio::time::timeout(tokio::time::Duration::from_millis(250), subscriber.next()).await;
        // Should timeout
        assert!(res.is_err());
    }

    #[test_log::test(tokio::test)]
    async fn test_large_payload_subscriber() {
        let nh = NodeHandle::new("http://localhost:11311", "/test_large_payload_subscriber")
            .await
            .unwrap();

        let publisher = nh
            .advertise::<test_msgs::RoundTripArrayRequest>("/large_payload_topic", 1, false)
            .await
            .unwrap();

        let mut subscriber = nh
            .subscribe::<test_msgs::RoundTripArrayRequest>("/large_payload_topic", 1)
            .await
            .unwrap();

        // Give some time for subscriber to connect to publisher
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

        for _i in 0..10 {
            let bytes = vec![0; 10_000];
            publisher
                .publish(&test_msgs::RoundTripArrayRequest {
                    bytes: bytes.clone(),
                })
                .await
                .unwrap();

            match subscriber.next().await {
                Some(Ok(msg)) => {
                    assert_eq!(msg.bytes, bytes);
                }
                Some(Err(e)) => {
                    panic!("Got error: {e:?}");
                }
                None => {
                    panic!("Got None");
                }
            }
        }
    }

    #[test_log::test(tokio::test)]
    async fn test_large_service_payload_client() {
        let nh = NodeHandle::new(
            "http://localhost:11311",
            "test_large_service_payload_client",
        )
        .await
        .unwrap();

        // Advertise a service that just echo's the bytes back
        let _handle = nh
            .advertise_service::<test_msgs::RoundTripArray, _>("large_service_payload", |request| {
                Ok(test_msgs::RoundTripArrayResponse {
                    bytes: request.bytes,
                })
            })
            .await
            .unwrap();

        // Picking random value that should be larger than MTU
        // Making sure the ROS message gets split over multiple TCP transactions
        // and that we correctly re-assemble it on the other end
        let bytes = vec![0; 10_000];

        info!("Starting service call");
        let response = nh
            .service_client::<test_msgs::RoundTripArray>("large_service_payload")
            .await
            .unwrap()
            .call(&test_msgs::RoundTripArrayRequest {
                bytes: bytes.clone(),
            })
            .await
            .unwrap();
        info!("Service call complete");

        assert_eq!(response.bytes, bytes);
    }

    #[test_log::test(tokio::test)]
    async fn error_on_unprovided_service() {
        let nh = NodeHandle::new("http://localhost:11311", "error_on_unprovided_service")
            .await
            .unwrap();

        let client = nh
            .service_client::<test_msgs::RoundTripArray>("unprovided_service")
            .await;
        assert!(client.is_err());
        // Note / TODO: this currently returns an IoError(Kind(ConnectionAborted))
        // which is better than hanging, but not a good error type to return
        if !matches!(client, Err(NodeError::IoError(_))) {
            panic!("Unexpected error type");
        }
    }

    #[test_log::test(tokio::test)]
    async fn persistent_client_can_be_called_multiple_times() {
        let nh = NodeHandle::new(
            "http://localhost:11311",
            "/persistent_client_can_be_called_multiple_times",
        )
        .await
        .unwrap();

        let server_fn = |request: test_msgs::AddTwoIntsRequest| {
            Ok(test_msgs::AddTwoIntsResponse {
                sum: request.a + request.b,
            })
        };

        let _handle = nh
            .advertise_service::<test_msgs::AddTwoInts, _>(
                "/persistent_client_can_be_called_multiple_times/add_two",
                server_fn,
            )
            .await
            .unwrap();

        let client = nh
            .service_client::<test_msgs::AddTwoInts>(
                "/persistent_client_can_be_called_multiple_times/add_two",
            )
            .await
            .unwrap();

        for i in 0..10 {
            let call: test_msgs::AddTwoIntsResponse = client
                .call(&test_msgs::AddTwoIntsRequest { a: 1, b: i })
                .await
                .unwrap();

            assert_eq!(call.sum, 1 + i);
        }
    }

    #[test_log::test(tokio::test)]
    async fn basic_service_server() {
        const TIMEOUT: std::time::Duration = std::time::Duration::from_secs(1);
        debug!("Getting node handle");
        let nh = NodeHandle::new("http://localhost:11311", "/basic_service_server")
            .await
            .unwrap();

        let server_fn = |request: test_msgs::AddTwoIntsRequest| {
            info!("Got request: {request:?}");
            return Ok(test_msgs::AddTwoIntsResponse {
                sum: request.a + request.b,
            });
        };

        // Create the server
        debug!("Creating server");
        let _handle = nh
            .advertise_service::<test_msgs::AddTwoInts, _>(
                "/basic_service_server/add_two",
                server_fn,
            )
            .await
            .unwrap();

        // Make the request
        debug!("Calling service");
        let call: test_msgs::AddTwoIntsResponse = timeout(
            TIMEOUT,
            timeout(
                TIMEOUT,
                nh.service_client::<test_msgs::AddTwoInts>("/basic_service_server/add_two"),
            )
            .await
            .unwrap()
            .unwrap()
            .call(&test_msgs::AddTwoIntsRequest { a: 1, b: 2 }),
        )
        .await
        .unwrap()
        .unwrap();

        assert_eq!(call.sum, 3);
        debug!("Got 3");
    }

    #[test_log::test(tokio::test)]
    async fn dropping_service_server_kill_correctly() {
        debug!("Getting node handle");
        let nh = NodeHandle::new("http://localhost:11311", "/dropping_service_node")
            .await
            .unwrap();

        let server_fn = |request: test_msgs::AddTwoIntsRequest| {
            info!("Got request: {request:?}");
            return Ok(test_msgs::AddTwoIntsResponse {
                sum: request.a + request.b,
            });
        };

        // Create the server
        let handle = nh
            .advertise_service::<test_msgs::AddTwoInts, _>("~/add_two", server_fn)
            .await
            .unwrap();

        // Make the request (should succeed)
        let client = nh
            .service_client::<test_msgs::AddTwoInts>("~/add_two")
            .await
            .unwrap();
        let _call: test_msgs::AddTwoIntsResponse = client
            .call(&test_msgs::AddTwoIntsRequest { a: 1, b: 2 })
            .await
            .unwrap();

        // Shut down the server
        std::mem::drop(handle);
        // Wait a little bit for server shut down to process
        tokio::time::sleep(std::time::Duration::from_millis(250)).await;

        // Make the request again (should fail)
        let call_2 = client
            .call(&test_msgs::AddTwoIntsRequest { a: 1, b: 2 })
            .await;
        debug!("Got call_2: {call_2:?}");
        assert!(
            call_2.is_err(),
            "Shouldn't be able to call after server is shut down"
        );

        // Create a new clinet
        let client = nh
            .service_client::<test_msgs::AddTwoInts>("~/add_two")
            .await;
        // Client should fail to create as there should be no provider of the service
        assert!(
            client.is_err(),
            "Shouldn't be able to connect again (no provider of service)"
        );

        // Confirm ros master no longer reports our service as provided (via rosapi for fun)
        let rosapi_client = nh
            .service_client::<rosapi::Services>("/rosapi/services")
            .await
            .unwrap();
        let service_list: rosapi::ServicesResponse = rosapi_client
            .call(&rosapi::ServicesRequest {})
            .await
            .unwrap();
        assert!(!service_list
            .services
            .contains(&"/dropping_service_node/add_two".to_string()));
    }

    #[test_log::test(tokio::test)]
    async fn service_error_behavior() {
        debug!("Getting node handle");
        let nh = NodeHandle::new("http://localhost:11311", "/service_error_behavior")
            .await
            .unwrap();

        let server_fn = |request: test_msgs::AddTwoIntsRequest| -> Result<
            test_msgs::AddTwoIntsResponse,
            Box<dyn std::error::Error + Send + Sync>,
        > {
            info!("Got request: {request:?}");
            return Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                "test message",
            )));
        };

        // Create the server
        let _handle = nh
            .advertise_service::<test_msgs::AddTwoInts, _>("~/add_two", server_fn)
            .await
            .unwrap();

        // Make the request (should fail)
        let client = nh
            .service_client::<test_msgs::AddTwoInts>("~/add_two")
            .await
            .unwrap();
        let call = client
            .call(&test_msgs::AddTwoIntsRequest { a: 1, b: 2 })
            .await;
        // Okay so this is logging the error message correctly, but the contents currently suck:
        // "Got call: Err(IoError(Custom { kind: Other, error: "Failure response from service server: Custom { kind: NotFound, error: \"test message\" }" }))"
        // We should someday clean up error types here, but frankly errors throughout the entire crate need an overhaul
        debug!("Got call: {call:?}");
        assert!(call.is_err());
    }
}
