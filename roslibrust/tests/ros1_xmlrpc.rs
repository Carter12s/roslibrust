#[cfg(all(feature = "ros1", feature = "ros1_test"))]
mod tests {
    use roslibrust_codegen::RosMessageType;
    use serde::de::DeserializeOwned;
    use serde_xmlrpc::Value;
    use tokio::time::timeout;
    const TIMEOUT: tokio::time::Duration = tokio::time::Duration::from_millis(500);

    roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces");

    async fn call_node_api_raw(uri: &str, endpoint: &str, args: Vec<Value>) -> String {
        let client = reqwest::Client::new();
        let body = serde_xmlrpc::request_to_string(endpoint, args).unwrap();
        let response = timeout(TIMEOUT, client.post(uri).body(body).send())
            .await
            .unwrap()
            .unwrap();
        timeout(TIMEOUT, response.text()).await.unwrap().unwrap()
    }

    async fn call_node_api<T: DeserializeOwned>(uri: &str, endpoint: &str, args: Vec<Value>) -> T {
        // Note: don't need timeout here as all operations in side call_node_api_raw are timeout()'d
        let response = call_node_api_raw(uri, endpoint, args).await;
        let (error_code, error_description, value): (i8, String, T) =
            serde_xmlrpc::response_from_str(&response).unwrap();

        assert_eq!(error_code, 1);
        assert_eq!(error_description.len(), 0);
        value
    }

    async fn test_with_watch_dog(test: impl std::future::Future<Output = ()>) {
        // Overall watchdog since I can't get these tests to timeout, but they take 20min in CI
        // Need the task to fail so I can get any debug information out
        // Can't get the failure to repro locally
        let watchdog = async {
            tokio::time::sleep(TIMEOUT * 5).await;
            log::error!("Test watchdog tripped!");
        };
        tokio::select! {
            _ = watchdog => {
                panic!("Test failed due to watchdog");
            }
            _ = test => {
                // Happy days!
            }
        }
    }

    #[test_log::test(tokio::test(flavor = "multi_thread", worker_threads = 2))]
    #[ignore]
    async fn verify_get_master_uri() {
        test_with_watch_dog(async {
            let node = timeout(
                TIMEOUT,
                roslibrust::NodeHandle::new("http://localhost:11311", "verify_get_master_uri"),
            )
            .await
            .unwrap()
            .unwrap();
            log::info!("Got new handle");

            let node_uri = timeout(TIMEOUT, node.get_client_uri())
                .await
                .unwrap()
                .unwrap();
            log::info!("Got uri");

            // Note: doesn't need timeout as it is internally timeout()'d
            let master_uri = call_node_api::<String>(
                &node_uri,
                "getMasterUri",
                vec!["/get_master_uri_test".into()],
            )
            .await;
            log::info!("Got master");
            assert_eq!(master_uri, "http://localhost:11311");
        })
        .await;
    }

    #[test_log::test(tokio::test(flavor = "multi_thread", worker_threads = 2))]
    #[ignore]
    async fn verify_get_publications() {
        test_with_watch_dog(async {
            let node = timeout(
                TIMEOUT,
                roslibrust::NodeHandle::new("http://localhost:11311", "verify_get_publications"),
            )
            .await
            .unwrap()
            .unwrap();
            log::info!("Got new handle");

            let node_uri = timeout(TIMEOUT, node.get_client_uri())
                .await
                .unwrap()
                .unwrap();
            log::info!("Got uri");

            // Note: timeout not needed as it is internally timeout'd
            let publications = call_node_api::<Vec<(String, String)>>(
                &node_uri,
                "getPublications",
                vec!["/verify_get_publications".into()],
            )
            .await;
            assert_eq!(publications.len(), 0);
            log::info!("Got publications");

            let _publisher = timeout(
                TIMEOUT,
                node.advertise::<std_msgs::String>("/test_topic", 1),
            )
            .await
            .unwrap()
            .unwrap();
            log::info!("advertised");

            // Note: internally timeout()'d
            let publications = timeout(
                TIMEOUT,
                call_node_api::<Vec<(String, String)>>(
                    &node_uri,
                    "getPublications",
                    vec!["/verify_get_publications".into()],
                ),
            )
            .await
            .unwrap();
            log::info!("Got post advertise publications");

            assert_eq!(publications.len(), 1);
            let (topic, topic_type) = publications.iter().nth(0).unwrap();
            assert_eq!(topic, "/test_topic");
            assert_eq!(topic_type, std_msgs::String::ROS_TYPE_NAME);
        })
        .await;
    }

    #[test_log::test(tokio::test(flavor = "multi_thread", worker_threads = 2))]
    #[ignore]
    async fn verify_shutdown() {
        test_with_watch_dog(async {
            let node = timeout(
                TIMEOUT,
                roslibrust::NodeHandle::new("http://localhost:11311", "verify_shutdown"),
            )
            .await
            .unwrap()
            .unwrap();
            log::info!("Got handle");
            let node_uri = timeout(TIMEOUT, node.get_client_uri())
                .await
                .unwrap()
                .unwrap();
            assert!(node.is_ok());
            log::info!("Got uri");

            // Note: internally timeout()'d
            call_node_api::<i32>(
                &node_uri,
                "shutdown",
                vec!["/verify_shutdown".into(), "".into()],
            )
            .await;
            log::info!("shutdown?");

            assert!(!node.is_ok());
        })
        .await;
    }

    #[test_log::test(tokio::test(flavor = "multi_thread", worker_threads = 2))]
    #[ignore]
    async fn verify_request_topic() {
        test_with_watch_dog(async {
            let node = timeout(
                TIMEOUT,
                roslibrust::NodeHandle::new("http://localhost:11311", "verify_request_topic"),
            )
            .await
            .unwrap()
            .unwrap();
            log::info!("Got handle");
            let node_uri = timeout(TIMEOUT, node.get_client_uri())
                .await
                .unwrap()
                .unwrap();
            log::info!("Got uri {node_uri:?}");

            let _publisher = timeout(
                TIMEOUT,
                node.advertise::<std_msgs::String>("/test_topic", 1),
            )
            .await
            .unwrap()
            .unwrap();
            log::info!("Got publisher");

            // Note: internally timeout()'d
            let response = call_node_api_raw(
                &node_uri,
                "requestTopic",
                vec![
                    "verify_request_topic".into(),
                    "/test_topic".into(),
                    serde_xmlrpc::Value::Array(vec![serde_xmlrpc::Value::Array(vec![
                        "TCPROS".into()
                    ])]),
                ],
            )
            .await;
            log::info!("Got response");

            let (code, _description, (protocol, host, port)): (i8, String, (String, String, u16)) =
                serde_xmlrpc::response_from_str(&response).unwrap();
            assert_eq!(code, 1);
            assert_eq!(protocol, "TCPROS");
            assert!(!host.is_empty());
            assert!(port != 0);
        })
        .await;
    }
}
