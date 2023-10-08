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

    #[tokio::test]
    #[ignore]
    async fn verify_get_master_uri() {
        let node = timeout(
            TIMEOUT,
            roslibrust::NodeHandle::new("http://localhost:11311", "verify_get_master_uri"),
        )
        .await
        .unwrap()
        .unwrap();
        let node_uri = timeout(TIMEOUT, node.get_client_uri())
            .await
            .unwrap()
            .unwrap();

        // Note: doesn't need timeout as it is internally timeout()'d
        let master_uri = call_node_api::<String>(
            &node_uri,
            "getMasterUri",
            vec!["/get_master_uri_test".into()],
        )
        .await;
        assert_eq!(master_uri, "http://localhost:11311");
    }

    #[tokio::test]
    #[ignore]
    async fn verify_get_publications() {
        let node = timeout(
            TIMEOUT,
            roslibrust::NodeHandle::new("http://localhost:11311", "verify_get_publications"),
        )
        .await
        .unwrap()
        .unwrap();

        let node_uri = timeout(TIMEOUT, node.get_client_uri())
            .await
            .unwrap()
            .unwrap();

        // Note: timeout not needed as it is internally timeout'd
        let publications = call_node_api::<Vec<(String, String)>>(
            &node_uri,
            "getPublications",
            vec!["/verify_get_publications".into()],
        )
        .await;
        assert_eq!(publications.len(), 0);

        let _publisher = timeout(
            TIMEOUT,
            node.advertise::<std_msgs::String>("/test_topic", 1),
        )
        .await
        .unwrap()
        .unwrap();

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
        assert_eq!(publications.len(), 1);
        let (topic, topic_type) = publications.iter().nth(0).unwrap();
        assert_eq!(topic, "/test_topic");
        assert_eq!(topic_type, std_msgs::String::ROS_TYPE_NAME);
    }

    #[tokio::test]
    #[ignore]
    async fn verify_shutdown() {
        let node = timeout(
            TIMEOUT,
            roslibrust::NodeHandle::new("http://localhost:11311", "verify_shutdown"),
        )
        .await
        .unwrap()
        .unwrap();
        let node_uri = timeout(TIMEOUT, node.get_client_uri())
            .await
            .unwrap()
            .unwrap();
        assert!(node.is_ok());

        // Note: internally timeout()'d
        call_node_api::<i32>(
            &node_uri,
            "shutdown",
            vec!["/verify_shutdown".into(), "".into()],
        )
        .await;

        assert!(!node.is_ok());
    }

    #[tokio::test]
    #[ignore]
    async fn verify_request_topic() {
        let node = timeout(
            TIMEOUT,
            roslibrust::NodeHandle::new("http://localhost:11311", "verify_request_topic"),
        )
        .await
        .unwrap()
        .unwrap();
        let node_uri = timeout(TIMEOUT, node.get_client_uri())
            .await
            .unwrap()
            .unwrap();

        let _publisher = timeout(
            TIMEOUT,
            node.advertise::<std_msgs::String>("/test_topic", 1),
        )
        .await
        .unwrap()
        .unwrap();

        // Note: internally timeout()'d
        let response = call_node_api_raw(
            &node_uri,
            "requestTopic",
            vec![
                "verify_request_topic".into(),
                "/test_topic".into(),
                serde_xmlrpc::Value::Array(vec![serde_xmlrpc::Value::Array(vec!["TCPROS".into()])]),
            ],
        )
        .await;

        let (code, _description, (protocol, host, port)): (i8, String, (String, String, u16)) =
            serde_xmlrpc::response_from_str(&response).unwrap();
        assert_eq!(code, 1);
        assert_eq!(protocol, "TCPROS");
        assert!(!host.is_empty());
        assert!(port != 0);
    }
}
