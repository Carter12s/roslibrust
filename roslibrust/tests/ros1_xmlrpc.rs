#[cfg(all(feature = "ros1", feature = "ros1_test"))]
mod tests {
    roslibrust_codegen_macro::find_and_generate_ros_messages_relative_to_manifest_dir!(
        "../assets/ros1_common_interfaces"
    );

    use roslibrust_codegen::RosMessageType;
    use serde::de::DeserializeOwned;
    use serde_xmlrpc::Value;

    roslibrust_codegen_macro::find_and_generate_ros_messages_relative_to_manifest_dir!(
        "../assets/ros1_common_interfaces"
    );

    async fn call_node_api_raw(uri: &str, endpoint: &str, args: Vec<Value>) -> String {
        let client = reqwest::Client::new();
        let body = serde_xmlrpc::request_to_string(endpoint, args).unwrap();
        client
            .post(uri)
            .body(body)
            .send()
            .await
            .unwrap()
            .text()
            .await
            .unwrap()
    }

    async fn call_node_api<T: DeserializeOwned>(uri: &str, endpoint: &str, args: Vec<Value>) -> T {
        let response = call_node_api_raw(uri, endpoint, args).await;
        let (error_code, error_description, value): (i8, String, T) =
            serde_xmlrpc::response_from_str(&response).unwrap();

        assert_eq!(error_code, 1);
        assert_eq!(error_description.len(), 0);
        value
    }

    #[tokio::test]
    async fn verify_get_master_uri() {
        let node = roslibrust::NodeHandle::new("http://localhost:11311", "verify_get_master_uri")
            .await
            .unwrap();
        let node_uri = node.get_client_uri().await.unwrap();

        let master_uri = call_node_api::<String>(
            &node_uri,
            "getMasterUri",
            vec!["/get_master_uri_test".into()],
        )
        .await;
        assert_eq!(master_uri, "http://localhost:11311");
    }

    #[tokio::test]
    async fn verify_get_publications() {
        let node = roslibrust::NodeHandle::new("http://localhost:11311", "verify_get_publications")
            .await
            .unwrap();
        let node_uri = node.get_client_uri().await.unwrap();

        let publications = call_node_api::<Vec<(String, String)>>(
            &node_uri,
            "getPublications",
            vec!["/verify_get_publications".into()],
        )
        .await;
        assert_eq!(publications.len(), 0);

        let _publisher = node
            .advertise::<std_msgs::String>("/test_topic", 1)
            .await
            .unwrap();

        let publications = call_node_api::<Vec<(String, String)>>(
            &node_uri,
            "getPublications",
            vec!["/verify_get_publications".into()],
        )
        .await;
        assert_eq!(publications.len(), 1);
        let (topic, topic_type) = publications.iter().nth(0).unwrap();
        assert_eq!(topic, "/test_topic");
        assert_eq!(topic_type, std_msgs::String::ROS_TYPE_NAME);
    }

    #[tokio::test]
    async fn verify_shutdown() {
        let node = roslibrust::NodeHandle::new("http://localhost:11311", "verify_shutdown")
            .await
            .unwrap();
        let node_uri = node.get_client_uri().await.unwrap();
        assert!(node.is_ok());

        call_node_api::<i32>(
            &node_uri,
            "shutdown",
            vec!["/verify_shutdown".into(), "".into()],
        )
        .await;

        assert!(!node.is_ok());
    }

    #[tokio::test]
    async fn verify_request_topic() {
        let node = roslibrust::NodeHandle::new("http://localhost:11311", "verify_request_topic")
            .await
            .unwrap();
        let node_uri = node.get_client_uri().await.unwrap();

        let _publisher = node
            .advertise::<std_msgs::String>("/test_topic", 1)
            .await
            .unwrap();

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
