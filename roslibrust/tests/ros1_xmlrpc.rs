#[cfg(all(feature = "ros1", feature = "ros1_test"))]
mod tests {
    use roslibrust::ros1::NodeHandle;
    use roslibrust_codegen::RosMessageType;
    use serde::de::DeserializeOwned;
    use serde_xmlrpc::Value;
    roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces");

    async fn call_node_api_raw(uri: &str, endpoint: &str, args: Vec<Value>) -> String {
        let client = reqwest::Client::new();
        let body = serde_xmlrpc::request_to_string(endpoint, args).unwrap();
        let response = client.post(uri).body(body).send().await.unwrap();
        response.text().await.unwrap()
    }

    async fn call_node_api<T: DeserializeOwned>(uri: &str, endpoint: &str, args: Vec<Value>) -> T {
        log::debug!("Start call api: {uri:?}, {endpoint:?}, {args:?}");
        // Note: don't need timeout here as all operations in side call_node_api_raw are timeout()'d
        let response = call_node_api_raw(uri, endpoint, args).await;
        log::debug!("Got api raw response");
        let (error_code, error_description, value): (i8, String, T) =
            serde_xmlrpc::response_from_str(&response).unwrap();

        assert_eq!(error_code, 1);
        assert_eq!(error_description.len(), 0);
        value
    }

    #[test_log::test(tokio::test)]
    async fn verify_get_master_uri() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let node = NodeHandle::new("http://localhost:11311", "verify_get_master_uri").await?;
        log::info!("Got new handle");

        let node_uri = node.get_client_uri().await?;
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
        Ok(())
    }

    #[test_log::test(tokio::test)]
    async fn verify_get_publications() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let node = NodeHandle::new("http://localhost:11311", "verify_get_publications").await?;
        log::info!("Got new handle");

        let node_uri = node.get_client_uri().await?;
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

        let _publisher = node.advertise::<std_msgs::String>("/test_topic", 1).await?;
        log::info!("advertised");

        // Note: internally timeout()'d
        let publications = call_node_api::<Vec<(String, String)>>(
            &node_uri,
            "getPublications",
            vec!["/verify_get_publications".into()],
        )
        .await;
        log::info!("Got post advertise publications");

        assert_eq!(publications.len(), 1);
        let (topic, topic_type) = publications.iter().nth(0).unwrap();
        assert_eq!(topic, "/test_topic");
        assert_eq!(topic_type, std_msgs::String::ROS_TYPE_NAME);
        Ok(())
    }

    #[test_log::test(tokio::test)]
    async fn verify_shutdown() {
        let node = NodeHandle::new("http://localhost:11311", "verify_shutdown")
            .await
            .unwrap();
        log::info!("Got handle");
        let node_uri = node.get_client_uri().await.unwrap();
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

        tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;

        assert!(!node.is_ok());
    }

    #[test_log::test(tokio::test)]
    async fn verify_request_topic() {
        let node = NodeHandle::new("http://localhost:11311", "verify_request_topic")
            .await
            .unwrap();
        log::info!("Got handle");
        let node_uri = node.get_client_uri().await.unwrap();
        log::info!("Got uri {node_uri:?}");

        let _publisher = node
            .advertise::<std_msgs::String>("/test_topic", 1)
            .await
            .unwrap();
        log::info!("Got publisher");

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
        log::info!("Got response");

        let (code, _description, (protocol, host, port)): (i8, String, (String, String, u16)) =
            serde_xmlrpc::response_from_str(&response).unwrap();
        assert_eq!(code, 1);
        assert_eq!(protocol, "TCPROS");
        assert!(!host.is_empty());
        assert!(port != 0);
    }
}
