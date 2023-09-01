#[cfg(all(feature = "ros1", feature = "ros1_test"))]
mod tests {
    use serde::de::DeserializeOwned;
    use serde_xmlrpc::Value;

    async fn call_node_api<T: DeserializeOwned>(uri: &str, endpoint: &str, args: Vec<Value>) -> T {
        let client = reqwest::Client::new();
        let body = serde_xmlrpc::request_to_string(endpoint, args).unwrap();
        let response = client
            .post(uri)
            .body(body)
            .send()
            .await
            .unwrap()
            .text()
            .await
            .unwrap();
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
    }
}
