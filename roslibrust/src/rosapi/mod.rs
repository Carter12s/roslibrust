//! This module is intended to provide a convenience wrapper around the capabilities
//! provided by the [rosapi](http://wiki.ros.org/rosapi) node.
//!
//! Ensure rosapi is running on your target system before attempting to utilize these features!

use crate::{ClientHandle, RosLibRustResult};

pub mod types;
use types::*;

pub struct RosApi {
    handle: ClientHandle,
}

impl RosApi {
    // TODO replace this by impl Trait on ClientHandle
    pub fn new(handle: ClientHandle) -> Self {
        RosApi { handle }
    }

    /// Get the current time
    pub async fn get_time(&self) -> RosLibRustResult<rosapi::GetTimeResponse> {
        self.handle
            .call_service("/rosapi/get_time", rosapi::GetTimeRequest {})
            .await
    }

    /// Get the list of topics active
    pub async fn topics(&self) -> RosLibRustResult<rosapi::TopicsResponse> {
        self.handle
            .call_service("/rosapi/topics", rosapi::TopicsRequest {})
            .await
    }

    /// Get the type of a given topic
    pub async fn get_topic_type(
        &self,
        topic: impl Into<String>,
    ) -> RosLibRustResult<rosapi::TopicTypeResponse> {
        self.handle
            .call_service(
                "/rosapi/topic_type",
                rosapi::TopicTypeRequest {
                    topic: topic.into(),
                },
            )
            .await
    }

    /// Returns a list of the topics active in the system that are of the type provided
    pub async fn get_topics_for_type(
        &self,
        topic_type: impl Into<String>,
    ) -> RosLibRustResult<rosapi::TopicsForTypeResponse> {
        self.handle
            .call_service(
                "/rosapi/topics_for_type",
                rosapi::TopicsForTypeRequest {
                    r#type: topic_type.into(),
                },
            )
            .await
    }

    /// Returns list of nodes active in a system
    pub async fn get_nodes(&self) -> RosLibRustResult<rosapi::NodesResponse> {
        self.handle
            .call_service("/rosapi/nodes", rosapi::NodesRequest {})
            .await
    }

    /// Returns the subscriptions, publishers, and service servers for a given node
    /// @param node Fully resolved ros node name e.g. "/rosapi", should match the names given by `rosnode list`
    pub async fn get_node_details(
        &self,
        node: impl Into<String>,
    ) -> RosLibRustResult<rosapi::NodeDetailsResponse> {
        self.handle
            .call_service(
                "/rosapi/node_details",
                rosapi::NodeDetailsRequest { node: node.into() },
            )
            .await
    }

    /// Given the name of service, get the name of the node that is providing that service
    pub async fn get_node_for_service(
        &self,
        service: impl Into<String>,
    ) -> RosLibRustResult<rosapi::ServiceNodeResponse> {
        self.handle
            .call_service(
                "/rosapi/service_node",
                rosapi::ServiceNodeRequest {
                    service: service.into(),
                },
            )
            .await
    }

    /// Sets a parameter. Unclear exactly how 'types' of parameters will be handled here.
    /// For now we're simply converting into a string.
    pub async fn set_param(
        &self,
        param_name: impl Into<String>,
        param_value: impl Into<String>,
    ) -> RosLibRustResult<rosapi::SetParamResponse> {
        self.handle
            .call_service(
                "/rosapi/set_param",
                rosapi::SetParamRequest {
                    name: param_name.into(),
                    value: param_value.into(),
                },
            )
            .await
    }

    /// Gets the current value for a parameter.
    /// Not handling any type safety or conversion.
    pub async fn get_param(
        &self,
        param_name: impl Into<String>,
    ) -> RosLibRustResult<rosapi::GetParamResponse> {
        self.handle
            .call_service(
                "/rosapi/get_param",
                rosapi::GetParamRequest {
                    name: param_name.into(),
                    default: "".to_string(),
                },
            )
            .await
    }

    /// Gets the list of currently known parameters.
    pub async fn get_param_names(&self) -> RosLibRustResult<rosapi::GetParamNamesResponse> {
        self.handle
            .call_service("/rosapi/get_param_names", rosapi::GetParamNamesRequest {})
            .await
    }

    /// Checks whether the given parameter is defined.
    pub async fn has_param(
        &self,
        param: impl Into<String>,
    ) -> RosLibRustResult<rosapi::HasParamResponse> {
        self.handle
            .call_service(
                "/rosapi/has_param",
                rosapi::HasParamRequest { name: param.into() },
            )
            .await
    }

    /// Deletes a parameter by name.
    pub async fn delete_param(
        &self,
        name: impl Into<String>,
    ) -> RosLibRustResult<rosapi::DeleteParamResponse> {
        self.handle
            .call_service(
                "/rosapi/delete_param",
                rosapi::DeleteParamRequest { name: name.into() },
            )
            .await
    }

    /// Returns detailed information about a given message type e.g. 'std_msgs/Header'
    pub async fn message_details(
        &self,
        message_name: impl Into<String>,
    ) -> RosLibRustResult<rosapi::MessageDetailsResponse> {
        self.handle
            .call_service(
                "/rosapi/message_details",
                rosapi::MessageDetailsRequest {
                    r#type: message_name.into(),
                },
            )
            .await
    }

    /// Gets a list of all nodes that are publishing to a given topic.
    pub async fn publishers(
        &self,
        topic: impl Into<String>,
    ) -> RosLibRustResult<rosapi::PublishersResponse> {
        self.handle
            .call_service(
                "/rosapi/publishers",
                rosapi::PublishersRequest {
                    topic: topic.into(),
                },
            )
            .await
    }

    /// Give the name of a service, returns the name of the machine on which that service is being hosted
    pub async fn service_host(
        &self,
        service: impl Into<String>,
    ) -> RosLibRustResult<rosapi::ServiceHostResponse> {
        self.handle
            .call_service(
                "/rosapi/service_host",
                rosapi::ServiceHostRequest {
                    service: service.into(),
                },
            )
            .await
    }

    /// Given the type of a service, returns a list of nodes which are providing services with that type
    pub async fn service_providers(
        &self,
        service_type: impl Into<String>,
    ) -> RosLibRustResult<rosapi::ServiceProvidersResponse> {
        self.handle
            .call_service(
                "/rosapi/service_providers",
                rosapi::ServiceProvidersRequest {
                    service: service_type.into(),
                },
            )
            .await
    }

    /// Given the type of a service (e.g. 'rosapi/SetParam'), returns details about the message format of the request
    pub async fn get_service_request_details(
        &self,
        service_type: impl Into<String>,
    ) -> RosLibRustResult<rosapi::ServiceRequestDetailsResponse> {
        self.handle
            .call_service(
                "/rosapi/service_request_details",
                rosapi::ServiceRequestDetailsRequest {
                    r#type: service_type.into(),
                },
            )
            .await
    }

    /// Given the type of a service (e.g. 'rosapi/SetParam'), returns details about the message format of the response
    pub async fn get_service_response_details(
        &self,
        service_type: impl Into<String>,
    ) -> RosLibRustResult<rosapi::ServiceRequestDetailsResponse> {
        self.handle
            .call_service(
                "/rosapi/service_response_details",
                rosapi::ServiceRequestDetailsRequest {
                    r#type: service_type.into(),
                },
            )
            .await
    }

    /// Given the name of a service (e.g. '/rosapi/publishers'), returns the type of the service ('rosapi/Publishers')
    pub async fn get_service_type(
        &self,
        service_name: impl Into<String>,
    ) -> RosLibRustResult<rosapi::ServiceTypeResponse> {
        self.handle
            .call_service(
                "/rosapi/service_type",
                rosapi::ServiceTypeRequest {
                    service: service_name.into(),
                },
            )
            .await
    }

    /// Get the list of services active on the system
    pub async fn get_services(&self) -> RosLibRustResult<rosapi::ServicesResponse> {
        self.handle
            .call_service("/rosapi/services", rosapi::ServicesRequest {})
            .await
    }

    /*
     List of rosapi services pulled from `rosservice list`
     /rosapi/action_servers - Probably won't support
     /rosapi/delete_param - Done
     /rosapi/get_loggers - ??
     /rosapi/get_param - Done
     /rosapi/get_param_names - Done
     /rosapi/get_time - Done
     /rosapi/has_param - Done
     /rosapi/message_details - Done
     /rosapi/node_details - Done
     /rosapi/nodes - Done
     /rosapi/publishers - Done
     /rosapi/search_param - ?? What does this do and why?
     /rosapi/service_host - Done
     /rosapi/service_node - Done
     /rosapi/service_providers - Done
     /rosapi/service_request_details - Done
     /rosapi/service_response_details - Done
     /rosapi/service_type - Done
     /rosapi/services - TODO
     /rosapi/services_for_type - Done
     /rosapi/set_logger_level - ??
     /rosapi/set_param - Done
     /rosapi/subscribers - Done
     /rosapi/topic_type - Done
     /rosapi/topics - Done
     /rosapi/topics_and_raw_types - ??
     /rosapi/topics_for_type - Done
    */
}

#[cfg(test)]
#[cfg(feature = "running_bridge")]
mod test {
    use super::RosApi;
    use crate::{ClientHandle, ClientHandleOptions};

    async fn fixture_client() -> RosApi {
        let opts = ClientHandleOptions::new("ws://localhost:9090")
            .timeout(std::time::Duration::from_millis(200));
        let client = ClientHandle::new_with_options(opts).await.unwrap();
        RosApi::new(client)
    }

    #[tokio::test]
    async fn rosapi_get_time() {
        let api = fixture_client().await;
        api.get_time().await.unwrap();
    }

    #[tokio::test]
    async fn rosapi_get_topic_type() {
        let api = fixture_client().await;
        let res = api
            .get_topic_type("/rosout")
            .await
            .expect("Failed to get topic type for rosout");
        assert_eq!(res.r#type, "rosgraph_msgs/Log");
    }

    #[tokio::test]
    async fn rosapi_get_topics_for_type() {
        let api = fixture_client().await;
        let res = api
            .get_topics_for_type("rosgraph_msgs/Log")
            .await
            .expect("Failed to get topics for type Log");
        assert!(res.topics.iter().any(|f| f == "/rosout"));
    }

    #[tokio::test]
    async fn rosapi_get_nodes() {
        let api = fixture_client().await;
        let nodes = api.get_nodes().await.expect("Failed to get nodes");
        assert!(nodes.nodes.iter().any(|f| f == "/rosapi"));
    }

    #[tokio::test]
    async fn rosapi_get_node_details() {
        let api = fixture_client().await;
        assert!(
            api.get_node_details("/rosapi")
                .await
                .expect("Failed to get node details for rosapi")
                .services
                .len()
                > 0
        );
    }

    #[tokio::test]
    async fn rosapi_get_node_for_service() {
        let api = fixture_client().await;
        assert_eq!(
            api.get_node_for_service("/rosapi/service_node")
                .await
                .expect("Failed to call service_node")
                .node,
            "/rosapi"
        );
    }

    #[tokio::test]
    async fn rosapi_param_roundtrip() {
        let api = fixture_client().await;
        const PARAM_NAME: &'static str = "/rosapi_param_roundtrip";
        // Set the parameter
        api.set_param(PARAM_NAME, 1.0.to_string())
            .await
            .expect("Failed to set");

        // read back the value we set
        let response = api
            .get_param(PARAM_NAME)
            .await
            .expect("Failed to read param back");
        assert_eq!(1.0, response.value.parse::<f64>().unwrap());

        // Confirm the parameter is in the list of parameters
        let param_names = api
            .get_param_names()
            .await
            .expect("Failed to get param names");
        assert!(param_names.names.contains(&PARAM_NAME.to_string()));

        // Confirm has_param sees it
        assert!(api.has_param(PARAM_NAME).await.unwrap().exists);

        // Delete it!
        api.delete_param(PARAM_NAME).await.unwrap();

        // Confirm it is gone
        assert!(!api.has_param(PARAM_NAME).await.unwrap().exists);
    }

    #[tokio::test]
    async fn rosapi_message_details() {
        let api = fixture_client().await;
        let response = api.message_details("std_msgs/Header").await.unwrap();
        // Spot check we actually got some data back
        assert!(response
            .typedefs
            .iter()
            .any(|t| t.fieldtypes.contains(&"time".to_string())));
    }

    #[tokio::test]
    async fn rosapi_publishers() {
        let api = fixture_client().await;
        let response = api.publishers("/rosout").await.unwrap();
        assert!(response.publishers.iter().any(|p| p == "/rosapi"));
    }

    #[tokio::test]
    async fn rosapi_service_providers() {
        let api = fixture_client().await;
        let response = api.service_providers("rosapi/ServiceHost").await.unwrap();
        assert!(response.providers.iter().any(|p| p == "/rosapi"));
    }

    #[tokio::test]
    async fn rosapi_service_request_details() {
        let api = fixture_client().await;
        let response = api
            .get_service_request_details("rosapi/SetParam")
            .await
            .unwrap();
        // Spot check we got some data back
        assert!(response.typedefs[0].fieldnames.len() == 2);
    }

    #[tokio::test]
    async fn rosapi_service_response_details() {
        let api = fixture_client().await;
        let response = api
            .get_service_response_details("rosapi/GetParam")
            .await
            .unwrap();
        // Spot check we got some data back
        assert_eq!(response.typedefs[0].fieldnames[0], "value");
    }

    #[tokio::test]
    async fn rosapi_get_service_type() {
        let api = fixture_client().await;
        let response = api.get_service_type("/rosapi/node_details").await.unwrap();
        assert_eq!(response.r#type, "rosapi/NodeDetails");
    }

    #[tokio::test]
    async fn rosapi_services() {
        let api = fixture_client().await;
        let response = api.get_services().await.unwrap();
        assert!(!response.services.is_empty());
    }
}
