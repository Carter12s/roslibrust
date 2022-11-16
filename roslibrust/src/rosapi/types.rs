//! This module is a copy of the auto generate types of rosapi
//! It is left here due to the inability of this crate to use its own proc macro
//! It will be remove once re-org is completed.

#[cfg(feature = "rosapi")]
#[allow(unused_imports)]
pub mod rosapi {

    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct DeleteParamRequest {
        pub r#name: std::string::String,
    }
    impl crate::RosMessageType for DeleteParamRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/DeleteParamRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct DeleteParamResponse {}
    impl crate::RosMessageType for DeleteParamResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/DeleteParamResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct GetActionServersRequest {}
    impl crate::RosMessageType for GetActionServersRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetActionServersRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct GetActionServersResponse {
        pub r#action_servers: std::vec::Vec<std::string::String>,
    }
    impl crate::RosMessageType for GetActionServersResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetActionServersResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct GetParamNamesRequest {}
    impl crate::RosMessageType for GetParamNamesRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetParamNamesRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct GetParamNamesResponse {
        pub r#names: std::vec::Vec<std::string::String>,
    }
    impl crate::RosMessageType for GetParamNamesResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetParamNamesResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct GetParamRequest {
        pub r#name: std::string::String,
        pub r#default: std::string::String,
    }
    impl crate::RosMessageType for GetParamRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetParamRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct GetParamResponse {
        pub r#value: std::string::String,
    }
    impl crate::RosMessageType for GetParamResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetParamResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct GetTimeRequest {}
    impl crate::RosMessageType for GetTimeRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetTimeRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct GetTimeResponse {
        pub r#time: crate::integral_types::Time,
    }
    impl crate::RosMessageType for GetTimeResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetTimeResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct HasParamRequest {
        pub r#name: std::string::String,
    }
    impl crate::RosMessageType for HasParamRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/HasParamRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct HasParamResponse {
        pub r#exists: bool,
    }
    impl crate::RosMessageType for HasParamResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/HasParamResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct MessageDetailsRequest {
        pub r#type: std::string::String,
    }
    impl crate::RosMessageType for MessageDetailsRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/MessageDetailsRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct MessageDetailsResponse {
        pub r#typedefs: std::vec::Vec<self::TypeDef>,
    }
    impl crate::RosMessageType for MessageDetailsResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/MessageDetailsResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct NodeDetailsRequest {
        pub r#node: std::string::String,
    }
    impl crate::RosMessageType for NodeDetailsRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/NodeDetailsRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct NodeDetailsResponse {
        pub r#subscribing: std::vec::Vec<std::string::String>,
        pub r#publishing: std::vec::Vec<std::string::String>,
        pub r#services: std::vec::Vec<std::string::String>,
    }
    impl crate::RosMessageType for NodeDetailsResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/NodeDetailsResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct NodesRequest {}
    impl crate::RosMessageType for NodesRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/NodesRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct NodesResponse {
        pub r#nodes: std::vec::Vec<std::string::String>,
    }
    impl crate::RosMessageType for NodesResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/NodesResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct PublishersRequest {
        pub r#topic: std::string::String,
    }
    impl crate::RosMessageType for PublishersRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/PublishersRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct PublishersResponse {
        pub r#publishers: std::vec::Vec<std::string::String>,
    }
    impl crate::RosMessageType for PublishersResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/PublishersResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct SearchParamRequest {
        pub r#name: std::string::String,
    }
    impl crate::RosMessageType for SearchParamRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/SearchParamRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct SearchParamResponse {
        pub r#global_name: std::string::String,
    }
    impl crate::RosMessageType for SearchParamResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/SearchParamResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct ServiceHostRequest {
        pub r#service: std::string::String,
    }
    impl crate::RosMessageType for ServiceHostRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceHostRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct ServiceHostResponse {
        pub r#host: std::string::String,
    }
    impl crate::RosMessageType for ServiceHostResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceHostResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct ServiceNodeRequest {
        pub r#service: std::string::String,
    }
    impl crate::RosMessageType for ServiceNodeRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceNodeRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct ServiceNodeResponse {
        pub r#node: std::string::String,
    }
    impl crate::RosMessageType for ServiceNodeResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceNodeResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct ServiceProvidersRequest {
        pub r#service: std::string::String,
    }
    impl crate::RosMessageType for ServiceProvidersRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceProvidersRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct ServiceProvidersResponse {
        pub r#providers: std::vec::Vec<std::string::String>,
    }
    impl crate::RosMessageType for ServiceProvidersResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceProvidersResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct ServiceRequestDetailsRequest {
        pub r#type: std::string::String,
    }
    impl crate::RosMessageType for ServiceRequestDetailsRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceRequestDetailsRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct ServiceRequestDetailsResponse {
        pub r#typedefs: std::vec::Vec<self::TypeDef>,
    }
    impl crate::RosMessageType for ServiceRequestDetailsResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceRequestDetailsResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct ServiceResponseDetailsRequest {
        pub r#type: std::string::String,
    }
    impl crate::RosMessageType for ServiceResponseDetailsRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceResponseDetailsRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct ServiceResponseDetailsResponse {
        pub r#typedefs: std::vec::Vec<self::TypeDef>,
    }
    impl crate::RosMessageType for ServiceResponseDetailsResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceResponseDetailsResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct ServiceTypeRequest {
        pub r#service: std::string::String,
    }
    impl crate::RosMessageType for ServiceTypeRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceTypeRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct ServiceTypeResponse {
        pub r#type: std::string::String,
    }
    impl crate::RosMessageType for ServiceTypeResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceTypeResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct ServicesForTypeRequest {
        pub r#type: std::string::String,
    }
    impl crate::RosMessageType for ServicesForTypeRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServicesForTypeRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct ServicesForTypeResponse {
        pub r#services: std::vec::Vec<std::string::String>,
    }
    impl crate::RosMessageType for ServicesForTypeResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServicesForTypeResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct ServicesRequest {}
    impl crate::RosMessageType for ServicesRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServicesRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct ServicesResponse {
        pub r#services: std::vec::Vec<std::string::String>,
    }
    impl crate::RosMessageType for ServicesResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServicesResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct SetParamRequest {
        pub r#name: std::string::String,
        pub r#value: std::string::String,
    }
    impl crate::RosMessageType for SetParamRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/SetParamRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct SetParamResponse {}
    impl crate::RosMessageType for SetParamResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/SetParamResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct SubscribersRequest {
        pub r#topic: std::string::String,
    }
    impl crate::RosMessageType for SubscribersRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/SubscribersRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct SubscribersResponse {
        pub r#subscribers: std::vec::Vec<std::string::String>,
    }
    impl crate::RosMessageType for SubscribersResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/SubscribersResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct TopicTypeRequest {
        pub r#topic: std::string::String,
    }
    impl crate::RosMessageType for TopicTypeRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/TopicTypeRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct TopicTypeResponse {
        pub r#type: std::string::String,
    }
    impl crate::RosMessageType for TopicTypeResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/TopicTypeResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct TopicsAndRawTypesRequest {}
    impl crate::RosMessageType for TopicsAndRawTypesRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/TopicsAndRawTypesRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct TopicsAndRawTypesResponse {
        pub r#topics: std::vec::Vec<std::string::String>,
        pub r#types: std::vec::Vec<std::string::String>,
        pub r#typedefs_full_text: std::vec::Vec<std::string::String>,
    }
    impl crate::RosMessageType for TopicsAndRawTypesResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/TopicsAndRawTypesResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct TopicsForTypeRequest {
        pub r#type: std::string::String,
    }
    impl crate::RosMessageType for TopicsForTypeRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/TopicsForTypeRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct TopicsForTypeResponse {
        pub r#topics: std::vec::Vec<std::string::String>,
    }
    impl crate::RosMessageType for TopicsForTypeResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/TopicsForTypeResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct TopicsRequest {}
    impl crate::RosMessageType for TopicsRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/TopicsRequest";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct TopicsResponse {
        pub r#topics: std::vec::Vec<std::string::String>,
        pub r#types: std::vec::Vec<std::string::String>,
    }
    impl crate::RosMessageType for TopicsResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/TopicsResponse";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct TypeDef {
        pub r#type: std::string::String,
        pub r#fieldnames: std::vec::Vec<std::string::String>,
        pub r#fieldtypes: std::vec::Vec<std::string::String>,
        pub r#fieldarraylen: std::vec::Vec<i32>,
        pub r#examples: std::vec::Vec<std::string::String>,
        pub r#constnames: std::vec::Vec<std::string::String>,
        pub r#constvalues: std::vec::Vec<std::string::String>,
    }
    impl crate::RosMessageType for TypeDef {
        const ROS_TYPE_NAME: &'static str = "rosapi/TypeDef";
    }
    pub struct DeleteParam {}
    impl crate::RosServiceType for DeleteParam {
        const ROS_SERVICE_NAME: &'static str = "rosapi/DeleteParam";
        type Request = DeleteParamRequest;
        type Response = DeleteParamResponse;
    }
    pub struct GetActionServers {}
    impl crate::RosServiceType for GetActionServers {
        const ROS_SERVICE_NAME: &'static str = "rosapi/GetActionServers";
        type Request = GetActionServersRequest;
        type Response = GetActionServersResponse;
    }
    pub struct GetParam {}
    impl crate::RosServiceType for GetParam {
        const ROS_SERVICE_NAME: &'static str = "rosapi/GetParam";
        type Request = GetParamRequest;
        type Response = GetParamResponse;
    }
    pub struct GetParamNames {}
    impl crate::RosServiceType for GetParamNames {
        const ROS_SERVICE_NAME: &'static str = "rosapi/GetParamNames";
        type Request = GetParamNamesRequest;
        type Response = GetParamNamesResponse;
    }
    pub struct GetTime {}
    impl crate::RosServiceType for GetTime {
        const ROS_SERVICE_NAME: &'static str = "rosapi/GetTime";
        type Request = GetTimeRequest;
        type Response = GetTimeResponse;
    }
    pub struct HasParam {}
    impl crate::RosServiceType for HasParam {
        const ROS_SERVICE_NAME: &'static str = "rosapi/HasParam";
        type Request = HasParamRequest;
        type Response = HasParamResponse;
    }
    pub struct MessageDetails {}
    impl crate::RosServiceType for MessageDetails {
        const ROS_SERVICE_NAME: &'static str = "rosapi/MessageDetails";
        type Request = MessageDetailsRequest;
        type Response = MessageDetailsResponse;
    }
    pub struct NodeDetails {}
    impl crate::RosServiceType for NodeDetails {
        const ROS_SERVICE_NAME: &'static str = "rosapi/NodeDetails";
        type Request = NodeDetailsRequest;
        type Response = NodeDetailsResponse;
    }
    pub struct Nodes {}
    impl crate::RosServiceType for Nodes {
        const ROS_SERVICE_NAME: &'static str = "rosapi/Nodes";
        type Request = NodesRequest;
        type Response = NodesResponse;
    }
    pub struct Publishers {}
    impl crate::RosServiceType for Publishers {
        const ROS_SERVICE_NAME: &'static str = "rosapi/Publishers";
        type Request = PublishersRequest;
        type Response = PublishersResponse;
    }
    pub struct SearchParam {}
    impl crate::RosServiceType for SearchParam {
        const ROS_SERVICE_NAME: &'static str = "rosapi/SearchParam";
        type Request = SearchParamRequest;
        type Response = SearchParamResponse;
    }
    pub struct ServiceHost {}
    impl crate::RosServiceType for ServiceHost {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceHost";
        type Request = ServiceHostRequest;
        type Response = ServiceHostResponse;
    }
    pub struct ServiceNode {}
    impl crate::RosServiceType for ServiceNode {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceNode";
        type Request = ServiceNodeRequest;
        type Response = ServiceNodeResponse;
    }
    pub struct ServiceProviders {}
    impl crate::RosServiceType for ServiceProviders {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceProviders";
        type Request = ServiceProvidersRequest;
        type Response = ServiceProvidersResponse;
    }
    pub struct ServiceRequestDetails {}
    impl crate::RosServiceType for ServiceRequestDetails {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceRequestDetails";
        type Request = ServiceRequestDetailsRequest;
        type Response = ServiceRequestDetailsResponse;
    }
    pub struct ServiceResponseDetails {}
    impl crate::RosServiceType for ServiceResponseDetails {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceResponseDetails";
        type Request = ServiceResponseDetailsRequest;
        type Response = ServiceResponseDetailsResponse;
    }
    pub struct ServiceType {}
    impl crate::RosServiceType for ServiceType {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceType";
        type Request = ServiceTypeRequest;
        type Response = ServiceTypeResponse;
    }
    pub struct Services {}
    impl crate::RosServiceType for Services {
        const ROS_SERVICE_NAME: &'static str = "rosapi/Services";
        type Request = ServicesRequest;
        type Response = ServicesResponse;
    }
    pub struct ServicesForType {}
    impl crate::RosServiceType for ServicesForType {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServicesForType";
        type Request = ServicesForTypeRequest;
        type Response = ServicesForTypeResponse;
    }
    pub struct SetParam {}
    impl crate::RosServiceType for SetParam {
        const ROS_SERVICE_NAME: &'static str = "rosapi/SetParam";
        type Request = SetParamRequest;
        type Response = SetParamResponse;
    }
    pub struct Subscribers {}
    impl crate::RosServiceType for Subscribers {
        const ROS_SERVICE_NAME: &'static str = "rosapi/Subscribers";
        type Request = SubscribersRequest;
        type Response = SubscribersResponse;
    }
    pub struct TopicType {}
    impl crate::RosServiceType for TopicType {
        const ROS_SERVICE_NAME: &'static str = "rosapi/TopicType";
        type Request = TopicTypeRequest;
        type Response = TopicTypeResponse;
    }
    pub struct Topics {}
    impl crate::RosServiceType for Topics {
        const ROS_SERVICE_NAME: &'static str = "rosapi/Topics";
        type Request = TopicsRequest;
        type Response = TopicsResponse;
    }
    pub struct TopicsAndRawTypes {}
    impl crate::RosServiceType for TopicsAndRawTypes {
        const ROS_SERVICE_NAME: &'static str = "rosapi/TopicsAndRawTypes";
        type Request = TopicsAndRawTypesRequest;
        type Response = TopicsAndRawTypesResponse;
    }
    pub struct TopicsForType {}
    impl crate::RosServiceType for TopicsForType {
        const ROS_SERVICE_NAME: &'static str = "rosapi/TopicsForType";
        type Request = TopicsForTypeRequest;
        type Response = TopicsForTypeResponse;
    }
}
