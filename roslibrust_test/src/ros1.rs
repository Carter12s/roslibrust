#[allow(unused_imports)]
pub mod actionlib_msgs {
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct GoalID {
        pub r#stamp: ::roslibrust_codegen::integral_types::Time,
        pub r#id: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for GoalID {
        const ROS_TYPE_NAME: &'static str = "actionlib_msgs/GoalID";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct GoalStatus {
        pub r#goal_id: self::GoalID,
        pub r#status: u8,
        pub r#text: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for GoalStatus {
        const ROS_TYPE_NAME: &'static str = "actionlib_msgs/GoalStatus";
    }
    impl GoalStatus {
        pub const r#PENDING: u8 = 0;
        pub const r#ACTIVE: u8 = 1;
        pub const r#PREEMPTED: u8 = 2;
        pub const r#SUCCEEDED: u8 = 3;
        pub const r#ABORTED: u8 = 4;
        pub const r#REJECTED: u8 = 5;
        pub const r#PREEMPTING: u8 = 6;
        pub const r#RECALLING: u8 = 7;
        pub const r#RECALLED: u8 = 8;
        pub const r#LOST: u8 = 9;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct GoalStatusArray {
        pub r#header: std_msgs::Header,
        pub r#status_list: ::std::vec::Vec<self::GoalStatus>,
    }
    impl ::roslibrust_codegen::RosMessageType for GoalStatusArray {
        const ROS_TYPE_NAME: &'static str = "actionlib_msgs/GoalStatusArray";
    }
}
#[allow(unused_imports)]
pub mod diagnostic_msgs {
    use super::actionlib_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct AddDiagnosticsRequest {
        pub r#load_namespace: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for AddDiagnosticsRequest {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/AddDiagnosticsRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct AddDiagnosticsResponse {
        pub r#success: bool,
        pub r#message: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for AddDiagnosticsResponse {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/AddDiagnosticsResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct DiagnosticArray {
        pub r#header: std_msgs::Header,
        pub r#status: ::std::vec::Vec<self::DiagnosticStatus>,
    }
    impl ::roslibrust_codegen::RosMessageType for DiagnosticArray {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/DiagnosticArray";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct DiagnosticStatus {
        pub r#level: u8,
        pub r#name: ::std::string::String,
        pub r#message: ::std::string::String,
        pub r#hardware_id: ::std::string::String,
        pub r#values: ::std::vec::Vec<self::KeyValue>,
    }
    impl ::roslibrust_codegen::RosMessageType for DiagnosticStatus {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/DiagnosticStatus";
    }
    impl DiagnosticStatus {
        pub const r#OK: u8 = 0;
        pub const r#WARN: u8 = 1;
        pub const r#ERROR: u8 = 2;
        pub const r#STALE: u8 = 3;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct KeyValue {
        pub r#key: ::std::string::String,
        pub r#value: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for KeyValue {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/KeyValue";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct SelfTestRequest {}
    impl ::roslibrust_codegen::RosMessageType for SelfTestRequest {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/SelfTestRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct SelfTestResponse {
        pub r#id: ::std::string::String,
        pub r#passed: u8,
        pub r#status: ::std::vec::Vec<self::DiagnosticStatus>,
    }
    impl ::roslibrust_codegen::RosMessageType for SelfTestResponse {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/SelfTestResponse";
    }
    pub struct AddDiagnostics {}
    impl ::roslibrust_codegen::RosServiceType for AddDiagnostics {
        const ROS_SERVICE_NAME: &'static str = "diagnostic_msgs/AddDiagnostics";
        type Request = AddDiagnosticsRequest;
        type Response = AddDiagnosticsResponse;
    }
    pub struct SelfTest {}
    impl ::roslibrust_codegen::RosServiceType for SelfTest {
        const ROS_SERVICE_NAME: &'static str = "diagnostic_msgs/SelfTest";
        type Request = SelfTestRequest;
        type Response = SelfTestResponse;
    }
}
#[allow(unused_imports)]
pub mod geometry_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Accel {
        pub r#linear: self::Vector3,
        pub r#angular: self::Vector3,
    }
    impl ::roslibrust_codegen::RosMessageType for Accel {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Accel";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct AccelStamped {
        pub r#header: std_msgs::Header,
        pub r#accel: self::Accel,
    }
    impl ::roslibrust_codegen::RosMessageType for AccelStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/AccelStamped";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct AccelWithCovariance {
        pub r#accel: self::Accel,
        pub r#covariance: ::std::vec::Vec<f64>,
    }
    impl ::roslibrust_codegen::RosMessageType for AccelWithCovariance {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/AccelWithCovariance";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct AccelWithCovarianceStamped {
        pub r#header: std_msgs::Header,
        pub r#accel: self::AccelWithCovariance,
    }
    impl ::roslibrust_codegen::RosMessageType for AccelWithCovarianceStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/AccelWithCovarianceStamped";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Inertia {
        pub r#m: f64,
        pub r#com: self::Vector3,
        pub r#ixx: f64,
        pub r#ixy: f64,
        pub r#ixz: f64,
        pub r#iyy: f64,
        pub r#iyz: f64,
        pub r#izz: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for Inertia {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Inertia";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct InertiaStamped {
        pub r#header: std_msgs::Header,
        pub r#inertia: self::Inertia,
    }
    impl ::roslibrust_codegen::RosMessageType for InertiaStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/InertiaStamped";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Point {
        pub r#x: f64,
        pub r#y: f64,
        pub r#z: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for Point {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Point";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Point32 {
        pub r#x: f32,
        pub r#y: f32,
        pub r#z: f32,
    }
    impl ::roslibrust_codegen::RosMessageType for Point32 {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Point32";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct PointStamped {
        pub r#header: std_msgs::Header,
        pub r#point: self::Point,
    }
    impl ::roslibrust_codegen::RosMessageType for PointStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PointStamped";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Polygon {
        pub r#points: ::std::vec::Vec<self::Point32>,
    }
    impl ::roslibrust_codegen::RosMessageType for Polygon {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Polygon";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct PolygonStamped {
        pub r#header: std_msgs::Header,
        pub r#polygon: self::Polygon,
    }
    impl ::roslibrust_codegen::RosMessageType for PolygonStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PolygonStamped";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Pose {
        pub r#position: self::Point,
        pub r#orientation: self::Quaternion,
    }
    impl ::roslibrust_codegen::RosMessageType for Pose {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Pose";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Pose2D {
        pub r#x: f64,
        pub r#y: f64,
        pub r#theta: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for Pose2D {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Pose2D";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct PoseArray {
        pub r#header: std_msgs::Header,
        pub r#poses: ::std::vec::Vec<self::Pose>,
    }
    impl ::roslibrust_codegen::RosMessageType for PoseArray {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseArray";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct PoseStamped {
        pub r#header: std_msgs::Header,
        pub r#pose: self::Pose,
    }
    impl ::roslibrust_codegen::RosMessageType for PoseStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseStamped";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct PoseWithCovariance {
        pub r#pose: self::Pose,
        pub r#covariance: ::std::vec::Vec<f64>,
    }
    impl ::roslibrust_codegen::RosMessageType for PoseWithCovariance {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseWithCovariance";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct PoseWithCovarianceStamped {
        pub r#header: std_msgs::Header,
        pub r#pose: self::PoseWithCovariance,
    }
    impl ::roslibrust_codegen::RosMessageType for PoseWithCovarianceStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseWithCovarianceStamped";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Quaternion {
        pub r#x: f64,
        pub r#y: f64,
        pub r#z: f64,
        pub r#w: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for Quaternion {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Quaternion";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct QuaternionStamped {
        pub r#header: std_msgs::Header,
        pub r#quaternion: self::Quaternion,
    }
    impl ::roslibrust_codegen::RosMessageType for QuaternionStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/QuaternionStamped";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Transform {
        pub r#translation: self::Vector3,
        pub r#rotation: self::Quaternion,
    }
    impl ::roslibrust_codegen::RosMessageType for Transform {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Transform";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct TransformStamped {
        pub r#header: std_msgs::Header,
        pub r#child_frame_id: ::std::string::String,
        pub r#transform: self::Transform,
    }
    impl ::roslibrust_codegen::RosMessageType for TransformStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/TransformStamped";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Twist {
        pub r#linear: self::Vector3,
        pub r#angular: self::Vector3,
    }
    impl ::roslibrust_codegen::RosMessageType for Twist {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Twist";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct TwistStamped {
        pub r#header: std_msgs::Header,
        pub r#twist: self::Twist,
    }
    impl ::roslibrust_codegen::RosMessageType for TwistStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/TwistStamped";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct TwistWithCovariance {
        pub r#twist: self::Twist,
        pub r#covariance: ::std::vec::Vec<f64>,
    }
    impl ::roslibrust_codegen::RosMessageType for TwistWithCovariance {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/TwistWithCovariance";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct TwistWithCovarianceStamped {
        pub r#header: std_msgs::Header,
        pub r#twist: self::TwistWithCovariance,
    }
    impl ::roslibrust_codegen::RosMessageType for TwistWithCovarianceStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/TwistWithCovarianceStamped";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Vector3 {
        pub r#x: f64,
        pub r#y: f64,
        pub r#z: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for Vector3 {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Vector3";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Vector3Stamped {
        pub r#header: std_msgs::Header,
        pub r#vector: self::Vector3,
    }
    impl ::roslibrust_codegen::RosMessageType for Vector3Stamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Vector3Stamped";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Wrench {
        pub r#force: self::Vector3,
        pub r#torque: self::Vector3,
    }
    impl ::roslibrust_codegen::RosMessageType for Wrench {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Wrench";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct WrenchStamped {
        pub r#header: std_msgs::Header,
        pub r#wrench: self::Wrench,
    }
    impl ::roslibrust_codegen::RosMessageType for WrenchStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/WrenchStamped";
    }
}
#[allow(unused_imports)]
pub mod nav_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct GetMapRequest {}
    impl ::roslibrust_codegen::RosMessageType for GetMapRequest {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetMapRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct GetMapResponse {
        pub r#map: self::OccupancyGrid,
    }
    impl ::roslibrust_codegen::RosMessageType for GetMapResponse {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetMapResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct GetPlanRequest {
        pub r#start: geometry_msgs::PoseStamped,
        pub r#goal: geometry_msgs::PoseStamped,
        pub r#tolerance: f32,
    }
    impl ::roslibrust_codegen::RosMessageType for GetPlanRequest {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetPlanRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct GetPlanResponse {
        pub r#plan: self::Path,
    }
    impl ::roslibrust_codegen::RosMessageType for GetPlanResponse {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetPlanResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct GridCells {
        pub r#header: std_msgs::Header,
        pub r#cell_width: f32,
        pub r#cell_height: f32,
        pub r#cells: ::std::vec::Vec<geometry_msgs::Point>,
    }
    impl ::roslibrust_codegen::RosMessageType for GridCells {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GridCells";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct LoadMapRequest {
        pub r#map_url: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for LoadMapRequest {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/LoadMapRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct LoadMapResponse {
        pub r#map: self::OccupancyGrid,
        pub r#result: u8,
    }
    impl ::roslibrust_codegen::RosMessageType for LoadMapResponse {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/LoadMapResponse";
    }
    impl LoadMapResponse {
        pub const r#RESULT_SUCCESS: u8 = 0;
        pub const r#RESULT_MAP_DOES_NOT_EXIST: u8 = 1;
        pub const r#RESULT_INVALID_MAP_DATA: u8 = 2;
        pub const r#RESULT_INVALID_MAP_METADATA: u8 = 3;
        pub const r#RESULT_UNDEFINED_FAILURE: u8 = 255;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct MapMetaData {
        pub r#map_load_time: ::roslibrust_codegen::integral_types::Time,
        pub r#resolution: f32,
        pub r#width: u32,
        pub r#height: u32,
        pub r#origin: geometry_msgs::Pose,
    }
    impl ::roslibrust_codegen::RosMessageType for MapMetaData {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/MapMetaData";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct OccupancyGrid {
        pub r#header: std_msgs::Header,
        pub r#info: self::MapMetaData,
        pub r#data: ::std::vec::Vec<i8>,
    }
    impl ::roslibrust_codegen::RosMessageType for OccupancyGrid {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/OccupancyGrid";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Odometry {
        pub r#header: std_msgs::Header,
        pub r#child_frame_id: ::std::string::String,
        pub r#pose: geometry_msgs::PoseWithCovariance,
        pub r#twist: geometry_msgs::TwistWithCovariance,
    }
    impl ::roslibrust_codegen::RosMessageType for Odometry {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/Odometry";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Path {
        pub r#header: std_msgs::Header,
        pub r#poses: ::std::vec::Vec<geometry_msgs::PoseStamped>,
    }
    impl ::roslibrust_codegen::RosMessageType for Path {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/Path";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct SetMapRequest {
        pub r#map: self::OccupancyGrid,
        pub r#initial_pose: geometry_msgs::PoseWithCovarianceStamped,
    }
    impl ::roslibrust_codegen::RosMessageType for SetMapRequest {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/SetMapRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct SetMapResponse {
        pub r#success: bool,
    }
    impl ::roslibrust_codegen::RosMessageType for SetMapResponse {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/SetMapResponse";
    }
    pub struct GetMap {}
    impl ::roslibrust_codegen::RosServiceType for GetMap {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/GetMap";
        type Request = GetMapRequest;
        type Response = GetMapResponse;
    }
    pub struct GetPlan {}
    impl ::roslibrust_codegen::RosServiceType for GetPlan {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/GetPlan";
        type Request = GetPlanRequest;
        type Response = GetPlanResponse;
    }
    pub struct LoadMap {}
    impl ::roslibrust_codegen::RosServiceType for LoadMap {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/LoadMap";
        type Request = LoadMapRequest;
        type Response = LoadMapResponse;
    }
    pub struct SetMap {}
    impl ::roslibrust_codegen::RosServiceType for SetMap {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/SetMap";
        type Request = SetMapRequest;
        type Response = SetMapResponse;
    }
}
#[allow(unused_imports)]
pub mod rosapi {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct DeleteParamRequest {
        pub r#name: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for DeleteParamRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/DeleteParamRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct DeleteParamResponse {}
    impl ::roslibrust_codegen::RosMessageType for DeleteParamResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/DeleteParamResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct GetActionServersRequest {}
    impl ::roslibrust_codegen::RosMessageType for GetActionServersRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetActionServersRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct GetActionServersResponse {
        pub r#action_servers: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for GetActionServersResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetActionServersResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct GetParamNamesRequest {}
    impl ::roslibrust_codegen::RosMessageType for GetParamNamesRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetParamNamesRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct GetParamNamesResponse {
        pub r#names: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for GetParamNamesResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetParamNamesResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct GetParamRequest {
        pub r#name: ::std::string::String,
        pub r#default: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for GetParamRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetParamRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct GetParamResponse {
        pub r#value: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for GetParamResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetParamResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct GetTimeRequest {}
    impl ::roslibrust_codegen::RosMessageType for GetTimeRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetTimeRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct GetTimeResponse {
        pub r#time: ::roslibrust_codegen::integral_types::Time,
    }
    impl ::roslibrust_codegen::RosMessageType for GetTimeResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetTimeResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct HasParamRequest {
        pub r#name: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for HasParamRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/HasParamRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct HasParamResponse {
        pub r#exists: bool,
    }
    impl ::roslibrust_codegen::RosMessageType for HasParamResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/HasParamResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct MessageDetailsRequest {
        pub r#type: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for MessageDetailsRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/MessageDetailsRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct MessageDetailsResponse {
        pub r#typedefs: ::std::vec::Vec<self::TypeDef>,
    }
    impl ::roslibrust_codegen::RosMessageType for MessageDetailsResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/MessageDetailsResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct NodeDetailsRequest {
        pub r#node: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for NodeDetailsRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/NodeDetailsRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct NodeDetailsResponse {
        pub r#subscribing: ::std::vec::Vec<::std::string::String>,
        pub r#publishing: ::std::vec::Vec<::std::string::String>,
        pub r#services: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for NodeDetailsResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/NodeDetailsResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct NodesRequest {}
    impl ::roslibrust_codegen::RosMessageType for NodesRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/NodesRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct NodesResponse {
        pub r#nodes: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for NodesResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/NodesResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct PublishersRequest {
        pub r#topic: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for PublishersRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/PublishersRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct PublishersResponse {
        pub r#publishers: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for PublishersResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/PublishersResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct SearchParamRequest {
        pub r#name: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for SearchParamRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/SearchParamRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct SearchParamResponse {
        pub r#global_name: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for SearchParamResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/SearchParamResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct ServiceHostRequest {
        pub r#service: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for ServiceHostRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceHostRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct ServiceHostResponse {
        pub r#host: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for ServiceHostResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceHostResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct ServiceNodeRequest {
        pub r#service: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for ServiceNodeRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceNodeRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct ServiceNodeResponse {
        pub r#node: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for ServiceNodeResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceNodeResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct ServiceProvidersRequest {
        pub r#service: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for ServiceProvidersRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceProvidersRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct ServiceProvidersResponse {
        pub r#providers: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for ServiceProvidersResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceProvidersResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct ServiceRequestDetailsRequest {
        pub r#type: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for ServiceRequestDetailsRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceRequestDetailsRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct ServiceRequestDetailsResponse {
        pub r#typedefs: ::std::vec::Vec<self::TypeDef>,
    }
    impl ::roslibrust_codegen::RosMessageType for ServiceRequestDetailsResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceRequestDetailsResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct ServiceResponseDetailsRequest {
        pub r#type: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for ServiceResponseDetailsRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceResponseDetailsRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct ServiceResponseDetailsResponse {
        pub r#typedefs: ::std::vec::Vec<self::TypeDef>,
    }
    impl ::roslibrust_codegen::RosMessageType for ServiceResponseDetailsResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceResponseDetailsResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct ServiceTypeRequest {
        pub r#service: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for ServiceTypeRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceTypeRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct ServiceTypeResponse {
        pub r#type: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for ServiceTypeResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServiceTypeResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct ServicesForTypeRequest {
        pub r#type: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for ServicesForTypeRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServicesForTypeRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct ServicesForTypeResponse {
        pub r#services: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for ServicesForTypeResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServicesForTypeResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct ServicesRequest {}
    impl ::roslibrust_codegen::RosMessageType for ServicesRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServicesRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct ServicesResponse {
        pub r#services: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for ServicesResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServicesResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct SetParamRequest {
        pub r#name: ::std::string::String,
        pub r#value: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for SetParamRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/SetParamRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct SetParamResponse {}
    impl ::roslibrust_codegen::RosMessageType for SetParamResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/SetParamResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct SubscribersRequest {
        pub r#topic: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for SubscribersRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/SubscribersRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct SubscribersResponse {
        pub r#subscribers: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for SubscribersResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/SubscribersResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct TopicTypeRequest {
        pub r#topic: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for TopicTypeRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/TopicTypeRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct TopicTypeResponse {
        pub r#type: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for TopicTypeResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/TopicTypeResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct TopicsAndRawTypesRequest {}
    impl ::roslibrust_codegen::RosMessageType for TopicsAndRawTypesRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/TopicsAndRawTypesRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct TopicsAndRawTypesResponse {
        pub r#topics: ::std::vec::Vec<::std::string::String>,
        pub r#types: ::std::vec::Vec<::std::string::String>,
        pub r#typedefs_full_text: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for TopicsAndRawTypesResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/TopicsAndRawTypesResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct TopicsForTypeRequest {
        pub r#type: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for TopicsForTypeRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/TopicsForTypeRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct TopicsForTypeResponse {
        pub r#topics: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for TopicsForTypeResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/TopicsForTypeResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct TopicsRequest {}
    impl ::roslibrust_codegen::RosMessageType for TopicsRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/TopicsRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct TopicsResponse {
        pub r#topics: ::std::vec::Vec<::std::string::String>,
        pub r#types: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for TopicsResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/TopicsResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct TypeDef {
        pub r#type: ::std::string::String,
        pub r#fieldnames: ::std::vec::Vec<::std::string::String>,
        pub r#fieldtypes: ::std::vec::Vec<::std::string::String>,
        pub r#fieldarraylen: ::std::vec::Vec<i32>,
        pub r#examples: ::std::vec::Vec<::std::string::String>,
        pub r#constnames: ::std::vec::Vec<::std::string::String>,
        pub r#constvalues: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for TypeDef {
        const ROS_TYPE_NAME: &'static str = "rosapi/TypeDef";
    }
    pub struct DeleteParam {}
    impl ::roslibrust_codegen::RosServiceType for DeleteParam {
        const ROS_SERVICE_NAME: &'static str = "rosapi/DeleteParam";
        type Request = DeleteParamRequest;
        type Response = DeleteParamResponse;
    }
    pub struct GetActionServers {}
    impl ::roslibrust_codegen::RosServiceType for GetActionServers {
        const ROS_SERVICE_NAME: &'static str = "rosapi/GetActionServers";
        type Request = GetActionServersRequest;
        type Response = GetActionServersResponse;
    }
    pub struct GetParam {}
    impl ::roslibrust_codegen::RosServiceType for GetParam {
        const ROS_SERVICE_NAME: &'static str = "rosapi/GetParam";
        type Request = GetParamRequest;
        type Response = GetParamResponse;
    }
    pub struct GetParamNames {}
    impl ::roslibrust_codegen::RosServiceType for GetParamNames {
        const ROS_SERVICE_NAME: &'static str = "rosapi/GetParamNames";
        type Request = GetParamNamesRequest;
        type Response = GetParamNamesResponse;
    }
    pub struct GetTime {}
    impl ::roslibrust_codegen::RosServiceType for GetTime {
        const ROS_SERVICE_NAME: &'static str = "rosapi/GetTime";
        type Request = GetTimeRequest;
        type Response = GetTimeResponse;
    }
    pub struct HasParam {}
    impl ::roslibrust_codegen::RosServiceType for HasParam {
        const ROS_SERVICE_NAME: &'static str = "rosapi/HasParam";
        type Request = HasParamRequest;
        type Response = HasParamResponse;
    }
    pub struct MessageDetails {}
    impl ::roslibrust_codegen::RosServiceType for MessageDetails {
        const ROS_SERVICE_NAME: &'static str = "rosapi/MessageDetails";
        type Request = MessageDetailsRequest;
        type Response = MessageDetailsResponse;
    }
    pub struct NodeDetails {}
    impl ::roslibrust_codegen::RosServiceType for NodeDetails {
        const ROS_SERVICE_NAME: &'static str = "rosapi/NodeDetails";
        type Request = NodeDetailsRequest;
        type Response = NodeDetailsResponse;
    }
    pub struct Nodes {}
    impl ::roslibrust_codegen::RosServiceType for Nodes {
        const ROS_SERVICE_NAME: &'static str = "rosapi/Nodes";
        type Request = NodesRequest;
        type Response = NodesResponse;
    }
    pub struct Publishers {}
    impl ::roslibrust_codegen::RosServiceType for Publishers {
        const ROS_SERVICE_NAME: &'static str = "rosapi/Publishers";
        type Request = PublishersRequest;
        type Response = PublishersResponse;
    }
    pub struct SearchParam {}
    impl ::roslibrust_codegen::RosServiceType for SearchParam {
        const ROS_SERVICE_NAME: &'static str = "rosapi/SearchParam";
        type Request = SearchParamRequest;
        type Response = SearchParamResponse;
    }
    pub struct ServiceHost {}
    impl ::roslibrust_codegen::RosServiceType for ServiceHost {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceHost";
        type Request = ServiceHostRequest;
        type Response = ServiceHostResponse;
    }
    pub struct ServiceNode {}
    impl ::roslibrust_codegen::RosServiceType for ServiceNode {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceNode";
        type Request = ServiceNodeRequest;
        type Response = ServiceNodeResponse;
    }
    pub struct ServiceProviders {}
    impl ::roslibrust_codegen::RosServiceType for ServiceProviders {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceProviders";
        type Request = ServiceProvidersRequest;
        type Response = ServiceProvidersResponse;
    }
    pub struct ServiceRequestDetails {}
    impl ::roslibrust_codegen::RosServiceType for ServiceRequestDetails {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceRequestDetails";
        type Request = ServiceRequestDetailsRequest;
        type Response = ServiceRequestDetailsResponse;
    }
    pub struct ServiceResponseDetails {}
    impl ::roslibrust_codegen::RosServiceType for ServiceResponseDetails {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceResponseDetails";
        type Request = ServiceResponseDetailsRequest;
        type Response = ServiceResponseDetailsResponse;
    }
    pub struct ServiceType {}
    impl ::roslibrust_codegen::RosServiceType for ServiceType {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceType";
        type Request = ServiceTypeRequest;
        type Response = ServiceTypeResponse;
    }
    pub struct Services {}
    impl ::roslibrust_codegen::RosServiceType for Services {
        const ROS_SERVICE_NAME: &'static str = "rosapi/Services";
        type Request = ServicesRequest;
        type Response = ServicesResponse;
    }
    pub struct ServicesForType {}
    impl ::roslibrust_codegen::RosServiceType for ServicesForType {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServicesForType";
        type Request = ServicesForTypeRequest;
        type Response = ServicesForTypeResponse;
    }
    pub struct SetParam {}
    impl ::roslibrust_codegen::RosServiceType for SetParam {
        const ROS_SERVICE_NAME: &'static str = "rosapi/SetParam";
        type Request = SetParamRequest;
        type Response = SetParamResponse;
    }
    pub struct Subscribers {}
    impl ::roslibrust_codegen::RosServiceType for Subscribers {
        const ROS_SERVICE_NAME: &'static str = "rosapi/Subscribers";
        type Request = SubscribersRequest;
        type Response = SubscribersResponse;
    }
    pub struct TopicType {}
    impl ::roslibrust_codegen::RosServiceType for TopicType {
        const ROS_SERVICE_NAME: &'static str = "rosapi/TopicType";
        type Request = TopicTypeRequest;
        type Response = TopicTypeResponse;
    }
    pub struct Topics {}
    impl ::roslibrust_codegen::RosServiceType for Topics {
        const ROS_SERVICE_NAME: &'static str = "rosapi/Topics";
        type Request = TopicsRequest;
        type Response = TopicsResponse;
    }
    pub struct TopicsAndRawTypes {}
    impl ::roslibrust_codegen::RosServiceType for TopicsAndRawTypes {
        const ROS_SERVICE_NAME: &'static str = "rosapi/TopicsAndRawTypes";
        type Request = TopicsAndRawTypesRequest;
        type Response = TopicsAndRawTypesResponse;
    }
    pub struct TopicsForType {}
    impl ::roslibrust_codegen::RosServiceType for TopicsForType {
        const ROS_SERVICE_NAME: &'static str = "rosapi/TopicsForType";
        type Request = TopicsForTypeRequest;
        type Response = TopicsForTypeResponse;
    }
}
#[allow(unused_imports)]
pub mod rosgraph_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Clock {
        pub r#clock: ::roslibrust_codegen::integral_types::Time,
    }
    impl ::roslibrust_codegen::RosMessageType for Clock {
        const ROS_TYPE_NAME: &'static str = "rosgraph_msgs/Clock";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Log {
        pub r#header: std_msgs::Header,
        pub r#level: u8,
        pub r#name: ::std::string::String,
        pub r#msg: ::std::string::String,
        pub r#file: ::std::string::String,
        pub r#function: ::std::string::String,
        pub r#line: u32,
        pub r#topics: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for Log {
        const ROS_TYPE_NAME: &'static str = "rosgraph_msgs/Log";
    }
    impl Log {
        pub const r#DEBUG: u8 = 1;
        pub const r#INFO: u8 = 2;
        pub const r#WARN: u8 = 4;
        pub const r#ERROR: u8 = 8;
        pub const r#FATAL: u8 = 16;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct TopicStatistics {
        pub r#topic: ::std::string::String,
        pub r#node_pub: ::std::string::String,
        pub r#node_sub: ::std::string::String,
        pub r#window_start: ::roslibrust_codegen::integral_types::Time,
        pub r#window_stop: ::roslibrust_codegen::integral_types::Time,
        pub r#delivered_msgs: i32,
        pub r#dropped_msgs: i32,
        pub r#traffic: i32,
        pub r#period_mean: ::roslibrust_codegen::integral_types::Duration,
        pub r#period_stddev: ::roslibrust_codegen::integral_types::Duration,
        pub r#period_max: ::roslibrust_codegen::integral_types::Duration,
        pub r#stamp_age_mean: ::roslibrust_codegen::integral_types::Duration,
        pub r#stamp_age_stddev: ::roslibrust_codegen::integral_types::Duration,
        pub r#stamp_age_max: ::roslibrust_codegen::integral_types::Duration,
    }
    impl ::roslibrust_codegen::RosMessageType for TopicStatistics {
        const ROS_TYPE_NAME: &'static str = "rosgraph_msgs/TopicStatistics";
    }
}
#[allow(unused_imports)]
pub mod sensor_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct BatteryState {
        pub r#header: std_msgs::Header,
        pub r#voltage: f32,
        pub r#temperature: f32,
        pub r#current: f32,
        pub r#charge: f32,
        pub r#capacity: f32,
        pub r#design_capacity: f32,
        pub r#percentage: f32,
        pub r#power_supply_status: u8,
        pub r#power_supply_health: u8,
        pub r#power_supply_technology: u8,
        pub r#present: bool,
        pub r#cell_voltage: ::std::vec::Vec<f32>,
        pub r#cell_temperature: ::std::vec::Vec<f32>,
        pub r#location: ::std::string::String,
        pub r#serial_number: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for BatteryState {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/BatteryState";
    }
    impl BatteryState {
        pub const r#POWER_SUPPLY_STATUS_UNKNOWN: u8 = 0;
        pub const r#POWER_SUPPLY_STATUS_CHARGING: u8 = 1;
        pub const r#POWER_SUPPLY_STATUS_DISCHARGING: u8 = 2;
        pub const r#POWER_SUPPLY_STATUS_NOT_CHARGING: u8 = 3;
        pub const r#POWER_SUPPLY_STATUS_FULL: u8 = 4;
        pub const r#POWER_SUPPLY_HEALTH_UNKNOWN: u8 = 0;
        pub const r#POWER_SUPPLY_HEALTH_GOOD: u8 = 1;
        pub const r#POWER_SUPPLY_HEALTH_OVERHEAT: u8 = 2;
        pub const r#POWER_SUPPLY_HEALTH_DEAD: u8 = 3;
        pub const r#POWER_SUPPLY_HEALTH_OVERVOLTAGE: u8 = 4;
        pub const r#POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: u8 = 5;
        pub const r#POWER_SUPPLY_HEALTH_COLD: u8 = 6;
        pub const r#POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE: u8 = 7;
        pub const r#POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE: u8 = 8;
        pub const r#POWER_SUPPLY_TECHNOLOGY_UNKNOWN: u8 = 0;
        pub const r#POWER_SUPPLY_TECHNOLOGY_NIMH: u8 = 1;
        pub const r#POWER_SUPPLY_TECHNOLOGY_LION: u8 = 2;
        pub const r#POWER_SUPPLY_TECHNOLOGY_LIPO: u8 = 3;
        pub const r#POWER_SUPPLY_TECHNOLOGY_LIFE: u8 = 4;
        pub const r#POWER_SUPPLY_TECHNOLOGY_NICD: u8 = 5;
        pub const r#POWER_SUPPLY_TECHNOLOGY_LIMN: u8 = 6;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct CameraInfo {
        pub r#header: std_msgs::Header,
        pub r#height: u32,
        pub r#width: u32,
        pub r#distortion_model: ::std::string::String,
        pub r#D: ::std::vec::Vec<f64>,
        pub r#K: ::std::vec::Vec<f64>,
        pub r#R: ::std::vec::Vec<f64>,
        pub r#P: ::std::vec::Vec<f64>,
        pub r#binning_x: u32,
        pub r#binning_y: u32,
        pub r#roi: self::RegionOfInterest,
    }
    impl ::roslibrust_codegen::RosMessageType for CameraInfo {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/CameraInfo";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct ChannelFloat32 {
        pub r#name: ::std::string::String,
        pub r#values: ::std::vec::Vec<f32>,
    }
    impl ::roslibrust_codegen::RosMessageType for ChannelFloat32 {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/ChannelFloat32";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct CompressedImage {
        pub r#header: std_msgs::Header,
        pub r#format: ::std::string::String,
        pub r#data: ::std::vec::Vec<u8>,
    }
    impl ::roslibrust_codegen::RosMessageType for CompressedImage {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/CompressedImage";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct FluidPressure {
        pub r#header: std_msgs::Header,
        pub r#fluid_pressure: f64,
        pub r#variance: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for FluidPressure {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/FluidPressure";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Illuminance {
        pub r#header: std_msgs::Header,
        pub r#illuminance: f64,
        pub r#variance: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for Illuminance {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Illuminance";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Image {
        pub r#header: std_msgs::Header,
        pub r#height: u32,
        pub r#width: u32,
        pub r#encoding: ::std::string::String,
        pub r#is_bigendian: u8,
        pub r#step: u32,
        pub r#data: ::std::vec::Vec<u8>,
    }
    impl ::roslibrust_codegen::RosMessageType for Image {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Image";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Imu {
        pub r#header: std_msgs::Header,
        pub r#orientation: geometry_msgs::Quaternion,
        pub r#orientation_covariance: ::std::vec::Vec<f64>,
        pub r#angular_velocity: geometry_msgs::Vector3,
        pub r#angular_velocity_covariance: ::std::vec::Vec<f64>,
        pub r#linear_acceleration: geometry_msgs::Vector3,
        pub r#linear_acceleration_covariance: ::std::vec::Vec<f64>,
    }
    impl ::roslibrust_codegen::RosMessageType for Imu {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Imu";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct JointState {
        pub r#header: std_msgs::Header,
        pub r#name: ::std::vec::Vec<::std::string::String>,
        pub r#position: ::std::vec::Vec<f64>,
        pub r#velocity: ::std::vec::Vec<f64>,
        pub r#effort: ::std::vec::Vec<f64>,
    }
    impl ::roslibrust_codegen::RosMessageType for JointState {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/JointState";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Joy {
        pub r#header: std_msgs::Header,
        pub r#axes: ::std::vec::Vec<f32>,
        pub r#buttons: ::std::vec::Vec<i32>,
    }
    impl ::roslibrust_codegen::RosMessageType for Joy {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Joy";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct JoyFeedback {
        pub r#type: u8,
        pub r#id: u8,
        pub r#intensity: f32,
    }
    impl ::roslibrust_codegen::RosMessageType for JoyFeedback {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/JoyFeedback";
    }
    impl JoyFeedback {
        pub const r#TYPE_LED: u8 = 0;
        pub const r#TYPE_RUMBLE: u8 = 1;
        pub const r#TYPE_BUZZER: u8 = 2;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct JoyFeedbackArray {
        pub r#array: ::std::vec::Vec<self::JoyFeedback>,
    }
    impl ::roslibrust_codegen::RosMessageType for JoyFeedbackArray {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/JoyFeedbackArray";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct LaserEcho {
        pub r#echoes: ::std::vec::Vec<f32>,
    }
    impl ::roslibrust_codegen::RosMessageType for LaserEcho {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/LaserEcho";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct LaserScan {
        pub r#header: std_msgs::Header,
        pub r#angle_min: f32,
        pub r#angle_max: f32,
        pub r#angle_increment: f32,
        pub r#time_increment: f32,
        pub r#scan_time: f32,
        pub r#range_min: f32,
        pub r#range_max: f32,
        pub r#ranges: ::std::vec::Vec<f32>,
        pub r#intensities: ::std::vec::Vec<f32>,
    }
    impl ::roslibrust_codegen::RosMessageType for LaserScan {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/LaserScan";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct MagneticField {
        pub r#header: std_msgs::Header,
        pub r#magnetic_field: geometry_msgs::Vector3,
        pub r#magnetic_field_covariance: ::std::vec::Vec<f64>,
    }
    impl ::roslibrust_codegen::RosMessageType for MagneticField {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/MagneticField";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct MultiDOFJointState {
        pub r#header: std_msgs::Header,
        pub r#joint_names: ::std::vec::Vec<::std::string::String>,
        pub r#transforms: ::std::vec::Vec<geometry_msgs::Transform>,
        pub r#twist: ::std::vec::Vec<geometry_msgs::Twist>,
        pub r#wrench: ::std::vec::Vec<geometry_msgs::Wrench>,
    }
    impl ::roslibrust_codegen::RosMessageType for MultiDOFJointState {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/MultiDOFJointState";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct MultiEchoLaserScan {
        pub r#header: std_msgs::Header,
        pub r#angle_min: f32,
        pub r#angle_max: f32,
        pub r#angle_increment: f32,
        pub r#time_increment: f32,
        pub r#scan_time: f32,
        pub r#range_min: f32,
        pub r#range_max: f32,
        pub r#ranges: ::std::vec::Vec<self::LaserEcho>,
        pub r#intensities: ::std::vec::Vec<self::LaserEcho>,
    }
    impl ::roslibrust_codegen::RosMessageType for MultiEchoLaserScan {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/MultiEchoLaserScan";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct NavSatFix {
        pub r#header: std_msgs::Header,
        pub r#status: self::NavSatStatus,
        pub r#latitude: f64,
        pub r#longitude: f64,
        pub r#altitude: f64,
        pub r#position_covariance: ::std::vec::Vec<f64>,
        pub r#position_covariance_type: u8,
    }
    impl ::roslibrust_codegen::RosMessageType for NavSatFix {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/NavSatFix";
    }
    impl NavSatFix {
        pub const r#COVARIANCE_TYPE_UNKNOWN: u8 = 0;
        pub const r#COVARIANCE_TYPE_APPROXIMATED: u8 = 1;
        pub const r#COVARIANCE_TYPE_DIAGONAL_KNOWN: u8 = 2;
        pub const r#COVARIANCE_TYPE_KNOWN: u8 = 3;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct NavSatStatus {
        pub r#status: i8,
        pub r#service: u16,
    }
    impl ::roslibrust_codegen::RosMessageType for NavSatStatus {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/NavSatStatus";
    }
    impl NavSatStatus {
        pub const r#STATUS_NO_FIX: i8 = -1;
        pub const r#STATUS_FIX: i8 = 0;
        pub const r#STATUS_SBAS_FIX: i8 = 1;
        pub const r#STATUS_GBAS_FIX: i8 = 2;
        pub const r#SERVICE_GPS: u16 = 1;
        pub const r#SERVICE_GLONASS: u16 = 2;
        pub const r#SERVICE_COMPASS: u16 = 4;
        pub const r#SERVICE_GALILEO: u16 = 8;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct PointCloud {
        pub r#header: std_msgs::Header,
        pub r#points: ::std::vec::Vec<geometry_msgs::Point32>,
        pub r#channels: ::std::vec::Vec<self::ChannelFloat32>,
    }
    impl ::roslibrust_codegen::RosMessageType for PointCloud {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/PointCloud";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct PointCloud2 {
        pub r#header: std_msgs::Header,
        pub r#height: u32,
        pub r#width: u32,
        pub r#fields: ::std::vec::Vec<self::PointField>,
        pub r#is_bigendian: bool,
        pub r#point_step: u32,
        pub r#row_step: u32,
        pub r#data: ::std::vec::Vec<u8>,
        pub r#is_dense: bool,
    }
    impl ::roslibrust_codegen::RosMessageType for PointCloud2 {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/PointCloud2";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct PointField {
        pub r#name: ::std::string::String,
        pub r#offset: u32,
        pub r#datatype: u8,
        pub r#count: u32,
    }
    impl ::roslibrust_codegen::RosMessageType for PointField {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/PointField";
    }
    impl PointField {
        pub const r#INT8: u8 = 1;
        pub const r#UINT8: u8 = 2;
        pub const r#INT16: u8 = 3;
        pub const r#UINT16: u8 = 4;
        pub const r#INT32: u8 = 5;
        pub const r#UINT32: u8 = 6;
        pub const r#FLOAT32: u8 = 7;
        pub const r#FLOAT64: u8 = 8;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Range {
        pub r#header: std_msgs::Header,
        pub r#radiation_type: u8,
        pub r#field_of_view: f32,
        pub r#min_range: f32,
        pub r#max_range: f32,
        pub r#range: f32,
    }
    impl ::roslibrust_codegen::RosMessageType for Range {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Range";
    }
    impl Range {
        pub const r#ULTRASOUND: u8 = 0;
        pub const r#INFRARED: u8 = 1;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct RegionOfInterest {
        pub r#x_offset: u32,
        pub r#y_offset: u32,
        pub r#height: u32,
        pub r#width: u32,
        pub r#do_rectify: bool,
    }
    impl ::roslibrust_codegen::RosMessageType for RegionOfInterest {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/RegionOfInterest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct RelativeHumidity {
        pub r#header: std_msgs::Header,
        pub r#relative_humidity: f64,
        pub r#variance: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for RelativeHumidity {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/RelativeHumidity";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct SetCameraInfoRequest {
        pub r#camera_info: self::CameraInfo,
    }
    impl ::roslibrust_codegen::RosMessageType for SetCameraInfoRequest {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/SetCameraInfoRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct SetCameraInfoResponse {
        pub r#success: bool,
        pub r#status_message: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for SetCameraInfoResponse {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/SetCameraInfoResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Temperature {
        pub r#header: std_msgs::Header,
        pub r#temperature: f64,
        pub r#variance: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for Temperature {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Temperature";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct TimeReference {
        pub r#header: std_msgs::Header,
        pub r#time_ref: ::roslibrust_codegen::integral_types::Time,
        pub r#source: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for TimeReference {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/TimeReference";
    }
    pub struct SetCameraInfo {}
    impl ::roslibrust_codegen::RosServiceType for SetCameraInfo {
        const ROS_SERVICE_NAME: &'static str = "sensor_msgs/SetCameraInfo";
        type Request = SetCameraInfoRequest;
        type Response = SetCameraInfoResponse;
    }
}
#[allow(unused_imports)]
pub mod shape_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Mesh {
        pub r#triangles: ::std::vec::Vec<self::MeshTriangle>,
        pub r#vertices: ::std::vec::Vec<geometry_msgs::Point>,
    }
    impl ::roslibrust_codegen::RosMessageType for Mesh {
        const ROS_TYPE_NAME: &'static str = "shape_msgs/Mesh";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct MeshTriangle {
        pub r#vertex_indices: ::std::vec::Vec<u32>,
    }
    impl ::roslibrust_codegen::RosMessageType for MeshTriangle {
        const ROS_TYPE_NAME: &'static str = "shape_msgs/MeshTriangle";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Plane {
        pub r#coef: ::std::vec::Vec<f64>,
    }
    impl ::roslibrust_codegen::RosMessageType for Plane {
        const ROS_TYPE_NAME: &'static str = "shape_msgs/Plane";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct SolidPrimitive {
        pub r#type: u8,
        pub r#dimensions: ::std::vec::Vec<f64>,
    }
    impl ::roslibrust_codegen::RosMessageType for SolidPrimitive {
        const ROS_TYPE_NAME: &'static str = "shape_msgs/SolidPrimitive";
    }
    impl SolidPrimitive {
        pub const r#BOX: u8 = 1;
        pub const r#SPHERE: u8 = 2;
        pub const r#CYLINDER: u8 = 3;
        pub const r#CONE: u8 = 4;
        pub const r#BOX_X: u8 = 0;
        pub const r#BOX_Y: u8 = 1;
        pub const r#BOX_Z: u8 = 2;
        pub const r#SPHERE_RADIUS: u8 = 0;
        pub const r#CYLINDER_HEIGHT: u8 = 0;
        pub const r#CYLINDER_RADIUS: u8 = 1;
        pub const r#CONE_HEIGHT: u8 = 0;
        pub const r#CONE_RADIUS: u8 = 1;
    }
}
#[allow(unused_imports)]
pub mod std_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Bool {
        pub r#data: bool,
    }
    impl ::roslibrust_codegen::RosMessageType for Bool {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Bool";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Byte {
        pub r#data: u8,
    }
    impl ::roslibrust_codegen::RosMessageType for Byte {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Byte";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct ByteMultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<u8>,
    }
    impl ::roslibrust_codegen::RosMessageType for ByteMultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/ByteMultiArray";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Char {
        pub r#data: u8,
    }
    impl ::roslibrust_codegen::RosMessageType for Char {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Char";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct ColorRGBA {
        pub r#r: f32,
        pub r#g: f32,
        pub r#b: f32,
        pub r#a: f32,
    }
    impl ::roslibrust_codegen::RosMessageType for ColorRGBA {
        const ROS_TYPE_NAME: &'static str = "std_msgs/ColorRGBA";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Duration {
        pub r#data: ::roslibrust_codegen::integral_types::Duration,
    }
    impl ::roslibrust_codegen::RosMessageType for Duration {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Duration";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Empty {}
    impl ::roslibrust_codegen::RosMessageType for Empty {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Empty";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Float32 {
        pub r#data: f32,
    }
    impl ::roslibrust_codegen::RosMessageType for Float32 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Float32";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Float32MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<f32>,
    }
    impl ::roslibrust_codegen::RosMessageType for Float32MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Float32MultiArray";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Float64 {
        pub r#data: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for Float64 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Float64";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Float64MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<f64>,
    }
    impl ::roslibrust_codegen::RosMessageType for Float64MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Float64MultiArray";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Header {
        pub r#seq: u32,
        pub r#stamp: ::roslibrust_codegen::integral_types::Time,
        pub r#frame_id: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for Header {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Header";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Int16 {
        pub r#data: i16,
    }
    impl ::roslibrust_codegen::RosMessageType for Int16 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int16";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Int16MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<i16>,
    }
    impl ::roslibrust_codegen::RosMessageType for Int16MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int16MultiArray";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Int32 {
        pub r#data: i32,
    }
    impl ::roslibrust_codegen::RosMessageType for Int32 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int32";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Int32MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<i32>,
    }
    impl ::roslibrust_codegen::RosMessageType for Int32MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int32MultiArray";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Int64 {
        pub r#data: i64,
    }
    impl ::roslibrust_codegen::RosMessageType for Int64 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int64";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Int64MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<i64>,
    }
    impl ::roslibrust_codegen::RosMessageType for Int64MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int64MultiArray";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Int8 {
        pub r#data: i8,
    }
    impl ::roslibrust_codegen::RosMessageType for Int8 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int8";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Int8MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<i8>,
    }
    impl ::roslibrust_codegen::RosMessageType for Int8MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int8MultiArray";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct MultiArrayDimension {
        pub r#label: ::std::string::String,
        pub r#size: u32,
        pub r#stride: u32,
    }
    impl ::roslibrust_codegen::RosMessageType for MultiArrayDimension {
        const ROS_TYPE_NAME: &'static str = "std_msgs/MultiArrayDimension";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct MultiArrayLayout {
        pub r#dim: ::std::vec::Vec<self::MultiArrayDimension>,
        pub r#data_offset: u32,
    }
    impl ::roslibrust_codegen::RosMessageType for MultiArrayLayout {
        const ROS_TYPE_NAME: &'static str = "std_msgs/MultiArrayLayout";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct String {
        pub r#data: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for String {
        const ROS_TYPE_NAME: &'static str = "std_msgs/String";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Time {
        pub r#data: ::roslibrust_codegen::integral_types::Time,
    }
    impl ::roslibrust_codegen::RosMessageType for Time {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Time";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct UInt16 {
        pub r#data: u16,
    }
    impl ::roslibrust_codegen::RosMessageType for UInt16 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt16";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct UInt16MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<u16>,
    }
    impl ::roslibrust_codegen::RosMessageType for UInt16MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt16MultiArray";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct UInt32 {
        pub r#data: u32,
    }
    impl ::roslibrust_codegen::RosMessageType for UInt32 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt32";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct UInt32MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<u32>,
    }
    impl ::roslibrust_codegen::RosMessageType for UInt32MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt32MultiArray";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct UInt64 {
        pub r#data: u64,
    }
    impl ::roslibrust_codegen::RosMessageType for UInt64 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt64";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct UInt64MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<u64>,
    }
    impl ::roslibrust_codegen::RosMessageType for UInt64MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt64MultiArray";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct UInt8 {
        pub r#data: u8,
    }
    impl ::roslibrust_codegen::RosMessageType for UInt8 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt8";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct UInt8MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<u8>,
    }
    impl ::roslibrust_codegen::RosMessageType for UInt8MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt8MultiArray";
    }
}
#[allow(unused_imports)]
pub mod std_srvs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct EmptyRequest {}
    impl ::roslibrust_codegen::RosMessageType for EmptyRequest {
        const ROS_TYPE_NAME: &'static str = "std_srvs/EmptyRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct EmptyResponse {}
    impl ::roslibrust_codegen::RosMessageType for EmptyResponse {
        const ROS_TYPE_NAME: &'static str = "std_srvs/EmptyResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct SetBoolRequest {
        pub r#data: bool,
    }
    impl ::roslibrust_codegen::RosMessageType for SetBoolRequest {
        const ROS_TYPE_NAME: &'static str = "std_srvs/SetBoolRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct SetBoolResponse {
        pub r#success: bool,
        pub r#message: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for SetBoolResponse {
        const ROS_TYPE_NAME: &'static str = "std_srvs/SetBoolResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct TriggerRequest {}
    impl ::roslibrust_codegen::RosMessageType for TriggerRequest {
        const ROS_TYPE_NAME: &'static str = "std_srvs/TriggerRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct TriggerResponse {
        pub r#success: bool,
        pub r#message: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for TriggerResponse {
        const ROS_TYPE_NAME: &'static str = "std_srvs/TriggerResponse";
    }
    pub struct Empty {}
    impl ::roslibrust_codegen::RosServiceType for Empty {
        const ROS_SERVICE_NAME: &'static str = "std_srvs/Empty";
        type Request = EmptyRequest;
        type Response = EmptyResponse;
    }
    pub struct SetBool {}
    impl ::roslibrust_codegen::RosServiceType for SetBool {
        const ROS_SERVICE_NAME: &'static str = "std_srvs/SetBool";
        type Request = SetBoolRequest;
        type Response = SetBoolResponse;
    }
    pub struct Trigger {}
    impl ::roslibrust_codegen::RosServiceType for Trigger {
        const ROS_SERVICE_NAME: &'static str = "std_srvs/Trigger";
        type Request = TriggerRequest;
        type Response = TriggerResponse;
    }
}
#[allow(unused_imports)]
pub mod stereo_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct DisparityImage {
        pub r#header: std_msgs::Header,
        pub r#image: sensor_msgs::Image,
        pub r#f: f32,
        pub r#T: f32,
        pub r#valid_window: sensor_msgs::RegionOfInterest,
        pub r#min_disparity: f32,
        pub r#max_disparity: f32,
        pub r#delta_d: f32,
    }
    impl ::roslibrust_codegen::RosMessageType for DisparityImage {
        const ROS_TYPE_NAME: &'static str = "stereo_msgs/DisparityImage";
    }
}
#[allow(unused_imports)]
pub mod test_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct AddTwoIntsRequest {
        pub r#a: i64,
        pub r#b: i64,
    }
    impl ::roslibrust_codegen::RosMessageType for AddTwoIntsRequest {
        const ROS_TYPE_NAME: &'static str = "test_msgs/AddTwoIntsRequest";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct AddTwoIntsResponse {
        pub r#sum: i64,
    }
    impl ::roslibrust_codegen::RosMessageType for AddTwoIntsResponse {
        const ROS_TYPE_NAME: &'static str = "test_msgs/AddTwoIntsResponse";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Constants {}
    impl ::roslibrust_codegen::RosMessageType for Constants {
        const ROS_TYPE_NAME: &'static str = "test_msgs/Constants";
    }
    impl Constants {
        pub const r#TEST_STR: &'static str = "/topic";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Float64Stamped {
        pub r#header: std_msgs::Header,
        pub r#value: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for Float64Stamped {
        const ROS_TYPE_NAME: &'static str = "test_msgs/Float64Stamped";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct LoggerLevel {
        pub r#level: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for LoggerLevel {
        const ROS_TYPE_NAME: &'static str = "test_msgs/LoggerLevel";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Metric {
        pub r#name: ::std::string::String,
        pub r#time: f64,
        pub r#data: ::std::vec::Vec<self::MetricPair>,
    }
    impl ::roslibrust_codegen::RosMessageType for Metric {
        const ROS_TYPE_NAME: &'static str = "test_msgs/Metric";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct MetricPair {
        pub r#key: ::std::string::String,
        pub r#value: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for MetricPair {
        const ROS_TYPE_NAME: &'static str = "test_msgs/MetricPair";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct NodeInfo {
        pub r#node_name: ::std::string::String,
        pub r#pid: i64,
        pub r#status: u8,
    }
    impl ::roslibrust_codegen::RosMessageType for NodeInfo {
        const ROS_TYPE_NAME: &'static str = "test_msgs/NodeInfo";
    }
    impl NodeInfo {
        pub const r#STATUS_UNINITIALIZED: u8 = 0;
        pub const r#STATUS_DISCONNECTED: u8 = 1;
        pub const r#STATUS_RUNNING: u8 = 2;
        pub const r#STATUS_RUN_ERROR: u8 = 3;
        pub const r#STATUS_SHUTTING_DOWN: u8 = 4;
        pub const r#STATUS_SHUTDOWN: u8 = 5;
    }
    pub struct AddTwoInts {}
    impl ::roslibrust_codegen::RosServiceType for AddTwoInts {
        const ROS_SERVICE_NAME: &'static str = "test_msgs/AddTwoInts";
        type Request = AddTwoIntsRequest;
        type Response = AddTwoIntsResponse;
    }
}
#[allow(unused_imports)]
pub mod trajectory_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct JointTrajectory {
        pub r#header: std_msgs::Header,
        pub r#joint_names: ::std::vec::Vec<::std::string::String>,
        pub r#points: ::std::vec::Vec<self::JointTrajectoryPoint>,
    }
    impl ::roslibrust_codegen::RosMessageType for JointTrajectory {
        const ROS_TYPE_NAME: &'static str = "trajectory_msgs/JointTrajectory";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct JointTrajectoryPoint {
        pub r#positions: ::std::vec::Vec<f64>,
        pub r#velocities: ::std::vec::Vec<f64>,
        pub r#accelerations: ::std::vec::Vec<f64>,
        pub r#effort: ::std::vec::Vec<f64>,
        pub r#time_from_start: ::roslibrust_codegen::integral_types::Duration,
    }
    impl ::roslibrust_codegen::RosMessageType for JointTrajectoryPoint {
        const ROS_TYPE_NAME: &'static str = "trajectory_msgs/JointTrajectoryPoint";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct MultiDOFJointTrajectory {
        pub r#header: std_msgs::Header,
        pub r#joint_names: ::std::vec::Vec<::std::string::String>,
        pub r#points: ::std::vec::Vec<self::MultiDOFJointTrajectoryPoint>,
    }
    impl ::roslibrust_codegen::RosMessageType for MultiDOFJointTrajectory {
        const ROS_TYPE_NAME: &'static str = "trajectory_msgs/MultiDOFJointTrajectory";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct MultiDOFJointTrajectoryPoint {
        pub r#transforms: ::std::vec::Vec<geometry_msgs::Transform>,
        pub r#velocities: ::std::vec::Vec<geometry_msgs::Twist>,
        pub r#accelerations: ::std::vec::Vec<geometry_msgs::Twist>,
        pub r#time_from_start: ::roslibrust_codegen::integral_types::Duration,
    }
    impl ::roslibrust_codegen::RosMessageType for MultiDOFJointTrajectoryPoint {
        const ROS_TYPE_NAME: &'static str = "trajectory_msgs/MultiDOFJointTrajectoryPoint";
    }
}
#[allow(unused_imports)]
pub mod visualization_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::rosgraph_msgs;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::std_srvs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct ImageMarker {
        pub r#header: std_msgs::Header,
        pub r#ns: ::std::string::String,
        pub r#id: i32,
        pub r#type: i32,
        pub r#action: i32,
        pub r#position: geometry_msgs::Point,
        pub r#scale: f32,
        pub r#outline_color: std_msgs::ColorRGBA,
        pub r#filled: u8,
        pub r#fill_color: std_msgs::ColorRGBA,
        pub r#lifetime: ::roslibrust_codegen::integral_types::Duration,
        pub r#points: ::std::vec::Vec<geometry_msgs::Point>,
        pub r#outline_colors: ::std::vec::Vec<std_msgs::ColorRGBA>,
    }
    impl ::roslibrust_codegen::RosMessageType for ImageMarker {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/ImageMarker";
    }
    impl ImageMarker {
        pub const r#CIRCLE: u8 = 0;
        pub const r#LINE_STRIP: u8 = 1;
        pub const r#LINE_LIST: u8 = 2;
        pub const r#POLYGON: u8 = 3;
        pub const r#POINTS: u8 = 4;
        pub const r#ADD: u8 = 0;
        pub const r#REMOVE: u8 = 1;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct InteractiveMarker {
        pub r#header: std_msgs::Header,
        pub r#pose: geometry_msgs::Pose,
        pub r#name: ::std::string::String,
        pub r#description: ::std::string::String,
        pub r#scale: f32,
        pub r#menu_entries: ::std::vec::Vec<self::MenuEntry>,
        pub r#controls: ::std::vec::Vec<self::InteractiveMarkerControl>,
    }
    impl ::roslibrust_codegen::RosMessageType for InteractiveMarker {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarker";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct InteractiveMarkerControl {
        pub r#name: ::std::string::String,
        pub r#orientation: geometry_msgs::Quaternion,
        pub r#orientation_mode: u8,
        pub r#interaction_mode: u8,
        pub r#always_visible: bool,
        pub r#markers: ::std::vec::Vec<self::Marker>,
        pub r#independent_marker_orientation: bool,
        pub r#description: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for InteractiveMarkerControl {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarkerControl";
    }
    impl InteractiveMarkerControl {
        pub const r#INHERIT: u8 = 0;
        pub const r#FIXED: u8 = 1;
        pub const r#VIEW_FACING: u8 = 2;
        pub const r#NONE: u8 = 0;
        pub const r#MENU: u8 = 1;
        pub const r#BUTTON: u8 = 2;
        pub const r#MOVE_AXIS: u8 = 3;
        pub const r#MOVE_PLANE: u8 = 4;
        pub const r#ROTATE_AXIS: u8 = 5;
        pub const r#MOVE_ROTATE: u8 = 6;
        pub const r#MOVE_3D: u8 = 7;
        pub const r#ROTATE_3D: u8 = 8;
        pub const r#MOVE_ROTATE_3D: u8 = 9;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct InteractiveMarkerFeedback {
        pub r#header: std_msgs::Header,
        pub r#client_id: ::std::string::String,
        pub r#marker_name: ::std::string::String,
        pub r#control_name: ::std::string::String,
        pub r#event_type: u8,
        pub r#pose: geometry_msgs::Pose,
        pub r#menu_entry_id: u32,
        pub r#mouse_point: geometry_msgs::Point,
        pub r#mouse_point_valid: bool,
    }
    impl ::roslibrust_codegen::RosMessageType for InteractiveMarkerFeedback {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarkerFeedback";
    }
    impl InteractiveMarkerFeedback {
        pub const r#KEEP_ALIVE: u8 = 0;
        pub const r#POSE_UPDATE: u8 = 1;
        pub const r#MENU_SELECT: u8 = 2;
        pub const r#BUTTON_CLICK: u8 = 3;
        pub const r#MOUSE_DOWN: u8 = 4;
        pub const r#MOUSE_UP: u8 = 5;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct InteractiveMarkerInit {
        pub r#server_id: ::std::string::String,
        pub r#seq_num: u64,
        pub r#markers: ::std::vec::Vec<self::InteractiveMarker>,
    }
    impl ::roslibrust_codegen::RosMessageType for InteractiveMarkerInit {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarkerInit";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct InteractiveMarkerPose {
        pub r#header: std_msgs::Header,
        pub r#pose: geometry_msgs::Pose,
        pub r#name: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for InteractiveMarkerPose {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarkerPose";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct InteractiveMarkerUpdate {
        pub r#server_id: ::std::string::String,
        pub r#seq_num: u64,
        pub r#type: u8,
        pub r#markers: ::std::vec::Vec<self::InteractiveMarker>,
        pub r#poses: ::std::vec::Vec<self::InteractiveMarkerPose>,
        pub r#erases: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for InteractiveMarkerUpdate {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarkerUpdate";
    }
    impl InteractiveMarkerUpdate {
        pub const r#KEEP_ALIVE: u8 = 0;
        pub const r#UPDATE: u8 = 1;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct Marker {
        pub r#header: std_msgs::Header,
        pub r#ns: ::std::string::String,
        pub r#id: i32,
        pub r#type: i32,
        pub r#action: i32,
        pub r#pose: geometry_msgs::Pose,
        pub r#scale: geometry_msgs::Vector3,
        pub r#color: std_msgs::ColorRGBA,
        pub r#lifetime: ::roslibrust_codegen::integral_types::Duration,
        pub r#frame_locked: bool,
        pub r#points: ::std::vec::Vec<geometry_msgs::Point>,
        pub r#colors: ::std::vec::Vec<std_msgs::ColorRGBA>,
        pub r#text: ::std::string::String,
        pub r#mesh_resource: ::std::string::String,
        pub r#mesh_use_embedded_materials: bool,
    }
    impl ::roslibrust_codegen::RosMessageType for Marker {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/Marker";
    }
    impl Marker {
        pub const r#ARROW: u8 = 0;
        pub const r#CUBE: u8 = 1;
        pub const r#SPHERE: u8 = 2;
        pub const r#CYLINDER: u8 = 3;
        pub const r#LINE_STRIP: u8 = 4;
        pub const r#LINE_LIST: u8 = 5;
        pub const r#CUBE_LIST: u8 = 6;
        pub const r#SPHERE_LIST: u8 = 7;
        pub const r#POINTS: u8 = 8;
        pub const r#TEXT_VIEW_FACING: u8 = 9;
        pub const r#MESH_RESOURCE: u8 = 10;
        pub const r#TRIANGLE_LIST: u8 = 11;
        pub const r#ADD: u8 = 0;
        pub const r#MODIFY: u8 = 0;
        pub const r#DELETE: u8 = 2;
        pub const r#DELETEALL: u8 = 3;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct MarkerArray {
        pub r#markers: ::std::vec::Vec<self::Marker>,
    }
    impl ::roslibrust_codegen::RosMessageType for MarkerArray {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/MarkerArray";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct MenuEntry {
        pub r#id: u32,
        pub r#parent_id: u32,
        pub r#title: ::std::string::String,
        pub r#command: ::std::string::String,
        pub r#command_type: u8,
    }
    impl ::roslibrust_codegen::RosMessageType for MenuEntry {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/MenuEntry";
    }
    impl MenuEntry {
        pub const r#FEEDBACK: u8 = 0;
        pub const r#ROSRUN: u8 = 1;
        pub const r#ROSLAUNCH: u8 = 2;
    }
}
