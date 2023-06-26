#[allow(unused_imports)]
pub mod actionlib_msgs {
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
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
        const MD5SUM: &'static str = "29380925936d499346662d2ed1573d06";
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
        const MD5SUM: &'static str = "c24a9e244d837a856267339807550287";
    }
    impl GoalStatus {
        pub const r#PENDING: u8 = 0u8;
        pub const r#ACTIVE: u8 = 1u8;
        pub const r#PREEMPTED: u8 = 2u8;
        pub const r#SUCCEEDED: u8 = 3u8;
        pub const r#ABORTED: u8 = 4u8;
        pub const r#REJECTED: u8 = 5u8;
        pub const r#PREEMPTING: u8 = 6u8;
        pub const r#RECALLING: u8 = 7u8;
        pub const r#RECALLED: u8 = 8u8;
        pub const r#LOST: u8 = 9u8;
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
        const MD5SUM: &'static str = "ad4c7a55992b9ff34d89596ca74a28e0";
    }
}
#[allow(unused_imports)]
pub mod diagnostic_msgs {
    use super::actionlib_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
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
    pub struct DiagnosticArray {
        pub r#header: std_msgs::Header,
        pub r#status: ::std::vec::Vec<self::DiagnosticStatus>,
    }
    impl ::roslibrust_codegen::RosMessageType for DiagnosticArray {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/DiagnosticArray";
        const MD5SUM: &'static str = "7f04f8332be7e46b680724aa4e9a9d71";
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
        const MD5SUM: &'static str = "d0ce08bc6e5ba34c7754f563a9cabaf1";
    }
    impl DiagnosticStatus {
        pub const r#OK: u8 = 0u8;
        pub const r#WARN: u8 = 1u8;
        pub const r#ERROR: u8 = 2u8;
        pub const r#STALE: u8 = 3u8;
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
        const MD5SUM: &'static str = "cf57fdc6617a881a88c16e768132149c";
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
    pub struct AddDiagnosticsRequest {
        pub r#load_namespace: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for AddDiagnosticsRequest {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/AddDiagnosticsRequest";
        const MD5SUM: &'static str = "c26cf6e164288fbc6050d74f838bcdf0";
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
        const MD5SUM: &'static str = "937c9679a518e3a18d831e57125ea522";
    }
    pub struct AddDiagnostics {}
    impl ::roslibrust_codegen::RosServiceType for AddDiagnostics {
        const ROS_SERVICE_NAME: &'static str = "diagnostic_msgs/AddDiagnostics";
        const MD5SUM: &'static str = "e6ac9bbde83d0d3186523c3687aecaee";
        type Request = AddDiagnosticsRequest;
        type Response = AddDiagnosticsResponse;
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
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
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
        const MD5SUM: &'static str = "ac21b1bab7ab17546986536c22eb34e9";
    }
    pub struct SelfTest {}
    impl ::roslibrust_codegen::RosServiceType for SelfTest {
        const ROS_SERVICE_NAME: &'static str = "diagnostic_msgs/SelfTest";
        const MD5SUM: &'static str = "ac21b1bab7ab17546986536c22eb34e9";
        type Request = SelfTestRequest;
        type Response = SelfTestResponse;
    }
}
#[allow(unused_imports)]
pub mod geometry_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::nav_msgs;
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
        const MD5SUM: &'static str = "9f195f881246fdfa2798d1d3eebca84a";
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
        const MD5SUM: &'static str = "19a31cf6d39a90e769a5539f9a293621";
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
        const MD5SUM: &'static str = "c6f49a48c87b365e434f7864bd9539b3";
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
        const MD5SUM: &'static str = "2ffaa449593251db0b3c12257ba4b247";
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
        const MD5SUM: &'static str = "1d26e4bb6c83ff141c5cf0d883c2b0fe";
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
        const MD5SUM: &'static str = "d4fb75ac056292d6c4bbec5e51d25080";
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
        const MD5SUM: &'static str = "4a842b65f413084dc2b10fb484ea7f17";
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
        const MD5SUM: &'static str = "cc153912f1453b708d221682bc23d9ac";
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
        const MD5SUM: &'static str = "938cb86faf4572821e49e2490701e6df";
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
        const MD5SUM: &'static str = "cd60a26494a087f577976f0329fa120e";
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
        const MD5SUM: &'static str = "56a3a2a80165092f696df3db62e18fbf";
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
        const MD5SUM: &'static str = "e45d45a5a1ce597b249e23fb30fc871f";
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
        const MD5SUM: &'static str = "938fa65709584ad8e77d238529be13b8";
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
        const MD5SUM: &'static str = "ea7300c78ec47498d5f226be74a155e8";
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
        const MD5SUM: &'static str = "c088ec4a70a5930b0ca46520d5745e2d";
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
        const MD5SUM: &'static str = "5b711e242c1ee70503c6bddce2439ca8";
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
        const MD5SUM: &'static str = "8d1f95db1811f91535f690acc723630a";
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
        #[default(0f64)]
        pub r#x: f64,
        #[default(0f64)]
        pub r#y: f64,
        #[default(0f64)]
        pub r#z: f64,
        #[default(1f64)]
        pub r#w: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for Quaternion {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Quaternion";
        const MD5SUM: &'static str = "a779879fadf0160734f906b8c19c7004";
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
        const MD5SUM: &'static str = "8f93ed7c8430d06bd82fefcc6f7a349e";
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
        const MD5SUM: &'static str = "ac9eff44abf714214112b05d54a3cf9b";
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
        const MD5SUM: &'static str = "09bf247c06cf7c69e8c55300b05a7a04";
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
        const MD5SUM: &'static str = "9f195f881246fdfa2798d1d3eebca84a";
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
        const MD5SUM: &'static str = "09f84400c1ca2e7e26a9da1232813bd0";
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
        const MD5SUM: &'static str = "75fcb7b0fec4e5cc35535e727ba979bd";
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
        const MD5SUM: &'static str = "5423aed9160897bf26a34981339cc85d";
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
        const MD5SUM: &'static str = "4a842b65f413084dc2b10fb484ea7f17";
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
        const MD5SUM: &'static str = "5cd361f2989a2e76d5aaf432c3bf0fb9";
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
        const MD5SUM: &'static str = "4f539cf138b23283b520fd271b567936";
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
        const MD5SUM: &'static str = "5bc71556ab354cd6274d262a7de094a5";
    }
}
#[allow(unused_imports)]
pub mod nav_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
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
    pub struct GridCells {
        pub r#header: std_msgs::Header,
        pub r#cell_width: f32,
        pub r#cell_height: f32,
        pub r#cells: ::std::vec::Vec<geometry_msgs::Point>,
    }
    impl ::roslibrust_codegen::RosMessageType for GridCells {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GridCells";
        const MD5SUM: &'static str = "bb9f07bfd2183241b5719f45a81f8cc5";
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
        const MD5SUM: &'static str = "d10232bae3de4ae536d98f679fce2cf2";
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
        const MD5SUM: &'static str = "2b0657f1993991bf3953916eb5ff5203";
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
        const MD5SUM: &'static str = "5554098bea214231bfb444a6df7a41cd";
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
        const MD5SUM: &'static str = "9f78b006a4c2cc2a146c12ed59d1bb7f";
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
    pub struct GetMapRequest {}
    impl ::roslibrust_codegen::RosMessageType for GetMapRequest {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetMapRequest";
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
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
        const MD5SUM: &'static str = "d6e8b0301af2dfe2244959ba20a4080a";
    }
    pub struct GetMap {}
    impl ::roslibrust_codegen::RosServiceType for GetMap {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/GetMap";
        const MD5SUM: &'static str = "d6e8b0301af2dfe2244959ba20a4080a";
        type Request = GetMapRequest;
        type Response = GetMapResponse;
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
        const MD5SUM: &'static str = "e4855e4d3c7377c76ec90e403202286a";
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
        const MD5SUM: &'static str = "37c13f9b42d0ee04e1dae0d4f7d14878";
    }
    pub struct GetPlan {}
    impl ::roslibrust_codegen::RosServiceType for GetPlan {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/GetPlan";
        const MD5SUM: &'static str = "135edd06523950427d2cf5e0bb9780a2";
        type Request = GetPlanRequest;
        type Response = GetPlanResponse;
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
        const MD5SUM: &'static str = "3813ba1ae85fbcd4dc88c90f1426b90b";
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
        const MD5SUM: &'static str = "cdb849e3dfaed8b5fe66776d7a64b83e";
    }
    impl LoadMapResponse {
        pub const r#RESULT_SUCCESS: u8 = 0u8;
        pub const r#RESULT_MAP_DOES_NOT_EXIST: u8 = 1u8;
        pub const r#RESULT_INVALID_MAP_DATA: u8 = 2u8;
        pub const r#RESULT_INVALID_MAP_METADATA: u8 = 3u8;
        pub const r#RESULT_UNDEFINED_FAILURE: u8 = 255u8;
    }
    pub struct LoadMap {}
    impl ::roslibrust_codegen::RosServiceType for LoadMap {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/LoadMap";
        const MD5SUM: &'static str = "96c8a15e8fe5c33ee245f610f020d6ba";
        type Request = LoadMapRequest;
        type Response = LoadMapResponse;
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
        const MD5SUM: &'static str = "1362070bf1594e3c3a800b8a926ae808";
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
        const MD5SUM: &'static str = "358e233cde0c8a8bcfea4ce193f8fc15";
    }
    pub struct SetMap {}
    impl ::roslibrust_codegen::RosServiceType for SetMap {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/SetMap";
        const MD5SUM: &'static str = "3259327d9856c8a615675f6c7f0d4420";
        type Request = SetMapRequest;
        type Response = SetMapResponse;
    }
}
#[allow(unused_imports)]
pub mod sensor_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
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
        const MD5SUM: &'static str = "0e25cedcd370a46961764fe3a9d2ddcb";
    }
    impl BatteryState {
        pub const r#POWER_SUPPLY_STATUS_UNKNOWN: u8 = 0u8;
        pub const r#POWER_SUPPLY_STATUS_CHARGING: u8 = 1u8;
        pub const r#POWER_SUPPLY_STATUS_DISCHARGING: u8 = 2u8;
        pub const r#POWER_SUPPLY_STATUS_NOT_CHARGING: u8 = 3u8;
        pub const r#POWER_SUPPLY_STATUS_FULL: u8 = 4u8;
        pub const r#POWER_SUPPLY_HEALTH_UNKNOWN: u8 = 0u8;
        pub const r#POWER_SUPPLY_HEALTH_GOOD: u8 = 1u8;
        pub const r#POWER_SUPPLY_HEALTH_OVERHEAT: u8 = 2u8;
        pub const r#POWER_SUPPLY_HEALTH_DEAD: u8 = 3u8;
        pub const r#POWER_SUPPLY_HEALTH_OVERVOLTAGE: u8 = 4u8;
        pub const r#POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: u8 = 5u8;
        pub const r#POWER_SUPPLY_HEALTH_COLD: u8 = 6u8;
        pub const r#POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE: u8 = 7u8;
        pub const r#POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE: u8 = 8u8;
        pub const r#POWER_SUPPLY_TECHNOLOGY_UNKNOWN: u8 = 0u8;
        pub const r#POWER_SUPPLY_TECHNOLOGY_NIMH: u8 = 1u8;
        pub const r#POWER_SUPPLY_TECHNOLOGY_LION: u8 = 2u8;
        pub const r#POWER_SUPPLY_TECHNOLOGY_LIPO: u8 = 3u8;
        pub const r#POWER_SUPPLY_TECHNOLOGY_LIFE: u8 = 4u8;
        pub const r#POWER_SUPPLY_TECHNOLOGY_NICD: u8 = 5u8;
        pub const r#POWER_SUPPLY_TECHNOLOGY_LIMN: u8 = 6u8;
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
        pub r#d: ::std::vec::Vec<f64>,
        pub r#k: ::std::vec::Vec<f64>,
        pub r#r: ::std::vec::Vec<f64>,
        pub r#p: ::std::vec::Vec<f64>,
        pub r#binning_x: u32,
        pub r#binning_y: u32,
        pub r#roi: self::RegionOfInterest,
    }
    impl ::roslibrust_codegen::RosMessageType for CameraInfo {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/CameraInfo";
        const MD5SUM: &'static str = "6ed1374c273af05e0a4310d74bacbaff";
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
        const MD5SUM: &'static str = "3d40139cdd33dfedcb71ffeeeb42ae7f";
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
        const MD5SUM: &'static str = "1df88053b24348f5f499666c7cb1d980";
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
        const MD5SUM: &'static str = "4967e6ff4dcf72e6b8fca0600661e0b6";
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
        const MD5SUM: &'static str = "94ccac1a1be684df74466dfc561512aa";
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
        const MD5SUM: &'static str = "9c8b3d25a28b72f070da359dbecf985b";
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
        const MD5SUM: &'static str = "0778670fb00acae60a84a9e24c1d1400";
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
        const MD5SUM: &'static str = "3f61f1439a9898cdd864497d378ce55c";
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
        const MD5SUM: &'static str = "967f985c9ca9013a4669430613e3e016";
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
        const MD5SUM: &'static str = "f4dcd73460360d98f36e55ee7f2e46f1";
    }
    impl JoyFeedback {
        pub const r#TYPE_LED: u8 = 0u8;
        pub const r#TYPE_RUMBLE: u8 = 1u8;
        pub const r#TYPE_BUZZER: u8 = 2u8;
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
        const MD5SUM: &'static str = "cde5730a895b1fc4dee6f91b754b213d";
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
        const MD5SUM: &'static str = "8bc5ae449b200fba4d552b4225586696";
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
        const MD5SUM: &'static str = "f86984b4383bf67523c75820e114e988";
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
        const MD5SUM: &'static str = "baa70b1dbaa90fd66e5241377ca453f6";
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
        const MD5SUM: &'static str = "9eb02d78422731545fd7e9b60069f261";
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
        const MD5SUM: &'static str = "fda674281c16cdee9a79d075ab27d12f";
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
        const MD5SUM: &'static str = "4772ea7752199e85449ef71667f9101e";
    }
    impl NavSatFix {
        pub const r#COVARIANCE_TYPE_UNKNOWN: u8 = 0u8;
        pub const r#COVARIANCE_TYPE_APPROXIMATED: u8 = 1u8;
        pub const r#COVARIANCE_TYPE_DIAGONAL_KNOWN: u8 = 2u8;
        pub const r#COVARIANCE_TYPE_KNOWN: u8 = 3u8;
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
        const MD5SUM: &'static str = "331cdbddfa4bc96ffc3b9ad98900a54c";
    }
    impl NavSatStatus {
        pub const r#STATUS_NO_FIX: i8 = -1i8;
        pub const r#STATUS_FIX: i8 = 0i8;
        pub const r#STATUS_SBAS_FIX: i8 = 1i8;
        pub const r#STATUS_GBAS_FIX: i8 = 2i8;
        pub const r#SERVICE_GPS: u16 = 1u16;
        pub const r#SERVICE_GLONASS: u16 = 2u16;
        pub const r#SERVICE_COMPASS: u16 = 4u16;
        pub const r#SERVICE_GALILEO: u16 = 8u16;
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
        const MD5SUM: &'static str = "95c9c548e015c235b38b961c79973db7";
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
        const MD5SUM: &'static str = "c61ffb665fe19735825e4dd31b53913d";
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
        const MD5SUM: &'static str = "268eacb2962780ceac86cbd17e328150";
    }
    impl PointField {
        pub const r#INT8: u8 = 1u8;
        pub const r#UINT8: u8 = 2u8;
        pub const r#INT16: u8 = 3u8;
        pub const r#UINT16: u8 = 4u8;
        pub const r#INT32: u8 = 5u8;
        pub const r#UINT32: u8 = 6u8;
        pub const r#FLOAT32: u8 = 7u8;
        pub const r#FLOAT64: u8 = 8u8;
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
        const MD5SUM: &'static str = "1ec40687acdf15b9559a6ff690722eae";
    }
    impl Range {
        pub const r#ULTRASOUND: u8 = 0u8;
        pub const r#INFRARED: u8 = 1u8;
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
        const MD5SUM: &'static str = "bdb633039d588fcccb441a4d43ccfe09";
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
        const MD5SUM: &'static str = "71cfefa31dcc94f47083b1e89e6fa5c9";
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
        const MD5SUM: &'static str = "c6df0674fcfebff84a15927a80ebb14b";
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
        const MD5SUM: &'static str = "7cb7ae5aa838323e9028637e304e0ad7";
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
        const MD5SUM: &'static str = "d01d173ac0f730f0076e8b9ecabcf422";
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
        const MD5SUM: &'static str = "2ec6f3eff0161f4257b808b12bc830c2";
    }
    pub struct SetCameraInfo {}
    impl ::roslibrust_codegen::RosServiceType for SetCameraInfo {
        const ROS_SERVICE_NAME: &'static str = "sensor_msgs/SetCameraInfo";
        const MD5SUM: &'static str = "05fe977980e126fa16e30568dad4042f";
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
        const MD5SUM: &'static str = "074b644fc5c01938c4e68ae14f4f0527";
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
        const MD5SUM: &'static str = "153f61dee254ce3792e80cfd99cbcee3";
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
        const MD5SUM: &'static str = "ab3614b72ca3f82aad54a53021284fb5";
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
        pub r#polygon: geometry_msgs::Polygon,
    }
    impl ::roslibrust_codegen::RosMessageType for SolidPrimitive {
        const ROS_TYPE_NAME: &'static str = "shape_msgs/SolidPrimitive";
        const MD5SUM: &'static str = "64566630a0f2fc6b0e02f71ceb36fd48";
    }
    impl SolidPrimitive {
        pub const r#BOX: u8 = 1u8;
        pub const r#SPHERE: u8 = 2u8;
        pub const r#CYLINDER: u8 = 3u8;
        pub const r#CONE: u8 = 4u8;
        pub const r#PRISM: u8 = 5u8;
        pub const r#BOX_X: u8 = 0u8;
        pub const r#BOX_Y: u8 = 1u8;
        pub const r#BOX_Z: u8 = 2u8;
        pub const r#SPHERE_RADIUS: u8 = 0u8;
        pub const r#CYLINDER_HEIGHT: u8 = 0u8;
        pub const r#CYLINDER_RADIUS: u8 = 1u8;
        pub const r#CONE_HEIGHT: u8 = 0u8;
        pub const r#CONE_RADIUS: u8 = 1u8;
        pub const r#PRISM_HEIGHT: u8 = 0u8;
    }
}
#[allow(unused_imports)]
pub mod std_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
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
        const MD5SUM: &'static str = "8b94c1b53db61fb6aed406028ad6332a";
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
        const MD5SUM: &'static str = "ad736a2e8818154c487bb80fe42ce43b";
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
        const MD5SUM: &'static str = "70ea476cbcfd65ac2f68f3cda1e891fe";
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
        const MD5SUM: &'static str = "1bf77f25acecdedba0e224b162199717";
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
        const MD5SUM: &'static str = "a29a96539573343b1310c73607334b00";
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
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
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
        const MD5SUM: &'static str = "73fcbf46b49191e672908e50842a83d4";
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
        const MD5SUM: &'static str = "6a40e0ffa6a17a503ac3f8616991b1f6";
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
        const MD5SUM: &'static str = "fdb28210bfa9d7c91146260178d9a584";
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
        const MD5SUM: &'static str = "4b7d974086d4060e7db4613a7e6c3ba4";
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
        pub r#stamp: ::roslibrust_codegen::integral_types::Time,
        pub r#frame_id: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for Header {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Header";
        const MD5SUM: &'static str = "5ed6b5dd1ef879ffb9c2ac51bab61a63";
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
        const MD5SUM: &'static str = "8524586e34fbd7cb1c08c5f5f1ca0e57";
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
        const MD5SUM: &'static str = "d9338d7f523fcb692fae9d0a0e9f067c";
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
        const MD5SUM: &'static str = "da5909fbe378aeaf85e547e830cc1bb7";
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
        const MD5SUM: &'static str = "1d99f79f8b325b44fee908053e9c945b";
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
        const MD5SUM: &'static str = "34add168574510e6e17f5d23ecc077ef";
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
        const MD5SUM: &'static str = "54865aa6c65be0448113a2afc6a49270";
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
        const MD5SUM: &'static str = "27ffa0c9c4b8fb8492252bcad9e5c57b";
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
        const MD5SUM: &'static str = "d7c1af35a1b4781bbe79e03dd94b7c13";
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
        const MD5SUM: &'static str = "4cd0c83a8683deae40ecdac60e53bfa8";
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
        const MD5SUM: &'static str = "0fed2a11c13e11c5571b4e2a995a91a3";
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
        const MD5SUM: &'static str = "992ce8a1687cec8c8bd883ec73ca41d1";
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
        const MD5SUM: &'static str = "1df79edf208b629fe6b81923a544552d";
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
        const MD5SUM: &'static str = "52f264f1c973c4b73790d384c6cb4484";
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
        const MD5SUM: &'static str = "304a39449588c7f8ce2df6e8001c5fce";
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
        const MD5SUM: &'static str = "4d6a180abc9be191b96a7eda6c8a233d";
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
        const MD5SUM: &'static str = "1b2a79973e8bf53d7b53acb71299cb57";
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
        const MD5SUM: &'static str = "6088f127afb1d6c72927aa1247e945af";
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
        const MD5SUM: &'static str = "7c8164229e7d2c17eb95e9231617fdee";
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
        const MD5SUM: &'static str = "82373f1612381bb6ee473b5cd6f5d89c";
    }
}
#[allow(unused_imports)]
pub mod std_srvs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
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
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
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
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
    }
    pub struct Empty {}
    impl ::roslibrust_codegen::RosServiceType for Empty {
        const ROS_SERVICE_NAME: &'static str = "std_srvs/Empty";
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
        type Request = EmptyRequest;
        type Response = EmptyResponse;
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
        const MD5SUM: &'static str = "8b94c1b53db61fb6aed406028ad6332a";
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
        const MD5SUM: &'static str = "937c9679a518e3a18d831e57125ea522";
    }
    pub struct SetBool {}
    impl ::roslibrust_codegen::RosServiceType for SetBool {
        const ROS_SERVICE_NAME: &'static str = "std_srvs/SetBool";
        const MD5SUM: &'static str = "09fb03525b03e7ea1fd3992bafd87e16";
        type Request = SetBoolRequest;
        type Response = SetBoolResponse;
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
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
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
        const MD5SUM: &'static str = "937c9679a518e3a18d831e57125ea522";
    }
    pub struct Trigger {}
    impl ::roslibrust_codegen::RosServiceType for Trigger {
        const ROS_SERVICE_NAME: &'static str = "std_srvs/Trigger";
        const MD5SUM: &'static str = "937c9679a518e3a18d831e57125ea522";
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
        pub r#t: f32,
        pub r#valid_window: sensor_msgs::RegionOfInterest,
        pub r#min_disparity: f32,
        pub r#max_disparity: f32,
        pub r#delta_d: f32,
    }
    impl ::roslibrust_codegen::RosMessageType for DisparityImage {
        const ROS_TYPE_NAME: &'static str = "stereo_msgs/DisparityImage";
        const MD5SUM: &'static str = "cb0de8feef04280238c7b77d74b2beca";
    }
}
#[allow(unused_imports)]
pub mod test_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
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
    pub struct Defaults {
        #[default(42u8)]
        pub r#x: u8,
        # [default (- 2000i16)]
        pub r#y: i16,
        #[default("John Doe")]
        pub r#full_name: ::std::string::String,
        #[default(_code = "vec![-200, -100, 0, 100, 200]")]
        pub r#samples: ::std::vec::Vec<i32>,
        #[default(_code = "vec![-200.0, -1.0, 0.0]")]
        pub r#f_samples: ::std::vec::Vec<f32>,
        #[default(_code = "[\"hello\", \"world\"].iter().map(|x| x.to_string()).collect()")]
        pub r#s_vec: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for Defaults {
        const ROS_TYPE_NAME: &'static str = "test_msgs/Defaults";
        const MD5SUM: &'static str = "43c441dc2b521c313f54affd982b5314";
    }
}
#[allow(unused_imports)]
pub mod trajectory_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
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
        const MD5SUM: &'static str = "d63e3b4556d9dbd9f48b5ab4a03f1fee";
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
        const MD5SUM: &'static str = "2c812f86aa886c93954e333721749ac5";
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
        const MD5SUM: &'static str = "d00d14d97bd70c5eb648278240cfb066";
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
        const MD5SUM: &'static str = "6731945e53cbc0fbc6e93c28f7416a71";
    }
}
#[allow(unused_imports)]
pub mod visualization_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
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
        const MD5SUM: &'static str = "829dd5d9ba39b8c3844252ebd8b47b96";
    }
    impl ImageMarker {
        pub const r#CIRCLE: i32 = 0i32;
        pub const r#LINE_STRIP: i32 = 1i32;
        pub const r#LINE_LIST: i32 = 2i32;
        pub const r#POLYGON: i32 = 3i32;
        pub const r#POINTS: i32 = 4i32;
        pub const r#ADD: i32 = 0i32;
        pub const r#REMOVE: i32 = 1i32;
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
        const MD5SUM: &'static str = "d71737fa44c5bdefd6bdb4fa9b2b86e5";
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
        const MD5SUM: &'static str = "7b945e790a2d68f430a6eb79f33bf8df";
    }
    impl InteractiveMarkerControl {
        pub const r#INHERIT: u8 = 0u8;
        pub const r#FIXED: u8 = 1u8;
        pub const r#VIEW_FACING: u8 = 2u8;
        pub const r#NONE: u8 = 0u8;
        pub const r#MENU: u8 = 1u8;
        pub const r#BUTTON: u8 = 2u8;
        pub const r#MOVE_AXIS: u8 = 3u8;
        pub const r#MOVE_PLANE: u8 = 4u8;
        pub const r#ROTATE_AXIS: u8 = 5u8;
        pub const r#MOVE_ROTATE: u8 = 6u8;
        pub const r#MOVE_3D: u8 = 7u8;
        pub const r#ROTATE_3D: u8 = 8u8;
        pub const r#MOVE_ROTATE_3D: u8 = 9u8;
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
        const MD5SUM: &'static str = "880e5141421ed8d30906fad686bc17bd";
    }
    impl InteractiveMarkerFeedback {
        pub const r#KEEP_ALIVE: u8 = 0u8;
        pub const r#POSE_UPDATE: u8 = 1u8;
        pub const r#MENU_SELECT: u8 = 2u8;
        pub const r#BUTTON_CLICK: u8 = 3u8;
        pub const r#MOUSE_DOWN: u8 = 4u8;
        pub const r#MOUSE_UP: u8 = 5u8;
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
        const MD5SUM: &'static str = "5d275694a5cb7ea4627f917a9eb1b4cd";
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
        const MD5SUM: &'static str = "b88540594a0f8e3fe46c720be41faa03";
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
        const MD5SUM: &'static str = "8f52c675c849441ae87da82eaa4d6eb5";
    }
    impl InteractiveMarkerUpdate {
        pub const r#KEEP_ALIVE: u8 = 0u8;
        pub const r#UPDATE: u8 = 1u8;
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
        pub r#texture_resource: ::std::string::String,
        pub r#texture: sensor_msgs::CompressedImage,
        pub r#uv_coordinates: ::std::vec::Vec<self::UVCoordinate>,
        pub r#text: ::std::string::String,
        pub r#mesh_resource: ::std::string::String,
        pub r#mesh_file: self::MeshFile,
        pub r#mesh_use_embedded_materials: bool,
    }
    impl ::roslibrust_codegen::RosMessageType for Marker {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/Marker";
        const MD5SUM: &'static str = "56c6324983a404ead7a426609371feed";
    }
    impl Marker {
        pub const r#ARROW: i32 = 0i32;
        pub const r#CUBE: i32 = 1i32;
        pub const r#SPHERE: i32 = 2i32;
        pub const r#CYLINDER: i32 = 3i32;
        pub const r#LINE_STRIP: i32 = 4i32;
        pub const r#LINE_LIST: i32 = 5i32;
        pub const r#CUBE_LIST: i32 = 6i32;
        pub const r#SPHERE_LIST: i32 = 7i32;
        pub const r#POINTS: i32 = 8i32;
        pub const r#TEXT_VIEW_FACING: i32 = 9i32;
        pub const r#MESH_RESOURCE: i32 = 10i32;
        pub const r#TRIANGLE_LIST: i32 = 11i32;
        pub const r#ADD: i32 = 0i32;
        pub const r#MODIFY: i32 = 0i32;
        pub const r#DELETE: i32 = 2i32;
        pub const r#DELETEALL: i32 = 3i32;
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
        const MD5SUM: &'static str = "11e38f15427197858cf46456867167bd";
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
        const MD5SUM: &'static str = "b90ec63024573de83b57aa93eb39be2d";
    }
    impl MenuEntry {
        pub const r#FEEDBACK: u8 = 0u8;
        pub const r#ROSRUN: u8 = 1u8;
        pub const r#ROSLAUNCH: u8 = 2u8;
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
    pub struct MeshFile {
        pub r#filename: ::std::string::String,
        pub r#data: ::std::vec::Vec<u8>,
    }
    impl ::roslibrust_codegen::RosMessageType for MeshFile {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/MeshFile";
        const MD5SUM: &'static str = "39f264648e441626a1045a7d9ef1ba17";
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
    pub struct UVCoordinate {
        pub r#u: f32,
        pub r#v: f32,
    }
    impl ::roslibrust_codegen::RosMessageType for UVCoordinate {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/UVCoordinate";
        const MD5SUM: &'static str = "4f5254e0e12914c461d4b17a0cd07f7f";
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
    pub struct GetInteractiveMarkersRequest {}
    impl ::roslibrust_codegen::RosMessageType for GetInteractiveMarkersRequest {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/GetInteractiveMarkersRequest";
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
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
    pub struct GetInteractiveMarkersResponse {
        pub r#sequence_number: u64,
        pub r#markers: ::std::vec::Vec<self::InteractiveMarker>,
    }
    impl ::roslibrust_codegen::RosMessageType for GetInteractiveMarkersResponse {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/GetInteractiveMarkersResponse";
        const MD5SUM: &'static str = "923b76ef2c497d4ff5f83a061d424d3b";
    }
    pub struct GetInteractiveMarkers {}
    impl ::roslibrust_codegen::RosServiceType for GetInteractiveMarkers {
        const ROS_SERVICE_NAME: &'static str = "visualization_msgs/GetInteractiveMarkers";
        const MD5SUM: &'static str = "923b76ef2c497d4ff5f83a061d424d3b";
        type Request = GetInteractiveMarkersRequest;
        type Response = GetInteractiveMarkersResponse;
    }
}
