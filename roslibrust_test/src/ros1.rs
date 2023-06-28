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
        const MD5SUM: &'static str = "302881f31927c1df708a2dbab0e80ee8";
    }
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
        const MD5SUM: &'static str = "d388f9b87b3c471f784434d671988d4a";
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
        const MD5SUM: &'static str = "8b2b82f13216d0a8ea88bd3af735e619";
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
    pub struct DiagnosticArray {
        pub r#header: std_msgs::Header,
        pub r#status: ::std::vec::Vec<self::DiagnosticStatus>,
    }
    impl ::roslibrust_codegen::RosMessageType for DiagnosticArray {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/DiagnosticArray";
        const MD5SUM: &'static str = "60810da900de1dd6ddd437c3503511da";
    }
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
        const MD5SUM: &'static str = "d8a98a5d81351b6eb0578c78557e7659";
    }
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
        const MD5SUM: &'static str = "ad5a718d699c6be72a02b8d6a139f334";
    }
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
        const MD5SUM: &'static str = "96adb295225031ec8d57fb4251b0a886";
    }
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
        const MD5SUM: &'static str = "ddee48caeab5a966c5e8d166654a9ac7";
    }
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
        const MD5SUM: &'static str = "c63aecb41bfdfd6b7e1fac37c7cbe7bf";
    }
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
        const MD5SUM: &'static str = "c6be8f7dc3bee7fe9e8d296070f53340";
    }
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
        const MD5SUM: &'static str = "916c28c5764443f268b296bb671b9d97";
    }
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
        const MD5SUM: &'static str = "d3812c3cbc69362b77dc0b19b345f8f5";
    }
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
        const MD5SUM: &'static str = "c23e848cf1b7533a8d7c259073a97e6f";
    }
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
        const MD5SUM: &'static str = "953b798c0f514ff060a53a3498ce6246";
    }
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
        const MD5SUM: &'static str = "e57f1e547e0e1fd13504588ffc8334e2";
    }
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
        const MD5SUM: &'static str = "b5764a33bfeb3588febc2682852579b0";
    }
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
        const MD5SUM: &'static str = "98d34b0043a2093cf9d9345ab6eef12e";
    }
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
        const MD5SUM: &'static str = "1fe8a28e6890a4cc3ae4c3ca5c7d82e6";
    }
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
        const MD5SUM: &'static str = "8927a1a12fb2607ceea095b2dc440a96";
    }
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
        const MD5SUM: &'static str = "7b324c7325e683bf02a9b14b01090ec7";
    }
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
        const MD5SUM: &'static str = "d78d3cb249ce23087ade7e7d0c40cfa7";
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
    pub struct GetMapAction {
        pub r#action_goal: self::GetMapActionGoal,
        pub r#action_result: self::GetMapActionResult,
        pub r#action_feedback: self::GetMapActionFeedback,
    }
    impl ::roslibrust_codegen::RosMessageType for GetMapAction {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetMapAction";
        const MD5SUM: &'static str = "e611ad23fbf237c031b7536416dc7cd7";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct GetMapActionFeedback {
        pub r#header: std_msgs::Header,
        pub r#status: actionlib_msgs::GoalStatus,
        pub r#feedback: self::GetMapFeedback,
    }
    impl ::roslibrust_codegen::RosMessageType for GetMapActionFeedback {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetMapActionFeedback";
        const MD5SUM: &'static str = "aae20e09065c3809e8a8e87c4c8953fd";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct GetMapActionGoal {
        pub r#header: std_msgs::Header,
        pub r#goal_id: actionlib_msgs::GoalID,
        pub r#goal: self::GetMapGoal,
    }
    impl ::roslibrust_codegen::RosMessageType for GetMapActionGoal {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetMapActionGoal";
        const MD5SUM: &'static str = "4b30be6cd12b9e72826df56b481f40e0";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct GetMapActionResult {
        pub r#header: std_msgs::Header,
        pub r#status: actionlib_msgs::GoalStatus,
        pub r#result: self::GetMapResult,
    }
    impl ::roslibrust_codegen::RosMessageType for GetMapActionResult {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetMapActionResult";
        const MD5SUM: &'static str = "ac66e5b9a79bb4bbd33dab245236c892";
    }
    #[allow(non_snake_case)]
    #[derive(
        :: serde :: Deserialize,
        :: serde :: Serialize,
        :: smart_default :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    pub struct GetMapFeedback {}
    impl ::roslibrust_codegen::RosMessageType for GetMapFeedback {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetMapFeedback";
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
    pub struct GetMapGoal {}
    impl ::roslibrust_codegen::RosMessageType for GetMapGoal {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetMapGoal";
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
    pub struct GetMapResult {
        pub r#map: self::OccupancyGrid,
    }
    impl ::roslibrust_codegen::RosMessageType for GetMapResult {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetMapResult";
        const MD5SUM: &'static str = "6cdd0a18e0aff5b0a3ca2326a89b54ff";
    }
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
        const MD5SUM: &'static str = "b9e4f5df6d28e272ebde00a3994830f5";
    }
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
        const MD5SUM: &'static str = "10cfc8a2818024d3248802c00c95f11b";
    }
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
        const MD5SUM: &'static str = "3381f2d731d4076ec5c71b0759edbe4e";
    }
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
        const MD5SUM: &'static str = "cd5e73d190d741a2f92e81eda573aca7";
    }
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
        const MD5SUM: &'static str = "6227e2b7e9cce15051f669a5e197bbf7";
    }
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
        const MD5SUM: &'static str = "6cdd0a18e0aff5b0a3ca2326a89b54ff";
    }
    pub struct GetMap {}
    impl ::roslibrust_codegen::RosServiceType for GetMap {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/GetMap";
        const MD5SUM: &'static str = "6cdd0a18e0aff5b0a3ca2326a89b54ff";
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
        const MD5SUM: &'static str = "e25a43e0752bcca599a8c2eef8282df8";
    }
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
        const MD5SUM: &'static str = "0002bc113c0259d71f6cf8cbc9430e18";
    }
    pub struct GetPlan {}
    impl ::roslibrust_codegen::RosServiceType for GetPlan {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/GetPlan";
        const MD5SUM: &'static str = "421c8ea4d21c6c9db7054b4bbdf1e024";
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
        const MD5SUM: &'static str = "079b9c828e9f7c1918bf86932fd7267e";
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
        const MD5SUM: &'static str = "22e647fdfbe3b23c8c9f419908afaebd";
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
        const MD5SUM: &'static str = "91149a20d7be299b87c340df8cc94fd4";
    }
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
        const MD5SUM: &'static str = "c36922319011e63ed7784112ad4fdd32";
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
        const MD5SUM: &'static str = "80597571d79bbeef6c9c4d98f30116a0";
    }
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
        const MD5SUM: &'static str = "c1f3d28f1b044c871e6eff2e9fc3c667";
    }
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
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
    }
    pub struct DeleteParam {}
    impl ::roslibrust_codegen::RosServiceType for DeleteParam {
        const ROS_SERVICE_NAME: &'static str = "rosapi/DeleteParam";
        const MD5SUM: &'static str = "c1f3d28f1b044c871e6eff2e9fc3c667";
        type Request = DeleteParamRequest;
        type Response = DeleteParamResponse;
    }
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
    pub struct GetActionServersResponse {
        pub r#action_servers: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for GetActionServersResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetActionServersResponse";
        const MD5SUM: &'static str = "46807ba271844ac5ba4730a47556b236";
    }
    pub struct GetActionServers {}
    impl ::roslibrust_codegen::RosServiceType for GetActionServers {
        const ROS_SERVICE_NAME: &'static str = "rosapi/GetActionServers";
        const MD5SUM: &'static str = "46807ba271844ac5ba4730a47556b236";
        type Request = GetActionServersRequest;
        type Response = GetActionServersResponse;
    }
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
        const MD5SUM: &'static str = "1cc3f281ee24ba9406c3e498e4da686f";
    }
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
        const MD5SUM: &'static str = "64e58419496c7248b4ef25731f88b8c3";
    }
    pub struct GetParam {}
    impl ::roslibrust_codegen::RosServiceType for GetParam {
        const ROS_SERVICE_NAME: &'static str = "rosapi/GetParam";
        const MD5SUM: &'static str = "e36fd90759dbac1c5159140a7fa8c644";
        type Request = GetParamRequest;
        type Response = GetParamResponse;
    }
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
    pub struct GetParamNamesResponse {
        pub r#names: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for GetParamNamesResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetParamNamesResponse";
        const MD5SUM: &'static str = "dc7ae3609524b18034e49294a4ce670e";
    }
    pub struct GetParamNames {}
    impl ::roslibrust_codegen::RosServiceType for GetParamNames {
        const ROS_SERVICE_NAME: &'static str = "rosapi/GetParamNames";
        const MD5SUM: &'static str = "dc7ae3609524b18034e49294a4ce670e";
        type Request = GetParamNamesRequest;
        type Response = GetParamNamesResponse;
    }
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
    pub struct GetTimeResponse {
        pub r#time: ::roslibrust_codegen::integral_types::Time,
    }
    impl ::roslibrust_codegen::RosMessageType for GetTimeResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetTimeResponse";
        const MD5SUM: &'static str = "556a4fb76023a469987922359d08a844";
    }
    pub struct GetTime {}
    impl ::roslibrust_codegen::RosServiceType for GetTime {
        const ROS_SERVICE_NAME: &'static str = "rosapi/GetTime";
        const MD5SUM: &'static str = "556a4fb76023a469987922359d08a844";
        type Request = GetTimeRequest;
        type Response = GetTimeResponse;
    }
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
        const MD5SUM: &'static str = "c1f3d28f1b044c871e6eff2e9fc3c667";
    }
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
        const MD5SUM: &'static str = "e8c90de4adc1219c86af9c2874c0c1b5";
    }
    pub struct HasParam {}
    impl ::roslibrust_codegen::RosServiceType for HasParam {
        const ROS_SERVICE_NAME: &'static str = "rosapi/HasParam";
        const MD5SUM: &'static str = "ed3df286bd6dff9b961770f577454ea9";
        type Request = HasParamRequest;
        type Response = HasParamResponse;
    }
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
        const MD5SUM: &'static str = "dc67331de85cf97091b7d45e5c64ab75";
    }
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
        const MD5SUM: &'static str = "a6b8995777f214f2ed97a1e4890feb10";
    }
    pub struct MessageDetails {}
    impl ::roslibrust_codegen::RosServiceType for MessageDetails {
        const ROS_SERVICE_NAME: &'static str = "rosapi/MessageDetails";
        const MD5SUM: &'static str = "f9c88144f6f6bd888dd99d4e0411905d";
        type Request = MessageDetailsRequest;
        type Response = MessageDetailsResponse;
    }
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
        const MD5SUM: &'static str = "a94c40e70a4b82863e6e52ec16732447";
    }
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
        const MD5SUM: &'static str = "3da1cb16c6ec5885ad291735b6244a48";
    }
    pub struct NodeDetails {}
    impl ::roslibrust_codegen::RosServiceType for NodeDetails {
        const ROS_SERVICE_NAME: &'static str = "rosapi/NodeDetails";
        const MD5SUM: &'static str = "e1d0ced5ab8d5edb5fc09c98eb1d46f6";
        type Request = NodeDetailsRequest;
        type Response = NodeDetailsResponse;
    }
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
    pub struct NodesResponse {
        pub r#nodes: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for NodesResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/NodesResponse";
        const MD5SUM: &'static str = "3d07bfda1268b4f76b16b7ba8a82665d";
    }
    pub struct Nodes {}
    impl ::roslibrust_codegen::RosServiceType for Nodes {
        const ROS_SERVICE_NAME: &'static str = "rosapi/Nodes";
        const MD5SUM: &'static str = "3d07bfda1268b4f76b16b7ba8a82665d";
        type Request = NodesRequest;
        type Response = NodesResponse;
    }
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
        const MD5SUM: &'static str = "d8f94bae31b356b24d0427f80426d0c3";
    }
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
        const MD5SUM: &'static str = "167d8030c4ca4018261dff8ae5083dc8";
    }
    pub struct Publishers {}
    impl ::roslibrust_codegen::RosServiceType for Publishers {
        const ROS_SERVICE_NAME: &'static str = "rosapi/Publishers";
        const MD5SUM: &'static str = "cb37f09944e7ba1fc08ee38f7a94291d";
        type Request = PublishersRequest;
        type Response = PublishersResponse;
    }
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
        const MD5SUM: &'static str = "c1f3d28f1b044c871e6eff2e9fc3c667";
    }
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
        const MD5SUM: &'static str = "87c264f142c2aeca13349d90aeec0386";
    }
    pub struct SearchParam {}
    impl ::roslibrust_codegen::RosServiceType for SearchParam {
        const ROS_SERVICE_NAME: &'static str = "rosapi/SearchParam";
        const MD5SUM: &'static str = "dfadc39f113c1cc6d7759508d8461d5a";
        type Request = SearchParamRequest;
        type Response = SearchParamResponse;
    }
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
        const MD5SUM: &'static str = "1cbcfa13b08f6d36710b9af8741e6112";
    }
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
        const MD5SUM: &'static str = "092ff9f63242a37704ce411703ec5eaf";
    }
    pub struct ServiceHost {}
    impl ::roslibrust_codegen::RosServiceType for ServiceHost {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceHost";
        const MD5SUM: &'static str = "a1b60006f8ee69637c856c94dd192f5a";
        type Request = ServiceHostRequest;
        type Response = ServiceHostResponse;
    }
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
        const MD5SUM: &'static str = "1cbcfa13b08f6d36710b9af8741e6112";
    }
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
        const MD5SUM: &'static str = "a94c40e70a4b82863e6e52ec16732447";
    }
    pub struct ServiceNode {}
    impl ::roslibrust_codegen::RosServiceType for ServiceNode {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceNode";
        const MD5SUM: &'static str = "bd2a0a45fd7a73a86c8d6051d5a6db8a";
        type Request = ServiceNodeRequest;
        type Response = ServiceNodeResponse;
    }
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
        const MD5SUM: &'static str = "1cbcfa13b08f6d36710b9af8741e6112";
    }
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
        const MD5SUM: &'static str = "945f6849f44f061c178ab393b12c1358";
    }
    pub struct ServiceProviders {}
    impl ::roslibrust_codegen::RosServiceType for ServiceProviders {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceProviders";
        const MD5SUM: &'static str = "f30b41d5e347454ae5483ee95eef5cc6";
        type Request = ServiceProvidersRequest;
        type Response = ServiceProvidersResponse;
    }
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
        const MD5SUM: &'static str = "dc67331de85cf97091b7d45e5c64ab75";
    }
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
        const MD5SUM: &'static str = "a6b8995777f214f2ed97a1e4890feb10";
    }
    pub struct ServiceRequestDetails {}
    impl ::roslibrust_codegen::RosServiceType for ServiceRequestDetails {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceRequestDetails";
        const MD5SUM: &'static str = "f9c88144f6f6bd888dd99d4e0411905d";
        type Request = ServiceRequestDetailsRequest;
        type Response = ServiceRequestDetailsResponse;
    }
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
        const MD5SUM: &'static str = "dc67331de85cf97091b7d45e5c64ab75";
    }
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
        const MD5SUM: &'static str = "a6b8995777f214f2ed97a1e4890feb10";
    }
    pub struct ServiceResponseDetails {}
    impl ::roslibrust_codegen::RosServiceType for ServiceResponseDetails {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceResponseDetails";
        const MD5SUM: &'static str = "f9c88144f6f6bd888dd99d4e0411905d";
        type Request = ServiceResponseDetailsRequest;
        type Response = ServiceResponseDetailsResponse;
    }
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
        const MD5SUM: &'static str = "1cbcfa13b08f6d36710b9af8741e6112";
    }
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
        const MD5SUM: &'static str = "dc67331de85cf97091b7d45e5c64ab75";
    }
    pub struct ServiceType {}
    impl ::roslibrust_codegen::RosServiceType for ServiceType {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServiceType";
        const MD5SUM: &'static str = "0e24a2dcdf70e483afc092a35a1f15f7";
        type Request = ServiceTypeRequest;
        type Response = ServiceTypeResponse;
    }
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
    pub struct ServicesResponse {
        pub r#services: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for ServicesResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/ServicesResponse";
        const MD5SUM: &'static str = "e44a7e7bcb900acadbcc28b132378f0c";
    }
    pub struct Services {}
    impl ::roslibrust_codegen::RosServiceType for Services {
        const ROS_SERVICE_NAME: &'static str = "rosapi/Services";
        const MD5SUM: &'static str = "e44a7e7bcb900acadbcc28b132378f0c";
        type Request = ServicesRequest;
        type Response = ServicesResponse;
    }
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
        const MD5SUM: &'static str = "dc67331de85cf97091b7d45e5c64ab75";
    }
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
        const MD5SUM: &'static str = "e44a7e7bcb900acadbcc28b132378f0c";
    }
    pub struct ServicesForType {}
    impl ::roslibrust_codegen::RosServiceType for ServicesForType {
        const ROS_SERVICE_NAME: &'static str = "rosapi/ServicesForType";
        const MD5SUM: &'static str = "93e9fe8ae5a9136008e260fe510bd2b0";
        type Request = ServicesForTypeRequest;
        type Response = ServicesForTypeResponse;
    }
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
        const MD5SUM: &'static str = "bc6ccc4a57f61779c8eaae61e9f422e0";
    }
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
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
    }
    pub struct SetParam {}
    impl ::roslibrust_codegen::RosServiceType for SetParam {
        const ROS_SERVICE_NAME: &'static str = "rosapi/SetParam";
        const MD5SUM: &'static str = "bc6ccc4a57f61779c8eaae61e9f422e0";
        type Request = SetParamRequest;
        type Response = SetParamResponse;
    }
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
        const MD5SUM: &'static str = "d8f94bae31b356b24d0427f80426d0c3";
    }
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
        const MD5SUM: &'static str = "22418cab5ba9531d8c2b738b4e56153b";
    }
    pub struct Subscribers {}
    impl ::roslibrust_codegen::RosServiceType for Subscribers {
        const ROS_SERVICE_NAME: &'static str = "rosapi/Subscribers";
        const MD5SUM: &'static str = "cb387b68f5b29bc1456398ee8476b973";
        type Request = SubscribersRequest;
        type Response = SubscribersResponse;
    }
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
        const MD5SUM: &'static str = "d8f94bae31b356b24d0427f80426d0c3";
    }
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
        const MD5SUM: &'static str = "dc67331de85cf97091b7d45e5c64ab75";
    }
    pub struct TopicType {}
    impl ::roslibrust_codegen::RosServiceType for TopicType {
        const ROS_SERVICE_NAME: &'static str = "rosapi/TopicType";
        const MD5SUM: &'static str = "0d30b3f53a0fd5036523a7141e524ddf";
        type Request = TopicTypeRequest;
        type Response = TopicTypeResponse;
    }
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
    pub struct TopicsResponse {
        pub r#topics: ::std::vec::Vec<::std::string::String>,
        pub r#types: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for TopicsResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/TopicsResponse";
        const MD5SUM: &'static str = "d966d98fc333fa1f3135af765eac1ba8";
    }
    pub struct Topics {}
    impl ::roslibrust_codegen::RosServiceType for Topics {
        const ROS_SERVICE_NAME: &'static str = "rosapi/Topics";
        const MD5SUM: &'static str = "d966d98fc333fa1f3135af765eac1ba8";
        type Request = TopicsRequest;
        type Response = TopicsResponse;
    }
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
    pub struct TopicsAndRawTypesResponse {
        pub r#topics: ::std::vec::Vec<::std::string::String>,
        pub r#types: ::std::vec::Vec<::std::string::String>,
        pub r#typedefs_full_text: ::std::vec::Vec<::std::string::String>,
    }
    impl ::roslibrust_codegen::RosMessageType for TopicsAndRawTypesResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/TopicsAndRawTypesResponse";
        const MD5SUM: &'static str = "e1432466c8f64316723276ba07c59d12";
    }
    pub struct TopicsAndRawTypes {}
    impl ::roslibrust_codegen::RosServiceType for TopicsAndRawTypes {
        const ROS_SERVICE_NAME: &'static str = "rosapi/TopicsAndRawTypes";
        const MD5SUM: &'static str = "e1432466c8f64316723276ba07c59d12";
        type Request = TopicsAndRawTypesRequest;
        type Response = TopicsAndRawTypesResponse;
    }
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
        const MD5SUM: &'static str = "dc67331de85cf97091b7d45e5c64ab75";
    }
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
        const MD5SUM: &'static str = "b0eef9a05d4e829092fc2f2c3c2aad3d";
    }
    pub struct TopicsForType {}
    impl ::roslibrust_codegen::RosServiceType for TopicsForType {
        const ROS_SERVICE_NAME: &'static str = "rosapi/TopicsForType";
        const MD5SUM: &'static str = "56f77ff6da756dd27c1ed16ec721072a";
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
        const MD5SUM: &'static str = "a9c97c1d230cfc112e270351a944ee47";
    }
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
        const MD5SUM: &'static str = "acffd30cd6b6de30f120938c17c593fb";
    }
    impl Log {
        pub const r#DEBUG: u8 = 1u8;
        pub const r#INFO: u8 = 2u8;
        pub const r#WARN: u8 = 4u8;
        pub const r#ERROR: u8 = 8u8;
        pub const r#FATAL: u8 = 16u8;
    }
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
        const MD5SUM: &'static str = "10152ed868c5097a5e2e4a89d7daa710";
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
        const MD5SUM: &'static str = "4ddae7f048e32fda22cac764685e3974";
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
        const MD5SUM: &'static str = "c9a58c1b0b154e0e6da7578cb991d214";
    }
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
        const MD5SUM: &'static str = "8f7a12909da2c9d3332d540a0977563f";
    }
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
        const MD5SUM: &'static str = "804dc5cea1c5306d6a2eb80b9833befe";
    }
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
        const MD5SUM: &'static str = "8cf5febb0952fca9d650c3d11a81a188";
    }
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
        const MD5SUM: &'static str = "060021388200f6f0f447d0fcd9c64743";
    }
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
        const MD5SUM: &'static str = "6a62c6daae103f4ff57a132d6f95cec2";
    }
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
        const MD5SUM: &'static str = "3066dcd76a6cfaef579bd0f34173e9fd";
    }
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
        const MD5SUM: &'static str = "5a9ea5f83505693b71e785041e67a8bb";
    }
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
        const MD5SUM: &'static str = "90c7ef2dc6895d81024acba2ac42f369";
    }
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
        const MD5SUM: &'static str = "2f3b0b43eed0c9501de0fa3ff89a45aa";
    }
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
        const MD5SUM: &'static str = "690f272f0640d2631c305eeb8301e59d";
    }
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
        const MD5SUM: &'static str = "6fefb0c6da89d7c8abe4b339f5c2f8fb";
    }
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
        const MD5SUM: &'static str = "2d3a8cd499b9b4a0249fb98fd05cfa48";
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
        const MD5SUM: &'static str = "d8e9c3f5afbdd8a130fd1d2763945fca";
    }
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
        const MD5SUM: &'static str = "1158d486dd51d683ce2f1be655c3c181";
    }
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
        const MD5SUM: &'static str = "c005c34273dc426c67a020a87bc24148";
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
        const MD5SUM: &'static str = "8730015b05955b7e992ce29a2678d90f";
    }
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
        const MD5SUM: &'static str = "ff71b307acdbe7c871a5a6d7ed359100";
    }
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
        const MD5SUM: &'static str = "fded64a0265108ba86c3d38fb11c0c16";
    }
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
        const MD5SUM: &'static str = "ee34be01fdeee563d0d99cd594d5581d";
    }
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
        const MD5SUM: &'static str = "bef1df590ed75ed1f393692395e15482";
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
        const MD5SUM: &'static str = "1ffdae9486cd3316a121c578b47a85cc";
    }
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
        const MD5SUM: &'static str = "23688b2e6d2de3d32fe8af104a903253";
    }
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
        const MD5SUM: &'static str = "2c1b92ed8f31492f8e73f6a4a44ca796";
    }
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
        const MD5SUM: &'static str = "d8f8cbc74c5ff283fca29569ccefb45d";
    }
    impl SolidPrimitive {
        pub const r#BOX: u8 = 1u8;
        pub const r#SPHERE: u8 = 2u8;
        pub const r#CYLINDER: u8 = 3u8;
        pub const r#CONE: u8 = 4u8;
        pub const r#BOX_X: u8 = 0u8;
        pub const r#BOX_Y: u8 = 1u8;
        pub const r#BOX_Z: u8 = 2u8;
        pub const r#SPHERE_RADIUS: u8 = 0u8;
        pub const r#CYLINDER_HEIGHT: u8 = 0u8;
        pub const r#CYLINDER_RADIUS: u8 = 1u8;
        pub const r#CONE_HEIGHT: u8 = 0u8;
        pub const r#CONE_RADIUS: u8 = 1u8;
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
    pub struct Duration {
        pub r#data: ::roslibrust_codegen::integral_types::Duration,
    }
    impl ::roslibrust_codegen::RosMessageType for Duration {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Duration";
        const MD5SUM: &'static str = "3e286caf4241d664e55f3ad380e2ae46";
    }
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
        pub r#seq: u32,
        pub r#stamp: ::roslibrust_codegen::integral_types::Time,
        pub r#frame_id: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for Header {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Header";
        const MD5SUM: &'static str = "2176decaecbce78abc3b96ef049fabed";
    }
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
    pub struct Time {
        pub r#data: ::roslibrust_codegen::integral_types::Time,
    }
    impl ::roslibrust_codegen::RosMessageType for Time {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Time";
        const MD5SUM: &'static str = "cd7166c74c552c311fbcc2fe5a7bc289";
    }
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
        const MD5SUM: &'static str = "04a177815f75271039fa21f16acad8c9";
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
    pub struct Constants {}
    impl ::roslibrust_codegen::RosMessageType for Constants {
        const ROS_TYPE_NAME: &'static str = "test_msgs/Constants";
        const MD5SUM: &'static str = "941e5be5614c8663086b50a15b29668b";
    }
    impl Constants {
        pub const r#TEST_STR: &'static str = "/topic";
        pub const r#TEST_STR_2: &'static str = "/topic_2";
        pub const r#TEST_FLOAT: f32 = 0f32;
    }
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
        const MD5SUM: &'static str = "d053817de0764f9ee90dbc89c4cdd751";
    }
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
        const MD5SUM: &'static str = "097b0e938d0dd7788057f4cdc9013238";
    }
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
        const MD5SUM: &'static str = "474be567370f515a7d5d3f3243aad369";
    }
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
        const MD5SUM: &'static str = "a681f679e1c39fbe570b7737e7cf183d";
    }
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
        const MD5SUM: &'static str = "7fab1acc377fd48898b00b7f3a897f47";
    }
    impl NodeInfo {
        pub const r#STATUS_UNINITIALIZED: u8 = 0u8;
        pub const r#STATUS_DISCONNECTED: u8 = 1u8;
        pub const r#STATUS_RUNNING: u8 = 2u8;
        pub const r#STATUS_RUN_ERROR: u8 = 3u8;
        pub const r#STATUS_SHUTTING_DOWN: u8 = 4u8;
        pub const r#STATUS_SHUTDOWN: u8 = 5u8;
    }
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
        const MD5SUM: &'static str = "36d09b846be0b371c5f190354dd3153e";
    }
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
        const MD5SUM: &'static str = "b88405221c77b1878a3cbbfff53428d7";
    }
    pub struct AddTwoInts {}
    impl ::roslibrust_codegen::RosServiceType for AddTwoInts {
        const ROS_SERVICE_NAME: &'static str = "test_msgs/AddTwoInts";
        const MD5SUM: &'static str = "6a2e34150c00229791cc89ff309fff21";
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
        const MD5SUM: &'static str = "65b4f94a94d1ed67169da35a02f33d3f";
    }
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
        const MD5SUM: &'static str = "f3cd1e1c4d320c79d6985c904ae5dcd3";
    }
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
        const MD5SUM: &'static str = "ef145a45a5f47b77b7f5cdde4b16c942";
    }
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
        const MD5SUM: &'static str = "3ebe08d1abd5b65862d50e09430db776";
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
        const MD5SUM: &'static str = "1de93c67ec8858b831025a08fbf1b35c";
    }
    impl ImageMarker {
        pub const r#CIRCLE: u8 = 0u8;
        pub const r#LINE_STRIP: u8 = 1u8;
        pub const r#LINE_LIST: u8 = 2u8;
        pub const r#POLYGON: u8 = 3u8;
        pub const r#POINTS: u8 = 4u8;
        pub const r#ADD: u8 = 0u8;
        pub const r#REMOVE: u8 = 1u8;
    }
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
        const MD5SUM: &'static str = "dd86d22909d5a3364b384492e35c10af";
    }
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
        const MD5SUM: &'static str = "b3c81e785788195d1840b86c28da1aac";
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
        const MD5SUM: &'static str = "ab0f1eee058667e28c19ff3ffc3f4b78";
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
        const MD5SUM: &'static str = "d5f2c5045a72456d228676ab91048734";
    }
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
        const MD5SUM: &'static str = "a6e6833209a196a38d798dadb02c81f8";
    }
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
        const MD5SUM: &'static str = "710d308d0a9276d65945e92dd30b3946";
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
        pub r#text: ::std::string::String,
        pub r#mesh_resource: ::std::string::String,
        pub r#mesh_use_embedded_materials: bool,
    }
    impl ::roslibrust_codegen::RosMessageType for Marker {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/Marker";
        const MD5SUM: &'static str = "4048c9de2a16f4ae8e0538085ebf1b97";
    }
    impl Marker {
        pub const r#ARROW: u8 = 0u8;
        pub const r#CUBE: u8 = 1u8;
        pub const r#SPHERE: u8 = 2u8;
        pub const r#CYLINDER: u8 = 3u8;
        pub const r#LINE_STRIP: u8 = 4u8;
        pub const r#LINE_LIST: u8 = 5u8;
        pub const r#CUBE_LIST: u8 = 6u8;
        pub const r#SPHERE_LIST: u8 = 7u8;
        pub const r#POINTS: u8 = 8u8;
        pub const r#TEXT_VIEW_FACING: u8 = 9u8;
        pub const r#MESH_RESOURCE: u8 = 10u8;
        pub const r#TRIANGLE_LIST: u8 = 11u8;
        pub const r#ADD: u8 = 0u8;
        pub const r#MODIFY: u8 = 0u8;
        pub const r#DELETE: u8 = 2u8;
        pub const r#DELETEALL: u8 = 3u8;
    }
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
        const MD5SUM: &'static str = "d155b9ce5188fbaf89745847fd5882d7";
    }
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
}
