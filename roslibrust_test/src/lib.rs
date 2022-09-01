#[allow(unused_imports)]
pub mod actionlib_msgs {
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct GoalID {
        pub stamp: std_msgs::TimeI,
        pub id: std::string::String,
    }
    impl ::roslibrust::RosMessageType for GoalID {
        const ROS_TYPE_NAME: &'static str = "actionlib_msgs/GoalID";
    }
    impl GoalID {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct GoalStatus {
        pub goal_id: self::GoalID,
        pub status: u8,
        pub text: std::string::String,
    }
    impl ::roslibrust::RosMessageType for GoalStatus {
        const ROS_TYPE_NAME: &'static str = "actionlib_msgs/GoalStatus";
    }
    impl GoalStatus {
        pub const PENDING: u8 = 0;
        pub const ACTIVE: u8 = 1;
        pub const PREEMPTED: u8 = 2;
        pub const SUCCEEDED: u8 = 3;
        pub const ABORTED: u8 = 4;
        pub const REJECTED: u8 = 5;
        pub const PREEMPTING: u8 = 6;
        pub const RECALLING: u8 = 7;
        pub const RECALLED: u8 = 8;
        pub const LOST: u8 = 9;
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct GoalStatusArray {
        pub header: std_msgs::Header,
        pub status_list: std::vec::Vec<self::GoalStatus>,
    }
    impl ::roslibrust::RosMessageType for GoalStatusArray {
        const ROS_TYPE_NAME: &'static str = "actionlib_msgs/GoalStatusArray";
    }
    impl GoalStatusArray {}
}
#[allow(unused_imports)]
pub mod diagnostic_msgs {
    use super::actionlib_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct AddDiagnosticsRequest {
        pub load_namespace: std::string::String,
    }
    impl ::roslibrust::RosMessageType for AddDiagnosticsRequest {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/AddDiagnosticsRequest";
    }
    impl AddDiagnosticsRequest {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct AddDiagnosticsResponse {
        pub success: bool,
        pub message: std::string::String,
    }
    impl ::roslibrust::RosMessageType for AddDiagnosticsResponse {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/AddDiagnosticsResponse";
    }
    impl AddDiagnosticsResponse {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct DiagnosticArray {
        pub header: std_msgs::Header,
        pub status: std::vec::Vec<self::DiagnosticStatus>,
    }
    impl ::roslibrust::RosMessageType for DiagnosticArray {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/DiagnosticArray";
    }
    impl DiagnosticArray {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct DiagnosticStatus {
        pub level: u8,
        pub name: std::string::String,
        pub message: std::string::String,
        pub hardware_id: std::string::String,
        pub values: std::vec::Vec<self::KeyValue>,
    }
    impl ::roslibrust::RosMessageType for DiagnosticStatus {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/DiagnosticStatus";
    }
    impl DiagnosticStatus {
        pub const OK: u8 = 0;
        pub const WARN: u8 = 1;
        pub const ERROR: u8 = 2;
        pub const STALE: u8 = 3;
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct KeyValue {
        pub key: std::string::String,
        pub value: std::string::String,
    }
    impl ::roslibrust::RosMessageType for KeyValue {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/KeyValue";
    }
    impl KeyValue {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct SelfTestRequest {}
    impl ::roslibrust::RosMessageType for SelfTestRequest {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/SelfTestRequest";
    }
    impl SelfTestRequest {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct SelfTestResponse {
        pub id: std::string::String,
        pub passed: u8,
        pub status: std::vec::Vec<self::DiagnosticStatus>,
    }
    impl ::roslibrust::RosMessageType for SelfTestResponse {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/SelfTestResponse";
    }
    impl SelfTestResponse {}
    pub struct AddDiagnostics {}
    impl ::roslibrust::RosServiceType for AddDiagnostics {
        const ROS_SERVICE_NAME: &'static str = "AddDiagnostics";
        type Request = AddDiagnosticsRequest;
        type Response = AddDiagnosticsResponse;
    }
    pub struct SelfTest {}
    impl ::roslibrust::RosServiceType for SelfTest {
        const ROS_SERVICE_NAME: &'static str = "SelfTest";
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
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Accel {
        pub linear: self::Vector3,
        pub angular: self::Vector3,
    }
    impl ::roslibrust::RosMessageType for Accel {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Accel";
    }
    impl Accel {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct AccelStamped {
        pub header: std_msgs::Header,
        pub accel: self::Accel,
    }
    impl ::roslibrust::RosMessageType for AccelStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/AccelStamped";
    }
    impl AccelStamped {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct AccelWithCovariance {
        pub accel: self::Accel,
        pub covariance: std::vec::Vec<f64>,
    }
    impl ::roslibrust::RosMessageType for AccelWithCovariance {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/AccelWithCovariance";
    }
    impl AccelWithCovariance {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct AccelWithCovarianceStamped {
        pub header: std_msgs::Header,
        pub accel: self::AccelWithCovariance,
    }
    impl ::roslibrust::RosMessageType for AccelWithCovarianceStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/AccelWithCovarianceStamped";
    }
    impl AccelWithCovarianceStamped {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Inertia {
        pub m: f64,
        pub com: self::Vector3,
        pub ixx: f64,
        pub ixy: f64,
        pub ixz: f64,
        pub iyy: f64,
        pub iyz: f64,
        pub izz: f64,
    }
    impl ::roslibrust::RosMessageType for Inertia {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Inertia";
    }
    impl Inertia {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct InertiaStamped {
        pub header: std_msgs::Header,
        pub inertia: self::Inertia,
    }
    impl ::roslibrust::RosMessageType for InertiaStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/InertiaStamped";
    }
    impl InertiaStamped {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Point {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }
    impl ::roslibrust::RosMessageType for Point {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Point";
    }
    impl Point {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Point32 {
        pub x: f32,
        pub y: f32,
        pub z: f32,
    }
    impl ::roslibrust::RosMessageType for Point32 {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Point32";
    }
    impl Point32 {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct PointStamped {
        pub header: std_msgs::Header,
        pub point: self::Point,
    }
    impl ::roslibrust::RosMessageType for PointStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PointStamped";
    }
    impl PointStamped {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Polygon {
        pub points: std::vec::Vec<self::Point32>,
    }
    impl ::roslibrust::RosMessageType for Polygon {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Polygon";
    }
    impl Polygon {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct PolygonStamped {
        pub header: std_msgs::Header,
        pub polygon: self::Polygon,
    }
    impl ::roslibrust::RosMessageType for PolygonStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PolygonStamped";
    }
    impl PolygonStamped {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Pose {
        pub position: self::Point,
        pub orientation: self::Quaternion,
    }
    impl ::roslibrust::RosMessageType for Pose {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Pose";
    }
    impl Pose {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Pose2D {
        pub x: f64,
        pub y: f64,
        pub theta: f64,
    }
    impl ::roslibrust::RosMessageType for Pose2D {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Pose2D";
    }
    impl Pose2D {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct PoseArray {
        pub header: std_msgs::Header,
        pub poses: std::vec::Vec<self::Pose>,
    }
    impl ::roslibrust::RosMessageType for PoseArray {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseArray";
    }
    impl PoseArray {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct PoseStamped {
        pub header: std_msgs::Header,
        pub pose: self::Pose,
    }
    impl ::roslibrust::RosMessageType for PoseStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseStamped";
    }
    impl PoseStamped {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct PoseWithCovariance {
        pub pose: self::Pose,
        pub covariance: std::vec::Vec<f64>,
    }
    impl ::roslibrust::RosMessageType for PoseWithCovariance {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseWithCovariance";
    }
    impl PoseWithCovariance {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct PoseWithCovarianceStamped {
        pub header: std_msgs::Header,
        pub pose: self::PoseWithCovariance,
    }
    impl ::roslibrust::RosMessageType for PoseWithCovarianceStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseWithCovarianceStamped";
    }
    impl PoseWithCovarianceStamped {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Quaternion {
        pub x: f64,
        pub y: f64,
        pub z: f64,
        pub w: f64,
    }
    impl ::roslibrust::RosMessageType for Quaternion {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Quaternion";
    }
    impl Quaternion {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct QuaternionStamped {
        pub header: std_msgs::Header,
        pub quaternion: self::Quaternion,
    }
    impl ::roslibrust::RosMessageType for QuaternionStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/QuaternionStamped";
    }
    impl QuaternionStamped {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Transform {
        pub translation: self::Vector3,
        pub rotation: self::Quaternion,
    }
    impl ::roslibrust::RosMessageType for Transform {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Transform";
    }
    impl Transform {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct TransformStamped {
        pub header: std_msgs::Header,
        pub child_frame_id: std::string::String,
        pub transform: self::Transform,
    }
    impl ::roslibrust::RosMessageType for TransformStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/TransformStamped";
    }
    impl TransformStamped {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Twist {
        pub linear: self::Vector3,
        pub angular: self::Vector3,
    }
    impl ::roslibrust::RosMessageType for Twist {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Twist";
    }
    impl Twist {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct TwistStamped {
        pub header: std_msgs::Header,
        pub twist: self::Twist,
    }
    impl ::roslibrust::RosMessageType for TwistStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/TwistStamped";
    }
    impl TwistStamped {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct TwistWithCovariance {
        pub twist: self::Twist,
        pub covariance: std::vec::Vec<f64>,
    }
    impl ::roslibrust::RosMessageType for TwistWithCovariance {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/TwistWithCovariance";
    }
    impl TwistWithCovariance {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct TwistWithCovarianceStamped {
        pub header: std_msgs::Header,
        pub twist: self::TwistWithCovariance,
    }
    impl ::roslibrust::RosMessageType for TwistWithCovarianceStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/TwistWithCovarianceStamped";
    }
    impl TwistWithCovarianceStamped {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Vector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }
    impl ::roslibrust::RosMessageType for Vector3 {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Vector3";
    }
    impl Vector3 {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Vector3Stamped {
        pub header: std_msgs::Header,
        pub vector: self::Vector3,
    }
    impl ::roslibrust::RosMessageType for Vector3Stamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Vector3Stamped";
    }
    impl Vector3Stamped {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Wrench {
        pub force: self::Vector3,
        pub torque: self::Vector3,
    }
    impl ::roslibrust::RosMessageType for Wrench {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Wrench";
    }
    impl Wrench {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct WrenchStamped {
        pub header: std_msgs::Header,
        pub wrench: self::Wrench,
    }
    impl ::roslibrust::RosMessageType for WrenchStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/WrenchStamped";
    }
    impl WrenchStamped {}
}
#[allow(unused_imports)]
pub mod nav_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::rosapi;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct GetMapRequest {}
    impl ::roslibrust::RosMessageType for GetMapRequest {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetMapRequest";
    }
    impl GetMapRequest {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct GetMapResponse {
        pub map: self::OccupancyGrid,
    }
    impl ::roslibrust::RosMessageType for GetMapResponse {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetMapResponse";
    }
    impl GetMapResponse {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct GetPlanRequest {
        pub start: geometry_msgs::PoseStamped,
        pub goal: geometry_msgs::PoseStamped,
        pub tolerance: f32,
    }
    impl ::roslibrust::RosMessageType for GetPlanRequest {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetPlanRequest";
    }
    impl GetPlanRequest {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct GetPlanResponse {
        pub plan: self::Path,
    }
    impl ::roslibrust::RosMessageType for GetPlanResponse {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetPlanResponse";
    }
    impl GetPlanResponse {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct GridCells {
        pub header: std_msgs::Header,
        pub cell_width: f32,
        pub cell_height: f32,
        pub cells: std::vec::Vec<geometry_msgs::Point>,
    }
    impl ::roslibrust::RosMessageType for GridCells {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GridCells";
    }
    impl GridCells {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct LoadMapRequest {
        pub map_url: std::string::String,
    }
    impl ::roslibrust::RosMessageType for LoadMapRequest {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/LoadMapRequest";
    }
    impl LoadMapRequest {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct LoadMapResponse {
        pub map: self::OccupancyGrid,
        pub result: u8,
    }
    impl ::roslibrust::RosMessageType for LoadMapResponse {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/LoadMapResponse";
    }
    impl LoadMapResponse {
        pub const RESULT_SUCCESS: u8 = 0;
        pub const RESULT_MAP_DOES_NOT_EXIST: u8 = 1;
        pub const RESULT_INVALID_MAP_DATA: u8 = 2;
        pub const RESULT_INVALID_MAP_METADATA: u8 = 3;
        pub const RESULT_UNDEFINED_FAILURE: u8 = 255;
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct MapMetaData {
        pub map_load_time: std_msgs::TimeI,
        pub resolution: f32,
        pub width: u32,
        pub height: u32,
        pub origin: geometry_msgs::Pose,
    }
    impl ::roslibrust::RosMessageType for MapMetaData {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/MapMetaData";
    }
    impl MapMetaData {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct OccupancyGrid {
        pub header: std_msgs::Header,
        pub info: self::MapMetaData,
        pub data: std::vec::Vec<i8>,
    }
    impl ::roslibrust::RosMessageType for OccupancyGrid {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/OccupancyGrid";
    }
    impl OccupancyGrid {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Odometry {
        pub header: std_msgs::Header,
        pub child_frame_id: std::string::String,
        pub pose: geometry_msgs::PoseWithCovariance,
        pub twist: geometry_msgs::TwistWithCovariance,
    }
    impl ::roslibrust::RosMessageType for Odometry {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/Odometry";
    }
    impl Odometry {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Path {
        pub header: std_msgs::Header,
        pub poses: std::vec::Vec<geometry_msgs::PoseStamped>,
    }
    impl ::roslibrust::RosMessageType for Path {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/Path";
    }
    impl Path {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct SetMapRequest {
        pub map: self::OccupancyGrid,
        pub initial_pose: geometry_msgs::PoseWithCovarianceStamped,
    }
    impl ::roslibrust::RosMessageType for SetMapRequest {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/SetMapRequest";
    }
    impl SetMapRequest {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct SetMapResponse {
        pub success: bool,
    }
    impl ::roslibrust::RosMessageType for SetMapResponse {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/SetMapResponse";
    }
    impl SetMapResponse {}
    pub struct GetPlan {}
    impl ::roslibrust::RosServiceType for GetPlan {
        const ROS_SERVICE_NAME: &'static str = "GetPlan";
        type Request = GetPlanRequest;
        type Response = GetPlanResponse;
    }
    pub struct GetMap {}
    impl ::roslibrust::RosServiceType for GetMap {
        const ROS_SERVICE_NAME: &'static str = "GetMap";
        type Request = GetMapRequest;
        type Response = GetMapResponse;
    }
    pub struct SetMap {}
    impl ::roslibrust::RosServiceType for SetMap {
        const ROS_SERVICE_NAME: &'static str = "SetMap";
        type Request = SetMapRequest;
        type Response = SetMapResponse;
    }
    pub struct LoadMap {}
    impl ::roslibrust::RosServiceType for LoadMap {
        const ROS_SERVICE_NAME: &'static str = "LoadMap";
        type Request = LoadMapRequest;
        type Response = LoadMapResponse;
    }
}
#[allow(unused_imports)]
pub mod rosapi {
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
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct GetTimeRequest {}
    impl ::roslibrust::RosMessageType for GetTimeRequest {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetTimeRequest";
    }
    impl GetTimeRequest {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct GetTimeResponse {
        pub data: std_msgs::TimeI,
    }
    impl ::roslibrust::RosMessageType for GetTimeResponse {
        const ROS_TYPE_NAME: &'static str = "rosapi/GetTimeResponse";
    }
    impl GetTimeResponse {}
    pub struct GetTime {}
    impl ::roslibrust::RosServiceType for GetTime {
        const ROS_SERVICE_NAME: &'static str = "GetTime";
        type Request = GetTimeRequest;
        type Response = GetTimeResponse;
    }
}
#[allow(unused_imports)]
pub mod sensor_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::shape_msgs;
    use super::std_msgs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct BatteryState {
        pub header: std_msgs::Header,
        pub voltage: f32,
        pub temperature: f32,
        pub current: f32,
        pub charge: f32,
        pub capacity: f32,
        pub design_capacity: f32,
        pub percentage: f32,
        pub power_supply_status: u8,
        pub power_supply_health: u8,
        pub power_supply_technology: u8,
        pub present: bool,
        pub cell_voltage: std::vec::Vec<f32>,
        pub cell_temperature: std::vec::Vec<f32>,
        pub location: std::string::String,
        pub serial_number: std::string::String,
    }
    impl ::roslibrust::RosMessageType for BatteryState {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/BatteryState";
    }
    impl BatteryState {
        pub const POWER_SUPPLY_STATUS_UNKNOWN: u8 = 0;
        pub const POWER_SUPPLY_STATUS_CHARGING: u8 = 1;
        pub const POWER_SUPPLY_STATUS_DISCHARGING: u8 = 2;
        pub const POWER_SUPPLY_STATUS_NOT_CHARGING: u8 = 3;
        pub const POWER_SUPPLY_STATUS_FULL: u8 = 4;
        pub const POWER_SUPPLY_HEALTH_UNKNOWN: u8 = 0;
        pub const POWER_SUPPLY_HEALTH_GOOD: u8 = 1;
        pub const POWER_SUPPLY_HEALTH_OVERHEAT: u8 = 2;
        pub const POWER_SUPPLY_HEALTH_DEAD: u8 = 3;
        pub const POWER_SUPPLY_HEALTH_OVERVOLTAGE: u8 = 4;
        pub const POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: u8 = 5;
        pub const POWER_SUPPLY_HEALTH_COLD: u8 = 6;
        pub const POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE: u8 = 7;
        pub const POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE: u8 = 8;
        pub const POWER_SUPPLY_TECHNOLOGY_UNKNOWN: u8 = 0;
        pub const POWER_SUPPLY_TECHNOLOGY_NIMH: u8 = 1;
        pub const POWER_SUPPLY_TECHNOLOGY_LION: u8 = 2;
        pub const POWER_SUPPLY_TECHNOLOGY_LIPO: u8 = 3;
        pub const POWER_SUPPLY_TECHNOLOGY_LIFE: u8 = 4;
        pub const POWER_SUPPLY_TECHNOLOGY_NICD: u8 = 5;
        pub const POWER_SUPPLY_TECHNOLOGY_LIMN: u8 = 6;
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct CameraInfo {
        pub header: std_msgs::Header,
        pub height: u32,
        pub width: u32,
        pub distortion_model: std::string::String,
        pub D: std::vec::Vec<f64>,
        pub K: std::vec::Vec<f64>,
        pub R: std::vec::Vec<f64>,
        pub P: std::vec::Vec<f64>,
        pub binning_x: u32,
        pub binning_y: u32,
        pub roi: self::RegionOfInterest,
    }
    impl ::roslibrust::RosMessageType for CameraInfo {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/CameraInfo";
    }
    impl CameraInfo {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct ChannelFloat32 {
        pub name: std::string::String,
        pub values: std::vec::Vec<f32>,
    }
    impl ::roslibrust::RosMessageType for ChannelFloat32 {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/ChannelFloat32";
    }
    impl ChannelFloat32 {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct CompressedImage {
        pub header: std_msgs::Header,
        pub format: std::string::String,
        pub data: std::vec::Vec<u8>,
    }
    impl ::roslibrust::RosMessageType for CompressedImage {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/CompressedImage";
    }
    impl CompressedImage {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct FluidPressure {
        pub header: std_msgs::Header,
        pub fluid_pressure: f64,
        pub variance: f64,
    }
    impl ::roslibrust::RosMessageType for FluidPressure {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/FluidPressure";
    }
    impl FluidPressure {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Illuminance {
        pub header: std_msgs::Header,
        pub illuminance: f64,
        pub variance: f64,
    }
    impl ::roslibrust::RosMessageType for Illuminance {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Illuminance";
    }
    impl Illuminance {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Image {
        pub header: std_msgs::Header,
        pub height: u32,
        pub width: u32,
        pub encoding: std::string::String,
        pub is_bigendian: u8,
        pub step: u32,
        pub data: std::vec::Vec<u8>,
    }
    impl ::roslibrust::RosMessageType for Image {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Image";
    }
    impl Image {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Imu {
        pub header: std_msgs::Header,
        pub orientation: geometry_msgs::Quaternion,
        pub orientation_covariance: std::vec::Vec<f64>,
        pub angular_velocity: geometry_msgs::Vector3,
        pub angular_velocity_covariance: std::vec::Vec<f64>,
        pub linear_acceleration: geometry_msgs::Vector3,
        pub linear_acceleration_covariance: std::vec::Vec<f64>,
    }
    impl ::roslibrust::RosMessageType for Imu {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Imu";
    }
    impl Imu {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct JointState {
        pub header: std_msgs::Header,
        pub name: std::vec::Vec<std::string::String>,
        pub position: std::vec::Vec<f64>,
        pub velocity: std::vec::Vec<f64>,
        pub effort: std::vec::Vec<f64>,
    }
    impl ::roslibrust::RosMessageType for JointState {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/JointState";
    }
    impl JointState {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Joy {
        pub header: std_msgs::Header,
        pub axes: std::vec::Vec<f32>,
        pub buttons: std::vec::Vec<i32>,
    }
    impl ::roslibrust::RosMessageType for Joy {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Joy";
    }
    impl Joy {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct JoyFeedback {
        pub r#type: u8,
        pub id: u8,
        pub intensity: f32,
    }
    impl ::roslibrust::RosMessageType for JoyFeedback {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/JoyFeedback";
    }
    impl JoyFeedback {
        pub const TYPE_LED: u8 = 0;
        pub const TYPE_RUMBLE: u8 = 1;
        pub const TYPE_BUZZER: u8 = 2;
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct JoyFeedbackArray {
        pub array: std::vec::Vec<self::JoyFeedback>,
    }
    impl ::roslibrust::RosMessageType for JoyFeedbackArray {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/JoyFeedbackArray";
    }
    impl JoyFeedbackArray {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct LaserEcho {
        pub echoes: std::vec::Vec<f32>,
    }
    impl ::roslibrust::RosMessageType for LaserEcho {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/LaserEcho";
    }
    impl LaserEcho {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct LaserScan {
        pub header: std_msgs::Header,
        pub angle_min: f32,
        pub angle_max: f32,
        pub angle_increment: f32,
        pub time_increment: f32,
        pub scan_time: f32,
        pub range_min: f32,
        pub range_max: f32,
        pub ranges: std::vec::Vec<f32>,
        pub intensities: std::vec::Vec<f32>,
    }
    impl ::roslibrust::RosMessageType for LaserScan {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/LaserScan";
    }
    impl LaserScan {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct MagneticField {
        pub header: std_msgs::Header,
        pub magnetic_field: geometry_msgs::Vector3,
        pub magnetic_field_covariance: std::vec::Vec<f64>,
    }
    impl ::roslibrust::RosMessageType for MagneticField {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/MagneticField";
    }
    impl MagneticField {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct MultiDOFJointState {
        pub header: std_msgs::Header,
        pub joint_names: std::vec::Vec<std::string::String>,
        pub transforms: std::vec::Vec<geometry_msgs::Transform>,
        pub twist: std::vec::Vec<geometry_msgs::Twist>,
        pub wrench: std::vec::Vec<geometry_msgs::Wrench>,
    }
    impl ::roslibrust::RosMessageType for MultiDOFJointState {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/MultiDOFJointState";
    }
    impl MultiDOFJointState {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct MultiEchoLaserScan {
        pub header: std_msgs::Header,
        pub angle_min: f32,
        pub angle_max: f32,
        pub angle_increment: f32,
        pub time_increment: f32,
        pub scan_time: f32,
        pub range_min: f32,
        pub range_max: f32,
        pub ranges: std::vec::Vec<self::LaserEcho>,
        pub intensities: std::vec::Vec<self::LaserEcho>,
    }
    impl ::roslibrust::RosMessageType for MultiEchoLaserScan {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/MultiEchoLaserScan";
    }
    impl MultiEchoLaserScan {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct NavSatFix {
        pub header: std_msgs::Header,
        pub status: self::NavSatStatus,
        pub latitude: f64,
        pub longitude: f64,
        pub altitude: f64,
        pub position_covariance: std::vec::Vec<f64>,
        pub position_covariance_type: u8,
    }
    impl ::roslibrust::RosMessageType for NavSatFix {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/NavSatFix";
    }
    impl NavSatFix {
        pub const COVARIANCE_TYPE_UNKNOWN: u8 = 0;
        pub const COVARIANCE_TYPE_APPROXIMATED: u8 = 1;
        pub const COVARIANCE_TYPE_DIAGONAL_KNOWN: u8 = 2;
        pub const COVARIANCE_TYPE_KNOWN: u8 = 3;
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct NavSatStatus {
        pub status: i8,
        pub service: u16,
    }
    impl ::roslibrust::RosMessageType for NavSatStatus {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/NavSatStatus";
    }
    impl NavSatStatus {
        pub const STATUS_NO_FIX: i8 = -1;
        pub const STATUS_FIX: i8 = 0;
        pub const STATUS_SBAS_FIX: i8 = 1;
        pub const STATUS_GBAS_FIX: i8 = 2;
        pub const SERVICE_GPS: u16 = 1;
        pub const SERVICE_GLONASS: u16 = 2;
        pub const SERVICE_COMPASS: u16 = 4;
        pub const SERVICE_GALILEO: u16 = 8;
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct PointCloud {
        pub header: std_msgs::Header,
        pub points: std::vec::Vec<geometry_msgs::Point32>,
        pub channels: std::vec::Vec<self::ChannelFloat32>,
    }
    impl ::roslibrust::RosMessageType for PointCloud {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/PointCloud";
    }
    impl PointCloud {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct PointCloud2 {
        pub header: std_msgs::Header,
        pub height: u32,
        pub width: u32,
        pub fields: std::vec::Vec<self::PointField>,
        pub is_bigendian: bool,
        pub point_step: u32,
        pub row_step: u32,
        pub data: std::vec::Vec<u8>,
        pub is_dense: bool,
    }
    impl ::roslibrust::RosMessageType for PointCloud2 {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/PointCloud2";
    }
    impl PointCloud2 {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct PointField {
        pub name: std::string::String,
        pub offset: u32,
        pub datatype: u8,
        pub count: u32,
    }
    impl ::roslibrust::RosMessageType for PointField {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/PointField";
    }
    impl PointField {
        pub const INT8: u8 = 1;
        pub const UINT8: u8 = 2;
        pub const INT16: u8 = 3;
        pub const UINT16: u8 = 4;
        pub const INT32: u8 = 5;
        pub const UINT32: u8 = 6;
        pub const FLOAT32: u8 = 7;
        pub const FLOAT64: u8 = 8;
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Range {
        pub header: std_msgs::Header,
        pub radiation_type: u8,
        pub field_of_view: f32,
        pub min_range: f32,
        pub max_range: f32,
        pub range: f32,
    }
    impl ::roslibrust::RosMessageType for Range {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Range";
    }
    impl Range {
        pub const ULTRASOUND: u8 = 0;
        pub const INFRARED: u8 = 1;
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct RegionOfInterest {
        pub x_offset: u32,
        pub y_offset: u32,
        pub height: u32,
        pub width: u32,
        pub do_rectify: bool,
    }
    impl ::roslibrust::RosMessageType for RegionOfInterest {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/RegionOfInterest";
    }
    impl RegionOfInterest {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct RelativeHumidity {
        pub header: std_msgs::Header,
        pub relative_humidity: f64,
        pub variance: f64,
    }
    impl ::roslibrust::RosMessageType for RelativeHumidity {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/RelativeHumidity";
    }
    impl RelativeHumidity {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct SetCameraInfoRequest {
        pub camera_info: self::CameraInfo,
    }
    impl ::roslibrust::RosMessageType for SetCameraInfoRequest {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/SetCameraInfoRequest";
    }
    impl SetCameraInfoRequest {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct SetCameraInfoResponse {
        pub success: bool,
        pub status_message: std::string::String,
    }
    impl ::roslibrust::RosMessageType for SetCameraInfoResponse {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/SetCameraInfoResponse";
    }
    impl SetCameraInfoResponse {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Temperature {
        pub header: std_msgs::Header,
        pub temperature: f64,
        pub variance: f64,
    }
    impl ::roslibrust::RosMessageType for Temperature {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Temperature";
    }
    impl Temperature {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct TimeReference {
        pub header: std_msgs::Header,
        pub time_ref: std_msgs::TimeI,
        pub source: std::string::String,
    }
    impl ::roslibrust::RosMessageType for TimeReference {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/TimeReference";
    }
    impl TimeReference {}
    pub struct SetCameraInfo {}
    impl ::roslibrust::RosServiceType for SetCameraInfo {
        const ROS_SERVICE_NAME: &'static str = "SetCameraInfo";
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
    use super::sensor_msgs;
    use super::std_msgs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Mesh {
        pub triangles: std::vec::Vec<self::MeshTriangle>,
        pub vertices: std::vec::Vec<geometry_msgs::Point>,
    }
    impl ::roslibrust::RosMessageType for Mesh {
        const ROS_TYPE_NAME: &'static str = "shape_msgs/Mesh";
    }
    impl Mesh {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct MeshTriangle {
        pub vertex_indices: std::vec::Vec<u32>,
    }
    impl ::roslibrust::RosMessageType for MeshTriangle {
        const ROS_TYPE_NAME: &'static str = "shape_msgs/MeshTriangle";
    }
    impl MeshTriangle {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Plane {
        pub coef: std::vec::Vec<f64>,
    }
    impl ::roslibrust::RosMessageType for Plane {
        const ROS_TYPE_NAME: &'static str = "shape_msgs/Plane";
    }
    impl Plane {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct SolidPrimitive {
        pub r#type: u8,
        pub dimensions: std::vec::Vec<f64>,
    }
    impl ::roslibrust::RosMessageType for SolidPrimitive {
        const ROS_TYPE_NAME: &'static str = "shape_msgs/SolidPrimitive";
    }
    impl SolidPrimitive {
        pub const BOX: u8 = 1;
        pub const SPHERE: u8 = 2;
        pub const CYLINDER: u8 = 3;
        pub const CONE: u8 = 4;
        pub const BOX_X: u8 = 0;
        pub const BOX_Y: u8 = 1;
        pub const BOX_Z: u8 = 2;
        pub const SPHERE_RADIUS: u8 = 0;
        pub const CYLINDER_HEIGHT: u8 = 0;
        pub const CYLINDER_RADIUS: u8 = 1;
        pub const CONE_HEIGHT: u8 = 0;
        pub const CONE_RADIUS: u8 = 1;
    }
}
#[allow(unused_imports)]
pub mod std_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Bool {
        pub data: bool,
    }
    impl ::roslibrust::RosMessageType for Bool {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Bool";
    }
    impl Bool {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Byte {
        pub data: u8,
    }
    impl ::roslibrust::RosMessageType for Byte {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Byte";
    }
    impl Byte {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct ByteMultiArray {
        pub layout: self::MultiArrayLayout,
        pub data: std::vec::Vec<u8>,
    }
    impl ::roslibrust::RosMessageType for ByteMultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/ByteMultiArray";
    }
    impl ByteMultiArray {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Char {
        pub data: char,
    }
    impl ::roslibrust::RosMessageType for Char {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Char";
    }
    impl Char {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct ColorRGBA {
        pub r: f32,
        pub g: f32,
        pub b: f32,
        pub a: f32,
    }
    impl ::roslibrust::RosMessageType for ColorRGBA {
        const ROS_TYPE_NAME: &'static str = "std_msgs/ColorRGBA";
    }
    impl ColorRGBA {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Duration {
        pub data: self::DurationI,
    }
    impl ::roslibrust::RosMessageType for Duration {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Duration";
    }
    impl Duration {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct DurationI {
        pub sec: i32,
        pub nsec: i32,
    }
    impl ::roslibrust::RosMessageType for DurationI {
        const ROS_TYPE_NAME: &'static str = "std_msgs/DurationI";
    }
    impl DurationI {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Empty {}
    impl ::roslibrust::RosMessageType for Empty {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Empty";
    }
    impl Empty {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Float32 {
        pub data: f32,
    }
    impl ::roslibrust::RosMessageType for Float32 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Float32";
    }
    impl Float32 {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Float32MultiArray {
        pub layout: self::MultiArrayLayout,
        pub data: std::vec::Vec<f32>,
    }
    impl ::roslibrust::RosMessageType for Float32MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Float32MultiArray";
    }
    impl Float32MultiArray {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Float64 {
        pub data: f64,
    }
    impl ::roslibrust::RosMessageType for Float64 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Float64";
    }
    impl Float64 {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Float64MultiArray {
        pub layout: self::MultiArrayLayout,
        pub data: std::vec::Vec<f64>,
    }
    impl ::roslibrust::RosMessageType for Float64MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Float64MultiArray";
    }
    impl Float64MultiArray {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Header {
        pub seq: u32,
        pub stamp: self::TimeI,
        pub frame_id: std::string::String,
    }
    impl ::roslibrust::RosMessageType for Header {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Header";
    }
    impl Header {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Int16 {
        pub data: i16,
    }
    impl ::roslibrust::RosMessageType for Int16 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int16";
    }
    impl Int16 {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Int16MultiArray {
        pub layout: self::MultiArrayLayout,
        pub data: std::vec::Vec<i16>,
    }
    impl ::roslibrust::RosMessageType for Int16MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int16MultiArray";
    }
    impl Int16MultiArray {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Int32 {
        pub data: i32,
    }
    impl ::roslibrust::RosMessageType for Int32 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int32";
    }
    impl Int32 {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Int32MultiArray {
        pub layout: self::MultiArrayLayout,
        pub data: std::vec::Vec<i32>,
    }
    impl ::roslibrust::RosMessageType for Int32MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int32MultiArray";
    }
    impl Int32MultiArray {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Int64 {
        pub data: i64,
    }
    impl ::roslibrust::RosMessageType for Int64 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int64";
    }
    impl Int64 {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Int64MultiArray {
        pub layout: self::MultiArrayLayout,
        pub data: std::vec::Vec<i64>,
    }
    impl ::roslibrust::RosMessageType for Int64MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int64MultiArray";
    }
    impl Int64MultiArray {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Int8 {
        pub data: i8,
    }
    impl ::roslibrust::RosMessageType for Int8 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int8";
    }
    impl Int8 {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Int8MultiArray {
        pub layout: self::MultiArrayLayout,
        pub data: std::vec::Vec<i8>,
    }
    impl ::roslibrust::RosMessageType for Int8MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int8MultiArray";
    }
    impl Int8MultiArray {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct MultiArrayDimension {
        pub label: std::string::String,
        pub size: u32,
        pub stride: u32,
    }
    impl ::roslibrust::RosMessageType for MultiArrayDimension {
        const ROS_TYPE_NAME: &'static str = "std_msgs/MultiArrayDimension";
    }
    impl MultiArrayDimension {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct MultiArrayLayout {
        pub dim: std::vec::Vec<self::MultiArrayDimension>,
        pub data_offset: u32,
    }
    impl ::roslibrust::RosMessageType for MultiArrayLayout {
        const ROS_TYPE_NAME: &'static str = "std_msgs/MultiArrayLayout";
    }
    impl MultiArrayLayout {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct String {
        pub data: std::string::String,
    }
    impl ::roslibrust::RosMessageType for String {
        const ROS_TYPE_NAME: &'static str = "std_msgs/String";
    }
    impl String {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Time {
        pub data: self::TimeI,
    }
    impl ::roslibrust::RosMessageType for Time {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Time";
    }
    impl Time {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct TimeI {
        pub secs: u32,
        pub nsecs: u32,
    }
    impl ::roslibrust::RosMessageType for TimeI {
        const ROS_TYPE_NAME: &'static str = "std_msgs/TimeI";
    }
    impl TimeI {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct UInt16 {
        pub data: u16,
    }
    impl ::roslibrust::RosMessageType for UInt16 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt16";
    }
    impl UInt16 {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct UInt16MultiArray {
        pub layout: self::MultiArrayLayout,
        pub data: std::vec::Vec<u16>,
    }
    impl ::roslibrust::RosMessageType for UInt16MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt16MultiArray";
    }
    impl UInt16MultiArray {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct UInt32 {
        pub data: u32,
    }
    impl ::roslibrust::RosMessageType for UInt32 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt32";
    }
    impl UInt32 {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct UInt32MultiArray {
        pub layout: self::MultiArrayLayout,
        pub data: std::vec::Vec<u32>,
    }
    impl ::roslibrust::RosMessageType for UInt32MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt32MultiArray";
    }
    impl UInt32MultiArray {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct UInt64 {
        pub data: u64,
    }
    impl ::roslibrust::RosMessageType for UInt64 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt64";
    }
    impl UInt64 {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct UInt64MultiArray {
        pub layout: self::MultiArrayLayout,
        pub data: std::vec::Vec<u64>,
    }
    impl ::roslibrust::RosMessageType for UInt64MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt64MultiArray";
    }
    impl UInt64MultiArray {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct UInt8 {
        pub data: u8,
    }
    impl ::roslibrust::RosMessageType for UInt8 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt8";
    }
    impl UInt8 {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct UInt8MultiArray {
        pub layout: self::MultiArrayLayout,
        pub data: std::vec::Vec<u8>,
    }
    impl ::roslibrust::RosMessageType for UInt8MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt8MultiArray";
    }
    impl UInt8MultiArray {}
}
#[allow(unused_imports)]
pub mod stereo_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct DisparityImage {
        pub header: std_msgs::Header,
        pub image: sensor_msgs::Image,
        pub f: f32,
        pub T: f32,
        pub valid_window: sensor_msgs::RegionOfInterest,
        pub min_disparity: f32,
        pub max_disparity: f32,
        pub delta_d: f32,
    }
    impl ::roslibrust::RosMessageType for DisparityImage {
        const ROS_TYPE_NAME: &'static str = "stereo_msgs/DisparityImage";
    }
    impl DisparityImage {}
}
#[allow(unused_imports)]
pub mod test_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::stereo_msgs;
    use super::trajectory_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct AddTwoIntsRequest {
        pub a: i64,
        pub b: i64,
    }
    impl ::roslibrust::RosMessageType for AddTwoIntsRequest {
        const ROS_TYPE_NAME: &'static str = "test_msgs/AddTwoIntsRequest";
    }
    impl AddTwoIntsRequest {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct AddTwoIntsResponse {
        pub sum: i64,
    }
    impl ::roslibrust::RosMessageType for AddTwoIntsResponse {
        const ROS_TYPE_NAME: &'static str = "test_msgs/AddTwoIntsResponse";
    }
    impl AddTwoIntsResponse {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Constants {}
    impl ::roslibrust::RosMessageType for Constants {
        const ROS_TYPE_NAME: &'static str = "test_msgs/Constants";
    }
    impl Constants {
        pub const TEST_STR: &'static str = "/topic";
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Float64Stamped {
        pub header: std_msgs::Header,
        pub value: f64,
    }
    impl ::roslibrust::RosMessageType for Float64Stamped {
        const ROS_TYPE_NAME: &'static str = "test_msgs/Float64Stamped";
    }
    impl Float64Stamped {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct LoggerLevel {
        pub level: std::string::String,
    }
    impl ::roslibrust::RosMessageType for LoggerLevel {
        const ROS_TYPE_NAME: &'static str = "test_msgs/LoggerLevel";
    }
    impl LoggerLevel {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Metric {
        pub name: std::string::String,
        pub time: f64,
        pub data: std::vec::Vec<self::MetricPair>,
    }
    impl ::roslibrust::RosMessageType for Metric {
        const ROS_TYPE_NAME: &'static str = "test_msgs/Metric";
    }
    impl Metric {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct MetricPair {
        pub key: std::string::String,
        pub value: f64,
    }
    impl ::roslibrust::RosMessageType for MetricPair {
        const ROS_TYPE_NAME: &'static str = "test_msgs/MetricPair";
    }
    impl MetricPair {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct NodeInfo {
        pub node_name: std::string::String,
        pub pid: i64,
        pub status: u8,
    }
    impl ::roslibrust::RosMessageType for NodeInfo {
        const ROS_TYPE_NAME: &'static str = "test_msgs/NodeInfo";
    }
    impl NodeInfo {
        pub const STATUS_UNINITIALIZED: u8 = 0;
        pub const STATUS_DISCONNECTED: u8 = 1;
        pub const STATUS_RUNNING: u8 = 2;
        pub const STATUS_RUN_ERROR: u8 = 3;
        pub const STATUS_SHUTTING_DOWN: u8 = 4;
        pub const STATUS_SHUTDOWN: u8 = 5;
    }
    pub struct AddTwoInts {}
    impl ::roslibrust::RosServiceType for AddTwoInts {
        const ROS_SERVICE_NAME: &'static str = "AddTwoInts";
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
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::visualization_msgs;
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct JointTrajectory {
        pub header: std_msgs::Header,
        pub joint_names: std::vec::Vec<std::string::String>,
        pub points: std::vec::Vec<self::JointTrajectoryPoint>,
    }
    impl ::roslibrust::RosMessageType for JointTrajectory {
        const ROS_TYPE_NAME: &'static str = "trajectory_msgs/JointTrajectory";
    }
    impl JointTrajectory {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct JointTrajectoryPoint {
        pub positions: std::vec::Vec<f64>,
        pub velocities: std::vec::Vec<f64>,
        pub accelerations: std::vec::Vec<f64>,
        pub effort: std::vec::Vec<f64>,
        pub time_from_start: std_msgs::DurationI,
    }
    impl ::roslibrust::RosMessageType for JointTrajectoryPoint {
        const ROS_TYPE_NAME: &'static str = "trajectory_msgs/JointTrajectoryPoint";
    }
    impl JointTrajectoryPoint {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct MultiDOFJointTrajectory {
        pub header: std_msgs::Header,
        pub joint_names: std::vec::Vec<std::string::String>,
        pub points: std::vec::Vec<self::MultiDOFJointTrajectoryPoint>,
    }
    impl ::roslibrust::RosMessageType for MultiDOFJointTrajectory {
        const ROS_TYPE_NAME: &'static str = "trajectory_msgs/MultiDOFJointTrajectory";
    }
    impl MultiDOFJointTrajectory {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct MultiDOFJointTrajectoryPoint {
        pub transforms: std::vec::Vec<geometry_msgs::Transform>,
        pub velocities: std::vec::Vec<geometry_msgs::Twist>,
        pub accelerations: std::vec::Vec<geometry_msgs::Twist>,
        pub time_from_start: std_msgs::DurationI,
    }
    impl ::roslibrust::RosMessageType for MultiDOFJointTrajectoryPoint {
        const ROS_TYPE_NAME: &'static str = "trajectory_msgs/MultiDOFJointTrajectoryPoint";
    }
    impl MultiDOFJointTrajectoryPoint {}
}
#[allow(unused_imports)]
pub mod visualization_msgs {
    use super::actionlib_msgs;
    use super::diagnostic_msgs;
    use super::geometry_msgs;
    use super::nav_msgs;
    use super::rosapi;
    use super::sensor_msgs;
    use super::shape_msgs;
    use super::std_msgs;
    use super::stereo_msgs;
    use super::test_msgs;
    use super::trajectory_msgs;
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct ImageMarker {
        pub header: std_msgs::Header,
        pub ns: std::string::String,
        pub id: i32,
        pub r#type: i32,
        pub action: i32,
        pub position: geometry_msgs::Point,
        pub scale: f32,
        pub outline_color: std_msgs::ColorRGBA,
        pub filled: u8,
        pub fill_color: std_msgs::ColorRGBA,
        pub lifetime: std_msgs::DurationI,
        pub points: std::vec::Vec<geometry_msgs::Point>,
        pub outline_colors: std::vec::Vec<std_msgs::ColorRGBA>,
    }
    impl ::roslibrust::RosMessageType for ImageMarker {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/ImageMarker";
    }
    impl ImageMarker {
        pub const CIRCLE: u8 = 0;
        pub const LINE_STRIP: u8 = 1;
        pub const LINE_LIST: u8 = 2;
        pub const POLYGON: u8 = 3;
        pub const POINTS: u8 = 4;
        pub const ADD: u8 = 0;
        pub const REMOVE: u8 = 1;
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct InteractiveMarker {
        pub header: std_msgs::Header,
        pub pose: geometry_msgs::Pose,
        pub name: std::string::String,
        pub description: std::string::String,
        pub scale: f32,
        pub menu_entries: std::vec::Vec<self::MenuEntry>,
        pub controls: std::vec::Vec<self::InteractiveMarkerControl>,
    }
    impl ::roslibrust::RosMessageType for InteractiveMarker {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarker";
    }
    impl InteractiveMarker {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct InteractiveMarkerControl {
        pub name: std::string::String,
        pub orientation: geometry_msgs::Quaternion,
        pub orientation_mode: u8,
        pub interaction_mode: u8,
        pub always_visible: bool,
        pub markers: std::vec::Vec<self::Marker>,
        pub independent_marker_orientation: bool,
        pub description: std::string::String,
    }
    impl ::roslibrust::RosMessageType for InteractiveMarkerControl {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarkerControl";
    }
    impl InteractiveMarkerControl {
        pub const INHERIT: u8 = 0;
        pub const FIXED: u8 = 1;
        pub const VIEW_FACING: u8 = 2;
        pub const NONE: u8 = 0;
        pub const MENU: u8 = 1;
        pub const BUTTON: u8 = 2;
        pub const MOVE_AXIS: u8 = 3;
        pub const MOVE_PLANE: u8 = 4;
        pub const ROTATE_AXIS: u8 = 5;
        pub const MOVE_ROTATE: u8 = 6;
        pub const MOVE_3D: u8 = 7;
        pub const ROTATE_3D: u8 = 8;
        pub const MOVE_ROTATE_3D: u8 = 9;
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct InteractiveMarkerFeedback {
        pub header: std_msgs::Header,
        pub client_id: std::string::String,
        pub marker_name: std::string::String,
        pub control_name: std::string::String,
        pub event_type: u8,
        pub pose: geometry_msgs::Pose,
        pub menu_entry_id: u32,
        pub mouse_point: geometry_msgs::Point,
        pub mouse_point_valid: bool,
    }
    impl ::roslibrust::RosMessageType for InteractiveMarkerFeedback {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarkerFeedback";
    }
    impl InteractiveMarkerFeedback {
        pub const KEEP_ALIVE: u8 = 0;
        pub const POSE_UPDATE: u8 = 1;
        pub const MENU_SELECT: u8 = 2;
        pub const BUTTON_CLICK: u8 = 3;
        pub const MOUSE_DOWN: u8 = 4;
        pub const MOUSE_UP: u8 = 5;
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct InteractiveMarkerInit {
        pub server_id: std::string::String,
        pub seq_num: u64,
        pub markers: std::vec::Vec<self::InteractiveMarker>,
    }
    impl ::roslibrust::RosMessageType for InteractiveMarkerInit {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarkerInit";
    }
    impl InteractiveMarkerInit {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct InteractiveMarkerPose {
        pub header: std_msgs::Header,
        pub pose: geometry_msgs::Pose,
        pub name: std::string::String,
    }
    impl ::roslibrust::RosMessageType for InteractiveMarkerPose {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarkerPose";
    }
    impl InteractiveMarkerPose {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct InteractiveMarkerUpdate {
        pub server_id: std::string::String,
        pub seq_num: u64,
        pub r#type: u8,
        pub markers: std::vec::Vec<self::InteractiveMarker>,
        pub poses: std::vec::Vec<self::InteractiveMarkerPose>,
        pub erases: std::vec::Vec<std::string::String>,
    }
    impl ::roslibrust::RosMessageType for InteractiveMarkerUpdate {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarkerUpdate";
    }
    impl InteractiveMarkerUpdate {
        pub const KEEP_ALIVE: u8 = 0;
        pub const UPDATE: u8 = 1;
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct Marker {
        pub header: std_msgs::Header,
        pub ns: std::string::String,
        pub id: i32,
        pub r#type: i32,
        pub action: i32,
        pub pose: geometry_msgs::Pose,
        pub scale: geometry_msgs::Vector3,
        pub color: std_msgs::ColorRGBA,
        pub lifetime: std_msgs::DurationI,
        pub frame_locked: bool,
        pub points: std::vec::Vec<geometry_msgs::Point>,
        pub colors: std::vec::Vec<std_msgs::ColorRGBA>,
        pub text: std::string::String,
        pub mesh_resource: std::string::String,
        pub mesh_use_embedded_materials: bool,
    }
    impl ::roslibrust::RosMessageType for Marker {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/Marker";
    }
    impl Marker {
        pub const ARROW: u8 = 0;
        pub const CUBE: u8 = 1;
        pub const SPHERE: u8 = 2;
        pub const CYLINDER: u8 = 3;
        pub const LINE_STRIP: u8 = 4;
        pub const LINE_LIST: u8 = 5;
        pub const CUBE_LIST: u8 = 6;
        pub const SPHERE_LIST: u8 = 7;
        pub const POINTS: u8 = 8;
        pub const TEXT_VIEW_FACING: u8 = 9;
        pub const MESH_RESOURCE: u8 = 10;
        pub const TRIANGLE_LIST: u8 = 11;
        pub const ADD: u8 = 0;
        pub const MODIFY: u8 = 0;
        pub const DELETE: u8 = 2;
        pub const DELETEALL: u8 = 3;
    }
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct MarkerArray {
        pub markers: std::vec::Vec<self::Marker>,
    }
    impl ::roslibrust::RosMessageType for MarkerArray {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/MarkerArray";
    }
    impl MarkerArray {}
    #[allow(non_snake_case)]
    #[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
    pub struct MenuEntry {
        pub id: u32,
        pub parent_id: u32,
        pub title: std::string::String,
        pub command: std::string::String,
        pub command_type: u8,
    }
    impl ::roslibrust::RosMessageType for MenuEntry {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/MenuEntry";
    }
    impl MenuEntry {
        pub const FEEDBACK: u8 = 0;
        pub const ROSRUN: u8 = 1;
        pub const ROSLAUNCH: u8 = 2;
    }
}
