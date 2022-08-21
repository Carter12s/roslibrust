#![allow(non_snake_case)]
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct GoalID {
    pub stamp: TimeI,
    pub id: std::string::String,
}
impl ::roslibrust::RosMessageType for GoalID {
    const ROS_TYPE_NAME: &'static str = "actionlib_msgs/GoalID";
}
impl GoalID {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct GoalStatus {
    pub goal_id: GoalID,
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct GoalStatusArray {
    pub header: Header,
    pub status_list: std::vec::Vec<GoalStatus>,
}
impl ::roslibrust::RosMessageType for GoalStatusArray {
    const ROS_TYPE_NAME: &'static str = "actionlib_msgs/GoalStatusArray";
}
impl GoalStatusArray {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct DiagnosticArray {
    pub header: Header,
    pub status: std::vec::Vec<DiagnosticStatus>,
}
impl ::roslibrust::RosMessageType for DiagnosticArray {
    const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/DiagnosticArray";
}
impl DiagnosticArray {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct DiagnosticStatus {
    pub level: u8,
    pub name: std::string::String,
    pub message: std::string::String,
    pub hardware_id: std::string::String,
    pub values: std::vec::Vec<KeyValue>,
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct KeyValue {
    pub key: std::string::String,
    pub value: std::string::String,
}
impl ::roslibrust::RosMessageType for KeyValue {
    const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/KeyValue";
}
impl KeyValue {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Accel {
    pub linear: Vector3,
    pub angular: Vector3,
}
impl ::roslibrust::RosMessageType for Accel {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/Accel";
}
impl Accel {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct AccelStamped {
    pub header: Header,
    pub accel: Accel,
}
impl ::roslibrust::RosMessageType for AccelStamped {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/AccelStamped";
}
impl AccelStamped {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct AccelWithCovariance {
    pub accel: Accel,
    pub covariance: std::vec::Vec<f64>,
}
impl ::roslibrust::RosMessageType for AccelWithCovariance {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/AccelWithCovariance";
}
impl AccelWithCovariance {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct AccelWithCovarianceStamped {
    pub header: Header,
    pub accel: AccelWithCovariance,
}
impl ::roslibrust::RosMessageType for AccelWithCovarianceStamped {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/AccelWithCovarianceStamped";
}
impl AccelWithCovarianceStamped {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Inertia {
    pub m: f64,
    pub com: Vector3,
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct InertiaStamped {
    pub header: Header,
    pub inertia: Inertia,
}
impl ::roslibrust::RosMessageType for InertiaStamped {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/InertiaStamped";
}
impl InertiaStamped {}
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct PointStamped {
    pub header: Header,
    pub point: Point,
}
impl ::roslibrust::RosMessageType for PointStamped {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/PointStamped";
}
impl PointStamped {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Polygon {
    pub points: std::vec::Vec<Point32>,
}
impl ::roslibrust::RosMessageType for Polygon {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/Polygon";
}
impl Polygon {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct PolygonStamped {
    pub header: Header,
    pub polygon: Polygon,
}
impl ::roslibrust::RosMessageType for PolygonStamped {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/PolygonStamped";
}
impl PolygonStamped {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Pose {
    pub position: Point,
    pub orientation: Quaternion,
}
impl ::roslibrust::RosMessageType for Pose {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/Pose";
}
impl Pose {}
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct PoseArray {
    pub header: Header,
    pub poses: std::vec::Vec<Pose>,
}
impl ::roslibrust::RosMessageType for PoseArray {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseArray";
}
impl PoseArray {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct PoseStamped {
    pub header: Header,
    pub pose: Pose,
}
impl ::roslibrust::RosMessageType for PoseStamped {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseStamped";
}
impl PoseStamped {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct PoseWithCovariance {
    pub pose: Pose,
    pub covariance: std::vec::Vec<f64>,
}
impl ::roslibrust::RosMessageType for PoseWithCovariance {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseWithCovariance";
}
impl PoseWithCovariance {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct PoseWithCovarianceStamped {
    pub header: Header,
    pub pose: PoseWithCovariance,
}
impl ::roslibrust::RosMessageType for PoseWithCovarianceStamped {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseWithCovarianceStamped";
}
impl PoseWithCovarianceStamped {}
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct QuaternionStamped {
    pub header: Header,
    pub quaternion: Quaternion,
}
impl ::roslibrust::RosMessageType for QuaternionStamped {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/QuaternionStamped";
}
impl QuaternionStamped {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Transform {
    pub translation: Vector3,
    pub rotation: Quaternion,
}
impl ::roslibrust::RosMessageType for Transform {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/Transform";
}
impl Transform {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct TransformStamped {
    pub header: Header,
    pub child_frame_id: std::string::String,
    pub transform: Transform,
}
impl ::roslibrust::RosMessageType for TransformStamped {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/TransformStamped";
}
impl TransformStamped {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}
impl ::roslibrust::RosMessageType for Twist {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/Twist";
}
impl Twist {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct TwistStamped {
    pub header: Header,
    pub twist: Twist,
}
impl ::roslibrust::RosMessageType for TwistStamped {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/TwistStamped";
}
impl TwistStamped {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct TwistWithCovariance {
    pub twist: Twist,
    pub covariance: std::vec::Vec<f64>,
}
impl ::roslibrust::RosMessageType for TwistWithCovariance {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/TwistWithCovariance";
}
impl TwistWithCovariance {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct TwistWithCovarianceStamped {
    pub header: Header,
    pub twist: TwistWithCovariance,
}
impl ::roslibrust::RosMessageType for TwistWithCovarianceStamped {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/TwistWithCovarianceStamped";
}
impl TwistWithCovarianceStamped {}
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Vector3Stamped {
    pub header: Header,
    pub vector: Vector3,
}
impl ::roslibrust::RosMessageType for Vector3Stamped {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/Vector3Stamped";
}
impl Vector3Stamped {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Wrench {
    pub force: Vector3,
    pub torque: Vector3,
}
impl ::roslibrust::RosMessageType for Wrench {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/Wrench";
}
impl Wrench {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct WrenchStamped {
    pub header: Header,
    pub wrench: Wrench,
}
impl ::roslibrust::RosMessageType for WrenchStamped {
    const ROS_TYPE_NAME: &'static str = "geometry_msgs/WrenchStamped";
}
impl WrenchStamped {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct GridCells {
    pub header: Header,
    pub cell_width: f32,
    pub cell_height: f32,
    pub cells: std::vec::Vec<Point>,
}
impl ::roslibrust::RosMessageType for GridCells {
    const ROS_TYPE_NAME: &'static str = "nav_msgs/GridCells";
}
impl GridCells {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct MapMetaData {
    pub map_load_time: TimeI,
    pub resolution: f32,
    pub width: u32,
    pub height: u32,
    pub origin: Pose,
}
impl ::roslibrust::RosMessageType for MapMetaData {
    const ROS_TYPE_NAME: &'static str = "nav_msgs/MapMetaData";
}
impl MapMetaData {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct OccupancyGrid {
    pub header: Header,
    pub info: MapMetaData,
    pub data: std::vec::Vec<i8>,
}
impl ::roslibrust::RosMessageType for OccupancyGrid {
    const ROS_TYPE_NAME: &'static str = "nav_msgs/OccupancyGrid";
}
impl OccupancyGrid {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Odometry {
    pub header: Header,
    pub child_frame_id: std::string::String,
    pub pose: PoseWithCovariance,
    pub twist: TwistWithCovariance,
}
impl ::roslibrust::RosMessageType for Odometry {
    const ROS_TYPE_NAME: &'static str = "nav_msgs/Odometry";
}
impl Odometry {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Path {
    pub header: Header,
    pub poses: std::vec::Vec<PoseStamped>,
}
impl ::roslibrust::RosMessageType for Path {
    const ROS_TYPE_NAME: &'static str = "nav_msgs/Path";
}
impl Path {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct BatteryState {
    pub header: Header,
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct CameraInfo {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub distortion_model: std::string::String,
    pub D: std::vec::Vec<f64>,
    pub K: std::vec::Vec<f64>,
    pub R: std::vec::Vec<f64>,
    pub P: std::vec::Vec<f64>,
    pub binning_x: u32,
    pub binning_y: u32,
    pub roi: RegionOfInterest,
}
impl ::roslibrust::RosMessageType for CameraInfo {
    const ROS_TYPE_NAME: &'static str = "sensor_msgs/CameraInfo";
}
impl CameraInfo {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct ChannelFloat32 {
    pub name: std::string::String,
    pub values: std::vec::Vec<f32>,
}
impl ::roslibrust::RosMessageType for ChannelFloat32 {
    const ROS_TYPE_NAME: &'static str = "sensor_msgs/ChannelFloat32";
}
impl ChannelFloat32 {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct CompressedImage {
    pub header: Header,
    pub format: std::string::String,
    pub data: std::vec::Vec<u8>,
}
impl ::roslibrust::RosMessageType for CompressedImage {
    const ROS_TYPE_NAME: &'static str = "sensor_msgs/CompressedImage";
}
impl CompressedImage {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct FluidPressure {
    pub header: Header,
    pub fluid_pressure: f64,
    pub variance: f64,
}
impl ::roslibrust::RosMessageType for FluidPressure {
    const ROS_TYPE_NAME: &'static str = "sensor_msgs/FluidPressure";
}
impl FluidPressure {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Illuminance {
    pub header: Header,
    pub illuminance: f64,
    pub variance: f64,
}
impl ::roslibrust::RosMessageType for Illuminance {
    const ROS_TYPE_NAME: &'static str = "sensor_msgs/Illuminance";
}
impl Illuminance {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Image {
    pub header: Header,
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Imu {
    pub header: Header,
    pub orientation: Quaternion,
    pub orientation_covariance: std::vec::Vec<f64>,
    pub angular_velocity: Vector3,
    pub angular_velocity_covariance: std::vec::Vec<f64>,
    pub linear_acceleration: Vector3,
    pub linear_acceleration_covariance: std::vec::Vec<f64>,
}
impl ::roslibrust::RosMessageType for Imu {
    const ROS_TYPE_NAME: &'static str = "sensor_msgs/Imu";
}
impl Imu {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct JointState {
    pub header: Header,
    pub name: std::vec::Vec<std::string::String>,
    pub position: std::vec::Vec<f64>,
    pub velocity: std::vec::Vec<f64>,
    pub effort: std::vec::Vec<f64>,
}
impl ::roslibrust::RosMessageType for JointState {
    const ROS_TYPE_NAME: &'static str = "sensor_msgs/JointState";
}
impl JointState {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Joy {
    pub header: Header,
    pub axes: std::vec::Vec<f32>,
    pub buttons: std::vec::Vec<i32>,
}
impl ::roslibrust::RosMessageType for Joy {
    const ROS_TYPE_NAME: &'static str = "sensor_msgs/Joy";
}
impl Joy {}
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct JoyFeedbackArray {
    pub array: std::vec::Vec<JoyFeedback>,
}
impl ::roslibrust::RosMessageType for JoyFeedbackArray {
    const ROS_TYPE_NAME: &'static str = "sensor_msgs/JoyFeedbackArray";
}
impl JoyFeedbackArray {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct LaserEcho {
    pub echoes: std::vec::Vec<f32>,
}
impl ::roslibrust::RosMessageType for LaserEcho {
    const ROS_TYPE_NAME: &'static str = "sensor_msgs/LaserEcho";
}
impl LaserEcho {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct LaserScan {
    pub header: Header,
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct MagneticField {
    pub header: Header,
    pub magnetic_field: Vector3,
    pub magnetic_field_covariance: std::vec::Vec<f64>,
}
impl ::roslibrust::RosMessageType for MagneticField {
    const ROS_TYPE_NAME: &'static str = "sensor_msgs/MagneticField";
}
impl MagneticField {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct MultiDOFJointState {
    pub header: Header,
    pub joint_names: std::vec::Vec<std::string::String>,
    pub transforms: std::vec::Vec<Transform>,
    pub twist: std::vec::Vec<Twist>,
    pub wrench: std::vec::Vec<Wrench>,
}
impl ::roslibrust::RosMessageType for MultiDOFJointState {
    const ROS_TYPE_NAME: &'static str = "sensor_msgs/MultiDOFJointState";
}
impl MultiDOFJointState {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct MultiEchoLaserScan {
    pub header: Header,
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub time_increment: f32,
    pub scan_time: f32,
    pub range_min: f32,
    pub range_max: f32,
    pub ranges: std::vec::Vec<LaserEcho>,
    pub intensities: std::vec::Vec<LaserEcho>,
}
impl ::roslibrust::RosMessageType for MultiEchoLaserScan {
    const ROS_TYPE_NAME: &'static str = "sensor_msgs/MultiEchoLaserScan";
}
impl MultiEchoLaserScan {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct NavSatFix {
    pub header: Header,
    pub status: NavSatStatus,
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct PointCloud {
    pub header: Header,
    pub points: std::vec::Vec<Point32>,
    pub channels: std::vec::Vec<ChannelFloat32>,
}
impl ::roslibrust::RosMessageType for PointCloud {
    const ROS_TYPE_NAME: &'static str = "sensor_msgs/PointCloud";
}
impl PointCloud {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct PointCloud2 {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub fields: std::vec::Vec<PointField>,
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Range {
    pub header: Header,
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct RelativeHumidity {
    pub header: Header,
    pub relative_humidity: f64,
    pub variance: f64,
}
impl ::roslibrust::RosMessageType for RelativeHumidity {
    const ROS_TYPE_NAME: &'static str = "sensor_msgs/RelativeHumidity";
}
impl RelativeHumidity {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Temperature {
    pub header: Header,
    pub temperature: f64,
    pub variance: f64,
}
impl ::roslibrust::RosMessageType for Temperature {
    const ROS_TYPE_NAME: &'static str = "sensor_msgs/Temperature";
}
impl Temperature {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct TimeReference {
    pub header: Header,
    pub time_ref: TimeI,
    pub source: std::string::String,
}
impl ::roslibrust::RosMessageType for TimeReference {
    const ROS_TYPE_NAME: &'static str = "sensor_msgs/TimeReference";
}
impl TimeReference {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Mesh {
    pub triangles: std::vec::Vec<MeshTriangle>,
    pub vertices: std::vec::Vec<Point>,
}
impl ::roslibrust::RosMessageType for Mesh {
    const ROS_TYPE_NAME: &'static str = "shape_msgs/Mesh";
}
impl Mesh {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct MeshTriangle {
    pub vertex_indices: std::vec::Vec<u32>,
}
impl ::roslibrust::RosMessageType for MeshTriangle {
    const ROS_TYPE_NAME: &'static str = "shape_msgs/MeshTriangle";
}
impl MeshTriangle {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Plane {
    pub coef: std::vec::Vec<f64>,
}
impl ::roslibrust::RosMessageType for Plane {
    const ROS_TYPE_NAME: &'static str = "shape_msgs/Plane";
}
impl Plane {}
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Bool {
    pub data: bool,
}
impl ::roslibrust::RosMessageType for Bool {
    const ROS_TYPE_NAME: &'static str = "std_msgs/Bool";
}
impl Bool {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Byte {
    pub data: u8,
}
impl ::roslibrust::RosMessageType for Byte {
    const ROS_TYPE_NAME: &'static str = "std_msgs/Byte";
}
impl Byte {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct ByteMultiArray {
    pub layout: MultiArrayLayout,
    pub data: std::vec::Vec<u8>,
}
impl ::roslibrust::RosMessageType for ByteMultiArray {
    const ROS_TYPE_NAME: &'static str = "std_msgs/ByteMultiArray";
}
impl ByteMultiArray {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Char {
    pub data: char,
}
impl ::roslibrust::RosMessageType for Char {
    const ROS_TYPE_NAME: &'static str = "std_msgs/Char";
}
impl Char {}
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Duration {
    pub data: DurationI,
}
impl ::roslibrust::RosMessageType for Duration {
    const ROS_TYPE_NAME: &'static str = "std_msgs/Duration";
}
impl Duration {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct DurationI {
    pub sec: i32,
    pub nsec: i32,
}
impl ::roslibrust::RosMessageType for DurationI {
    const ROS_TYPE_NAME: &'static str = "std_msgs/DurationI";
}
impl DurationI {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Empty {}
impl ::roslibrust::RosMessageType for Empty {
    const ROS_TYPE_NAME: &'static str = "std_msgs/Empty";
}
impl Empty {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Float32 {
    pub data: f32,
}
impl ::roslibrust::RosMessageType for Float32 {
    const ROS_TYPE_NAME: &'static str = "std_msgs/Float32";
}
impl Float32 {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Float32MultiArray {
    pub layout: MultiArrayLayout,
    pub data: std::vec::Vec<f32>,
}
impl ::roslibrust::RosMessageType for Float32MultiArray {
    const ROS_TYPE_NAME: &'static str = "std_msgs/Float32MultiArray";
}
impl Float32MultiArray {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Float64 {
    pub data: f64,
}
impl ::roslibrust::RosMessageType for Float64 {
    const ROS_TYPE_NAME: &'static str = "std_msgs/Float64";
}
impl Float64 {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Float64MultiArray {
    pub layout: MultiArrayLayout,
    pub data: std::vec::Vec<f64>,
}
impl ::roslibrust::RosMessageType for Float64MultiArray {
    const ROS_TYPE_NAME: &'static str = "std_msgs/Float64MultiArray";
}
impl Float64MultiArray {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Header {
    pub seq: u32,
    pub stamp: TimeI,
    pub frame_id: std::string::String,
}
impl ::roslibrust::RosMessageType for Header {
    const ROS_TYPE_NAME: &'static str = "std_msgs/Header";
}
impl Header {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Int16 {
    pub data: i16,
}
impl ::roslibrust::RosMessageType for Int16 {
    const ROS_TYPE_NAME: &'static str = "std_msgs/Int16";
}
impl Int16 {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Int16MultiArray {
    pub layout: MultiArrayLayout,
    pub data: std::vec::Vec<i16>,
}
impl ::roslibrust::RosMessageType for Int16MultiArray {
    const ROS_TYPE_NAME: &'static str = "std_msgs/Int16MultiArray";
}
impl Int16MultiArray {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Int32 {
    pub data: i32,
}
impl ::roslibrust::RosMessageType for Int32 {
    const ROS_TYPE_NAME: &'static str = "std_msgs/Int32";
}
impl Int32 {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Int32MultiArray {
    pub layout: MultiArrayLayout,
    pub data: std::vec::Vec<i32>,
}
impl ::roslibrust::RosMessageType for Int32MultiArray {
    const ROS_TYPE_NAME: &'static str = "std_msgs/Int32MultiArray";
}
impl Int32MultiArray {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Int64 {
    pub data: i64,
}
impl ::roslibrust::RosMessageType for Int64 {
    const ROS_TYPE_NAME: &'static str = "std_msgs/Int64";
}
impl Int64 {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Int64MultiArray {
    pub layout: MultiArrayLayout,
    pub data: std::vec::Vec<i64>,
}
impl ::roslibrust::RosMessageType for Int64MultiArray {
    const ROS_TYPE_NAME: &'static str = "std_msgs/Int64MultiArray";
}
impl Int64MultiArray {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Int8 {
    pub data: i8,
}
impl ::roslibrust::RosMessageType for Int8 {
    const ROS_TYPE_NAME: &'static str = "std_msgs/Int8";
}
impl Int8 {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Int8MultiArray {
    pub layout: MultiArrayLayout,
    pub data: std::vec::Vec<i8>,
}
impl ::roslibrust::RosMessageType for Int8MultiArray {
    const ROS_TYPE_NAME: &'static str = "std_msgs/Int8MultiArray";
}
impl Int8MultiArray {}
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct MultiArrayLayout {
    pub dim: std::vec::Vec<MultiArrayDimension>,
    pub data_offset: u32,
}
impl ::roslibrust::RosMessageType for MultiArrayLayout {
    const ROS_TYPE_NAME: &'static str = "std_msgs/MultiArrayLayout";
}
impl MultiArrayLayout {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct String {
    pub data: std::string::String,
}
impl ::roslibrust::RosMessageType for String {
    const ROS_TYPE_NAME: &'static str = "std_msgs/String";
}
impl String {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Time {
    pub data: TimeI,
}
impl ::roslibrust::RosMessageType for Time {
    const ROS_TYPE_NAME: &'static str = "std_msgs/Time";
}
impl Time {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct TimeI {
    pub secs: u32,
    pub nsecs: u32,
}
impl ::roslibrust::RosMessageType for TimeI {
    const ROS_TYPE_NAME: &'static str = "std_msgs/TimeI";
}
impl TimeI {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct UInt16 {
    pub data: u16,
}
impl ::roslibrust::RosMessageType for UInt16 {
    const ROS_TYPE_NAME: &'static str = "std_msgs/UInt16";
}
impl UInt16 {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct UInt16MultiArray {
    pub layout: MultiArrayLayout,
    pub data: std::vec::Vec<u16>,
}
impl ::roslibrust::RosMessageType for UInt16MultiArray {
    const ROS_TYPE_NAME: &'static str = "std_msgs/UInt16MultiArray";
}
impl UInt16MultiArray {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct UInt32 {
    pub data: u32,
}
impl ::roslibrust::RosMessageType for UInt32 {
    const ROS_TYPE_NAME: &'static str = "std_msgs/UInt32";
}
impl UInt32 {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct UInt32MultiArray {
    pub layout: MultiArrayLayout,
    pub data: std::vec::Vec<u32>,
}
impl ::roslibrust::RosMessageType for UInt32MultiArray {
    const ROS_TYPE_NAME: &'static str = "std_msgs/UInt32MultiArray";
}
impl UInt32MultiArray {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct UInt64 {
    pub data: u64,
}
impl ::roslibrust::RosMessageType for UInt64 {
    const ROS_TYPE_NAME: &'static str = "std_msgs/UInt64";
}
impl UInt64 {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct UInt64MultiArray {
    pub layout: MultiArrayLayout,
    pub data: std::vec::Vec<u64>,
}
impl ::roslibrust::RosMessageType for UInt64MultiArray {
    const ROS_TYPE_NAME: &'static str = "std_msgs/UInt64MultiArray";
}
impl UInt64MultiArray {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct UInt8 {
    pub data: u8,
}
impl ::roslibrust::RosMessageType for UInt8 {
    const ROS_TYPE_NAME: &'static str = "std_msgs/UInt8";
}
impl UInt8 {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct UInt8MultiArray {
    pub layout: MultiArrayLayout,
    pub data: std::vec::Vec<u8>,
}
impl ::roslibrust::RosMessageType for UInt8MultiArray {
    const ROS_TYPE_NAME: &'static str = "std_msgs/UInt8MultiArray";
}
impl UInt8MultiArray {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct DisparityImage {
    pub header: Header,
    pub image: Image,
    pub f: f32,
    pub T: f32,
    pub valid_window: RegionOfInterest,
    pub min_disparity: f32,
    pub max_disparity: f32,
    pub delta_d: f32,
}
impl ::roslibrust::RosMessageType for DisparityImage {
    const ROS_TYPE_NAME: &'static str = "stereo_msgs/DisparityImage";
}
impl DisparityImage {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Float64Stamped {
    pub header: Header,
    pub value: f64,
}
impl ::roslibrust::RosMessageType for Float64Stamped {
    const ROS_TYPE_NAME: &'static str = "test_msgs/Float64Stamped";
}
impl Float64Stamped {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct LoggerLevel {
    pub level: std::string::String,
}
impl ::roslibrust::RosMessageType for LoggerLevel {
    const ROS_TYPE_NAME: &'static str = "test_msgs/LoggerLevel";
}
impl LoggerLevel {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Metric {
    pub name: std::string::String,
    pub time: f64,
    pub data: std::vec::Vec<MetricPair>,
}
impl ::roslibrust::RosMessageType for Metric {
    const ROS_TYPE_NAME: &'static str = "test_msgs/Metric";
}
impl Metric {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct MetricPair {
    pub key: std::string::String,
    pub value: f64,
}
impl ::roslibrust::RosMessageType for MetricPair {
    const ROS_TYPE_NAME: &'static str = "test_msgs/MetricPair";
}
impl MetricPair {}
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct JointTrajectory {
    pub header: Header,
    pub joint_names: std::vec::Vec<std::string::String>,
    pub points: std::vec::Vec<JointTrajectoryPoint>,
}
impl ::roslibrust::RosMessageType for JointTrajectory {
    const ROS_TYPE_NAME: &'static str = "trajectory_msgs/JointTrajectory";
}
impl JointTrajectory {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct JointTrajectoryPoint {
    pub positions: std::vec::Vec<f64>,
    pub velocities: std::vec::Vec<f64>,
    pub accelerations: std::vec::Vec<f64>,
    pub effort: std::vec::Vec<f64>,
    pub time_from_start: DurationI,
}
impl ::roslibrust::RosMessageType for JointTrajectoryPoint {
    const ROS_TYPE_NAME: &'static str = "trajectory_msgs/JointTrajectoryPoint";
}
impl JointTrajectoryPoint {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct MultiDOFJointTrajectory {
    pub header: Header,
    pub joint_names: std::vec::Vec<std::string::String>,
    pub points: std::vec::Vec<MultiDOFJointTrajectoryPoint>,
}
impl ::roslibrust::RosMessageType for MultiDOFJointTrajectory {
    const ROS_TYPE_NAME: &'static str = "trajectory_msgs/MultiDOFJointTrajectory";
}
impl MultiDOFJointTrajectory {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct MultiDOFJointTrajectoryPoint {
    pub transforms: std::vec::Vec<Transform>,
    pub velocities: std::vec::Vec<Twist>,
    pub accelerations: std::vec::Vec<Twist>,
    pub time_from_start: DurationI,
}
impl ::roslibrust::RosMessageType for MultiDOFJointTrajectoryPoint {
    const ROS_TYPE_NAME: &'static str = "trajectory_msgs/MultiDOFJointTrajectoryPoint";
}
impl MultiDOFJointTrajectoryPoint {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct ImageMarker {
    pub header: Header,
    pub ns: std::string::String,
    pub id: i32,
    pub r#type: i32,
    pub action: i32,
    pub position: Point,
    pub scale: f32,
    pub outline_color: ColorRGBA,
    pub filled: u8,
    pub fill_color: ColorRGBA,
    pub lifetime: DurationI,
    pub points: std::vec::Vec<Point>,
    pub outline_colors: std::vec::Vec<ColorRGBA>,
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct InteractiveMarker {
    pub header: Header,
    pub pose: Pose,
    pub name: std::string::String,
    pub description: std::string::String,
    pub scale: f32,
    pub menu_entries: std::vec::Vec<MenuEntry>,
    pub controls: std::vec::Vec<InteractiveMarkerControl>,
}
impl ::roslibrust::RosMessageType for InteractiveMarker {
    const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarker";
}
impl InteractiveMarker {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct InteractiveMarkerControl {
    pub name: std::string::String,
    pub orientation: Quaternion,
    pub orientation_mode: u8,
    pub interaction_mode: u8,
    pub always_visible: bool,
    pub markers: std::vec::Vec<Marker>,
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct InteractiveMarkerFeedback {
    pub header: Header,
    pub client_id: std::string::String,
    pub marker_name: std::string::String,
    pub control_name: std::string::String,
    pub event_type: u8,
    pub pose: Pose,
    pub menu_entry_id: u32,
    pub mouse_point: Point,
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct InteractiveMarkerInit {
    pub server_id: std::string::String,
    pub seq_num: u64,
    pub markers: std::vec::Vec<InteractiveMarker>,
}
impl ::roslibrust::RosMessageType for InteractiveMarkerInit {
    const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarkerInit";
}
impl InteractiveMarkerInit {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct InteractiveMarkerPose {
    pub header: Header,
    pub pose: Pose,
    pub name: std::string::String,
}
impl ::roslibrust::RosMessageType for InteractiveMarkerPose {
    const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarkerPose";
}
impl InteractiveMarkerPose {}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct InteractiveMarkerUpdate {
    pub server_id: std::string::String,
    pub seq_num: u64,
    pub r#type: u8,
    pub markers: std::vec::Vec<InteractiveMarker>,
    pub poses: std::vec::Vec<InteractiveMarkerPose>,
    pub erases: std::vec::Vec<std::string::String>,
}
impl ::roslibrust::RosMessageType for InteractiveMarkerUpdate {
    const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarkerUpdate";
}
impl InteractiveMarkerUpdate {
    pub const KEEP_ALIVE: u8 = 0;
    pub const UPDATE: u8 = 1;
}
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Marker {
    pub header: Header,
    pub ns: std::string::String,
    pub id: i32,
    pub r#type: i32,
    pub action: i32,
    pub pose: Pose,
    pub scale: Vector3,
    pub color: ColorRGBA,
    pub lifetime: DurationI,
    pub frame_locked: bool,
    pub points: std::vec::Vec<Point>,
    pub colors: std::vec::Vec<ColorRGBA>,
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
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct MarkerArray {
    pub markers: std::vec::Vec<Marker>,
}
impl ::roslibrust::RosMessageType for MarkerArray {
    const ROS_TYPE_NAME: &'static str = "visualization_msgs/MarkerArray";
}
impl MarkerArray {}
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
