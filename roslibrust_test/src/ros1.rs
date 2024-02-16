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
        const DEFINITION : & 'static str = "# The stamp should store the time at which this goal was requested.\n# It is used by an action server when it tries to preempt all\n# goals that were requested before a certain time\ntime stamp\n\n# The id provides a way to associate feedback and\n# result message with specific goal requests. The id\n# specified must be unique.\nstring id" ;
    }
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
        const DEFINITION : & 'static str = "GoalID goal_id\nuint8 status\nuint8 PENDING         = 0   # The goal has yet to be processed by the action server\nuint8 ACTIVE          = 1   # The goal is currently being processed by the action server\nuint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n                            #   and has since completed its execution (Terminal State)\nuint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\nuint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n                            #    to some failure (Terminal State)\nuint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n                            #    because the goal was unattainable or invalid (Terminal State)\nuint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n                            #    and has not yet completed execution\nuint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n                            #    but the action server has not yet confirmed that the goal is canceled\nuint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n                            #    and was successfully cancelled (Terminal State)\nuint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n                            #    sent over the wire by an action server\n\n#Allow for the user to associate a string with GoalStatus for debugging\nstring text" ;
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
        const DEFINITION : & 'static str = "# Stores the statuses for goals that are currently being tracked\n# by an action server\nHeader header\nGoalStatus[] status_list" ;
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
        const DEFINITION : & 'static str = "# This message is used to send diagnostic information about the state of the robot\nHeader header #for timestamp\nDiagnosticStatus[] status # an array of components being reported on" ;
    }
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
        const DEFINITION : & 'static str = "# This message holds the status of an individual component of the robot.\n# \n\n# Possible levels of operations\nbyte OK=0\nbyte WARN=1\nbyte ERROR=2\nbyte STALE=3\n\nbyte level # level of operation enumerated above \nstring name # a description of the test/component reporting\nstring message # a description of the status\nstring hardware_id # a hardware unique string\nKeyValue[] values # an array of values associated with the status" ;
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
        const DEFINITION : & 'static str = "string key # what to label this value when viewing\nstring value # a value to track over time" ;
    }
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
        const DEFINITION : & 'static str = "# This service is used as part of the process for loading analyzers at runtime,\n# and should be used by a loader script or program, not as a standalone service.\n# Information about dynamic addition of analyzers can be found at\n# http://wiki.ros.org/diagnostics/Tutorials/Adding%20Analyzers%20at%20Runtime\n\n# The load_namespace parameter defines the namespace where parameters for the\n# initialization of analyzers in the diagnostic aggregator have been loaded. The\n# value should be a global name (i.e. /my/name/space), not a relative\n# (my/name/space) or private (~my/name/space) name. Analyzers will not be added\n# if a non-global name is used. The call will also fail if the namespace\n# contains parameters that follow a namespace structure that does not conform to\n# that expected by the analyzer definitions. See\n# http://wiki.ros.org/diagnostics/Tutorials/Configuring%20Diagnostic%20Aggregators\n# and http://wiki.ros.org/diagnostics/Tutorials/Using%20the%20GenericAnalyzer\n# for examples of the structure of yaml files which are expected to have been\n# loaded into the namespace.\nstring load_namespace" ;
    }
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
        const DEFINITION : & 'static str = "# True if diagnostic aggregator was updated with new diagnostics, False\n# otherwise. A false return value means that either there is a bond in the\n# aggregator which already used the requested namespace, or the initialization\n# of analyzers failed.\nbool success\n\n# Message with additional information about the success or failure\nstring message" ;
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
        const DEFINITION: &'static str = "";
    }
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
        const DEFINITION: &'static str = "string id\nbyte passed\nDiagnosticStatus[] status";
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
        const DEFINITION : & 'static str = "# This expresses acceleration in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular" ;
    }
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
        const DEFINITION: &'static str =
            "# An accel with reference coordinate frame and timestamp\nHeader header\nAccel accel";
    }
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
        #[default(_code = "[Default::default(); 36]")]
        #[serde(with = "::serde_big_array::BigArray")]
        pub r#covariance: [f64; 36],
    }
    impl ::roslibrust_codegen::RosMessageType for AccelWithCovariance {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/AccelWithCovariance";
        const MD5SUM: &'static str = "ad5a718d699c6be72a02b8d6a139f334";
        const DEFINITION : & 'static str = "# This expresses acceleration in free space with uncertainty.\n\nAccel accel\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance" ;
    }
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
        const DEFINITION : & 'static str = "# This represents an estimated accel with reference coordinate frame and timestamp.\nHeader header\nAccelWithCovariance accel" ;
    }
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
        const DEFINITION : & 'static str = "# Mass [kg]\nfloat64 m\n\n# Center of mass [m]\ngeometry_msgs/Vector3 com\n\n# Inertia Tensor [kg-m^2]\n#     | ixx ixy ixz |\n# I = | ixy iyy iyz |\n#     | ixz iyz izz |\nfloat64 ixx\nfloat64 ixy\nfloat64 ixz\nfloat64 iyy\nfloat64 iyz\nfloat64 izz" ;
    }
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
        const DEFINITION: &'static str = "Header header\nInertia inertia";
    }
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
        const DEFINITION : & 'static str = "# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z" ;
    }
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
        const DEFINITION : & 'static str = "# This contains the position of a point in free space(with 32 bits of precision).\n# It is recommeded to use Point wherever possible instead of Point32.  \n# \n# This recommendation is to promote interoperability.  \n#\n# This message is designed to take up less space when sending\n# lots of points at once, as in the case of a PointCloud.  \n\nfloat32 x\nfloat32 y\nfloat32 z" ;
    }
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
        const DEFINITION : & 'static str = "# This represents a Point with reference coordinate frame and timestamp\nHeader header\nPoint point" ;
    }
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
        const DEFINITION : & 'static str = "#A specification of a polygon where the first and last points are assumed to be connected\nPoint32[] points" ;
    }
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
        const DEFINITION : & 'static str = "# This represents a Polygon with reference coordinate frame and timestamp\nHeader header\nPolygon polygon" ;
    }
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
        const DEFINITION : & 'static str = "# A representation of pose in free space, composed of position and orientation. \nPoint position\nQuaternion orientation" ;
    }
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
        const DEFINITION : & 'static str = "# Deprecated\n# Please use the full 3D pose.\n\n# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.\n\n# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.\n\n\n# This expresses a position and orientation on a 2D manifold.\n\nfloat64 x\nfloat64 y\nfloat64 theta" ;
    }
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
        const DEFINITION : & 'static str = "# An array of poses with a header for global reference.\n\nHeader header\n\nPose[] poses" ;
    }
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
        const DEFINITION: &'static str =
            "# A Pose with reference coordinate frame and timestamp\nHeader header\nPose pose";
    }
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
        #[default(_code = "[Default::default(); 36]")]
        #[serde(with = "::serde_big_array::BigArray")]
        pub r#covariance: [f64; 36],
    }
    impl ::roslibrust_codegen::RosMessageType for PoseWithCovariance {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseWithCovariance";
        const MD5SUM: &'static str = "c23e848cf1b7533a8d7c259073a97e6f";
        const DEFINITION : & 'static str = "# This represents a pose in free space with uncertainty.\n\nPose pose\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance" ;
    }
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
        const DEFINITION : & 'static str = "# This expresses an estimated pose with a reference coordinate frame and timestamp\n\nHeader header\nPoseWithCovariance pose" ;
    }
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
        const DEFINITION : & 'static str = "# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w" ;
    }
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
        const DEFINITION : & 'static str = "# This represents an orientation with reference coordinate frame and timestamp.\n\nHeader header\nQuaternion quaternion" ;
    }
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
        const DEFINITION : & 'static str = "# This represents the transform between two coordinate frames in free space.\n\nVector3 translation\nQuaternion rotation" ;
    }
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
        const DEFINITION : & 'static str = "# This expresses a transform from coordinate frame header.frame_id\n# to the coordinate frame child_frame_id\n#\n# This message is mostly used by the \n# <a href=\"http://wiki.ros.org/tf\">tf</a> package. \n# See its documentation for more information.\n\nHeader header\nstring child_frame_id # the frame id of the child frame\nTransform transform" ;
    }
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
        const DEFINITION : & 'static str = "# This expresses velocity in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular" ;
    }
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
        const DEFINITION: &'static str =
            "# A twist with reference coordinate frame and timestamp\nHeader header\nTwist twist";
    }
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
        #[default(_code = "[Default::default(); 36]")]
        #[serde(with = "::serde_big_array::BigArray")]
        pub r#covariance: [f64; 36],
    }
    impl ::roslibrust_codegen::RosMessageType for TwistWithCovariance {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/TwistWithCovariance";
        const MD5SUM: &'static str = "1fe8a28e6890a4cc3ae4c3ca5c7d82e6";
        const DEFINITION : & 'static str = "# This expresses velocity in free space with uncertainty.\n\nTwist twist\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance" ;
    }
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
        const DEFINITION : & 'static str = "# This represents an estimated twist with reference coordinate frame and timestamp.\nHeader header\nTwistWithCovariance twist" ;
    }
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
        const DEFINITION : & 'static str = "# This represents a vector in free space. \n# It is only meant to represent a direction. Therefore, it does not\n# make sense to apply a translation to it (e.g., when applying a \n# generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n# geometry_msgs/Point message instead.\n\nfloat64 x\nfloat64 y\nfloat64 z" ;
    }
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
        const DEFINITION : & 'static str = "# This represents a Vector3 with reference coordinate frame and timestamp\nHeader header\nVector3 vector" ;
    }
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
        const DEFINITION : & 'static str = "# This represents force in free space, separated into\n# its linear and angular parts.\nVector3  force\nVector3  torque" ;
    }
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
        const DEFINITION : & 'static str = "# A wrench with reference coordinate frame and timestamp\nHeader header\nWrench wrench" ;
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
        const DEFINITION : & 'static str = "GetMapActionGoal action_goal\nGetMapActionResult action_result\nGetMapActionFeedback action_feedback" ;
    }
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
        const DEFINITION: &'static str =
            "Header header\nactionlib_msgs/GoalStatus status\nGetMapFeedback feedback";
    }
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
        const DEFINITION: &'static str =
            "Header header\nactionlib_msgs/GoalID goal_id\nGetMapGoal goal";
    }
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
        const DEFINITION: &'static str =
            "Header header\nactionlib_msgs/GoalStatus status\nGetMapResult result";
    }
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
        const DEFINITION: &'static str = "# no feedback";
    }
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
        const DEFINITION: &'static str = "# Get the map as a nav_msgs/OccupancyGrid";
    }
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
        const DEFINITION: &'static str = "nav_msgs/OccupancyGrid map";
    }
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
        const DEFINITION : & 'static str = "#an array of cells in a 2D grid\nHeader header\nfloat32 cell_width\nfloat32 cell_height\ngeometry_msgs/Point[] cells" ;
    }
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
        const DEFINITION : & 'static str = "# This hold basic information about the characterists of the OccupancyGrid\n\n# The time at which the map was loaded\ntime map_load_time\n# The map resolution [m/cell]\nfloat32 resolution\n# Map width [cells]\nuint32 width\n# Map height [cells]\nuint32 height\n# The origin of the map [m, m, rad].  This is the real-world pose of the\n# cell (0,0) in the map.\ngeometry_msgs/Pose origin" ;
    }
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
        const DEFINITION : & 'static str = "# This represents a 2-D grid map, in which each cell represents the probability of\n# occupancy.\n\nHeader header \n\n#MetaData for the map\nMapMetaData info\n\n# The map data, in row-major order, starting with (0,0).  Occupancy\n# probabilities are in the range [0,100].  Unknown is -1.\nint8[] data" ;
    }
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
        const DEFINITION : & 'static str = "# This represents an estimate of a position and velocity in free space.  \n# The pose in this message should be specified in the coordinate frame given by header.frame_id.\n# The twist in this message should be specified in the coordinate frame given by the child_frame_id\nHeader header\nstring child_frame_id\ngeometry_msgs/PoseWithCovariance pose\ngeometry_msgs/TwistWithCovariance twist" ;
    }
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
        const DEFINITION : & 'static str = "#An array of poses that represents a Path for a robot to follow\nHeader header\ngeometry_msgs/PoseStamped[] poses" ;
    }
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
        const DEFINITION: &'static str = "# Get the map as a nav_msgs/OccupancyGrid";
    }
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
        const DEFINITION: &'static str = "nav_msgs/OccupancyGrid map";
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
        const DEFINITION : & 'static str = "# Get a plan from the current position to the goal Pose \n\n# The start pose for the plan\ngeometry_msgs/PoseStamped start\n\n# The final pose of the goal position\ngeometry_msgs/PoseStamped goal\n\n# If the goal is obstructed, how many meters the planner can \n# relax the constraint in x and y before failing. \nfloat32 tolerance" ;
    }
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
        const DEFINITION: &'static str = "nav_msgs/Path plan";
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
        const DEFINITION : & 'static str = "# URL of map resource\n# Can be an absolute path to a file: file:///path/to/maps/floor1.yaml\n# Or, relative to a ROS package: package://my_ros_package/maps/floor2.yaml\nstring map_url" ;
    }
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
        const DEFINITION : & 'static str = "# Result code defintions\nuint8 RESULT_SUCCESS=0\nuint8 RESULT_MAP_DOES_NOT_EXIST=1\nuint8 RESULT_INVALID_MAP_DATA=2\nuint8 RESULT_INVALID_MAP_METADATA=3\nuint8 RESULT_UNDEFINED_FAILURE=255\n\n# Returned map is only valid if result equals RESULT_SUCCESS\nnav_msgs/OccupancyGrid map\nuint8 result" ;
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
        const DEFINITION : & 'static str = "# Set a new map together with an initial pose\nnav_msgs/OccupancyGrid map\ngeometry_msgs/PoseWithCovarianceStamped initial_pose" ;
    }
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
        const DEFINITION: &'static str = "bool success";
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
        const DEFINITION : & 'static str = "string type\nstring[] fieldnames\nstring[] fieldtypes\nint32[] fieldarraylen\nstring[] examples\nstring[] constnames\nstring[] constvalues" ;
    }
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
        const DEFINITION: &'static str = "string name";
    }
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
        const DEFINITION: &'static str = "";
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
        const DEFINITION: &'static str = "";
    }
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
        const DEFINITION: &'static str = "string[] action_servers";
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
        const DEFINITION: &'static str = "string name\nstring default";
    }
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
        const DEFINITION: &'static str = "string value";
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
        const DEFINITION: &'static str = "";
    }
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
        const DEFINITION: &'static str = "string[] names";
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
        const DEFINITION: &'static str = "";
    }
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
        const DEFINITION: &'static str = "time time";
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
        const DEFINITION: &'static str = "string name";
    }
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
        const DEFINITION: &'static str = "bool exists";
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
        const DEFINITION: &'static str = "string type";
    }
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
        const DEFINITION: &'static str = "TypeDef[] typedefs";
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
        const DEFINITION: &'static str = "string node";
    }
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
        const DEFINITION: &'static str =
            "string[] subscribing\nstring[] publishing\nstring[] services";
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
        const DEFINITION: &'static str = "";
    }
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
        const DEFINITION: &'static str = "string[] nodes";
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
        const DEFINITION: &'static str = "string topic";
    }
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
        const DEFINITION: &'static str = "string[] publishers";
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
        const DEFINITION: &'static str = "string name";
    }
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
        const DEFINITION: &'static str = "string global_name";
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
        const DEFINITION: &'static str = "string service";
    }
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
        const DEFINITION: &'static str = "string host";
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
        const DEFINITION: &'static str = "string service";
    }
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
        const DEFINITION: &'static str = "string node";
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
        const DEFINITION: &'static str = "string service";
    }
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
        const DEFINITION: &'static str = "string[] providers";
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
        const DEFINITION: &'static str = "string type";
    }
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
        const DEFINITION: &'static str = "TypeDef[] typedefs";
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
        const DEFINITION: &'static str = "string type";
    }
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
        const DEFINITION: &'static str = "TypeDef[] typedefs";
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
        const DEFINITION: &'static str = "string service";
    }
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
        const DEFINITION: &'static str = "string type";
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
        const DEFINITION: &'static str = "";
    }
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
        const DEFINITION: &'static str = "string[] services";
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
        const DEFINITION: &'static str = "string type";
    }
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
        const DEFINITION: &'static str = "string[] services";
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
        const DEFINITION: &'static str = "string name\nstring value";
    }
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
        const DEFINITION: &'static str = "";
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
        const DEFINITION: &'static str = "string topic";
    }
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
        const DEFINITION: &'static str = "string[] subscribers";
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
        const DEFINITION: &'static str = "string topic";
    }
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
        const DEFINITION: &'static str = "string type";
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
        const DEFINITION: &'static str = "";
    }
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
        const DEFINITION: &'static str = "string[] topics\nstring[] types";
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
        const DEFINITION: &'static str = "";
    }
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
        const DEFINITION: &'static str =
            "string[] topics\nstring[] types\nstring[] typedefs_full_text";
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
        const DEFINITION: &'static str = "string type";
    }
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
        const DEFINITION: &'static str = "string[] topics";
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
        const DEFINITION : & 'static str = "# roslib/Clock is used for publishing simulated time in ROS. \n# This message simply communicates the current time.\n# For more information, see http://www.ros.org/wiki/Clock\ntime clock" ;
    }
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
        const DEFINITION : & 'static str = "##\n## Severity level constants\n##\nbyte DEBUG=1 #debug level\nbyte INFO=2  #general level\nbyte WARN=4  #warning level\nbyte ERROR=8 #error level\nbyte FATAL=16 #fatal/critical level\n##\n## Fields\n##\nHeader header\nbyte level\nstring name # name of the node\nstring msg # message \nstring file # file the message came from\nstring function # function the message came from\nuint32 line # line the message came from\nstring[] topics # topic names that the node publishes" ;
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
        const DEFINITION : & 'static str = "# name of the topic\nstring topic\n\n# node id of the publisher\nstring node_pub\n\n# node id of the subscriber\nstring node_sub\n\n# the statistics apply to this time window\ntime window_start\ntime window_stop\n\n# number of messages delivered during the window\nint32 delivered_msgs\n# numbers of messages dropped during the window\nint32 dropped_msgs\n\n# traffic during the window, in bytes\nint32 traffic\n\n# mean/stddev/max period between two messages\nduration period_mean\nduration period_stddev\nduration period_max\n\n# mean/stddev/max age of the message based on the\n# timestamp in the message header. In case the\n# message does not have a header, it will be 0.\nduration stamp_age_mean\nduration stamp_age_stddev\nduration stamp_age_max" ;
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
        const DEFINITION : & 'static str = "# Constants are chosen to match the enums in the linux kernel\n# defined in include/linux/power_supply.h as of version 3.7\n# The one difference is for style reasons the constants are\n# all uppercase not mixed case.\n\n# Power supply status constants\nuint8 POWER_SUPPLY_STATUS_UNKNOWN = 0\nuint8 POWER_SUPPLY_STATUS_CHARGING = 1\nuint8 POWER_SUPPLY_STATUS_DISCHARGING = 2\nuint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3\nuint8 POWER_SUPPLY_STATUS_FULL = 4\n\n# Power supply health constants\nuint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0\nuint8 POWER_SUPPLY_HEALTH_GOOD = 1\nuint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2\nuint8 POWER_SUPPLY_HEALTH_DEAD = 3\nuint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4\nuint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5\nuint8 POWER_SUPPLY_HEALTH_COLD = 6\nuint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7\nuint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8\n\n# Power supply technology (chemistry) constants\nuint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0\nuint8 POWER_SUPPLY_TECHNOLOGY_NIMH = 1\nuint8 POWER_SUPPLY_TECHNOLOGY_LION = 2\nuint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3\nuint8 POWER_SUPPLY_TECHNOLOGY_LIFE = 4\nuint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5\nuint8 POWER_SUPPLY_TECHNOLOGY_LIMN = 6\n\nHeader  header\nfloat32 voltage          # Voltage in Volts (Mandatory)\nfloat32 temperature      # Temperature in Degrees Celsius (If unmeasured NaN)\nfloat32 current          # Negative when discharging (A)  (If unmeasured NaN)\nfloat32 charge           # Current charge in Ah  (If unmeasured NaN)\nfloat32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)\nfloat32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)\nfloat32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)\nuint8   power_supply_status     # The charging status as reported. Values defined above\nuint8   power_supply_health     # The battery health metric. Values defined above\nuint8   power_supply_technology # The battery chemistry. Values defined above\nbool    present          # True if the battery is present\n\nfloat32[] cell_voltage   # An array of individual cell voltages for each cell in the pack\n                         # If individual voltages unknown but number of cells known set each to NaN\nfloat32[] cell_temperature  # An array of individual cell temperatures for each cell in the pack\n                            # If individual temperatures unknown but number of cells known set each to NaN\nstring location          # The location into which the battery is inserted. (slot number or plug)\nstring serial_number     # The best approximation of the battery serial number" ;
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
        pub r#K: [f64; 9],
        pub r#R: [f64; 9],
        pub r#P: [f64; 12],
        pub r#binning_x: u32,
        pub r#binning_y: u32,
        pub r#roi: self::RegionOfInterest,
    }
    impl ::roslibrust_codegen::RosMessageType for CameraInfo {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/CameraInfo";
        const MD5SUM: &'static str = "c9a58c1b0b154e0e6da7578cb991d214";
        const DEFINITION : & 'static str = "# This message defines meta information for a camera. It should be in a\n# camera namespace on topic \"camera_info\" and accompanied by up to five\n# image topics named:\n#\n#   image_raw - raw data from the camera driver, possibly Bayer encoded\n#   image            - monochrome, distorted\n#   image_color      - color, distorted\n#   image_rect       - monochrome, rectified\n#   image_rect_color - color, rectified\n#\n# The image_pipeline contains packages (image_proc, stereo_image_proc)\n# for producing the four processed image topics from image_raw and\n# camera_info. The meaning of the camera parameters are described in\n# detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.\n#\n# The image_geometry package provides a user-friendly interface to\n# common operations using this meta information. If you want to, e.g.,\n# project a 3d point into image coordinates, we strongly recommend\n# using image_geometry.\n#\n# If the camera is uncalibrated, the matrices D, K, R, P should be left\n# zeroed out. In particular, clients may assume that K[0] == 0.0\n# indicates an uncalibrated camera.\n\n#######################################################################\n#                     Image acquisition info                          #\n#######################################################################\n\n# Time of image acquisition, camera coordinate frame ID\nHeader header    # Header timestamp should be acquisition time of image\n                 # Header frame_id should be optical frame of camera\n                 # origin of frame should be optical center of camera\n                 # +x should point to the right in the image\n                 # +y should point down in the image\n                 # +z should point into the plane of the image\n\n\n#######################################################################\n#                      Calibration Parameters                         #\n#######################################################################\n# These are fixed during camera calibration. Their values will be the #\n# same in all messages until the camera is recalibrated. Note that    #\n# self-calibrating systems may \"recalibrate\" frequently.              #\n#                                                                     #\n# The internal parameters can be used to warp a raw (distorted) image #\n# to:                                                                 #\n#   1. An undistorted image (requires D and K)                        #\n#   2. A rectified image (requires D, K, R)                           #\n# The projection matrix P projects 3D points into the rectified image.#\n#######################################################################\n\n# The image dimensions with which the camera was calibrated. Normally\n# this will be the full camera resolution in pixels.\nuint32 height\nuint32 width\n\n# The distortion model used. Supported models are listed in\n# sensor_msgs/distortion_models.h. For most cameras, \"plumb_bob\" - a\n# simple model of radial and tangential distortion - is sufficient.\nstring distortion_model\n\n# The distortion parameters, size depending on the distortion model.\n# For \"plumb_bob\", the 5 parameters are: (k1, k2, t1, t2, k3).\nfloat64[] D\n\n# Intrinsic camera matrix for the raw (distorted) images.\n#     [fx  0 cx]\n# K = [ 0 fy cy]\n#     [ 0  0  1]\n# Projects 3D points in the camera coordinate frame to 2D pixel\n# coordinates using the focal lengths (fx, fy) and principal point\n# (cx, cy).\nfloat64[9]  K # 3x3 row-major matrix\n\n# Rectification matrix (stereo cameras only)\n# A rotation matrix aligning the camera coordinate system to the ideal\n# stereo image plane so that epipolar lines in both stereo images are\n# parallel.\nfloat64[9]  R # 3x3 row-major matrix\n\n# Projection/camera matrix\n#     [fx'  0  cx' Tx]\n# P = [ 0  fy' cy' Ty]\n#     [ 0   0   1   0]\n# By convention, this matrix specifies the intrinsic (camera) matrix\n#  of the processed (rectified) image. That is, the left 3x3 portion\n#  is the normal camera intrinsic matrix for the rectified image.\n# It projects 3D points in the camera coordinate frame to 2D pixel\n#  coordinates using the focal lengths (fx', fy') and principal point\n#  (cx', cy') - these may differ from the values in K.\n# For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will\n#  also have R = the identity and P[1:3,1:3] = K.\n# For a stereo pair, the fourth column [Tx Ty 0]' is related to the\n#  position of the optical center of the second camera in the first\n#  camera's frame. We assume Tz = 0 so both cameras are in the same\n#  stereo image plane. The first camera always has Tx = Ty = 0. For\n#  the right (second) camera of a horizontal stereo pair, Ty = 0 and\n#  Tx = -fx' * B, where B is the baseline between the cameras.\n# Given a 3D point [X Y Z]', the projection (x, y) of the point onto\n#  the rectified image is given by:\n#  [u v w]' = P * [X Y Z 1]'\n#         x = u / w\n#         y = v / w\n#  This holds for both images of a stereo pair.\nfloat64[12] P # 3x4 row-major matrix\n\n\n#######################################################################\n#                      Operational Parameters                         #\n#######################################################################\n# These define the image region actually captured by the camera       #\n# driver. Although they affect the geometry of the output image, they #\n# may be changed freely without recalibrating the camera.             #\n#######################################################################\n\n# Binning refers here to any camera setting which combines rectangular\n#  neighborhoods of pixels into larger \"super-pixels.\" It reduces the\n#  resolution of the output image to\n#  (width / binning_x) x (height / binning_y).\n# The default values binning_x = binning_y = 0 is considered the same\n#  as binning_x = binning_y = 1 (no subsampling).\nuint32 binning_x\nuint32 binning_y\n\n# Region of interest (subwindow of full camera resolution), given in\n#  full resolution (unbinned) image coordinates. A particular ROI\n#  always denotes the same window of pixels on the camera sensor,\n#  regardless of binning settings.\n# The default setting of roi (all values 0) is considered the same as\n#  full resolution (roi.width = width, roi.height = height).\nRegionOfInterest roi" ;
    }
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
        const DEFINITION : & 'static str = "# This message is used by the PointCloud message to hold optional data\n# associated with each point in the cloud. The length of the values\n# array should be the same as the length of the points array in the\n# PointCloud, and each value should be associated with the corresponding\n# point.\n\n# Channel names in existing practice include:\n#   \"u\", \"v\" - row and column (respectively) in the left stereo image.\n#              This is opposite to usual conventions but remains for\n#              historical reasons. The newer PointCloud2 message has no\n#              such problem.\n#   \"rgb\" - For point clouds produced by color stereo cameras. uint8\n#           (R,G,B) values packed into the least significant 24 bits,\n#           in order.\n#   \"intensity\" - laser or pixel intensity.\n#   \"distance\"\n\n# The channel name should give semantics of the channel (e.g.\n# \"intensity\" instead of \"value\").\nstring name\n\n# The values array should be 1-1 with the elements of the associated\n# PointCloud.\nfloat32[] values" ;
    }
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
        const DEFINITION : & 'static str = "# This message contains a compressed image\n\nHeader header        # Header timestamp should be acquisition time of image\n                     # Header frame_id should be optical frame of camera\n                     # origin of frame should be optical center of camera\n                     # +x should point to the right in the image\n                     # +y should point down in the image\n                     # +z should point into to plane of the image\n\nstring format        # Specifies the format of the data\n                     #   Acceptable values:\n                     #     jpeg, png\nuint8[] data         # Compressed image buffer" ;
    }
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
        const DEFINITION : & 'static str = "# Single pressure reading.  This message is appropriate for measuring the\n # pressure inside of a fluid (air, water, etc).  This also includes\n # atmospheric or barometric pressure.\n\n # This message is not appropriate for force/pressure contact sensors.\n\n Header header           # timestamp of the measurement\n                         # frame_id is the location of the pressure sensor\n\n float64 fluid_pressure  # Absolute pressure reading in Pascals.\n\n float64 variance        # 0 is interpreted as variance unknown" ;
    }
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
        const DEFINITION : & 'static str = "# Single photometric illuminance measurement.  Light should be assumed to be\n # measured along the sensor's x-axis (the area of detection is the y-z plane).\n # The illuminance should have a 0 or positive value and be received with\n # the sensor's +X axis pointing toward the light source.\n\n # Photometric illuminance is the measure of the human eye's sensitivity of the\n # intensity of light encountering or passing through a surface.\n\n # All other Photometric and Radiometric measurements should\n # not use this message.\n # This message cannot represent:\n # Luminous intensity (candela/light source output)\n # Luminance (nits/light output per area)\n # Irradiance (watt/area), etc.\n\n Header header           # timestamp is the time the illuminance was measured\n                         # frame_id is the location and direction of the reading\n\n float64 illuminance     # Measurement of the Photometric Illuminance in Lux.\n\n float64 variance        # 0 is interpreted as variance unknown" ;
    }
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
        const DEFINITION : & 'static str = "# This message contains an uncompressed image\n# (0, 0) is at top-left corner of image\n#\n\nHeader header        # Header timestamp should be acquisition time of image\n                     # Header frame_id should be optical frame of camera\n                     # origin of frame should be optical center of camera\n                     # +x should point to the right in the image\n                     # +y should point down in the image\n                     # +z should point into to plane of the image\n                     # If the frame_id here and the frame_id of the CameraInfo\n                     # message associated with the image conflict\n                     # the behavior is undefined\n\nuint32 height         # image height, that is, number of rows\nuint32 width          # image width, that is, number of columns\n\n# The legal values for encoding are in file src/image_encodings.cpp\n# If you want to standardize a new string format, join\n# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n\nstring encoding       # Encoding of pixels -- channel meaning, ordering, size\n                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n\nuint8 is_bigendian    # is this data bigendian?\nuint32 step           # Full row length in bytes\nuint8[] data          # actual matrix data, size is (step * rows)" ;
    }
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
        pub r#orientation_covariance: [f64; 9],
        pub r#angular_velocity: geometry_msgs::Vector3,
        pub r#angular_velocity_covariance: [f64; 9],
        pub r#linear_acceleration: geometry_msgs::Vector3,
        pub r#linear_acceleration_covariance: [f64; 9],
    }
    impl ::roslibrust_codegen::RosMessageType for Imu {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Imu";
        const MD5SUM: &'static str = "6a62c6daae103f4ff57a132d6f95cec2";
        const DEFINITION : & 'static str = "# This is a message to hold data from an IMU (Inertial Measurement Unit)\n#\n# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec\n#\n# If the covariance of the measurement is known, it should be filled in (if all you know is the \n# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)\n# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the\n# data a covariance will have to be assumed or gotten from some other source\n#\n# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation \n# estimate), please set element 0 of the associated covariance matrix to -1\n# If you are interpreting this message, please check for a value of -1 in the first element of each \n# covariance matrix, and disregard the associated estimate.\n\nHeader header\n\ngeometry_msgs/Quaternion orientation\nfloat64[9] orientation_covariance # Row major about x, y, z axes\n\ngeometry_msgs/Vector3 angular_velocity\nfloat64[9] angular_velocity_covariance # Row major about x, y, z axes\n\ngeometry_msgs/Vector3 linear_acceleration\nfloat64[9] linear_acceleration_covariance # Row major x, y z" ;
    }
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
        const DEFINITION : & 'static str = "# This is a message that holds data to describe the state of a set of torque controlled joints. \n#\n# The state of each joint (revolute or prismatic) is defined by:\n#  * the position of the joint (rad or m),\n#  * the velocity of the joint (rad/s or m/s) and \n#  * the effort that is applied in the joint (Nm or N).\n#\n# Each joint is uniquely identified by its name\n# The header specifies the time at which the joint states were recorded. All the joint states\n# in one message have to be recorded at the same time.\n#\n# This message consists of a multiple arrays, one for each part of the joint state. \n# The goal is to make each of the fields optional. When e.g. your joints have no\n# effort associated with them, you can leave the effort array empty. \n#\n# All arrays in this message should have the same size, or be empty.\n# This is the only way to uniquely associate the joint name with the correct\n# states.\n\n\nHeader header\n\nstring[] name\nfloat64[] position\nfloat64[] velocity\nfloat64[] effort" ;
    }
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
        const DEFINITION : & 'static str = "# Reports the state of a joysticks axes and buttons.\nHeader header           # timestamp in the header is the time the data is received from the joystick\nfloat32[] axes          # the axes measurements from a joystick\nint32[] buttons         # the buttons measurements from a joystick" ;
    }
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
        const DEFINITION : & 'static str = "# Declare of the type of feedback\nuint8 TYPE_LED    = 0\nuint8 TYPE_RUMBLE = 1\nuint8 TYPE_BUZZER = 2\n\nuint8 type\n\n# This will hold an id number for each type of each feedback.\n# Example, the first led would be id=0, the second would be id=1\nuint8 id\n\n# Intensity of the feedback, from 0.0 to 1.0, inclusive.  If device is\n# actually binary, driver should treat 0<=x<0.5 as off, 0.5<=x<=1 as on.\nfloat32 intensity" ;
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
        const DEFINITION: &'static str =
            "# This message publishes values for multiple feedback at once. \nJoyFeedback[] array";
    }
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
        const DEFINITION : & 'static str = "# This message is a submessage of MultiEchoLaserScan and is not intended\n# to be used separately.\n\nfloat32[] echoes  # Multiple values of ranges or intensities.\n                  # Each array represents data from the same angle increment." ;
    }
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
        const DEFINITION : & 'static str = "# Single scan from a planar laser range-finder\n#\n# If you have another ranging device with different behavior (e.g. a sonar\n# array), please find or create a different message, since applications\n# will make fairly laser-specific assumptions about this data\n\nHeader header            # timestamp in the header is the acquisition time of \n                         # the first ray in the scan.\n                         #\n                         # in frame frame_id, angles are measured around \n                         # the positive Z axis (counterclockwise, if Z is up)\n                         # with zero angle being forward along the x axis\n                         \nfloat32 angle_min        # start angle of the scan [rad]\nfloat32 angle_max        # end angle of the scan [rad]\nfloat32 angle_increment  # angular distance between measurements [rad]\n\nfloat32 time_increment   # time between measurements [seconds] - if your scanner\n                         # is moving, this will be used in interpolating position\n                         # of 3d points\nfloat32 scan_time        # time between scans [seconds]\n\nfloat32 range_min        # minimum range value [m]\nfloat32 range_max        # maximum range value [m]\n\nfloat32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)\nfloat32[] intensities    # intensity data [device-specific units].  If your\n                         # device does not provide intensities, please leave\n                         # the array empty." ;
    }
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
        pub r#magnetic_field_covariance: [f64; 9],
    }
    impl ::roslibrust_codegen::RosMessageType for MagneticField {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/MagneticField";
        const MD5SUM: &'static str = "2f3b0b43eed0c9501de0fa3ff89a45aa";
        const DEFINITION : & 'static str = "# Measurement of the Magnetic Field vector at a specific location.\n\n # If the covariance of the measurement is known, it should be filled in\n # (if all you know is the variance of each measurement, e.g. from the datasheet,\n #just put those along the diagonal)\n # A covariance matrix of all zeros will be interpreted as \"covariance unknown\",\n # and to use the data a covariance will have to be assumed or gotten from some\n # other source\n\n\n Header header                        # timestamp is the time the\n                                      # field was measured\n                                      # frame_id is the location and orientation\n                                      # of the field measurement\n\n geometry_msgs/Vector3 magnetic_field # x, y, and z components of the\n                                      # field vector in Tesla\n                                      # If your sensor does not output 3 axes,\n                                      # put NaNs in the components not reported.\n\n float64[9] magnetic_field_covariance # Row major about x, y, z axes\n                                      # 0 is interpreted as variance unknown" ;
    }
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
        const DEFINITION : & 'static str = "# Representation of state for joints with multiple degrees of freedom, \n# following the structure of JointState.\n#\n# It is assumed that a joint in a system corresponds to a transform that gets applied \n# along the kinematic chain. For example, a planar joint (as in URDF) is 3DOF (x, y, yaw)\n# and those 3DOF can be expressed as a transformation matrix, and that transformation\n# matrix can be converted back to (x, y, yaw)\n#\n# Each joint is uniquely identified by its name\n# The header specifies the time at which the joint states were recorded. All the joint states\n# in one message have to be recorded at the same time.\n#\n# This message consists of a multiple arrays, one for each part of the joint state. \n# The goal is to make each of the fields optional. When e.g. your joints have no\n# wrench associated with them, you can leave the wrench array empty. \n#\n# All arrays in this message should have the same size, or be empty.\n# This is the only way to uniquely associate the joint name with the correct\n# states.\n\nHeader header\n\nstring[] joint_names\ngeometry_msgs/Transform[] transforms\ngeometry_msgs/Twist[] twist\ngeometry_msgs/Wrench[] wrench" ;
    }
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
        const DEFINITION : & 'static str = "# Single scan from a multi-echo planar laser range-finder\n#\n# If you have another ranging device with different behavior (e.g. a sonar\n# array), please find or create a different message, since applications\n# will make fairly laser-specific assumptions about this data\n\nHeader header            # timestamp in the header is the acquisition time of \n                         # the first ray in the scan.\n                         #\n                         # in frame frame_id, angles are measured around \n                         # the positive Z axis (counterclockwise, if Z is up)\n                         # with zero angle being forward along the x axis\n                         \nfloat32 angle_min        # start angle of the scan [rad]\nfloat32 angle_max        # end angle of the scan [rad]\nfloat32 angle_increment  # angular distance between measurements [rad]\n\nfloat32 time_increment   # time between measurements [seconds] - if your scanner\n                         # is moving, this will be used in interpolating position\n                         # of 3d points\nfloat32 scan_time        # time between scans [seconds]\n\nfloat32 range_min        # minimum range value [m]\nfloat32 range_max        # maximum range value [m]\n\nLaserEcho[] ranges       # range data [m] (Note: NaNs, values < range_min or > range_max should be discarded)\n                         # +Inf measurements are out of range\n                         # -Inf measurements are too close to determine exact distance.\nLaserEcho[] intensities  # intensity data [device-specific units].  If your\n                         # device does not provide intensities, please leave\n                         # the array empty." ;
    }
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
        pub r#position_covariance: [f64; 9],
        pub r#position_covariance_type: u8,
    }
    impl ::roslibrust_codegen::RosMessageType for NavSatFix {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/NavSatFix";
        const MD5SUM: &'static str = "2d3a8cd499b9b4a0249fb98fd05cfa48";
        const DEFINITION : & 'static str = "# Navigation Satellite fix for any Global Navigation Satellite System\n#\n# Specified using the WGS 84 reference ellipsoid\n\n# header.stamp specifies the ROS time for this measurement (the\n#        corresponding satellite time may be reported using the\n#        sensor_msgs/TimeReference message).\n#\n# header.frame_id is the frame of reference reported by the satellite\n#        receiver, usually the location of the antenna.  This is a\n#        Euclidean frame relative to the vehicle, not a reference\n#        ellipsoid.\nHeader header\n\n# satellite fix status information\nNavSatStatus status\n\n# Latitude [degrees]. Positive is north of equator; negative is south.\nfloat64 latitude\n\n# Longitude [degrees]. Positive is east of prime meridian; negative is west.\nfloat64 longitude\n\n# Altitude [m]. Positive is above the WGS 84 ellipsoid\n# (quiet NaN if no altitude is available).\nfloat64 altitude\n\n# Position covariance [m^2] defined relative to a tangential plane\n# through the reported position. The components are East, North, and\n# Up (ENU), in row-major order.\n#\n# Beware: this coordinate system exhibits singularities at the poles.\n\nfloat64[9] position_covariance\n\n# If the covariance of the fix is known, fill it in completely. If the\n# GPS receiver provides the variance of each measurement, put them\n# along the diagonal. If only Dilution of Precision is available,\n# estimate an approximate covariance from that.\n\nuint8 COVARIANCE_TYPE_UNKNOWN = 0\nuint8 COVARIANCE_TYPE_APPROXIMATED = 1\nuint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2\nuint8 COVARIANCE_TYPE_KNOWN = 3\n\nuint8 position_covariance_type" ;
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
        const DEFINITION : & 'static str = "# Navigation Satellite fix status for any Global Navigation Satellite System\n\n# Whether to output an augmented fix is determined by both the fix\n# type and the last time differential corrections were received.  A\n# fix is valid when status >= STATUS_FIX.\n\nint8 STATUS_NO_FIX =  -1        # unable to fix position\nint8 STATUS_FIX =      0        # unaugmented fix\nint8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation\nint8 STATUS_GBAS_FIX = 2        # with ground-based augmentation\n\nint8 status\n\n# Bits defining which Global Navigation Satellite System signals were\n# used by the receiver.\n\nuint16 SERVICE_GPS =     1\nuint16 SERVICE_GLONASS = 2\nuint16 SERVICE_COMPASS = 4      # includes BeiDou.\nuint16 SERVICE_GALILEO = 8\n\nuint16 service" ;
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
        const DEFINITION : & 'static str = "# This message holds a collection of 3d points, plus optional additional\n# information about each point.\n\n# Time of sensor data acquisition, coordinate frame ID.\nHeader header\n\n# Array of 3d points. Each Point32 should be interpreted as a 3d point\n# in the frame given in the header.\ngeometry_msgs/Point32[] points\n\n# Each channel should have the same number of elements as points array,\n# and the data in each channel should correspond 1:1 with each point.\n# Channel names in common practice are listed in ChannelFloat32.msg.\nChannelFloat32[] channels" ;
    }
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
        const DEFINITION : & 'static str = "# This message holds a collection of N-dimensional points, which may\n# contain additional information such as normals, intensity, etc. The\n# point data is stored as a binary blob, its layout described by the\n# contents of the \"fields\" array.\n\n# The point cloud data may be organized 2d (image-like) or 1d\n# (unordered). Point clouds organized as 2d images may be produced by\n# camera depth sensors such as stereo or time-of-flight.\n\n# Time of sensor data acquisition, and the coordinate frame ID (for 3d\n# points).\nHeader header\n\n# 2D structure of the point cloud. If the cloud is unordered, height is\n# 1 and width is the length of the point cloud.\nuint32 height\nuint32 width\n\n# Describes the channels and their layout in the binary data blob.\nPointField[] fields\n\nbool    is_bigendian # Is this data bigendian?\nuint32  point_step   # Length of a point in bytes\nuint32  row_step     # Length of a row in bytes\nuint8[] data         # Actual point data, size is (row_step*height)\n\nbool is_dense        # True if there are no invalid points" ;
    }
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
        const DEFINITION : & 'static str = "# This message holds the description of one point entry in the\n# PointCloud2 message format.\nuint8 INT8    = 1\nuint8 UINT8   = 2\nuint8 INT16   = 3\nuint8 UINT16  = 4\nuint8 INT32   = 5\nuint8 UINT32  = 6\nuint8 FLOAT32 = 7\nuint8 FLOAT64 = 8\n\nstring name      # Name of field\nuint32 offset    # Offset from start of point struct\nuint8  datatype  # Datatype enumeration, see above\nuint32 count     # How many elements in the field" ;
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
        const DEFINITION : & 'static str = "# Single range reading from an active ranger that emits energy and reports\n# one range reading that is valid along an arc at the distance measured. \n# This message is  not appropriate for laser scanners. See the LaserScan\n# message if you are working with a laser scanner.\n\n# This message also can represent a fixed-distance (binary) ranger.  This\n# sensor will have min_range===max_range===distance of detection.\n# These sensors follow REP 117 and will output -Inf if the object is detected\n# and +Inf if the object is outside of the detection range.\n\nHeader header           # timestamp in the header is the time the ranger\n                        # returned the distance reading\n\n# Radiation type enums\n# If you want a value added to this list, send an email to the ros-users list\nuint8 ULTRASOUND=0\nuint8 INFRARED=1\n\nuint8 radiation_type    # the type of radiation used by the sensor\n                        # (sound, IR, etc) [enum]\n\nfloat32 field_of_view   # the size of the arc that the distance reading is\n                        # valid for [rad]\n                        # the object causing the range reading may have\n                        # been anywhere within -field_of_view/2 and\n                        # field_of_view/2 at the measured range. \n                        # 0 angle corresponds to the x-axis of the sensor.\n\nfloat32 min_range       # minimum range value [m]\nfloat32 max_range       # maximum range value [m]\n                        # Fixed distance rangers require min_range==max_range\n\nfloat32 range           # range data [m]\n                        # (Note: values < range_min or > range_max\n                        # should be discarded)\n                        # Fixed distance rangers only output -Inf or +Inf.\n                        # -Inf represents a detection within fixed distance.\n                        # (Detection too close to the sensor to quantify)\n                        # +Inf represents no detection within the fixed distance.\n                        # (Object out of range)" ;
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
        const DEFINITION : & 'static str = "# This message is used to specify a region of interest within an image.\n#\n# When used to specify the ROI setting of the camera when the image was\n# taken, the height and width fields should either match the height and\n# width fields for the associated image; or height = width = 0\n# indicates that the full resolution image was captured.\n\nuint32 x_offset  # Leftmost pixel of the ROI\n                 # (0 if the ROI includes the left edge of the image)\nuint32 y_offset  # Topmost pixel of the ROI\n                 # (0 if the ROI includes the top edge of the image)\nuint32 height    # Height of ROI\nuint32 width     # Width of ROI\n\n# True if a distinct rectified ROI should be calculated from the \"raw\"\n# ROI in this message. Typically this should be False if the full image\n# is captured (ROI not used), and True if a subwindow is captured (ROI\n# used).\nbool do_rectify" ;
    }
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
        const DEFINITION : & 'static str = "# Single reading from a relative humidity sensor.  Defines the ratio of partial\n # pressure of water vapor to the saturated vapor pressure at a temperature.\n\n Header header             # timestamp of the measurement\n                           # frame_id is the location of the humidity sensor\n\n float64 relative_humidity # Expression of the relative humidity\n                           # from 0.0 to 1.0.\n                           # 0.0 is no partial pressure of water vapor\n                           # 1.0 represents partial pressure of saturation\n\n float64 variance          # 0 is interpreted as variance unknown" ;
    }
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
        const DEFINITION : & 'static str = "# Single temperature reading.\n\n Header header           # timestamp is the time the temperature was measured\n                         # frame_id is the location of the temperature reading\n\n float64 temperature     # Measurement of the Temperature in Degrees Celsius\n\n float64 variance        # 0 is interpreted as variance unknown" ;
    }
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
        const DEFINITION : & 'static str = "# Measurement from an external time source not actively synchronized with the system clock.\n\nHeader header    # stamp is system time for which measurement was valid\n                 # frame_id is not used \n\ntime   time_ref  # corresponding time from this external source\nstring source    # (optional) name of time source" ;
    }
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
        const DEFINITION : & 'static str = "# This service requests that a camera stores the given CameraInfo \n# as that camera's calibration information.\n#\n# The width and height in the camera_info field should match what the\n# camera is currently outputting on its camera_info topic, and the camera\n# will assume that the region of the imager that is being referred to is\n# the region that the camera is currently capturing.\n\nsensor_msgs/CameraInfo camera_info # The camera_info to store" ;
    }
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
        const DEFINITION : & 'static str = "bool success          # True if the call succeeded\nstring status_message # Used to give details about success" ;
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
        const DEFINITION : & 'static str = "# Definition of a mesh\n\n# list of triangles; the index values refer to positions in vertices[]\nMeshTriangle[] triangles\n\n# the actual vertices that make up the mesh\ngeometry_msgs/Point[] vertices" ;
    }
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
        pub r#vertex_indices: [u32; 3],
    }
    impl ::roslibrust_codegen::RosMessageType for MeshTriangle {
        const ROS_TYPE_NAME: &'static str = "shape_msgs/MeshTriangle";
        const MD5SUM: &'static str = "23688b2e6d2de3d32fe8af104a903253";
        const DEFINITION: &'static str =
            "# Definition of a triangle's vertices\nuint32[3] vertex_indices";
    }
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
        pub r#coef: [f64; 4],
    }
    impl ::roslibrust_codegen::RosMessageType for Plane {
        const ROS_TYPE_NAME: &'static str = "shape_msgs/Plane";
        const MD5SUM: &'static str = "2c1b92ed8f31492f8e73f6a4a44ca796";
        const DEFINITION : & 'static str = "# Representation of a plane, using the plane equation ax + by + cz + d = 0\n\n# a := coef[0]\n# b := coef[1]\n# c := coef[2]\n# d := coef[3]\n\nfloat64[4] coef" ;
    }
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
        const DEFINITION : & 'static str = "# Define box, sphere, cylinder, cone \n# All shapes are defined to have their bounding boxes centered around 0,0,0.\n\nuint8 BOX=1\nuint8 SPHERE=2\nuint8 CYLINDER=3\nuint8 CONE=4\n\n# The type of the shape\nuint8 type\n\n\n# The dimensions of the shape\nfloat64[] dimensions\n\n# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array\n\n# For the BOX type, the X, Y, and Z dimensions are the length of the corresponding\n# sides of the box.\nuint8 BOX_X=0\nuint8 BOX_Y=1\nuint8 BOX_Z=2\n\n\n# For the SPHERE type, only one component is used, and it gives the radius of\n# the sphere.\nuint8 SPHERE_RADIUS=0\n\n\n# For the CYLINDER and CONE types, the center line is oriented along\n# the Z axis.  Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component\n# of dimensions gives the height of the cylinder (cone).  The\n# CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the\n# radius of the base of the cylinder (cone).  Cone and cylinder\n# primitives are defined to be circular. The tip of the cone is\n# pointing up, along +Z axis.\n\nuint8 CYLINDER_HEIGHT=0\nuint8 CYLINDER_RADIUS=1\n\nuint8 CONE_HEIGHT=0\nuint8 CONE_RADIUS=1" ;
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
        const DEFINITION: &'static str = "bool data";
    }
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
        const DEFINITION: &'static str = "byte data";
    }
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
        const DEFINITION : & 'static str = "# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nbyte[]            data          # array of data" ;
    }
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
        const DEFINITION: &'static str = "char data";
    }
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
        const DEFINITION: &'static str = "float32 r\nfloat32 g\nfloat32 b\nfloat32 a";
    }
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
        const DEFINITION: &'static str = "duration data";
    }
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
        const DEFINITION: &'static str = "";
    }
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
        const DEFINITION: &'static str = "float32 data";
    }
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
        const DEFINITION : & 'static str = "# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nfloat32[]         data          # array of data" ;
    }
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
        const DEFINITION: &'static str = "float64 data";
    }
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
        const DEFINITION : & 'static str = "# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nfloat64[]         data          # array of data" ;
    }
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
        const DEFINITION : & 'static str = "# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id" ;
    }
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
        const DEFINITION: &'static str = "int16 data";
    }
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
        const DEFINITION : & 'static str = "# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nint16[]           data          # array of data" ;
    }
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
        const DEFINITION: &'static str = "int32 data";
    }
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
        const DEFINITION : & 'static str = "# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nint32[]           data          # array of data" ;
    }
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
        const DEFINITION: &'static str = "int64 data";
    }
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
        const DEFINITION : & 'static str = "# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nint64[]           data          # array of data" ;
    }
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
        const DEFINITION: &'static str = "int8 data";
    }
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
        const DEFINITION : & 'static str = "# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nint8[]            data          # array of data" ;
    }
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
        const DEFINITION : & 'static str = "string label   # label of given dimension\nuint32 size    # size of given dimension (in type units)\nuint32 stride  # stride of given dimension" ;
    }
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
        const DEFINITION : & 'static str = "# The multiarray declares a generic multi-dimensional array of a\n# particular data type.  Dimensions are ordered from outer most\n# to inner most.\n\nMultiArrayDimension[] dim # Array of dimension properties\nuint32 data_offset        # padding elements at front of data\n\n# Accessors should ALWAYS be written in terms of dimension stride\n# and specified outer-most dimension first.\n# \n# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n#\n# A standard, 3-channel 640x480 image with interleaved color channels\n# would be specified as:\n#\n# dim[0].label  = \"height\"\n# dim[0].size   = 480\n# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n# dim[1].label  = \"width\"\n# dim[1].size   = 640\n# dim[1].stride = 3*640 = 1920\n# dim[2].label  = \"channel\"\n# dim[2].size   = 3\n# dim[2].stride = 3\n#\n# multiarray(i,j,k) refers to the ith row, jth column, and kth channel." ;
    }
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
        const DEFINITION: &'static str = "string data";
    }
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
        const DEFINITION: &'static str = "time data";
    }
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
        const DEFINITION: &'static str = "uint16 data";
    }
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
        const DEFINITION : & 'static str = "# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nuint16[]            data        # array of data" ;
    }
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
        const DEFINITION: &'static str = "uint32 data";
    }
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
        const DEFINITION : & 'static str = "# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nuint32[]          data          # array of data" ;
    }
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
        const DEFINITION: &'static str = "uint64 data";
    }
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
        const DEFINITION : & 'static str = "# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nuint64[]          data          # array of data" ;
    }
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
        const DEFINITION: &'static str = "uint8 data";
    }
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
        const DEFINITION : & 'static str = "# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nuint8[]           data          # array of data" ;
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
        const DEFINITION: &'static str = "";
    }
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
        const DEFINITION: &'static str = "";
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
        const DEFINITION: &'static str = "bool data # e.g. for hardware enabling / disabling";
    }
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
        const DEFINITION : & 'static str = "bool success   # indicate successful run of triggered service\nstring message # informational, e.g. for error messages" ;
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
        const DEFINITION: &'static str = "";
    }
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
        const DEFINITION : & 'static str = "bool success   # indicate successful run of triggered service\nstring message # informational, e.g. for error messages" ;
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
        const DEFINITION : & 'static str = "# Separate header for compatibility with current TimeSynchronizer.\n# Likely to be removed in a later release, use image.header instead.\nHeader header\n\n# Floating point disparity image. The disparities are pre-adjusted for any\n# x-offset between the principal points of the two cameras (in the case\n# that they are verged). That is: d = x_l - x_r - (cx_l - cx_r)\nsensor_msgs/Image image\n\n# Stereo geometry. For disparity d, the depth from the camera is Z = fT/d.\nfloat32 f # Focal length, pixels\nfloat32 T # Baseline, world units\n\n# Subwindow of (potentially) valid disparity values.\nsensor_msgs/RegionOfInterest valid_window\n\n# The range of disparities searched.\n# In the disparity image, any disparity less than min_disparity is invalid.\n# The disparity search range defines the horopter, or 3D volume that the\n# stereo algorithm can \"see\". Points with Z outside of:\n#     Z_min = fT / max_disparity\n#     Z_max = fT / min_disparity\n# could not be found.\nfloat32 min_disparity\nfloat32 max_disparity\n\n# Smallest allowed disparity increment. The smallest achievable depth range\n# resolution is delta_Z = (Z^2/fT)*delta_d.\nfloat32 delta_d" ;
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
        const MD5SUM: &'static str = "027df5f26b72c57b1e40902038ca3eec";
        const DEFINITION : & 'static str = "string TEST_STR=\"/topic\"\nstring TEST_STR_2 = '/topic_2'\n# Apparently unquoted strings are also valid?\n# Pulled from https://github.com/ros/bond_core/blob/kinetic-devel/bond/msg/Constants.msg\nstring DISABLE_HEARTBEAT_TIMEOUT_PARAM=/bond_disable_heartbeat_timeout\nfloat32 TEST_FLOAT=0 # testing" ;
    }
    impl Constants {
        pub const r#TEST_STR: &'static str = "\"/topic\"";
        pub const r#TEST_STR_2: &'static str = "'/topic_2'";
        pub const r#DISABLE_HEARTBEAT_TIMEOUT_PARAM: &'static str =
            "/bond_disable_heartbeat_timeout";
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
        const DEFINITION: &'static str = "Header header\nfloat64 value";
    }
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
        const DEFINITION: &'static str = "string level";
    }
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
        const DEFINITION : & 'static str = "#Metric data type\n#For logging a set of points, e.g. for a pie chart\n\nstring name\nfloat64 time\nMetricPair[] data" ;
    }
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
        const DEFINITION : & 'static str = "#Data type for storing the key/value pairs from the Metric.data map\n\nstring key\nfloat64 value" ;
    }
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
        const DEFINITION : & 'static str = "string node_name\nint64 pid\n\n# Node is created, but is not yet initialized.\nuint8 STATUS_UNINITIALIZED=0\n# Node is initialized, but not connected.\nuint8 STATUS_DISCONNECTED=1\n# Node is initialized, connected, and running successfully.\nuint8 STATUS_RUNNING=2\n# Node is initialized and connected, but has a run error.\nuint8 STATUS_RUN_ERROR=3\n# Node was running, and is now shutting down.\nuint8 STATUS_SHUTTING_DOWN=4\n# Node is stopped.\nuint8 STATUS_SHUTDOWN=5\nuint8 status" ;
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
        const DEFINITION : & 'static str = "# AddTwoInts.srv\n# --- for funsies\n# From this ROS tutorial: http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv\nint64 a\nint64 b" ;
    }
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
        const DEFINITION: &'static str = "# Overflow? What overflow?\nint64 sum";
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
        const DEFINITION: &'static str =
            "Header header\nstring[] joint_names\nJointTrajectoryPoint[] points";
    }
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
        const DEFINITION : & 'static str = "# Each trajectory point specifies either positions[, velocities[, accelerations]]\n# or positions[, effort] for the trajectory to be executed.\n# All specified values are in the same order as the joint names in JointTrajectory.msg\n\nfloat64[] positions\nfloat64[] velocities\nfloat64[] accelerations\nfloat64[] effort\nduration time_from_start" ;
    }
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
        const DEFINITION : & 'static str = "# The header is used to specify the coordinate frame and the reference time for the trajectory durations\nHeader header\n\n# A representation of a multi-dof joint trajectory (each point is a transformation)\n# Each point along the trajectory will include an array of positions/velocities/accelerations\n# that has the same length as the array of joint names, and has the same order of joints as \n# the joint names array.\n\nstring[] joint_names\nMultiDOFJointTrajectoryPoint[] points" ;
    }
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
        const DEFINITION : & 'static str = "# Each multi-dof joint can specify a transform (up to 6 DOF)\ngeometry_msgs/Transform[] transforms\n\n# There can be a velocity specified for the origin of the joint \ngeometry_msgs/Twist[] velocities\n\n# There can be an acceleration specified for the origin of the joint \ngeometry_msgs/Twist[] accelerations\n\nduration time_from_start" ;
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
        const DEFINITION : & 'static str = "uint8 CIRCLE=0\nuint8 LINE_STRIP=1\nuint8 LINE_LIST=2\nuint8 POLYGON=3\nuint8 POINTS=4\n\nuint8 ADD=0\nuint8 REMOVE=1\n\nHeader header\nstring ns\t\t# namespace, used with id to form a unique id\nint32 id          \t# unique id within the namespace\nint32 type        \t# CIRCLE/LINE_STRIP/etc.\nint32 action      \t# ADD/REMOVE\ngeometry_msgs/Point position # 2D, in pixel-coords\nfloat32 scale\t \t# the diameter for a circle, etc.\nstd_msgs/ColorRGBA outline_color\nuint8 filled\t\t# whether to fill in the shape with color\nstd_msgs/ColorRGBA fill_color # color [0.0-1.0]\nduration lifetime       # How long the object should last before being automatically deleted.  0 means forever\n\n\ngeometry_msgs/Point[] points # used for LINE_STRIP/LINE_LIST/POINTS/etc., 2D in pixel coords\nstd_msgs/ColorRGBA[] outline_colors # a color for each line, point, etc." ;
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
        const DEFINITION : & 'static str = "# Time/frame info.\n# If header.time is set to 0, the marker will be retransformed into\n# its frame on each timestep. You will receive the pose feedback\n# in the same frame.\n# Otherwise, you might receive feedback in a different frame.\n# For rviz, this will be the current 'fixed frame' set by the user.\nHeader header\n\n# Initial pose. Also, defines the pivot point for rotations.\ngeometry_msgs/Pose pose\n\n# Identifying string. Must be globally unique in\n# the topic that this message is sent through.\nstring name\n\n# Short description (< 40 characters).\nstring description\n\n# Scale to be used for default controls (default=1).\nfloat32 scale\n\n# All menu and submenu entries associated with this marker.\nMenuEntry[] menu_entries\n\n# List of controls displayed for this marker.\nInteractiveMarkerControl[] controls" ;
    }
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
        const DEFINITION : & 'static str = "# Represents a control that is to be displayed together with an interactive marker\n\n# Identifying string for this control.\n# You need to assign a unique value to this to receive feedback from the GUI\n# on what actions the user performs on this control (e.g. a button click).\nstring name\n\n\n# Defines the local coordinate frame (relative to the pose of the parent\n# interactive marker) in which is being rotated and translated.\n# Default: Identity\ngeometry_msgs/Quaternion orientation\n\n\n# Orientation mode: controls how orientation changes.\n# INHERIT: Follow orientation of interactive marker\n# FIXED: Keep orientation fixed at initial state\n# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).\nuint8 INHERIT = 0 \nuint8 FIXED = 1\nuint8 VIEW_FACING = 2\n\nuint8 orientation_mode\n\n# Interaction mode for this control\n# \n# NONE: This control is only meant for visualization; no context menu.\n# MENU: Like NONE, but right-click menu is active.\n# BUTTON: Element can be left-clicked.\n# MOVE_AXIS: Translate along local x-axis.\n# MOVE_PLANE: Translate in local y-z plane.\n# ROTATE_AXIS: Rotate around local x-axis.\n# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.\nuint8 NONE = 0 \nuint8 MENU = 1\nuint8 BUTTON = 2\nuint8 MOVE_AXIS = 3 \nuint8 MOVE_PLANE = 4\nuint8 ROTATE_AXIS = 5\nuint8 MOVE_ROTATE = 6\n# \"3D\" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.\n# MOVE_3D: Translate freely in 3D space.\n# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.\n# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.\nuint8 MOVE_3D = 7\nuint8 ROTATE_3D = 8\nuint8 MOVE_ROTATE_3D = 9\n\nuint8 interaction_mode\n\n\n# If true, the contained markers will also be visible\n# when the gui is not in interactive mode.\nbool always_visible\n\n\n# Markers to be displayed as custom visual representation.\n# Leave this empty to use the default control handles.\n#\n# Note: \n# - The markers can be defined in an arbitrary coordinate frame,\n#   but will be transformed into the local frame of the interactive marker.\n# - If the header of a marker is empty, its pose will be interpreted as \n#   relative to the pose of the parent interactive marker.\nMarker[] markers\n\n\n# In VIEW_FACING mode, set this to true if you don't want the markers\n# to be aligned with the camera view point. The markers will show up\n# as in INHERIT mode.\nbool independent_marker_orientation\n\n\n# Short description (< 40 characters) of what this control does,\n# e.g. \"Move the robot\". \n# Default: A generic description based on the interaction mode\nstring description" ;
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
        const DEFINITION : & 'static str = "# Time/frame info.\nHeader header\n\n# Identifying string. Must be unique in the topic namespace.\nstring client_id\n\n# Feedback message sent back from the GUI, e.g.\n# when the status of an interactive marker was modified by the user.\n\n# Specifies which interactive marker and control this message refers to\nstring marker_name\nstring control_name\n\n# Type of the event\n# KEEP_ALIVE: sent while dragging to keep up control of the marker\n# MENU_SELECT: a menu entry has been selected\n# BUTTON_CLICK: a button control has been clicked\n# POSE_UPDATE: the pose has been changed using one of the controls\nuint8 KEEP_ALIVE = 0\nuint8 POSE_UPDATE = 1\nuint8 MENU_SELECT = 2\nuint8 BUTTON_CLICK = 3\n\nuint8 MOUSE_DOWN = 4\nuint8 MOUSE_UP = 5\n\nuint8 event_type\n\n# Current pose of the marker\n# Note: Has to be valid for all feedback types.\ngeometry_msgs/Pose pose\n\n# Contains the ID of the selected menu entry\n# Only valid for MENU_SELECT events.\nuint32 menu_entry_id\n\n# If event_type is BUTTON_CLICK, MOUSE_DOWN, or MOUSE_UP, mouse_point\n# may contain the 3 dimensional position of the event on the\n# control.  If it does, mouse_point_valid will be true.  mouse_point\n# will be relative to the frame listed in the header.\ngeometry_msgs/Point mouse_point\nbool mouse_point_valid" ;
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
        const DEFINITION : & 'static str = "# Identifying string. Must be unique in the topic namespace\n# that this server works on.\nstring server_id\n\n# Sequence number.\n# The client will use this to detect if it has missed a subsequent\n# update.  Every update message will have the same sequence number as\n# an init message.  Clients will likely want to unsubscribe from the\n# init topic after a successful initialization to avoid receiving\n# duplicate data.\nuint64 seq_num\n\n# All markers.\nInteractiveMarker[] markers" ;
    }
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
        const DEFINITION : & 'static str = "# Time/frame info.\nHeader header\n\n# Initial pose. Also, defines the pivot point for rotations.\ngeometry_msgs/Pose pose\n\n# Identifying string. Must be globally unique in\n# the topic that this message is sent through.\nstring name" ;
    }
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
        const DEFINITION : & 'static str = "# Identifying string. Must be unique in the topic namespace\n# that this server works on.\nstring server_id\n\n# Sequence number.\n# The client will use this to detect if it has missed an update.\nuint64 seq_num\n\n# Type holds the purpose of this message.  It must be one of UPDATE or KEEP_ALIVE.\n# UPDATE: Incremental update to previous state. \n#         The sequence number must be 1 higher than for\n#         the previous update.\n# KEEP_ALIVE: Indicates the that the server is still living.\n#             The sequence number does not increase.\n#             No payload data should be filled out (markers, poses, or erases).\nuint8 KEEP_ALIVE = 0\nuint8 UPDATE = 1\n\nuint8 type\n\n#Note: No guarantees on the order of processing.\n#      Contents must be kept consistent by sender.\n\n#Markers to be added or updated\nInteractiveMarker[] markers\n\n#Poses of markers that should be moved\nInteractiveMarkerPose[] poses\n\n#Names of markers to be erased\nstring[] erases" ;
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
        const DEFINITION : & 'static str = "# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz\n\nuint8 ARROW=0\nuint8 CUBE=1\nuint8 SPHERE=2\nuint8 CYLINDER=3\nuint8 LINE_STRIP=4\nuint8 LINE_LIST=5\nuint8 CUBE_LIST=6\nuint8 SPHERE_LIST=7\nuint8 POINTS=8\nuint8 TEXT_VIEW_FACING=9\nuint8 MESH_RESOURCE=10\nuint8 TRIANGLE_LIST=11\n\nuint8 ADD=0\nuint8 MODIFY=0\nuint8 DELETE=2\nuint8 DELETEALL=3\n\nHeader header                        # header for time/frame information\nstring ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object\nint32 id \t\t                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later\nint32 type \t\t                       # Type of object\nint32 action \t                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects\ngeometry_msgs/Pose pose                 # Pose of the object\ngeometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)\nstd_msgs/ColorRGBA color             # Color [0.0-1.0]\nduration lifetime                    # How long the object should last before being automatically deleted.  0 means forever\nbool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep\n\n#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)\ngeometry_msgs/Point[] points\n#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)\n#number of colors must either be 0 or equal to the number of points\n#NOTE: alpha is not yet used\nstd_msgs/ColorRGBA[] colors\n\n# NOTE: only used for text markers\nstring text\n\n# NOTE: only used for MESH_RESOURCE markers\nstring mesh_resource\nbool mesh_use_embedded_materials" ;
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
        const DEFINITION: &'static str = "Marker[] markers";
    }
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
        const DEFINITION : & 'static str = "# MenuEntry message.\n\n# Each InteractiveMarker message has an array of MenuEntry messages.\n# A collection of MenuEntries together describe a\n# menu/submenu/subsubmenu/etc tree, though they are stored in a flat\n# array.  The tree structure is represented by giving each menu entry\n# an ID number and a \"parent_id\" field.  Top-level entries are the\n# ones with parent_id = 0.  Menu entries are ordered within their\n# level the same way they are ordered in the containing array.  Parent\n# entries must appear before their children.\n\n# Example:\n# - id = 3\n#   parent_id = 0\n#   title = \"fun\"\n# - id = 2\n#   parent_id = 0\n#   title = \"robot\"\n# - id = 4\n#   parent_id = 2\n#   title = \"pr2\"\n# - id = 5\n#   parent_id = 2\n#   title = \"turtle\"\n#\n# Gives a menu tree like this:\n#  - fun\n#  - robot\n#    - pr2\n#    - turtle\n\n# ID is a number for each menu entry.  Must be unique within the\n# control, and should never be 0.\nuint32 id\n\n# ID of the parent of this menu entry, if it is a submenu.  If this\n# menu entry is a top-level entry, set parent_id to 0.\nuint32 parent_id\n\n# menu / entry title\nstring title\n\n# Arguments to command indicated by command_type (below)\nstring command\n\n# Command_type stores the type of response desired when this menu\n# entry is clicked.\n# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.\n# ROSRUN: execute \"rosrun\" with arguments given in the command field (above).\n# ROSLAUNCH: execute \"roslaunch\" with arguments given in the command field (above).\nuint8 FEEDBACK=0\nuint8 ROSRUN=1\nuint8 ROSLAUNCH=2\nuint8 command_type" ;
    }
    impl MenuEntry {
        pub const r#FEEDBACK: u8 = 0u8;
        pub const r#ROSRUN: u8 = 1u8;
        pub const r#ROSLAUNCH: u8 = 2u8;
    }
}
