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
        const DEFINITION : & 'static str = "# The stamp should store the time at which this goal was requested.\n# It is used by an action server when it tries to preempt all\n# goals that were requested before a certain time\nbuiltin_interfaces/Time stamp\n\n# The id provides a way to associate feedback and\n# result message with specific goal requests. The id\n# specified must be unique.\nstring id" ;
    }
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
        const DEFINITION : & 'static str = "GoalID goal_id\nuint8 status\nuint8 PENDING         = 0   # The goal has yet to be processed by the action server.\nuint8 ACTIVE          = 1   # The goal is currently being processed by the action server.\nuint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n                            #   and has since completed its execution (Terminal State).\nuint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server\n                            #   (Terminal State).\nuint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n                            #    to some failure (Terminal State).\nuint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n                            #    because the goal was unattainable or invalid (Terminal State).\nuint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n                            #    and has not yet completed execution.\nuint8 RECALLING       = 7   # The goal received a cancel request before it started executing, but\n                            #    the action server has not yet confirmed that the goal is canceled.\nuint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n                            #    and was successfully cancelled (Terminal State).\nuint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not\n                            #    be sent over the wire by an action server.\n\n# Allow for the user to associate a string with GoalStatus for debugging.\nstring text" ;
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
        const DEFINITION : & 'static str = "# Stores the statuses for goals that are currently being tracked\n# by an action server\nstd_msgs/Header header\nGoalStatus[] status_list" ;
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
        const DEFINITION : & 'static str = "# This message is used to send diagnostic information about the state of the robot.\nstd_msgs/Header header # for timestamp\nDiagnosticStatus[] status # an array of components being reported on" ;
    }
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
        const DEFINITION : & 'static str = "# This message holds the status of an individual component of the robot.\n\n# Possible levels of operations.\nbyte OK=0\nbyte WARN=1\nbyte ERROR=2\nbyte STALE=3\n\n# Level of operation enumerated above.\nbyte level\n# A description of the test/component reporting.\nstring name\n# A description of the status.\nstring message\n# A hardware unique string.\nstring hardware_id\n# An array of values associated with the status.\nKeyValue[] values" ;
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
        const DEFINITION : & 'static str = "# What to label this value when viewing.\nstring key\n# A value to track over time.\nstring value" ;
    }
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
        const MD5SUM: &'static str = "19a31cf6d39a90e769a5539f9a293621";
        const DEFINITION : & 'static str = "# An accel with reference coordinate frame and timestamp\nstd_msgs/Header header\nAccel accel" ;
    }
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
        const MD5SUM: &'static str = "36b6f1177d3c3f476d4c306279c6f18a";
        const DEFINITION : & 'static str = "# This represents an estimated accel with reference coordinate frame and timestamp.\nstd_msgs/Header header\nAccelWithCovariance accel" ;
    }
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
        const MD5SUM: &'static str = "d4fb75ac056292d6c4bbec5e51d25080";
        const DEFINITION : & 'static str = "# An Inertia with a time stamp and reference frame.\n\nstd_msgs/Header header\nInertia inertia" ;
    }
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
        const DEFINITION : & 'static str = "# This contains the position of a point in free space(with 32 bits of precision).\n# It is recommended to use Point wherever possible instead of Point32.\n#\n# This recommendation is to promote interoperability.\n#\n# This message is designed to take up less space when sending\n# lots of points at once, as in the case of a PointCloud.\n\nfloat32 x\nfloat32 y\nfloat32 z" ;
    }
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
        const DEFINITION : & 'static str = "# This represents a Point with reference coordinate frame and timestamp\n\nstd_msgs/Header header\nPoint point" ;
    }
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
        const DEFINITION : & 'static str = "# A specification of a polygon where the first and last points are assumed to be connected\n\nPoint32[] points" ;
    }
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
        const DEFINITION : & 'static str = "# This represents a Polygon with reference coordinate frame and timestamp\n\nstd_msgs/Header header\nPolygon polygon" ;
    }
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
        const DEFINITION : & 'static str = "# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation" ;
    }
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
        const DEFINITION : & 'static str = "# Deprecated as of Foxy and will potentially be removed in any following release.\n# Please use the full 3D pose.\n\n# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.\n\n# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.# This expresses a position and orientation on a 2D manifold.\n\nfloat64 x\nfloat64 y\nfloat64 theta" ;
    }
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
        const DEFINITION : & 'static str = "# An array of poses with a header for global reference.\n\nstd_msgs/Header header\n\nPose[] poses" ;
    }
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
        const DEFINITION : & 'static str = "# A Pose with reference coordinate frame and timestamp\n\nstd_msgs/Header header\nPose pose" ;
    }
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
        const MD5SUM: &'static str = "2178452bf195c1abe1e99b07b4e6c8f0";
        const DEFINITION : & 'static str = "# This expresses an estimated pose with a reference coordinate frame and timestamp\n\nstd_msgs/Header header\nPoseWithCovariance pose" ;
    }
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
        const DEFINITION : & 'static str = "# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1" ;
    }
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
        const DEFINITION : & 'static str = "# This represents an orientation with reference coordinate frame and timestamp.\n\nstd_msgs/Header header\nQuaternion quaternion" ;
    }
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
        const MD5SUM: &'static str = "09bf247c06cf7c69e8c55300b05a7a04";
        const DEFINITION : & 'static str = "# This expresses a transform from coordinate frame header.frame_id\n# to the coordinate frame child_frame_id at the time of header.stamp\n#\n# This message is mostly used by the\n# <a href=\"https://index.ros.org/p/tf2/\">tf2</a> package.\n# See its documentation for more information.\n#\n# The child_frame_id is necessary in addition to the frame_id\n# in the Header to communicate the full reference for the transform\n# in a self contained message.\n\n# The frame id in the header is used as the reference frame of this transform.\nstd_msgs/Header header\n\n# The frame id of the child frame to which this transform points.\nstring child_frame_id\n\n# Translation and rotation in 3-dimensions of child_frame_id from header.frame_id.\nTransform transform" ;
    }
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
        const DEFINITION : & 'static str = "# This expresses velocity in free space broken into its linear and angular parts.\n\nVector3  linear\nVector3  angular" ;
    }
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
        const DEFINITION : & 'static str = "# A twist with reference coordinate frame and timestamp\n\nstd_msgs/Header header\nTwist twist" ;
    }
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
        const MD5SUM: &'static str = "7019807c85ce8602fb83180366470670";
        const DEFINITION : & 'static str = "# This represents an estimated twist with reference coordinate frame and timestamp.\n\nstd_msgs/Header header\nTwistWithCovariance twist" ;
    }
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
        const DEFINITION : & 'static str = "# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z" ;
    }
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
        const DEFINITION : & 'static str = "# This represents a Vector3 with reference coordinate frame and timestamp\n\n# Note that this follows vector semantics with it always anchored at the origin,\n# so the rotational elements of a transform are the only parts applied when transforming.\n\nstd_msgs/Header header\nVector3 vector" ;
    }
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
        const DEFINITION : & 'static str = "# This represents force in free space, separated into its linear and angular parts.\n\nVector3  force\nVector3  torque" ;
    }
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
        const DEFINITION : & 'static str = "# A wrench with reference coordinate frame and timestamp\n\nstd_msgs/Header header\nWrench wrench" ;
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
        const DEFINITION : & 'static str = "# An array of cells in a 2D grid\n\nstd_msgs/Header header\n\n# Width of each cell\nfloat32 cell_width\n\n# Height of each cell\nfloat32 cell_height\n\n# Each cell is represented by the Point at the center of the cell\ngeometry_msgs/Point[] cells" ;
    }
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
        const DEFINITION : & 'static str = "# This hold basic information about the characteristics of the OccupancyGrid\n\n# The time at which the map was loaded\nbuiltin_interfaces/Time map_load_time\n\n# The map resolution [m/cell]\nfloat32 resolution\n\n# Map width [cells]\nuint32 width\n\n# Map height [cells]\nuint32 height\n\n# The origin of the map [m, m, rad].  This is the real-world pose of the\n# bottom left corner of cell (0,0) in the map.\ngeometry_msgs/Pose origin" ;
    }
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
        const DEFINITION : & 'static str = "# This represents a 2-D grid map\nstd_msgs/Header header\n\n# MetaData for the map\nMapMetaData info\n\n# The map data, in row-major order, starting with (0,0). \n# Cell (1, 0) will be listed second, representing the next cell in the x direction. \n# Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).\n# The values inside are application dependent, but frequently, \n# 0 represents unoccupied, 1 represents definitely occupied, and\n# -1 represents unknown. \nint8[] data" ;
    }
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
        const MD5SUM: &'static str = "81a0900daae2c6c0acc71c9f8df88947";
        const DEFINITION : & 'static str = "# This represents an estimate of a position and velocity in free space.\n# The pose in this message should be specified in the coordinate frame given by header.frame_id\n# The twist in this message should be specified in the coordinate frame given by the child_frame_id\n\n# Includes the frame id of the pose parent.\nstd_msgs/Header header\n\n# Frame id the pose points to. The twist is in this coordinate frame.\nstring child_frame_id\n\n# Estimated pose that is typically relative to a fixed world frame.\ngeometry_msgs/PoseWithCovariance pose\n\n# Estimated linear and angular velocity relative to child_frame_id.\ngeometry_msgs/TwistWithCovariance twist" ;
    }
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
        const DEFINITION : & 'static str = "# An array of poses that represents a Path for a robot to follow.\n\n# Indicates the frame_id of the path.\nstd_msgs/Header header\n\n# Array of poses to follow.\ngeometry_msgs/PoseStamped[] poses" ;
    }
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
        const MD5SUM: &'static str = "d6e8b0301af2dfe2244959ba20a4080a";
        const DEFINITION: &'static str =
            "# The current map hosted by this map service.\nOccupancyGrid map";
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
        const DEFINITION : & 'static str = "# Get a plan from the current position to the goal Pose\n\n# The start pose for the plan\ngeometry_msgs/PoseStamped start\n\n# The final pose of the goal position\ngeometry_msgs/PoseStamped goal\n\n# If the goal is obstructed, how many meters the planner can\n# relax the constraint in x and y before failing.\nfloat32 tolerance" ;
    }
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
        const DEFINITION: &'static str =
            "# Array of poses from start to goal if one was successfully found.\nPath plan";
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
        const MD5SUM: &'static str = "cdb849e3dfaed8b5fe66776d7a64b83e";
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
        const MD5SUM: &'static str = "98782a373ad73e1165352caf85923850";
        const DEFINITION : & 'static str = "# Set a new map together with an initial pose\n\n# Requested 2D map to be set.\nnav_msgs/OccupancyGrid map\n\n# Estimated initial pose when setting new map.\ngeometry_msgs/PoseWithCovarianceStamped initial_pose" ;
    }
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
        const DEFINITION: &'static str =
            "# True if the map was successfully set, false otherwise.\nbool success";
    }
    pub struct SetMap {}
    impl ::roslibrust_codegen::RosServiceType for SetMap {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/SetMap";
        const MD5SUM: &'static str = "6c3f8182fbcb3d4ee7aef02d1dcd1e16";
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
        const DEFINITION : & 'static str = "# Constants are chosen to match the enums in the linux kernel\n# defined in include/linux/power_supply.h as of version 3.7\n# The one difference is for style reasons the constants are\n# all uppercase not mixed case.\n\n# Power supply status constants\nuint8 POWER_SUPPLY_STATUS_UNKNOWN = 0\nuint8 POWER_SUPPLY_STATUS_CHARGING = 1\nuint8 POWER_SUPPLY_STATUS_DISCHARGING = 2\nuint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3\nuint8 POWER_SUPPLY_STATUS_FULL = 4\n\n# Power supply health constants\nuint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0\nuint8 POWER_SUPPLY_HEALTH_GOOD = 1\nuint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2\nuint8 POWER_SUPPLY_HEALTH_DEAD = 3\nuint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4\nuint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5\nuint8 POWER_SUPPLY_HEALTH_COLD = 6\nuint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7\nuint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8\n\n# Power supply technology (chemistry) constants\nuint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0\nuint8 POWER_SUPPLY_TECHNOLOGY_NIMH = 1\nuint8 POWER_SUPPLY_TECHNOLOGY_LION = 2\nuint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3\nuint8 POWER_SUPPLY_TECHNOLOGY_LIFE = 4\nuint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5\nuint8 POWER_SUPPLY_TECHNOLOGY_LIMN = 6\n\nstd_msgs/Header  header\nfloat32 voltage          # Voltage in Volts (Mandatory)\nfloat32 temperature      # Temperature in Degrees Celsius (If unmeasured NaN)\nfloat32 current          # Negative when discharging (A)  (If unmeasured NaN)\nfloat32 charge           # Current charge in Ah  (If unmeasured NaN)\nfloat32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)\nfloat32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)\nfloat32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)\nuint8   power_supply_status     # The charging status as reported. Values defined above\nuint8   power_supply_health     # The battery health metric. Values defined above\nuint8   power_supply_technology # The battery chemistry. Values defined above\nbool    present          # True if the battery is present\n\nfloat32[] cell_voltage   # An array of individual cell voltages for each cell in the pack\n                         # If individual voltages unknown but number of cells known set each to NaN\nfloat32[] cell_temperature # An array of individual cell temperatures for each cell in the pack\n                           # If individual temperatures unknown but number of cells known set each to NaN\nstring location          # The location into which the battery is inserted. (slot number or plug)\nstring serial_number     # The best approximation of the battery serial number" ;
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
        pub r#k: [f64; 9],
        pub r#r: [f64; 9],
        pub r#p: [f64; 12],
        pub r#binning_x: u32,
        pub r#binning_y: u32,
        pub r#roi: self::RegionOfInterest,
    }
    impl ::roslibrust_codegen::RosMessageType for CameraInfo {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/CameraInfo";
        const MD5SUM: &'static str = "47b55ddbbf2ec398f94cddf328bbc2ac";
        const DEFINITION : & 'static str = "# This message defines meta information for a camera. It should be in a\n# camera namespace on topic \"camera_info\" and accompanied by up to five\n# image topics named:\n#\n#   image_raw - raw data from the camera driver, possibly Bayer encoded\n#   image            - monochrome, distorted\n#   image_color      - color, distorted\n#   image_rect       - monochrome, rectified\n#   image_rect_color - color, rectified\n#\n# The image_pipeline contains packages (image_proc, stereo_image_proc)\n# for producing the four processed image topics from image_raw and\n# camera_info. The meaning of the camera parameters are described in\n# detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.\n#\n# The image_geometry package provides a user-friendly interface to\n# common operations using this meta information. If you want to, e.g.,\n# project a 3d point into image coordinates, we strongly recommend\n# using image_geometry.\n#\n# If the camera is uncalibrated, the matrices D, K, R, P should be left\n# zeroed out. In particular, clients may assume that K[0] == 0.0\n# indicates an uncalibrated camera.\n\n#######################################################################\n#                     Image acquisition info                          #\n#######################################################################\n\n# Time of image acquisition, camera coordinate frame ID\nstd_msgs/Header header # Header timestamp should be acquisition time of image\n                             # Header frame_id should be optical frame of camera\n                             # origin of frame should be optical center of camera\n                             # +x should point to the right in the image\n                             # +y should point down in the image\n                             # +z should point into the plane of the image\n\n\n#######################################################################\n#                      Calibration Parameters                         #\n#######################################################################\n# These are fixed during camera calibration. Their values will be the #\n# same in all messages until the camera is recalibrated. Note that    #\n# self-calibrating systems may \"recalibrate\" frequently.              #\n#                                                                     #\n# The internal parameters can be used to warp a raw (distorted) image #\n# to:                                                                 #\n#   1. An undistorted image (requires D and K)                        #\n#   2. A rectified image (requires D, K, R)                           #\n# The projection matrix P projects 3D points into the rectified image.#\n#######################################################################\n\n# The image dimensions with which the camera was calibrated.\n# Normally this will be the full camera resolution in pixels.\nuint32 height\nuint32 width\n\n# The distortion model used. Supported models are listed in\n# sensor_msgs/distortion_models.hpp. For most cameras, \"plumb_bob\" - a\n# simple model of radial and tangential distortion - is sufficent.\nstring distortion_model\n\n# The distortion parameters, size depending on the distortion model.\n# For \"plumb_bob\", the 5 parameters are: (k1, k2, t1, t2, k3).\nfloat64[] d\n\n# Intrinsic camera matrix for the raw (distorted) images.\n#     [fx  0 cx]\n# K = [ 0 fy cy]\n#     [ 0  0  1]\n# Projects 3D points in the camera coordinate frame to 2D pixel\n# coordinates using the focal lengths (fx, fy) and principal point\n# (cx, cy).\nfloat64[9]  k # 3x3 row-major matrix\n\n# Rectification matrix (stereo cameras only)\n# A rotation matrix aligning the camera coordinate system to the ideal\n# stereo image plane so that epipolar lines in both stereo images are\n# parallel.\nfloat64[9]  r # 3x3 row-major matrix\n\n# Projection/camera matrix\n#     [fx'  0  cx' Tx]\n# P = [ 0  fy' cy' Ty]\n#     [ 0   0   1   0]\n# By convention, this matrix specifies the intrinsic (camera) matrix\n#  of the processed (rectified) image. That is, the left 3x3 portion\n#  is the normal camera intrinsic matrix for the rectified image.\n# It projects 3D points in the camera coordinate frame to 2D pixel\n#  coordinates using the focal lengths (fx', fy') and principal point\n#  (cx', cy') - these may differ from the values in K.\n# For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will\n#  also have R = the identity and P[1:3,1:3] = K.\n# For a stereo pair, the fourth column [Tx Ty 0]' is related to the\n#  position of the optical center of the second camera in the first\n#  camera's frame. We assume Tz = 0 so both cameras are in the same\n#  stereo image plane. The first camera always has Tx = Ty = 0. For\n#  the right (second) camera of a horizontal stereo pair, Ty = 0 and\n#  Tx = -fx' * B, where B is the baseline between the cameras.\n# Given a 3D point [X Y Z]', the projection (x, y) of the point onto\n#  the rectified image is given by:\n#  [u v w]' = P * [X Y Z 1]'\n#         x = u / w\n#         y = v / w\n#  This holds for both images of a stereo pair.\nfloat64[12] p # 3x4 row-major matrix\n\n\n#######################################################################\n#                      Operational Parameters                         #\n#######################################################################\n# These define the image region actually captured by the camera       #\n# driver. Although they affect the geometry of the output image, they #\n# may be changed freely without recalibrating the camera.             #\n#######################################################################\n\n# Binning refers here to any camera setting which combines rectangular\n#  neighborhoods of pixels into larger \"super-pixels.\" It reduces the\n#  resolution of the output image to\n#  (width / binning_x) x (height / binning_y).\n# The default values binning_x = binning_y = 0 is considered the same\n#  as binning_x = binning_y = 1 (no subsampling).\nuint32 binning_x\nuint32 binning_y\n\n# Region of interest (subwindow of full camera resolution), given in\n#  full resolution (unbinned) image coordinates. A particular ROI\n#  always denotes the same window of pixels on the camera sensor,\n#  regardless of binning settings.\n# The default setting of roi (all values 0) is considered the same as\n#  full resolution (roi.width = width, roi.height = height).\nRegionOfInterest roi" ;
    }
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
        const DEFINITION : & 'static str = "# This message is used by the PointCloud message to hold optional data\n# associated with each point in the cloud. The length of the values\n# array should be the same as the length of the points array in the\n# PointCloud, and each value should be associated with the corresponding\n# point.\n#\n# Channel names in existing practice include:\n#   \"u\", \"v\" - row and column (respectively) in the left stereo image.\n#              This is opposite to usual conventions but remains for\n#              historical reasons. The newer PointCloud2 message has no\n#              such problem.\n#   \"rgb\" - For point clouds produced by color stereo cameras. uint8\n#           (R,G,B) values packed into the least significant 24 bits,\n#           in order.\n#   \"intensity\" - laser or pixel intensity.\n#   \"distance\"\n\n# The channel name should give semantics of the channel (e.g.\n# \"intensity\" instead of \"value\").\nstring name\n\n# The values array should be 1-1 with the elements of the associated\n# PointCloud.\nfloat32[] values" ;
    }
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
        const DEFINITION : & 'static str = "# This message contains a compressed image.\n\nstd_msgs/Header header # Header timestamp should be acquisition time of image\n                             # Header frame_id should be optical frame of camera\n                             # origin of frame should be optical center of cameara\n                             # +x should point to the right in the image\n                             # +y should point down in the image\n                             # +z should point into to plane of the image\n\nstring format                # Specifies the format of the data\n                             #   Acceptable values:\n                             #     jpeg, png, tiff\n\nuint8[] data                 # Compressed image buffer" ;
    }
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
        const DEFINITION : & 'static str = "# Single pressure reading.  This message is appropriate for measuring the\n# pressure inside of a fluid (air, water, etc).  This also includes\n# atmospheric or barometric pressure.\n#\n# This message is not appropriate for force/pressure contact sensors.\n\nstd_msgs/Header header # timestamp of the measurement\n                             # frame_id is the location of the pressure sensor\n\nfloat64 fluid_pressure       # Absolute pressure reading in Pascals.\n\nfloat64 variance             # 0 is interpreted as variance unknown" ;
    }
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
        const DEFINITION : & 'static str = "# Single photometric illuminance measurement.  Light should be assumed to be\n# measured along the sensor's x-axis (the area of detection is the y-z plane).\n# The illuminance should have a 0 or positive value and be received with\n# the sensor's +X axis pointing toward the light source.\n#\n# Photometric illuminance is the measure of the human eye's sensitivity of the\n# intensity of light encountering or passing through a surface.\n#\n# All other Photometric and Radiometric measurements should not use this message.\n# This message cannot represent:\n#  - Luminous intensity (candela/light source output)\n#  - Luminance (nits/light output per area)\n#  - Irradiance (watt/area), etc.\n\nstd_msgs/Header header # timestamp is the time the illuminance was measured\n                             # frame_id is the location and direction of the reading\n\nfloat64 illuminance          # Measurement of the Photometric Illuminance in Lux.\n\nfloat64 variance             # 0 is interpreted as variance unknown" ;
    }
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
        const DEFINITION : & 'static str = "# This message contains an uncompressed image\n# (0, 0) is at top-left corner of image\n\nstd_msgs/Header header # Header timestamp should be acquisition time of image\n                             # Header frame_id should be optical frame of camera\n                             # origin of frame should be optical center of cameara\n                             # +x should point to the right in the image\n                             # +y should point down in the image\n                             # +z should point into to plane of the image\n                             # If the frame_id here and the frame_id of the CameraInfo\n                             # message associated with the image conflict\n                             # the behavior is undefined\n\nuint32 height                # image height, that is, number of rows\nuint32 width                 # image width, that is, number of columns\n\n# The legal values for encoding are in file include/sensor_msgs/image_encodings.hpp\n# If you want to standardize a new string format, join\n# ros-users@lists.ros.org and send an email proposing a new encoding.\n\nstring encoding       # Encoding of pixels -- channel meaning, ordering, size\n                      # taken from the list of strings in include/sensor_msgs/image_encodings.hpp\n\nuint8 is_bigendian    # is this data bigendian?\nuint32 step           # Full row length in bytes\nuint8[] data          # actual matrix data, size is (step * rows)" ;
    }
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
        const MD5SUM: &'static str = "058a92f712764b4ade1563e82041c569";
        const DEFINITION : & 'static str = "# This is a message to hold data from an IMU (Inertial Measurement Unit)\n#\n# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec\n#\n# If the covariance of the measurement is known, it should be filled in (if all you know is the\n# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)\n# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the\n# data a covariance will have to be assumed or gotten from some other source\n#\n# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an\n# orientation estimate), please set element 0 of the associated covariance matrix to -1\n# If you are interpreting this message, please check for a value of -1 in the first element of each\n# covariance matrix, and disregard the associated estimate.\n\nstd_msgs/Header header\n\ngeometry_msgs/Quaternion orientation\nfloat64[9] orientation_covariance # Row major about x, y, z axes\n\ngeometry_msgs/Vector3 angular_velocity\nfloat64[9] angular_velocity_covariance # Row major about x, y, z axes\n\ngeometry_msgs/Vector3 linear_acceleration\nfloat64[9] linear_acceleration_covariance # Row major x, y z" ;
    }
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
        const DEFINITION : & 'static str = "# This is a message that holds data to describe the state of a set of torque controlled joints.\n#\n# The state of each joint (revolute or prismatic) is defined by:\n#  * the position of the joint (rad or m),\n#  * the velocity of the joint (rad/s or m/s) and\n#  * the effort that is applied in the joint (Nm or N).\n#\n# Each joint is uniquely identified by its name\n# The header specifies the time at which the joint states were recorded. All the joint states\n# in one message have to be recorded at the same time.\n#\n# This message consists of a multiple arrays, one for each part of the joint state.\n# The goal is to make each of the fields optional. When e.g. your joints have no\n# effort associated with them, you can leave the effort array empty.\n#\n# All arrays in this message should have the same size, or be empty.\n# This is the only way to uniquely associate the joint name with the correct\n# states.\n\nstd_msgs/Header header\n\nstring[] name\nfloat64[] position\nfloat64[] velocity\nfloat64[] effort" ;
    }
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
        const DEFINITION : & 'static str = "# Reports the state of a joystick's axes and buttons.\n\n# The timestamp is the time at which data is received from the joystick.\nstd_msgs/Header header\n\n# The axes measurements from a joystick.\nfloat32[] axes\n\n# The buttons measurements from a joystick.\nint32[] buttons" ;
    }
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
            "# This message publishes values for multiple feedback at once.\nJoyFeedback[] array";
    }
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
        const MD5SUM: &'static str = "f86984b4383bf67523c75820e114e988";
        const DEFINITION : & 'static str = "# Single scan from a planar laser range-finder\n#\n# If you have another ranging device with different behavior (e.g. a sonar\n# array), please find or create a different message, since applications\n# will make fairly laser-specific assumptions about this data\n\nstd_msgs/Header header # timestamp in the header is the acquisition time of\n                             # the first ray in the scan.\n                             #\n                             # in frame frame_id, angles are measured around\n                             # the positive Z axis (counterclockwise, if Z is up)\n                             # with zero angle being forward along the x axis\n\nfloat32 angle_min            # start angle of the scan [rad]\nfloat32 angle_max            # end angle of the scan [rad]\nfloat32 angle_increment      # angular distance between measurements [rad]\n\nfloat32 time_increment       # time between measurements [seconds] - if your scanner\n                             # is moving, this will be used in interpolating position\n                             # of 3d points\nfloat32 scan_time            # time between scans [seconds]\n\nfloat32 range_min            # minimum range value [m]\nfloat32 range_max            # maximum range value [m]\n\nfloat32[] ranges             # range data [m]\n                             # (Note: values < range_min or > range_max should be discarded)\nfloat32[] intensities        # intensity data [device-specific units].  If your\n                             # device does not provide intensities, please leave\n                             # the array empty." ;
    }
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
        const MD5SUM: &'static str = "c8761d20eb9dc59addd882f1d4de2266";
        const DEFINITION : & 'static str = "# Measurement of the Magnetic Field vector at a specific location.\n#\n# If the covariance of the measurement is known, it should be filled in.\n# If all you know is the variance of each measurement, e.g. from the datasheet,\n# just put those along the diagonal.\n# A covariance matrix of all zeros will be interpreted as \"covariance unknown\",\n# and to use the data a covariance will have to be assumed or gotten from some\n# other source.\n\nstd_msgs/Header header               # timestamp is the time the\n                                           # field was measured\n                                           # frame_id is the location and orientation\n                                           # of the field measurement\n\ngeometry_msgs/Vector3 magnetic_field # x, y, and z components of the\n                                           # field vector in Tesla\n                                           # If your sensor does not output 3 axes,\n                                           # put NaNs in the components not reported.\n\nfloat64[9] magnetic_field_covariance       # Row major about x, y, z axes\n                                           # 0 is interpreted as variance unknown" ;
    }
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
        const DEFINITION : & 'static str = "# Representation of state for joints with multiple degrees of freedom,\n# following the structure of JointState which can only represent a single degree of freedom.\n#\n# It is assumed that a joint in a system corresponds to a transform that gets applied\n# along the kinematic chain. For example, a planar joint (as in URDF) is 3DOF (x, y, yaw)\n# and those 3DOF can be expressed as a transformation matrix, and that transformation\n# matrix can be converted back to (x, y, yaw)\n#\n# Each joint is uniquely identified by its name\n# The header specifies the time at which the joint states were recorded. All the joint states\n# in one message have to be recorded at the same time.\n#\n# This message consists of a multiple arrays, one for each part of the joint state.\n# The goal is to make each of the fields optional. When e.g. your joints have no\n# wrench associated with them, you can leave the wrench array empty.\n#\n# All arrays in this message should have the same size, or be empty.\n# This is the only way to uniquely associate the joint name with the correct\n# states.\n\nstd_msgs/Header header\n\nstring[] joint_names\ngeometry_msgs/Transform[] transforms\ngeometry_msgs/Twist[] twist\ngeometry_msgs/Wrench[] wrench" ;
    }
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
        const DEFINITION : & 'static str = "# Single scan from a multi-echo planar laser range-finder\n#\n# If you have another ranging device with different behavior (e.g. a sonar\n# array), please find or create a different message, since applications\n# will make fairly laser-specific assumptions about this data\n\nstd_msgs/Header header # timestamp in the header is the acquisition time of\n                             # the first ray in the scan.\n                             #\n                             # in frame frame_id, angles are measured around\n                             # the positive Z axis (counterclockwise, if Z is up)\n                             # with zero angle being forward along the x axis\n\nfloat32 angle_min            # start angle of the scan [rad]\nfloat32 angle_max            # end angle of the scan [rad]\nfloat32 angle_increment      # angular distance between measurements [rad]\n\nfloat32 time_increment       # time between measurements [seconds] - if your scanner\n                             # is moving, this will be used in interpolating position\n                             # of 3d points\nfloat32 scan_time            # time between scans [seconds]\n\nfloat32 range_min            # minimum range value [m]\nfloat32 range_max            # maximum range value [m]\n\nLaserEcho[] ranges           # range data [m]\n                             # (Note: NaNs, values < range_min or > range_max should be discarded)\n                             # +Inf measurements are out of range\n                             # -Inf measurements are too close to determine exact distance.\nLaserEcho[] intensities      # intensity data [device-specific units].  If your\n                             # device does not provide intensities, please leave\n                             # the array empty." ;
    }
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
        const MD5SUM: &'static str = "faa1756146a6a934d7e4ef0e3855c531";
        const DEFINITION : & 'static str = "# Navigation Satellite fix for any Global Navigation Satellite System\n#\n# Specified using the WGS 84 reference ellipsoid\n\n# header.stamp specifies the ROS time for this measurement (the\n#        corresponding satellite time may be reported using the\n#        sensor_msgs/TimeReference message).\n#\n# header.frame_id is the frame of reference reported by the satellite\n#        receiver, usually the location of the antenna.  This is a\n#        Euclidean frame relative to the vehicle, not a reference\n#        ellipsoid.\nstd_msgs/Header header\n\n# Satellite fix status information.\nNavSatStatus status\n\n# Latitude [degrees]. Positive is north of equator; negative is south.\nfloat64 latitude\n\n# Longitude [degrees]. Positive is east of prime meridian; negative is west.\nfloat64 longitude\n\n# Altitude [m]. Positive is above the WGS 84 ellipsoid\n# (quiet NaN if no altitude is available).\nfloat64 altitude\n\n# Position covariance [m^2] defined relative to a tangential plane\n# through the reported position. The components are East, North, and\n# Up (ENU), in row-major order.\n#\n# Beware: this coordinate system exhibits singularities at the poles.\nfloat64[9] position_covariance\n\n# If the covariance of the fix is known, fill it in completely. If the\n# GPS receiver provides the variance of each measurement, put them\n# along the diagonal. If only Dilution of Precision is available,\n# estimate an approximate covariance from that.\n\nuint8 COVARIANCE_TYPE_UNKNOWN = 0\nuint8 COVARIANCE_TYPE_APPROXIMATED = 1\nuint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2\nuint8 COVARIANCE_TYPE_KNOWN = 3\n\nuint8 position_covariance_type" ;
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
        const DEFINITION : & 'static str = "# Navigation Satellite fix status for any Global Navigation Satellite System.\n#\n# Whether to output an augmented fix is determined by both the fix\n# type and the last time differential corrections were received.  A\n# fix is valid when status >= STATUS_FIX.\n\nint8 STATUS_NO_FIX =  -1        # unable to fix position\nint8 STATUS_FIX =      0        # unaugmented fix\nint8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation\nint8 STATUS_GBAS_FIX = 2        # with ground-based augmentation\n\nint8 status\n\n# Bits defining which Global Navigation Satellite System signals were\n# used by the receiver.\n\nuint16 SERVICE_GPS =     1\nuint16 SERVICE_GLONASS = 2\nuint16 SERVICE_COMPASS = 4      # includes BeiDou.\nuint16 SERVICE_GALILEO = 8\n\nuint16 service" ;
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
        const DEFINITION : & 'static str = "## THIS MESSAGE IS DEPRECATED AS OF FOXY\n## Please use sensor_msgs/PointCloud2\n\n# This message holds a collection of 3d points, plus optional additional\n# information about each point.\n\n# Time of sensor data acquisition, coordinate frame ID.\nstd_msgs/Header header\n\n# Array of 3d points. Each Point32 should be interpreted as a 3d point\n# in the frame given in the header.\ngeometry_msgs/Point32[] points\n\n# Each channel should have the same number of elements as points array,\n# and the data in each channel should correspond 1:1 with each point.\n# Channel names in common practice are listed in ChannelFloat32.msg.\nChannelFloat32[] channels" ;
    }
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
        const DEFINITION : & 'static str = "# This message holds a collection of N-dimensional points, which may\n# contain additional information such as normals, intensity, etc. The\n# point data is stored as a binary blob, its layout described by the\n# contents of the \"fields\" array.\n#\n# The point cloud data may be organized 2d (image-like) or 1d (unordered).\n# Point clouds organized as 2d images may be produced by camera depth sensors\n# such as stereo or time-of-flight.\n\n# Time of sensor data acquisition, and the coordinate frame ID (for 3d points).\nstd_msgs/Header header\n\n# 2D structure of the point cloud. If the cloud is unordered, height is\n# 1 and width is the length of the point cloud.\nuint32 height\nuint32 width\n\n# Describes the channels and their layout in the binary data blob.\nPointField[] fields\n\nbool    is_bigendian # Is this data bigendian?\nuint32  point_step   # Length of a point in bytes\nuint32  row_step     # Length of a row in bytes\nuint8[] data         # Actual point data, size is (row_step*height)\n\nbool is_dense        # True if there are no invalid points" ;
    }
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
        const DEFINITION : & 'static str = "# This message holds the description of one point entry in the\n# PointCloud2 message format.\nuint8 INT8    = 1\nuint8 UINT8   = 2\nuint8 INT16   = 3\nuint8 UINT16  = 4\nuint8 INT32   = 5\nuint8 UINT32  = 6\nuint8 FLOAT32 = 7\nuint8 FLOAT64 = 8\n\n# Common PointField names are x, y, z, intensity, rgb, rgba\nstring name      # Name of field\nuint32 offset    # Offset from start of point struct\nuint8  datatype  # Datatype enumeration, see above\nuint32 count     # How many elements in the field" ;
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
        const DEFINITION : & 'static str = "# Single range reading from an active ranger that emits energy and reports\n# one range reading that is valid along an arc at the distance measured.\n# This message is  not appropriate for laser scanners. See the LaserScan\n# message if you are working with a laser scanner.\n#\n# This message also can represent a fixed-distance (binary) ranger.  This\n# sensor will have min_range===max_range===distance of detection.\n# These sensors follow REP 117 and will output -Inf if the object is detected\n# and +Inf if the object is outside of the detection range.\n\nstd_msgs/Header header # timestamp in the header is the time the ranger\n                             # returned the distance reading\n\n# Radiation type enums\n# If you want a value added to this list, send an email to the ros-users list\nuint8 ULTRASOUND=0\nuint8 INFRARED=1\n\nuint8 radiation_type    # the type of radiation used by the sensor\n                        # (sound, IR, etc) [enum]\n\nfloat32 field_of_view   # the size of the arc that the distance reading is\n                        # valid for [rad]\n                        # the object causing the range reading may have\n                        # been anywhere within -field_of_view/2 and\n                        # field_of_view/2 at the measured range.\n                        # 0 angle corresponds to the x-axis of the sensor.\n\nfloat32 min_range       # minimum range value [m]\nfloat32 max_range       # maximum range value [m]\n                        # Fixed distance rangers require min_range==max_range\n\nfloat32 range           # range data [m]\n                        # (Note: values < range_min or > range_max should be discarded)\n                        # Fixed distance rangers only output -Inf or +Inf.\n                        # -Inf represents a detection within fixed distance.\n                        # (Detection too close to the sensor to quantify)\n                        # +Inf represents no detection within the fixed distance.\n                        # (Object out of range)" ;
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
        const MD5SUM: &'static str = "71cfefa31dcc94f47083b1e89e6fa5c9";
        const DEFINITION : & 'static str = "# Single reading from a relative humidity sensor.\n# Defines the ratio of partial pressure of water vapor to the saturated vapor\n# pressure at a temperature.\n\nstd_msgs/Header header # timestamp of the measurement\n                             # frame_id is the location of the humidity sensor\n\nfloat64 relative_humidity    # Expression of the relative humidity\n                             # from 0.0 to 1.0.\n                             # 0.0 is no partial pressure of water vapor\n                             # 1.0 represents partial pressure of saturation\n\nfloat64 variance             # 0 is interpreted as variance unknown" ;
    }
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
        const DEFINITION : & 'static str = "# Single temperature reading.\n\nstd_msgs/Header header # timestamp is the time the temperature was measured\n                             # frame_id is the location of the temperature reading\n\nfloat64 temperature          # Measurement of the Temperature in Degrees Celsius.\n\nfloat64 variance             # 0 is interpreted as variance unknown." ;
    }
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
        const DEFINITION : & 'static str = "# Measurement from an external time source not actively synchronized with the system clock.\n\nstd_msgs/Header header      # stamp is system time for which measurement was valid\n                                  # frame_id is not used\n\nbuiltin_interfaces/Time time_ref  # corresponding time from this external source\nstring source                     # (optional) name of time source" ;
    }
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
        const MD5SUM: &'static str = "251c96e357751cc7c699c496178141d5";
        const DEFINITION : & 'static str = "# This service requests that a camera stores the given CameraInfo as that\n# camera's calibration information.\n#\n# The width and height in the camera_info field should match what the\n# camera is currently outputting on its camera_info topic, and the camera\n# will assume that the region of the imager that is being referred to is\n# the region that the camera is currently capturing.\n\nsensor_msgs/CameraInfo camera_info # The camera_info to store" ;
    }
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
        const DEFINITION : & 'static str = "bool success                             # True if the call succeeded\nstring status_message                    # Used to give details about success" ;
    }
    pub struct SetCameraInfo {}
    impl ::roslibrust_codegen::RosServiceType for SetCameraInfo {
        const ROS_SERVICE_NAME: &'static str = "sensor_msgs/SetCameraInfo";
        const MD5SUM: &'static str = "c191a50a3d5730b8679f4b95b3948b15";
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
        const MD5SUM: &'static str = "1ffdae9486cd3316a121c578b47a85cc";
        const DEFINITION : & 'static str = "# Definition of a mesh.\n\n# List of triangles; the index values refer to positions in vertices[].\nMeshTriangle[] triangles\n\n# The actual vertices that make up the mesh.\ngeometry_msgs/Point[] vertices" ;
    }
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
            "# Definition of a triangle's vertices.\n\nuint32[3] vertex_indices";
    }
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
        const DEFINITION : & 'static str = "# Representation of a plane, using the plane equation ax + by + cz + d = 0.\n#\n# a := coef[0]\n# b := coef[1]\n# c := coef[2]\n# d := coef[3]\nfloat64[4] coef" ;
    }
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
        pub r#dimensions: [f64; 0],
        pub r#polygon: geometry_msgs::Polygon,
    }
    impl ::roslibrust_codegen::RosMessageType for SolidPrimitive {
        const ROS_TYPE_NAME: &'static str = "shape_msgs/SolidPrimitive";
        const MD5SUM: &'static str = "0cdf91a0a45ccd7bc1e0deb784cb2958";
        const DEFINITION : & 'static str = "# Defines box, sphere, cylinder, cone and prism.\n# All shapes are defined to have their bounding boxes centered around 0,0,0.\n\nuint8 BOX=1\nuint8 SPHERE=2\nuint8 CYLINDER=3\nuint8 CONE=4\nuint8 PRISM=5\n\n# The type of the shape\nuint8 type\n\n# The dimensions of the shape\nfloat64[<=3] dimensions  # At no point will dimensions have a length > 3.\n\n# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array.\n\n# For type BOX, the X, Y, and Z dimensions are the length of the corresponding sides of the box.\nuint8 BOX_X=0\nuint8 BOX_Y=1\nuint8 BOX_Z=2\n\n# For the SPHERE type, only one component is used, and it gives the radius of the sphere.\nuint8 SPHERE_RADIUS=0\n\n# For the CYLINDER and CONE types, the center line is oriented along the Z axis.\n# Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component of dimensions gives the\n# height of the cylinder (cone).\n# The CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the radius of\n# the base of the cylinder (cone).\n# Cone and cylinder primitives are defined to be circular. The tip of the cone\n# is pointing up, along +Z axis.\n\nuint8 CYLINDER_HEIGHT=0\nuint8 CYLINDER_RADIUS=1\n\nuint8 CONE_HEIGHT=0\nuint8 CONE_RADIUS=1\n\n# For the type PRISM, the center line is oriented along Z axis.\n# The PRISM_HEIGHT component of dimensions gives the\n# height of the prism.\n# The polygon defines the Z axis centered base of the prism.\n# The prism is constructed by extruding the base in +Z and -Z\n# directions by half of the PRISM_HEIGHT\n# Only x and y fields of the points are used in the polygon.\n# Points of the polygon are ordered counter-clockwise.\n\nuint8 PRISM_HEIGHT=0\ngeometry_msgs/Polygon polygon" ;
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nbool data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nbyte data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nbyte[]            data          # array of data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nchar data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nfloat32 data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nfloat32[]         data          # array of data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nfloat64 data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nfloat64[]         data          # array of data" ;
    }
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
        const DEFINITION : & 'static str = "# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nint16 data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nint16[]           data          # array of data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nint32 data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nint32[]           data          # array of data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nint64 data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nint64[]           data          # array of data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nint8 data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nint8[]            data          # array of data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nstring label   # label of given dimension\nuint32 size    # size of given dimension (in type units)\nuint32 stride  # stride of given dimension" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# The multiarray declares a generic multi-dimensional array of a\n# particular data type.  Dimensions are ordered from outer most\n# to inner most.\n#\n# Accessors should ALWAYS be written in terms of dimension stride\n# and specified outer-most dimension first.\n#\n# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n#\n# A standard, 3-channel 640x480 image with interleaved color channels\n# would be specified as:\n#\n# dim[0].label  = \"height\"\n# dim[0].size   = 480\n# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n# dim[1].label  = \"width\"\n# dim[1].size   = 640\n# dim[1].stride = 3*640 = 1920\n# dim[2].label  = \"channel\"\n# dim[2].size   = 3\n# dim[2].stride = 3\n#\n# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n\nMultiArrayDimension[] dim # Array of dimension properties\nuint32 data_offset        # padding bytes at front of data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nstring data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nuint16 data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nuint16[]            data        # array of data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nuint32 data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nuint32[]          data          # array of data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nuint64 data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nuint64[]          data          # array of data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nuint8 data" ;
    }
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
        const DEFINITION : & 'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nuint8[]           data          # array of data" ;
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
        const DEFINITION : & 'static str = "# Separate header for compatibility with current TimeSynchronizer.\n# Likely to be removed in a later release, use image.header instead.\nstd_msgs/Header header\n\n# Floating point disparity image. The disparities are pre-adjusted for any\n# x-offset between the principal points of the two cameras (in the case\n# that they are verged). That is: d = x_l - x_r - (cx_l - cx_r)\nsensor_msgs/Image image\n\n# Stereo geometry. For disparity d, the depth from the camera is Z = fT/d.\nfloat32 f # Focal length, pixels\nfloat32 t # Baseline, world units\n\n# Subwindow of (potentially) valid disparity values.\nsensor_msgs/RegionOfInterest valid_window\n\n# The range of disparities searched.\n# In the disparity image, any disparity less than min_disparity is invalid.\n# The disparity search range defines the horopter, or 3D volume that the\n# stereo algorithm can \"see\". Points with Z outside of:\n#     Z_min = fT / max_disparity\n#     Z_max = fT / min_disparity\n# could not be found.\nfloat32 min_disparity\nfloat32 max_disparity\n\n# Smallest allowed disparity increment. The smallest achievable depth range\n# resolution is delta_Z = (Z^2/fT)*delta_d.\nfloat32 delta_d" ;
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
        const DEFINITION : & 'static str = "# This message is specifically for testing generating of default values\n# Examples based on https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html\nuint8 x 42\nint16 y -2000\nstring full_name \"John Doe\"\nint32[] samples [-200, -100, 0, 100, 200]\n\n# More complicated examples to stress the system, floats with mixed precision\nfloat32[] f_samples [-200, -1.0, 0]\nstring[] s_vec [\"hello\", \"world\"]\n# This may or may not be valid ROS, it probably is, but we don't handle yet\n# TODO handle this somehow\n# string[] s_vec_2 ['hello', 'world']\n\n# TODO ROS says this is valid, but we currently don't handle\n#string single_quote 'Jane Doe'" ;
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
        const DEFINITION : & 'static str = "# The header is used to specify the coordinate frame and the reference time for\n# the trajectory durations\nstd_msgs/Header header\n\n# The names of the active joints in each trajectory point. These names are\n# ordered and must correspond to the values in each trajectory point.\nstring[] joint_names\n\n# Array of trajectory points, which describe the positions, velocities,\n# accelerations and/or efforts of the joints at each time point.\nJointTrajectoryPoint[] points" ;
    }
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
        const DEFINITION : & 'static str = "# Each trajectory point specifies either positions[, velocities[, accelerations]]\n# or positions[, effort] for the trajectory to be executed.\n# All specified values are in the same order as the joint names in JointTrajectory.msg.\n\n# Single DOF joint positions for each joint relative to their \"0\" position.\n# The units depend on the specific joint type: radians for revolute or\n# continuous joints, and meters for prismatic joints.\nfloat64[] positions\n\n# The rate of change in position of each joint. Units are joint type dependent.\n# Radians/second for revolute or continuous joints, and meters/second for\n# prismatic joints.\nfloat64[] velocities\n\n# Rate of change in velocity of each joint. Units are joint type dependent.\n# Radians/second^2 for revolute or continuous joints, and meters/second^2 for\n# prismatic joints.\nfloat64[] accelerations\n\n# The torque or the force to be applied at each joint. For revolute/continuous\n# joints effort denotes a torque in newton-meters. For prismatic joints, effort\n# denotes a force in newtons.\nfloat64[] effort\n\n# Desired time from the trajectory start to arrive at this trajectory point.\nbuiltin_interfaces/Duration time_from_start" ;
    }
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
        const DEFINITION : & 'static str = "# The header is used to specify the coordinate frame and the reference time for the trajectory durations\nstd_msgs/Header header\n\n# A representation of a multi-dof joint trajectory (each point is a transformation)\n# Each point along the trajectory will include an array of positions/velocities/accelerations\n# that has the same length as the array of joint names, and has the same order of joints as \n# the joint names array.\n\nstring[] joint_names\nMultiDOFJointTrajectoryPoint[] points" ;
    }
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
        const DEFINITION : & 'static str = "# Each multi-dof joint can specify a transform (up to 6 DOF).\ngeometry_msgs/Transform[] transforms\n\n# There can be a velocity specified for the origin of the joint.\ngeometry_msgs/Twist[] velocities\n\n# There can be an acceleration specified for the origin of the joint.\ngeometry_msgs/Twist[] accelerations\n\n# Desired time from the trajectory start to arrive at this trajectory point.\nbuiltin_interfaces/Duration time_from_start" ;
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
        const DEFINITION : & 'static str = "int32 CIRCLE=0\nint32 LINE_STRIP=1\nint32 LINE_LIST=2\nint32 POLYGON=3\nint32 POINTS=4\n\nint32 ADD=0\nint32 REMOVE=1\n\nstd_msgs/Header header\n# Namespace which is used with the id to form a unique id.\nstring ns\n# Unique id within the namespace.\nint32 id\n# One of the above types, e.g. CIRCLE, LINE_STRIP, etc.\nint32 type\n# Either ADD or REMOVE.\nint32 action\n# Two-dimensional coordinate position, in pixel-coordinates.\ngeometry_msgs/Point position\n# The scale of the object, e.g. the diameter for a CIRCLE.\nfloat32 scale\n# The outline color of the marker.\nstd_msgs/ColorRGBA outline_color\n# Whether or not to fill in the shape with color.\nuint8 filled\n# Fill color; in the range: [0.0-1.0]\nstd_msgs/ColorRGBA fill_color\n# How long the object should last before being automatically deleted.\n# 0 indicates forever.\nbuiltin_interfaces/Duration lifetime\n\n# Coordinates in 2D in pixel coords. Used for LINE_STRIP, LINE_LIST, POINTS, etc.\ngeometry_msgs/Point[] points\n# The color for each line, point, etc. in the points field.\nstd_msgs/ColorRGBA[] outline_colors" ;
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
        const DEFINITION : & 'static str = "# Time/frame info.\n# If header.time is set to 0, the marker will be retransformed into\n# its frame on each timestep. You will receive the pose feedback\n# in the same frame.\n# Otherwise, you might receive feedback in a different frame.\n# For rviz, this will be the current 'fixed frame' set by the user.\nstd_msgs/Header header\n\n# Initial pose. Also, defines the pivot point for rotations.\ngeometry_msgs/Pose pose\n\n# Identifying string. Must be globally unique in\n# the topic that this message is sent through.\nstring name\n\n# Short description (< 40 characters).\nstring description\n\n# Scale to be used for default controls (default=1).\nfloat32 scale\n\n# All menu and submenu entries associated with this marker.\nMenuEntry[] menu_entries\n\n# List of controls displayed for this marker.\nInteractiveMarkerControl[] controls" ;
    }
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
        const DEFINITION : & 'static str = "# Represents a control that is to be displayed together with an interactive marker\n\n# Identifying string for this control.\n# You need to assign a unique value to this to receive feedback from the GUI\n# on what actions the user performs on this control (e.g. a button click).\nstring name\n\n\n# Defines the local coordinate frame (relative to the pose of the parent\n# interactive marker) in which is being rotated and translated.\n# Default: Identity\ngeometry_msgs/Quaternion orientation\n\n\n# Orientation mode: controls how orientation changes.\n# INHERIT: Follow orientation of interactive marker\n# FIXED: Keep orientation fixed at initial state\n# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).\nuint8 INHERIT = 0\nuint8 FIXED = 1\nuint8 VIEW_FACING = 2\n\nuint8 orientation_mode\n\n# Interaction mode for this control\n#\n# NONE: This control is only meant for visualization; no context menu.\n# MENU: Like NONE, but right-click menu is active.\n# BUTTON: Element can be left-clicked.\n# MOVE_AXIS: Translate along local x-axis.\n# MOVE_PLANE: Translate in local y-z plane.\n# ROTATE_AXIS: Rotate around local x-axis.\n# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.\nuint8 NONE = 0\nuint8 MENU = 1\nuint8 BUTTON = 2\nuint8 MOVE_AXIS = 3\nuint8 MOVE_PLANE = 4\nuint8 ROTATE_AXIS = 5\nuint8 MOVE_ROTATE = 6\n# \"3D\" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.\n# MOVE_3D: Translate freely in 3D space.\n# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.\n# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.\nuint8 MOVE_3D = 7\nuint8 ROTATE_3D = 8\nuint8 MOVE_ROTATE_3D = 9\n\nuint8 interaction_mode\n\n\n# If true, the contained markers will also be visible\n# when the gui is not in interactive mode.\nbool always_visible\n\n\n# Markers to be displayed as custom visual representation.\n# Leave this empty to use the default control handles.\n#\n# Note:\n# - The markers can be defined in an arbitrary coordinate frame,\n#   but will be transformed into the local frame of the interactive marker.\n# - If the header of a marker is empty, its pose will be interpreted as\n#   relative to the pose of the parent interactive marker.\nMarker[] markers\n\n\n# In VIEW_FACING mode, set this to true if you don't want the markers\n# to be aligned with the camera view point. The markers will show up\n# as in INHERIT mode.\nbool independent_marker_orientation\n\n\n# Short description (< 40 characters) of what this control does,\n# e.g. \"Move the robot\".\n# Default: A generic description based on the interaction mode\nstring description" ;
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
        const DEFINITION : & 'static str = "# Time/frame info.\nstd_msgs/Header header\n\n# Identifying string. Must be unique in the topic namespace.\nstring client_id\n\n# Feedback message sent back from the GUI, e.g.\n# when the status of an interactive marker was modified by the user.\n\n# Specifies which interactive marker and control this message refers to\nstring marker_name\nstring control_name\n\n# Type of the event\n# KEEP_ALIVE: sent while dragging to keep up control of the marker\n# MENU_SELECT: a menu entry has been selected\n# BUTTON_CLICK: a button control has been clicked\n# POSE_UPDATE: the pose has been changed using one of the controls\nuint8 KEEP_ALIVE = 0\nuint8 POSE_UPDATE = 1\nuint8 MENU_SELECT = 2\nuint8 BUTTON_CLICK = 3\n\nuint8 MOUSE_DOWN = 4\nuint8 MOUSE_UP = 5\n\nuint8 event_type\n\n# Current pose of the marker\n# Note: Has to be valid for all feedback types.\ngeometry_msgs/Pose pose\n\n# Contains the ID of the selected menu entry\n# Only valid for MENU_SELECT events.\nuint32 menu_entry_id\n\n# If event_type is BUTTON_CLICK, MOUSE_DOWN, or MOUSE_UP, mouse_point\n# may contain the 3 dimensional position of the event on the\n# control.  If it does, mouse_point_valid will be true.  mouse_point\n# will be relative to the frame listed in the header.\ngeometry_msgs/Point mouse_point\nbool mouse_point_valid" ;
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
        const MD5SUM: &'static str = "b88540594a0f8e3fe46c720be41faa03";
        const DEFINITION : & 'static str = "# Time/frame info.\nstd_msgs/Header header\n\n# Initial pose. Also, defines the pivot point for rotations.\ngeometry_msgs/Pose pose\n\n# Identifying string. Must be globally unique in\n# the topic that this message is sent through.\nstring name" ;
    }
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
        const DEFINITION : & 'static str = "# Identifying string. Must be unique in the topic namespace\n# that this server works on.\nstring server_id\n\n# Sequence number.\n# The client will use this to detect if it has missed an update.\nuint64 seq_num\n\n# Type holds the purpose of this message.  It must be one of UPDATE or KEEP_ALIVE.\n# UPDATE: Incremental update to previous state.\n#         The sequence number must be 1 higher than for\n#         the previous update.\n# KEEP_ALIVE: Indicates the that the server is still living.\n#             The sequence number does not increase.\n#             No payload data should be filled out (markers, poses, or erases).\nuint8 KEEP_ALIVE = 0\nuint8 UPDATE = 1\n\nuint8 type\n\n# Note: No guarantees on the order of processing.\n#       Contents must be kept consistent by sender.\n\n# Markers to be added or updated\nInteractiveMarker[] markers\n\n# Poses of markers that should be moved\nInteractiveMarkerPose[] poses\n\n# Names of markers to be erased\nstring[] erases" ;
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
        const DEFINITION : & 'static str = "# See:\n#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker\n#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes\n#\n# for more information on using this message with rviz.\n\nint32 ARROW=0\nint32 CUBE=1\nint32 SPHERE=2\nint32 CYLINDER=3\nint32 LINE_STRIP=4\nint32 LINE_LIST=5\nint32 CUBE_LIST=6\nint32 SPHERE_LIST=7\nint32 POINTS=8\nint32 TEXT_VIEW_FACING=9\nint32 MESH_RESOURCE=10\nint32 TRIANGLE_LIST=11\n\nint32 ADD=0\nint32 MODIFY=0\nint32 DELETE=2\nint32 DELETEALL=3\n\n# Header for timestamp and frame id.\nstd_msgs/Header header\n# Namespace in which to place the object.\n# Used in conjunction with id to create a unique name for the object.\nstring ns\n# Object ID used in conjunction with the namespace for manipulating and deleting the object later.\nint32 id\n# Type of object.\nint32 type\n# Action to take; one of:\n#  - 0 add/modify an object\n#  - 1 (deprecated)\n#  - 2 deletes an object (with the given ns and id)\n#  - 3 deletes all objects (or those with the given ns if any)\nint32 action\n# Pose of the object with respect the frame_id specified in the header.\ngeometry_msgs/Pose pose\n# Scale of the object; 1,1,1 means default (usually 1 meter square).\ngeometry_msgs/Vector3 scale\n# Color of the object; in the range: [0.0-1.0]\nstd_msgs/ColorRGBA color\n# How long the object should last before being automatically deleted.\n# 0 indicates forever.\nbuiltin_interfaces/Duration lifetime\n# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.\nbool frame_locked\n\n# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)\ngeometry_msgs/Point[] points\n# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)\n# The number of colors provided must either be 0 or equal to the number of points provided.\n# NOTE: alpha is not yet used\nstd_msgs/ColorRGBA[] colors\n\n# Texture resource is a special URI that can either reference a texture file in\n# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]\n# or an embedded texture via a string matching the format:\n#   \"embedded://texture_name\"\nstring texture_resource\n# An image to be loaded into the rendering engine as the texture for this marker.\n# This will be used iff texture_resource is set to embedded.\nsensor_msgs/CompressedImage texture\n# Location of each vertex within the texture; in the range: [0.0-1.0]\nUVCoordinate[] uv_coordinates\n\n# Only used for text markers\nstring text\n\n# Only used for MESH_RESOURCE markers.\n# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.\n# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,\n# use the following format for mesh_resource:\n#   \"embedded://mesh_name\"\nstring mesh_resource\nMeshFile mesh_file\nbool mesh_use_embedded_materials" ;
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
        const DEFINITION : & 'static str = "# MenuEntry message.\n#\n# Each InteractiveMarker message has an array of MenuEntry messages.\n# A collection of MenuEntries together describe a\n# menu/submenu/subsubmenu/etc tree, though they are stored in a flat\n# array.  The tree structure is represented by giving each menu entry\n# an ID number and a \"parent_id\" field.  Top-level entries are the\n# ones with parent_id = 0.  Menu entries are ordered within their\n# level the same way they are ordered in the containing array.  Parent\n# entries must appear before their children.\n#\n# Example:\n# - id = 3\n#   parent_id = 0\n#   title = \"fun\"\n# - id = 2\n#   parent_id = 0\n#   title = \"robot\"\n# - id = 4\n#   parent_id = 2\n#   title = \"pr2\"\n# - id = 5\n#   parent_id = 2\n#   title = \"turtle\"\n#\n# Gives a menu tree like this:\n#  - fun\n#  - robot\n#    - pr2\n#    - turtle\n\n# ID is a number for each menu entry.  Must be unique within the\n# control, and should never be 0.\nuint32 id\n\n# ID of the parent of this menu entry, if it is a submenu.  If this\n# menu entry is a top-level entry, set parent_id to 0.\nuint32 parent_id\n\n# menu / entry title\nstring title\n\n# Arguments to command indicated by command_type (below)\nstring command\n\n# Command_type stores the type of response desired when this menu\n# entry is clicked.\n# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.\n# ROSRUN: execute \"rosrun\" with arguments given in the command field (above).\n# ROSLAUNCH: execute \"roslaunch\" with arguments given in the command field (above).\nuint8 FEEDBACK=0\nuint8 ROSRUN=1\nuint8 ROSLAUNCH=2\nuint8 command_type" ;
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
        const DEFINITION : & 'static str = "# Used to send raw mesh files.\n\n# The filename is used for both debug purposes and to provide a file extension\n# for whatever parser is used.\nstring filename\n\n# This stores the raw text of the mesh file.\nuint8[] data" ;
    }
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
        const DEFINITION : & 'static str = "# Location of the pixel as a ratio of the width of a 2D texture.\n# Values should be in range: [0.0-1.0].\nfloat32 u\nfloat32 v" ;
    }
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
    pub struct GetInteractiveMarkersResponse {
        pub r#sequence_number: u64,
        pub r#markers: ::std::vec::Vec<self::InteractiveMarker>,
    }
    impl ::roslibrust_codegen::RosMessageType for GetInteractiveMarkersResponse {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/GetInteractiveMarkersResponse";
        const MD5SUM: &'static str = "923b76ef2c497d4ff5f83a061d424d3b";
        const DEFINITION : & 'static str = "# Sequence number.\n# Set to the sequence number of the latest update message\n# at the time the server received the request.\n# Clients use this to detect if any updates were missed.\nuint64 sequence_number\n\n# All interactive markers provided by the server.\nInteractiveMarker[] markers" ;
    }
    pub struct GetInteractiveMarkers {}
    impl ::roslibrust_codegen::RosServiceType for GetInteractiveMarkers {
        const ROS_SERVICE_NAME: &'static str = "visualization_msgs/GetInteractiveMarkers";
        const MD5SUM: &'static str = "923b76ef2c497d4ff5f83a061d424d3b";
        type Request = GetInteractiveMarkersRequest;
        type Response = GetInteractiveMarkersResponse;
    }
}
