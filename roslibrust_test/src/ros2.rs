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
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct GoalID {
        pub r#stamp: ::roslibrust_codegen::integral_types::Time,
        pub r#id: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for GoalID {
        const ROS_TYPE_NAME: &'static str = "actionlib_msgs/GoalID";
        const MD5SUM: &'static str = "29380925936d499346662d2ed1573d06";
        const DEFINITION: &'static str = r#"# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
builtin_interfaces/Time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct GoalStatus {
        pub r#goal_id: self::GoalID,
        pub r#status: u8,
        pub r#text: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for GoalStatus {
        const ROS_TYPE_NAME: &'static str = "actionlib_msgs/GoalStatus";
        const MD5SUM: &'static str = "c24a9e244d837a856267339807550287";
        const DEFINITION: &'static str = r#"GoalID goal_id
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server.
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server.
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State).
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server
                            #   (Terminal State).
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State).
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State).
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution.
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing, but
                            #    the action server has not yet confirmed that the goal is canceled.
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State).
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not
                            #    be sent over the wire by an action server.

# Allow for the user to associate a string with GoalStatus for debugging.
string text
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
builtin_interfaces/Time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id"#;
    }
    #[allow(unused)]
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
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct GoalStatusArray {
        pub r#header: std_msgs::Header,
        pub r#status_list: ::std::vec::Vec<self::GoalStatus>,
    }
    impl ::roslibrust_codegen::RosMessageType for GoalStatusArray {
        const ROS_TYPE_NAME: &'static str = "actionlib_msgs/GoalStatusArray";
        const MD5SUM: &'static str = "ad4c7a55992b9ff34d89596ca74a28e0";
        const DEFINITION: &'static str = r#"# Stores the statuses for goals that are currently being tracked
# by an action server
std_msgs/Header header
GoalStatus[] status_list
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
builtin_interfaces/Time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
================================================================================
MSG: actionlib_msgs/GoalStatus
GoalID goal_id
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server.
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server.
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State).
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server
                            #   (Terminal State).
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State).
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State).
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution.
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing, but
                            #    the action server has not yet confirmed that the goal is canceled.
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State).
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not
                            #    be sent over the wire by an action server.

# Allow for the user to associate a string with GoalStatus for debugging.
string text
================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
builtin_interfaces/Time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
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
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct DiagnosticArray {
        pub r#header: std_msgs::Header,
        pub r#status: ::std::vec::Vec<self::DiagnosticStatus>,
    }
    impl ::roslibrust_codegen::RosMessageType for DiagnosticArray {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/DiagnosticArray";
        const MD5SUM: &'static str = "7f04f8332be7e46b680724aa4e9a9d71";
        const DEFINITION: &'static str = r#"# This message is used to send diagnostic information about the state of the robot.
std_msgs/Header header # for timestamp
DiagnosticStatus[] status # an array of components being reported on
================================================================================
MSG: diagnostic_msgs/DiagnosticStatus
# This message holds the status of an individual component of the robot.

# Possible levels of operations.
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

# Level of operation enumerated above.
byte level
# A description of the test/component reporting.
string name
# A description of the status.
string message
# A hardware unique string.
string hardware_id
# An array of values associated with the status.
KeyValue[] values
================================================================================
MSG: diagnostic_msgs/KeyValue
# What to label this value when viewing.
string key
# A value to track over time.
string value
================================================================================
MSG: diagnostic_msgs/KeyValue
# What to label this value when viewing.
string key
# A value to track over time.
string value
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# This message holds the status of an individual component of the robot.

# Possible levels of operations.
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

# Level of operation enumerated above.
byte level
# A description of the test/component reporting.
string name
# A description of the status.
string message
# A hardware unique string.
string hardware_id
# An array of values associated with the status.
KeyValue[] values
================================================================================
MSG: diagnostic_msgs/KeyValue
# What to label this value when viewing.
string key
# A value to track over time.
string value"#;
    }
    #[allow(unused)]
    impl DiagnosticStatus {
        pub const r#OK: u8 = 0u8;
        pub const r#WARN: u8 = 1u8;
        pub const r#ERROR: u8 = 2u8;
        pub const r#STALE: u8 = 3u8;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct KeyValue {
        pub r#key: ::std::string::String,
        pub r#value: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for KeyValue {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/KeyValue";
        const MD5SUM: &'static str = "cf57fdc6617a881a88c16e768132149c";
        const DEFINITION: &'static str = r#"# What to label this value when viewing.
string key
# A value to track over time.
string value"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct AddDiagnosticsRequest {
        pub r#load_namespace: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for AddDiagnosticsRequest {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/AddDiagnosticsRequest";
        const MD5SUM: &'static str = "c26cf6e164288fbc6050d74f838bcdf0";
        const DEFINITION: &'static str = r#"# This service is used as part of the process for loading analyzers at runtime,
# and should be used by a loader script or program, not as a standalone service.
# Information about dynamic addition of analyzers can be found at
# http://wiki.ros.org/diagnostics/Tutorials/Adding%20Analyzers%20at%20Runtime

# The load_namespace parameter defines the namespace where parameters for the
# initialization of analyzers in the diagnostic aggregator have been loaded. The
# value should be a global name (i.e. /my/name/space), not a relative
# (my/name/space) or private (~my/name/space) name. Analyzers will not be added
# if a non-global name is used. The call will also fail if the namespace
# contains parameters that follow a namespace structure that does not conform to
# that expected by the analyzer definitions. See
# http://wiki.ros.org/diagnostics/Tutorials/Configuring%20Diagnostic%20Aggregators
# and http://wiki.ros.org/diagnostics/Tutorials/Using%20the%20GenericAnalyzer
# for examples of the structure of yaml files which are expected to have been
# loaded into the namespace.
string load_namespace"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct AddDiagnosticsResponse {
        pub r#success: bool,
        pub r#message: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for AddDiagnosticsResponse {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/AddDiagnosticsResponse";
        const MD5SUM: &'static str = "937c9679a518e3a18d831e57125ea522";
        const DEFINITION: &'static str = r#"# True if diagnostic aggregator was updated with new diagnostics, False
# otherwise. A false return value means that either there is a bond in the
# aggregator which already used the requested namespace, or the initialization
# of analyzers failed.
bool success

# Message with additional information about the success or failure
string message"#;
    }
    #[allow(dead_code)]
    pub struct AddDiagnostics {}
    impl ::roslibrust_codegen::RosServiceType for AddDiagnostics {
        const ROS_SERVICE_NAME: &'static str = "diagnostic_msgs/AddDiagnostics";
        const MD5SUM: &'static str = "e6ac9bbde83d0d3186523c3687aecaee";
        type Request = AddDiagnosticsRequest;
        type Response = AddDiagnosticsResponse;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct SelfTestRequest {}
    impl ::roslibrust_codegen::RosMessageType for SelfTestRequest {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/SelfTestRequest";
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
        const DEFINITION: &'static str = r#""#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct SelfTestResponse {
        pub r#id: ::std::string::String,
        pub r#passed: u8,
        pub r#status: ::std::vec::Vec<self::DiagnosticStatus>,
    }
    impl ::roslibrust_codegen::RosMessageType for SelfTestResponse {
        const ROS_TYPE_NAME: &'static str = "diagnostic_msgs/SelfTestResponse";
        const MD5SUM: &'static str = "ac21b1bab7ab17546986536c22eb34e9";
        const DEFINITION: &'static str = r#"string id
byte passed
DiagnosticStatus[] status
================================================================================
MSG: diagnostic_msgs/DiagnosticStatus
# This message holds the status of an individual component of the robot.

# Possible levels of operations.
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

# Level of operation enumerated above.
byte level
# A description of the test/component reporting.
string name
# A description of the status.
string message
# A hardware unique string.
string hardware_id
# An array of values associated with the status.
KeyValue[] values
================================================================================
MSG: diagnostic_msgs/KeyValue
# What to label this value when viewing.
string key
# A value to track over time.
string value
================================================================================
MSG: diagnostic_msgs/KeyValue
# What to label this value when viewing.
string key
# A value to track over time.
string value"#;
    }
    #[allow(dead_code)]
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
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Accel {
        pub r#linear: self::Vector3,
        pub r#angular: self::Vector3,
    }
    impl ::roslibrust_codegen::RosMessageType for Accel {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Accel";
        const MD5SUM: &'static str = "9f195f881246fdfa2798d1d3eebca84a";
        const DEFINITION: &'static str = r#"# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct AccelStamped {
        pub r#header: std_msgs::Header,
        pub r#accel: self::Accel,
    }
    impl ::roslibrust_codegen::RosMessageType for AccelStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/AccelStamped";
        const MD5SUM: &'static str = "19a31cf6d39a90e769a5539f9a293621";
        const DEFINITION: &'static str = r#"# An accel with reference coordinate frame and timestamp
std_msgs/Header header
Accel accel
================================================================================
MSG: geometry_msgs/Accel
# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct AccelWithCovariance {
        pub r#accel: self::Accel,
        #[default(_code = "[Default::default(); 36]")]
        #[serde(with = "::roslibrust_codegen::BigArray")]
        pub r#covariance: [f64; 36],
    }
    impl ::roslibrust_codegen::RosMessageType for AccelWithCovariance {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/AccelWithCovariance";
        const MD5SUM: &'static str = "ad5a718d699c6be72a02b8d6a139f334";
        const DEFINITION: &'static str = r#"# This expresses acceleration in free space with uncertainty.

Accel accel

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
================================================================================
MSG: geometry_msgs/Accel
# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct AccelWithCovarianceStamped {
        pub r#header: std_msgs::Header,
        pub r#accel: self::AccelWithCovariance,
    }
    impl ::roslibrust_codegen::RosMessageType for AccelWithCovarianceStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/AccelWithCovarianceStamped";
        const MD5SUM: &'static str = "36b6f1177d3c3f476d4c306279c6f18a";
        const DEFINITION: &'static str = r#"# This represents an estimated accel with reference coordinate frame and timestamp.
std_msgs/Header header
AccelWithCovariance accel
================================================================================
MSG: geometry_msgs/Accel
# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/AccelWithCovariance
# This expresses acceleration in free space with uncertainty.

Accel accel

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
================================================================================
MSG: geometry_msgs/Accel
# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# Mass [kg]
float64 m

# Center of mass [m]
geometry_msgs/Vector3 com

# Inertia Tensor [kg-m^2]
#     | ixx ixy ixz |
# I = | ixy iyy iyz |
#     | ixz iyz izz |
float64 ixx
float64 ixy
float64 ixz
float64 iyy
float64 iyz
float64 izz
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct InertiaStamped {
        pub r#header: std_msgs::Header,
        pub r#inertia: self::Inertia,
    }
    impl ::roslibrust_codegen::RosMessageType for InertiaStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/InertiaStamped";
        const MD5SUM: &'static str = "d4fb75ac056292d6c4bbec5e51d25080";
        const DEFINITION: &'static str = r#"# An Inertia with a time stamp and reference frame.

std_msgs/Header header
Inertia inertia
================================================================================
MSG: geometry_msgs/Inertia
# Mass [kg]
float64 m

# Center of mass [m]
geometry_msgs/Vector3 com

# Inertia Tensor [kg-m^2]
#     | ixx ixy ixz |
# I = | ixy iyy iyz |
#     | ixz iyz izz |
float64 ixx
float64 ixy
float64 ixz
float64 iyy
float64 iyz
float64 izz
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Point {
        pub r#x: f64,
        pub r#y: f64,
        pub r#z: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for Point {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Point";
        const MD5SUM: &'static str = "4a842b65f413084dc2b10fb484ea7f17";
        const DEFINITION: &'static str = r#"# This contains the position of a point in free space
float64 x
float64 y
float64 z"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Point32 {
        pub r#x: f32,
        pub r#y: f32,
        pub r#z: f32,
    }
    impl ::roslibrust_codegen::RosMessageType for Point32 {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Point32";
        const MD5SUM: &'static str = "cc153912f1453b708d221682bc23d9ac";
        const DEFINITION: &'static str = r#"# This contains the position of a point in free space(with 32 bits of precision).
# It is recommended to use Point wherever possible instead of Point32.
#
# This recommendation is to promote interoperability.
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.

float32 x
float32 y
float32 z"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct PointStamped {
        pub r#header: std_msgs::Header,
        pub r#point: self::Point,
    }
    impl ::roslibrust_codegen::RosMessageType for PointStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PointStamped";
        const MD5SUM: &'static str = "938cb86faf4572821e49e2490701e6df";
        const DEFINITION: &'static str = r#"# This represents a Point with reference coordinate frame and timestamp

std_msgs/Header header
Point point
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Polygon {
        pub r#points: ::std::vec::Vec<self::Point32>,
    }
    impl ::roslibrust_codegen::RosMessageType for Polygon {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Polygon";
        const MD5SUM: &'static str = "cd60a26494a087f577976f0329fa120e";
        const DEFINITION: &'static str = r#"# A specification of a polygon where the first and last points are assumed to be connected

Point32[] points
================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommended to use Point wherever possible instead of Point32.
#
# This recommendation is to promote interoperability.
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.

float32 x
float32 y
float32 z"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct PolygonStamped {
        pub r#header: std_msgs::Header,
        pub r#polygon: self::Polygon,
    }
    impl ::roslibrust_codegen::RosMessageType for PolygonStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PolygonStamped";
        const MD5SUM: &'static str = "56a3a2a80165092f696df3db62e18fbf";
        const DEFINITION: &'static str = r#"# This represents a Polygon with reference coordinate frame and timestamp

std_msgs/Header header
Polygon polygon
================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommended to use Point wherever possible instead of Point32.
#
# This recommendation is to promote interoperability.
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.

float32 x
float32 y
float32 z
================================================================================
MSG: geometry_msgs/Polygon
# A specification of a polygon where the first and last points are assumed to be connected

Point32[] points
================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommended to use Point wherever possible instead of Point32.
#
# This recommendation is to promote interoperability.
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.

float32 x
float32 y
float32 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Pose {
        pub r#position: self::Point,
        pub r#orientation: self::Quaternion,
    }
    impl ::roslibrust_codegen::RosMessageType for Pose {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Pose";
        const MD5SUM: &'static str = "e45d45a5a1ce597b249e23fb30fc871f";
        const DEFINITION: &'static str = r#"# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Pose2D {
        pub r#x: f64,
        pub r#y: f64,
        pub r#theta: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for Pose2D {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Pose2D";
        const MD5SUM: &'static str = "938fa65709584ad8e77d238529be13b8";
        const DEFINITION: &'static str = r#"# Deprecated as of Foxy and will potentially be removed in any following release.
# Please use the full 3D pose.

# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct PoseArray {
        pub r#header: std_msgs::Header,
        pub r#poses: ::std::vec::Vec<self::Pose>,
    }
    impl ::roslibrust_codegen::RosMessageType for PoseArray {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseArray";
        const MD5SUM: &'static str = "ea7300c78ec47498d5f226be74a155e8";
        const DEFINITION: &'static str = r#"# An array of poses with a header for global reference.

std_msgs/Header header

Pose[] poses
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct PoseStamped {
        pub r#header: std_msgs::Header,
        pub r#pose: self::Pose,
    }
    impl ::roslibrust_codegen::RosMessageType for PoseStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseStamped";
        const MD5SUM: &'static str = "c088ec4a70a5930b0ca46520d5745e2d";
        const DEFINITION: &'static str = r#"# A Pose with reference coordinate frame and timestamp

std_msgs/Header header
Pose pose
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct PoseWithCovariance {
        pub r#pose: self::Pose,
        #[default(_code = "[Default::default(); 36]")]
        #[serde(with = "::roslibrust_codegen::BigArray")]
        pub r#covariance: [f64; 36],
    }
    impl ::roslibrust_codegen::RosMessageType for PoseWithCovariance {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseWithCovariance";
        const MD5SUM: &'static str = "c23e848cf1b7533a8d7c259073a97e6f";
        const DEFINITION: &'static str = r#"# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct PoseWithCovarianceStamped {
        pub r#header: std_msgs::Header,
        pub r#pose: self::PoseWithCovariance,
    }
    impl ::roslibrust_codegen::RosMessageType for PoseWithCovarianceStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/PoseWithCovarianceStamped";
        const MD5SUM: &'static str = "2178452bf195c1abe1e99b07b4e6c8f0";
        const DEFINITION: &'static str = r#"# This expresses an estimated pose with a reference coordinate frame and timestamp

std_msgs/Header header
PoseWithCovariance pose
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct QuaternionStamped {
        pub r#header: std_msgs::Header,
        pub r#quaternion: self::Quaternion,
    }
    impl ::roslibrust_codegen::RosMessageType for QuaternionStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/QuaternionStamped";
        const MD5SUM: &'static str = "8f93ed7c8430d06bd82fefcc6f7a349e";
        const DEFINITION: &'static str = r#"# This represents an orientation with reference coordinate frame and timestamp.

std_msgs/Header header
Quaternion quaternion
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Transform {
        pub r#translation: self::Vector3,
        pub r#rotation: self::Quaternion,
    }
    impl ::roslibrust_codegen::RosMessageType for Transform {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Transform";
        const MD5SUM: &'static str = "ac9eff44abf714214112b05d54a3cf9b";
        const DEFINITION: &'static str = r#"# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct TransformStamped {
        pub r#header: std_msgs::Header,
        pub r#child_frame_id: ::std::string::String,
        pub r#transform: self::Transform,
    }
    impl ::roslibrust_codegen::RosMessageType for TransformStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/TransformStamped";
        const MD5SUM: &'static str = "09bf247c06cf7c69e8c55300b05a7a04";
        const DEFINITION: &'static str = r#"# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id at the time of header.stamp
#
# This message is mostly used by the
# <a href="https://index.ros.org/p/tf2/">tf2</a> package.
# See its documentation for more information.
#
# The child_frame_id is necessary in addition to the frame_id
# in the Header to communicate the full reference for the transform
# in a self contained message.

# The frame id in the header is used as the reference frame of this transform.
std_msgs/Header header

# The frame id of the child frame to which this transform points.
string child_frame_id

# Translation and rotation in 3-dimensions of child_frame_id from header.frame_id.
Transform transform
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Twist {
        pub r#linear: self::Vector3,
        pub r#angular: self::Vector3,
    }
    impl ::roslibrust_codegen::RosMessageType for Twist {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Twist";
        const MD5SUM: &'static str = "9f195f881246fdfa2798d1d3eebca84a";
        const DEFINITION: &'static str = r#"# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct TwistStamped {
        pub r#header: std_msgs::Header,
        pub r#twist: self::Twist,
    }
    impl ::roslibrust_codegen::RosMessageType for TwistStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/TwistStamped";
        const MD5SUM: &'static str = "09f84400c1ca2e7e26a9da1232813bd0";
        const DEFINITION: &'static str = r#"# A twist with reference coordinate frame and timestamp

std_msgs/Header header
Twist twist
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct TwistWithCovariance {
        pub r#twist: self::Twist,
        #[default(_code = "[Default::default(); 36]")]
        #[serde(with = "::roslibrust_codegen::BigArray")]
        pub r#covariance: [f64; 36],
    }
    impl ::roslibrust_codegen::RosMessageType for TwistWithCovariance {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/TwistWithCovariance";
        const MD5SUM: &'static str = "1fe8a28e6890a4cc3ae4c3ca5c7d82e6";
        const DEFINITION: &'static str = r#"# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct TwistWithCovarianceStamped {
        pub r#header: std_msgs::Header,
        pub r#twist: self::TwistWithCovariance,
    }
    impl ::roslibrust_codegen::RosMessageType for TwistWithCovarianceStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/TwistWithCovarianceStamped";
        const MD5SUM: &'static str = "7019807c85ce8602fb83180366470670";
        const DEFINITION: &'static str = r#"# This represents an estimated twist with reference coordinate frame and timestamp.

std_msgs/Header header
TwistWithCovariance twist
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Vector3 {
        pub r#x: f64,
        pub r#y: f64,
        pub r#z: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for Vector3 {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Vector3";
        const MD5SUM: &'static str = "4a842b65f413084dc2b10fb484ea7f17";
        const DEFINITION: &'static str = r#"# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Vector3Stamped {
        pub r#header: std_msgs::Header,
        pub r#vector: self::Vector3,
    }
    impl ::roslibrust_codegen::RosMessageType for Vector3Stamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Vector3Stamped";
        const MD5SUM: &'static str = "5cd361f2989a2e76d5aaf432c3bf0fb9";
        const DEFINITION: &'static str = r#"# This represents a Vector3 with reference coordinate frame and timestamp

# Note that this follows vector semantics with it always anchored at the origin,
# so the rotational elements of a transform are the only parts applied when transforming.

std_msgs/Header header
Vector3 vector
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Wrench {
        pub r#force: self::Vector3,
        pub r#torque: self::Vector3,
    }
    impl ::roslibrust_codegen::RosMessageType for Wrench {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/Wrench";
        const MD5SUM: &'static str = "4f539cf138b23283b520fd271b567936";
        const DEFINITION: &'static str = r#"# This represents force in free space, separated into its linear and angular parts.

Vector3  force
Vector3  torque
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct WrenchStamped {
        pub r#header: std_msgs::Header,
        pub r#wrench: self::Wrench,
    }
    impl ::roslibrust_codegen::RosMessageType for WrenchStamped {
        const ROS_TYPE_NAME: &'static str = "geometry_msgs/WrenchStamped";
        const MD5SUM: &'static str = "5bc71556ab354cd6274d262a7de094a5";
        const DEFINITION: &'static str = r#"# A wrench with reference coordinate frame and timestamp

std_msgs/Header header
Wrench wrench
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into its linear and angular parts.

Vector3  force
Vector3  torque
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
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
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct GridCells {
        pub r#header: std_msgs::Header,
        pub r#cell_width: f32,
        pub r#cell_height: f32,
        pub r#cells: ::std::vec::Vec<geometry_msgs::Point>,
    }
    impl ::roslibrust_codegen::RosMessageType for GridCells {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GridCells";
        const MD5SUM: &'static str = "bb9f07bfd2183241b5719f45a81f8cc5";
        const DEFINITION: &'static str = r#"# An array of cells in a 2D grid

std_msgs/Header header

# Width of each cell
float32 cell_width

# Height of each cell
float32 cell_height

# Each cell is represented by the Point at the center of the cell
geometry_msgs/Point[] cells
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# This hold basic information about the characteristics of the OccupancyGrid

# The time at which the map was loaded
builtin_interfaces/Time map_load_time

# The map resolution [m/cell]
float32 resolution

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height

# The origin of the map [m, m, rad].  This is the real-world pose of the
# bottom left corner of cell (0,0) in the map.
geometry_msgs/Pose origin
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct OccupancyGrid {
        pub r#header: std_msgs::Header,
        pub r#info: self::MapMetaData,
        pub r#data: ::std::vec::Vec<i8>,
    }
    impl ::roslibrust_codegen::RosMessageType for OccupancyGrid {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/OccupancyGrid";
        const MD5SUM: &'static str = "2b0657f1993991bf3953916eb5ff5203";
        const DEFINITION: &'static str = r#"# This represents a 2-D grid map
std_msgs/Header header

# MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0). 
# Cell (1, 0) will be listed second, representing the next cell in the x direction. 
# Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).
# The values inside are application dependent, but frequently, 
# 0 represents unoccupied, 1 represents definitely occupied, and
# -1 represents unknown. 
int8[] data
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characteristics of the OccupancyGrid

# The time at which the map was loaded
builtin_interfaces/Time map_load_time

# The map resolution [m/cell]
float32 resolution

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height

# The origin of the map [m, m, rad].  This is the real-world pose of the
# bottom left corner of cell (0,0) in the map.
geometry_msgs/Pose origin
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Odometry {
        pub r#header: std_msgs::Header,
        pub r#child_frame_id: ::std::string::String,
        pub r#pose: geometry_msgs::PoseWithCovariance,
        pub r#twist: geometry_msgs::TwistWithCovariance,
    }
    impl ::roslibrust_codegen::RosMessageType for Odometry {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/Odometry";
        const MD5SUM: &'static str = "81a0900daae2c6c0acc71c9f8df88947";
        const DEFINITION: &'static str = r#"# This represents an estimate of a position and velocity in free space.
# The pose in this message should be specified in the coordinate frame given by header.frame_id
# The twist in this message should be specified in the coordinate frame given by the child_frame_id

# Includes the frame id of the pose parent.
std_msgs/Header header

# Frame id the pose points to. The twist is in this coordinate frame.
string child_frame_id

# Estimated pose that is typically relative to a fixed world frame.
geometry_msgs/PoseWithCovariance pose

# Estimated linear and angular velocity relative to child_frame_id.
geometry_msgs/TwistWithCovariance twist
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Path {
        pub r#header: std_msgs::Header,
        pub r#poses: ::std::vec::Vec<geometry_msgs::PoseStamped>,
    }
    impl ::roslibrust_codegen::RosMessageType for Path {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/Path";
        const MD5SUM: &'static str = "9f78b006a4c2cc2a146c12ed59d1bb7f";
        const DEFINITION: &'static str = r#"# An array of poses that represents a Path for a robot to follow.

# Indicates the frame_id of the path.
std_msgs/Header header

# Array of poses to follow.
geometry_msgs/PoseStamped[] poses
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp

std_msgs/Header header
Pose pose
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct GetMapRequest {}
    impl ::roslibrust_codegen::RosMessageType for GetMapRequest {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetMapRequest";
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
        const DEFINITION: &'static str = r#"# Get the map as a nav_msgs/OccupancyGrid"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct GetMapResponse {
        pub r#map: self::OccupancyGrid,
    }
    impl ::roslibrust_codegen::RosMessageType for GetMapResponse {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetMapResponse";
        const MD5SUM: &'static str = "d6e8b0301af2dfe2244959ba20a4080a";
        const DEFINITION: &'static str = r#"# The current map hosted by this map service.
OccupancyGrid map
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characteristics of the OccupancyGrid

# The time at which the map was loaded
builtin_interfaces/Time map_load_time

# The map resolution [m/cell]
float32 resolution

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height

# The origin of the map [m, m, rad].  This is the real-world pose of the
# bottom left corner of cell (0,0) in the map.
geometry_msgs/Pose origin
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/OccupancyGrid
# This represents a 2-D grid map
std_msgs/Header header

# MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0). 
# Cell (1, 0) will be listed second, representing the next cell in the x direction. 
# Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).
# The values inside are application dependent, but frequently, 
# 0 represents unoccupied, 1 represents definitely occupied, and
# -1 represents unknown. 
int8[] data
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characteristics of the OccupancyGrid

# The time at which the map was loaded
builtin_interfaces/Time map_load_time

# The map resolution [m/cell]
float32 resolution

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height

# The origin of the map [m, m, rad].  This is the real-world pose of the
# bottom left corner of cell (0,0) in the map.
geometry_msgs/Pose origin
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(dead_code)]
    pub struct GetMap {}
    impl ::roslibrust_codegen::RosServiceType for GetMap {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/GetMap";
        const MD5SUM: &'static str = "d6e8b0301af2dfe2244959ba20a4080a";
        type Request = GetMapRequest;
        type Response = GetMapResponse;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct GetPlanRequest {
        pub r#start: geometry_msgs::PoseStamped,
        pub r#goal: geometry_msgs::PoseStamped,
        pub r#tolerance: f32,
    }
    impl ::roslibrust_codegen::RosMessageType for GetPlanRequest {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetPlanRequest";
        const MD5SUM: &'static str = "e4855e4d3c7377c76ec90e403202286a";
        const DEFINITION: &'static str = r#"# Get a plan from the current position to the goal Pose

# The start pose for the plan
geometry_msgs/PoseStamped start

# The final pose of the goal position
geometry_msgs/PoseStamped goal

# If the goal is obstructed, how many meters the planner can
# relax the constraint in x and y before failing.
float32 tolerance
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp

std_msgs/Header header
Pose pose
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct GetPlanResponse {
        pub r#plan: self::Path,
    }
    impl ::roslibrust_codegen::RosMessageType for GetPlanResponse {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/GetPlanResponse";
        const MD5SUM: &'static str = "37c13f9b42d0ee04e1dae0d4f7d14878";
        const DEFINITION: &'static str = r#"# Array of poses from start to goal if one was successfully found.
Path plan
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp

std_msgs/Header header
Pose pose
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/Path
# An array of poses that represents a Path for a robot to follow.

# Indicates the frame_id of the path.
std_msgs/Header header

# Array of poses to follow.
geometry_msgs/PoseStamped[] poses
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp

std_msgs/Header header
Pose pose
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(dead_code)]
    pub struct GetPlan {}
    impl ::roslibrust_codegen::RosServiceType for GetPlan {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/GetPlan";
        const MD5SUM: &'static str = "135edd06523950427d2cf5e0bb9780a2";
        type Request = GetPlanRequest;
        type Response = GetPlanResponse;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct LoadMapRequest {
        pub r#map_url: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for LoadMapRequest {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/LoadMapRequest";
        const MD5SUM: &'static str = "3813ba1ae85fbcd4dc88c90f1426b90b";
        const DEFINITION: &'static str = r#"# URL of map resource
# Can be an absolute path to a file: file:///path/to/maps/floor1.yaml
# Or, relative to a ROS package: package://my_ros_package/maps/floor2.yaml
string map_url"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct LoadMapResponse {
        pub r#map: self::OccupancyGrid,
        pub r#result: u8,
    }
    impl ::roslibrust_codegen::RosMessageType for LoadMapResponse {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/LoadMapResponse";
        const MD5SUM: &'static str = "cdb849e3dfaed8b5fe66776d7a64b83e";
        const DEFINITION: &'static str = r#"# Result code defintions
uint8 RESULT_SUCCESS=0
uint8 RESULT_MAP_DOES_NOT_EXIST=1
uint8 RESULT_INVALID_MAP_DATA=2
uint8 RESULT_INVALID_MAP_METADATA=3
uint8 RESULT_UNDEFINED_FAILURE=255

# Returned map is only valid if result equals RESULT_SUCCESS
nav_msgs/OccupancyGrid map
uint8 result
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characteristics of the OccupancyGrid

# The time at which the map was loaded
builtin_interfaces/Time map_load_time

# The map resolution [m/cell]
float32 resolution

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height

# The origin of the map [m, m, rad].  This is the real-world pose of the
# bottom left corner of cell (0,0) in the map.
geometry_msgs/Pose origin
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/OccupancyGrid
# This represents a 2-D grid map
std_msgs/Header header

# MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0). 
# Cell (1, 0) will be listed second, representing the next cell in the x direction. 
# Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).
# The values inside are application dependent, but frequently, 
# 0 represents unoccupied, 1 represents definitely occupied, and
# -1 represents unknown. 
int8[] data
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characteristics of the OccupancyGrid

# The time at which the map was loaded
builtin_interfaces/Time map_load_time

# The map resolution [m/cell]
float32 resolution

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height

# The origin of the map [m, m, rad].  This is the real-world pose of the
# bottom left corner of cell (0,0) in the map.
geometry_msgs/Pose origin
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(unused)]
    impl LoadMapResponse {
        pub const r#RESULT_SUCCESS: u8 = 0u8;
        pub const r#RESULT_MAP_DOES_NOT_EXIST: u8 = 1u8;
        pub const r#RESULT_INVALID_MAP_DATA: u8 = 2u8;
        pub const r#RESULT_INVALID_MAP_METADATA: u8 = 3u8;
        pub const r#RESULT_UNDEFINED_FAILURE: u8 = 255u8;
    }
    #[allow(dead_code)]
    pub struct LoadMap {}
    impl ::roslibrust_codegen::RosServiceType for LoadMap {
        const ROS_SERVICE_NAME: &'static str = "nav_msgs/LoadMap";
        const MD5SUM: &'static str = "96c8a15e8fe5c33ee245f610f020d6ba";
        type Request = LoadMapRequest;
        type Response = LoadMapResponse;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct SetMapRequest {
        pub r#map: self::OccupancyGrid,
        pub r#initial_pose: geometry_msgs::PoseWithCovarianceStamped,
    }
    impl ::roslibrust_codegen::RosMessageType for SetMapRequest {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/SetMapRequest";
        const MD5SUM: &'static str = "98782a373ad73e1165352caf85923850";
        const DEFINITION: &'static str = r#"# Set a new map together with an initial pose

# Requested 2D map to be set.
nav_msgs/OccupancyGrid map

# Estimated initial pose when setting new map.
geometry_msgs/PoseWithCovarianceStamped initial_pose
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/PoseWithCovarianceStamped
# This expresses an estimated pose with a reference coordinate frame and timestamp

std_msgs/Header header
PoseWithCovariance pose
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characteristics of the OccupancyGrid

# The time at which the map was loaded
builtin_interfaces/Time map_load_time

# The map resolution [m/cell]
float32 resolution

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height

# The origin of the map [m, m, rad].  This is the real-world pose of the
# bottom left corner of cell (0,0) in the map.
geometry_msgs/Pose origin
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/OccupancyGrid
# This represents a 2-D grid map
std_msgs/Header header

# MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0). 
# Cell (1, 0) will be listed second, representing the next cell in the x direction. 
# Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).
# The values inside are application dependent, but frequently, 
# 0 represents unoccupied, 1 represents definitely occupied, and
# -1 represents unknown. 
int8[] data
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: nav_msgs/MapMetaData
# This hold basic information about the characteristics of the OccupancyGrid

# The time at which the map was loaded
builtin_interfaces/Time map_load_time

# The map resolution [m/cell]
float32 resolution

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height

# The origin of the map [m, m, rad].  This is the real-world pose of the
# bottom left corner of cell (0,0) in the map.
geometry_msgs/Pose origin
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct SetMapResponse {
        pub r#success: bool,
    }
    impl ::roslibrust_codegen::RosMessageType for SetMapResponse {
        const ROS_TYPE_NAME: &'static str = "nav_msgs/SetMapResponse";
        const MD5SUM: &'static str = "358e233cde0c8a8bcfea4ce193f8fc15";
        const DEFINITION: &'static str = r#"# True if the map was successfully set, false otherwise.
bool success"#;
    }
    #[allow(dead_code)]
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
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# Constants are chosen to match the enums in the linux kernel
# defined in include/linux/power_supply.h as of version 3.7
# The one difference is for style reasons the constants are
# all uppercase not mixed case.

# Power supply status constants
uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0
uint8 POWER_SUPPLY_STATUS_CHARGING = 1
uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2
uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3
uint8 POWER_SUPPLY_STATUS_FULL = 4

# Power supply health constants
uint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0
uint8 POWER_SUPPLY_HEALTH_GOOD = 1
uint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2
uint8 POWER_SUPPLY_HEALTH_DEAD = 3
uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4
uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5
uint8 POWER_SUPPLY_HEALTH_COLD = 6
uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7
uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8

# Power supply technology (chemistry) constants
uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0
uint8 POWER_SUPPLY_TECHNOLOGY_NIMH = 1
uint8 POWER_SUPPLY_TECHNOLOGY_LION = 2
uint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3
uint8 POWER_SUPPLY_TECHNOLOGY_LIFE = 4
uint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5
uint8 POWER_SUPPLY_TECHNOLOGY_LIMN = 6

std_msgs/Header  header
float32 voltage          # Voltage in Volts (Mandatory)
float32 temperature      # Temperature in Degrees Celsius (If unmeasured NaN)
float32 current          # Negative when discharging (A)  (If unmeasured NaN)
float32 charge           # Current charge in Ah  (If unmeasured NaN)
float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)
float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)
float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)
uint8   power_supply_status     # The charging status as reported. Values defined above
uint8   power_supply_health     # The battery health metric. Values defined above
uint8   power_supply_technology # The battery chemistry. Values defined above
bool    present          # True if the battery is present

float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack
                         # If individual voltages unknown but number of cells known set each to NaN
float32[] cell_temperature # An array of individual cell temperatures for each cell in the pack
                           # If individual temperatures unknown but number of cells known set each to NaN
string location          # The location into which the battery is inserted. (slot number or plug)
string serial_number     # The best approximation of the battery serial number
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(unused)]
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
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# This message defines meta information for a camera. It should be in a
# camera namespace on topic "camera_info" and accompanied by up to five
# image topics named:
#
#   image_raw - raw data from the camera driver, possibly Bayer encoded
#   image            - monochrome, distorted
#   image_color      - color, distorted
#   image_rect       - monochrome, rectified
#   image_rect_color - color, rectified
#
# The image_pipeline contains packages (image_proc, stereo_image_proc)
# for producing the four processed image topics from image_raw and
# camera_info. The meaning of the camera parameters are described in
# detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.
#
# The image_geometry package provides a user-friendly interface to
# common operations using this meta information. If you want to, e.g.,
# project a 3d point into image coordinates, we strongly recommend
# using image_geometry.
#
# If the camera is uncalibrated, the matrices D, K, R, P should be left
# zeroed out. In particular, clients may assume that K[0] == 0.0
# indicates an uncalibrated camera.

#######################################################################
#                     Image acquisition info                          #
#######################################################################

# Time of image acquisition, camera coordinate frame ID
std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of camera
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into the plane of the image


#######################################################################
#                      Calibration Parameters                         #
#######################################################################
# These are fixed during camera calibration. Their values will be the #
# same in all messages until the camera is recalibrated. Note that    #
# self-calibrating systems may "recalibrate" frequently.              #
#                                                                     #
# The internal parameters can be used to warp a raw (distorted) image #
# to:                                                                 #
#   1. An undistorted image (requires D and K)                        #
#   2. A rectified image (requires D, K, R)                           #
# The projection matrix P projects 3D points into the rectified image.#
#######################################################################

# The image dimensions with which the camera was calibrated.
# Normally this will be the full camera resolution in pixels.
uint32 height
uint32 width

# The distortion model used. Supported models are listed in
# sensor_msgs/distortion_models.hpp. For most cameras, "plumb_bob" - a
# simple model of radial and tangential distortion - is sufficent.
string distortion_model

# The distortion parameters, size depending on the distortion model.
# For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
float64[] d

# Intrinsic camera matrix for the raw (distorted) images.
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]
# Projects 3D points in the camera coordinate frame to 2D pixel
# coordinates using the focal lengths (fx, fy) and principal point
# (cx, cy).
float64[9]  k # 3x3 row-major matrix

# Rectification matrix (stereo cameras only)
# A rotation matrix aligning the camera coordinate system to the ideal
# stereo image plane so that epipolar lines in both stereo images are
# parallel.
float64[9]  r # 3x3 row-major matrix

# Projection/camera matrix
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]
# By convention, this matrix specifies the intrinsic (camera) matrix
#  of the processed (rectified) image. That is, the left 3x3 portion
#  is the normal camera intrinsic matrix for the rectified image.
# It projects 3D points in the camera coordinate frame to 2D pixel
#  coordinates using the focal lengths (fx', fy') and principal point
#  (cx', cy') - these may differ from the values in K.
# For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
#  also have R = the identity and P[1:3,1:3] = K.
# For a stereo pair, the fourth column [Tx Ty 0]' is related to the
#  position of the optical center of the second camera in the first
#  camera's frame. We assume Tz = 0 so both cameras are in the same
#  stereo image plane. The first camera always has Tx = Ty = 0. For
#  the right (second) camera of a horizontal stereo pair, Ty = 0 and
#  Tx = -fx' * B, where B is the baseline between the cameras.
# Given a 3D point [X Y Z]', the projection (x, y) of the point onto
#  the rectified image is given by:
#  [u v w]' = P * [X Y Z 1]'
#         x = u / w
#         y = v / w
#  This holds for both images of a stereo pair.
float64[12] p # 3x4 row-major matrix


#######################################################################
#                      Operational Parameters                         #
#######################################################################
# These define the image region actually captured by the camera       #
# driver. Although they affect the geometry of the output image, they #
# may be changed freely without recalibrating the camera.             #
#######################################################################

# Binning refers here to any camera setting which combines rectangular
#  neighborhoods of pixels into larger "super-pixels." It reduces the
#  resolution of the output image to
#  (width / binning_x) x (height / binning_y).
# The default values binning_x = binning_y = 0 is considered the same
#  as binning_x = binning_y = 1 (no subsampling).
uint32 binning_x
uint32 binning_y

# Region of interest (subwindow of full camera resolution), given in
#  full resolution (unbinned) image coordinates. A particular ROI
#  always denotes the same window of pixels on the camera sensor,
#  regardless of binning settings.
# The default setting of roi (all values 0) is considered the same as
#  full resolution (roi.width = width, roi.height = height).
RegionOfInterest roi
================================================================================
MSG: sensor_msgs/RegionOfInterest
# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the "raw"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct ChannelFloat32 {
        pub r#name: ::std::string::String,
        pub r#values: ::std::vec::Vec<f32>,
    }
    impl ::roslibrust_codegen::RosMessageType for ChannelFloat32 {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/ChannelFloat32";
        const MD5SUM: &'static str = "3d40139cdd33dfedcb71ffeeeb42ae7f";
        const DEFINITION: &'static str = r#"# This message is used by the PointCloud message to hold optional data
# associated with each point in the cloud. The length of the values
# array should be the same as the length of the points array in the
# PointCloud, and each value should be associated with the corresponding
# point.
#
# Channel names in existing practice include:
#   "u", "v" - row and column (respectively) in the left stereo image.
#              This is opposite to usual conventions but remains for
#              historical reasons. The newer PointCloud2 message has no
#              such problem.
#   "rgb" - For point clouds produced by color stereo cameras. uint8
#           (R,G,B) values packed into the least significant 24 bits,
#           in order.
#   "intensity" - laser or pixel intensity.
#   "distance"

# The channel name should give semantics of the channel (e.g.
# "intensity" instead of "value").
string name

# The values array should be 1-1 with the elements of the associated
# PointCloud.
float32[] values"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct CompressedImage {
        pub r#header: std_msgs::Header,
        pub r#format: ::std::string::String,
        #[serde(with = "::roslibrust_codegen::serde_bytes")]
        pub r#data: ::std::vec::Vec<u8>,
    }
    impl ::roslibrust_codegen::RosMessageType for CompressedImage {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/CompressedImage";
        const MD5SUM: &'static str = "1df88053b24348f5f499666c7cb1d980";
        const DEFINITION: &'static str = r#"# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct FluidPressure {
        pub r#header: std_msgs::Header,
        pub r#fluid_pressure: f64,
        pub r#variance: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for FluidPressure {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/FluidPressure";
        const MD5SUM: &'static str = "4967e6ff4dcf72e6b8fca0600661e0b6";
        const DEFINITION: &'static str = r#"# Single pressure reading.  This message is appropriate for measuring the
# pressure inside of a fluid (air, water, etc).  This also includes
# atmospheric or barometric pressure.
#
# This message is not appropriate for force/pressure contact sensors.

std_msgs/Header header # timestamp of the measurement
                             # frame_id is the location of the pressure sensor

float64 fluid_pressure       # Absolute pressure reading in Pascals.

float64 variance             # 0 is interpreted as variance unknown
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Illuminance {
        pub r#header: std_msgs::Header,
        pub r#illuminance: f64,
        pub r#variance: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for Illuminance {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Illuminance";
        const MD5SUM: &'static str = "94ccac1a1be684df74466dfc561512aa";
        const DEFINITION: &'static str = r#"# Single photometric illuminance measurement.  Light should be assumed to be
# measured along the sensor's x-axis (the area of detection is the y-z plane).
# The illuminance should have a 0 or positive value and be received with
# the sensor's +X axis pointing toward the light source.
#
# Photometric illuminance is the measure of the human eye's sensitivity of the
# intensity of light encountering or passing through a surface.
#
# All other Photometric and Radiometric measurements should not use this message.
# This message cannot represent:
#  - Luminous intensity (candela/light source output)
#  - Luminance (nits/light output per area)
#  - Irradiance (watt/area), etc.

std_msgs/Header header # timestamp is the time the illuminance was measured
                             # frame_id is the location and direction of the reading

float64 illuminance          # Measurement of the Photometric Illuminance in Lux.

float64 variance             # 0 is interpreted as variance unknown
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Image {
        pub r#header: std_msgs::Header,
        pub r#height: u32,
        pub r#width: u32,
        pub r#encoding: ::std::string::String,
        pub r#is_bigendian: u8,
        pub r#step: u32,
        #[serde(with = "::roslibrust_codegen::serde_bytes")]
        pub r#data: ::std::vec::Vec<u8>,
    }
    impl ::roslibrust_codegen::RosMessageType for Image {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Image";
        const MD5SUM: &'static str = "9c8b3d25a28b72f070da359dbecf985b";
        const DEFINITION: &'static str = r#"# This message contains an uncompressed image
# (0, 0) is at top-left corner of image

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image
                             # If the frame_id here and the frame_id of the CameraInfo
                             # message associated with the image conflict
                             # the behavior is undefined

uint32 height                # image height, that is, number of rows
uint32 width                 # image width, that is, number of columns

# The legal values for encoding are in file include/sensor_msgs/image_encodings.hpp
# If you want to standardize a new string format, join
# ros-users@lists.ros.org and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.hpp

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an
# orientation estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each
# covariance matrix, and disregard the associated estimate.

std_msgs/Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# This is a message that holds data to describe the state of a set of torque controlled joints.
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state.
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty.
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.

std_msgs/Header header

string[] name
float64[] position
float64[] velocity
float64[] effort
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Joy {
        pub r#header: std_msgs::Header,
        pub r#axes: ::std::vec::Vec<f32>,
        pub r#buttons: ::std::vec::Vec<i32>,
    }
    impl ::roslibrust_codegen::RosMessageType for Joy {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Joy";
        const MD5SUM: &'static str = "967f985c9ca9013a4669430613e3e016";
        const DEFINITION: &'static str = r#"# Reports the state of a joystick's axes and buttons.

# The timestamp is the time at which data is received from the joystick.
std_msgs/Header header

# The axes measurements from a joystick.
float32[] axes

# The buttons measurements from a joystick.
int32[] buttons
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct JoyFeedback {
        pub r#type: u8,
        pub r#id: u8,
        pub r#intensity: f32,
    }
    impl ::roslibrust_codegen::RosMessageType for JoyFeedback {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/JoyFeedback";
        const MD5SUM: &'static str = "f4dcd73460360d98f36e55ee7f2e46f1";
        const DEFINITION: &'static str = r#"# Declare of the type of feedback
uint8 TYPE_LED    = 0
uint8 TYPE_RUMBLE = 1
uint8 TYPE_BUZZER = 2

uint8 type

# This will hold an id number for each type of each feedback.
# Example, the first led would be id=0, the second would be id=1
uint8 id

# Intensity of the feedback, from 0.0 to 1.0, inclusive.  If device is
# actually binary, driver should treat 0<=x<0.5 as off, 0.5<=x<=1 as on.
float32 intensity"#;
    }
    #[allow(unused)]
    impl JoyFeedback {
        pub const r#TYPE_LED: u8 = 0u8;
        pub const r#TYPE_RUMBLE: u8 = 1u8;
        pub const r#TYPE_BUZZER: u8 = 2u8;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct JoyFeedbackArray {
        pub r#array: ::std::vec::Vec<self::JoyFeedback>,
    }
    impl ::roslibrust_codegen::RosMessageType for JoyFeedbackArray {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/JoyFeedbackArray";
        const MD5SUM: &'static str = "cde5730a895b1fc4dee6f91b754b213d";
        const DEFINITION: &'static str = r#"# This message publishes values for multiple feedback at once.
JoyFeedback[] array
================================================================================
MSG: sensor_msgs/JoyFeedback
# Declare of the type of feedback
uint8 TYPE_LED    = 0
uint8 TYPE_RUMBLE = 1
uint8 TYPE_BUZZER = 2

uint8 type

# This will hold an id number for each type of each feedback.
# Example, the first led would be id=0, the second would be id=1
uint8 id

# Intensity of the feedback, from 0.0 to 1.0, inclusive.  If device is
# actually binary, driver should treat 0<=x<0.5 as off, 0.5<=x<=1 as on.
float32 intensity"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct LaserEcho {
        pub r#echoes: ::std::vec::Vec<f32>,
    }
    impl ::roslibrust_codegen::RosMessageType for LaserEcho {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/LaserEcho";
        const MD5SUM: &'static str = "8bc5ae449b200fba4d552b4225586696";
        const DEFINITION: &'static str = r#"# This message is a submessage of MultiEchoLaserScan and is not intended
# to be used separately.

float32[] echoes  # Multiple values of ranges or intensities.
                  # Each array represents data from the same angle increment."#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

std_msgs/Header header # timestamp in the header is the acquisition time of
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around
                             # the positive Z axis (counterclockwise, if Z is up)
                             # with zero angle being forward along the x axis

float32 angle_min            # start angle of the scan [rad]
float32 angle_max            # end angle of the scan [rad]
float32 angle_increment      # angular distance between measurements [rad]

float32 time_increment       # time between measurements [seconds] - if your scanner
                             # is moving, this will be used in interpolating position
                             # of 3d points
float32 scan_time            # time between scans [seconds]

float32 range_min            # minimum range value [m]
float32 range_max            # maximum range value [m]

float32[] ranges             # range data [m]
                             # (Note: values < range_min or > range_max should be discarded)
float32[] intensities        # intensity data [device-specific units].  If your
                             # device does not provide intensities, please leave
                             # the array empty.
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct MagneticField {
        pub r#header: std_msgs::Header,
        pub r#magnetic_field: geometry_msgs::Vector3,
        pub r#magnetic_field_covariance: [f64; 9],
    }
    impl ::roslibrust_codegen::RosMessageType for MagneticField {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/MagneticField";
        const MD5SUM: &'static str = "c8761d20eb9dc59addd882f1d4de2266";
        const DEFINITION: &'static str = r#"# Measurement of the Magnetic Field vector at a specific location.
#
# If the covariance of the measurement is known, it should be filled in.
# If all you know is the variance of each measurement, e.g. from the datasheet,
# just put those along the diagonal.
# A covariance matrix of all zeros will be interpreted as "covariance unknown",
# and to use the data a covariance will have to be assumed or gotten from some
# other source.

std_msgs/Header header               # timestamp is the time the
                                           # field was measured
                                           # frame_id is the location and orientation
                                           # of the field measurement

geometry_msgs/Vector3 magnetic_field # x, y, and z components of the
                                           # field vector in Tesla
                                           # If your sensor does not output 3 axes,
                                           # put NaNs in the components not reported.

float64[9] magnetic_field_covariance       # Row major about x, y, z axes
                                           # 0 is interpreted as variance unknown
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# Representation of state for joints with multiple degrees of freedom,
# following the structure of JointState which can only represent a single degree of freedom.
#
# It is assumed that a joint in a system corresponds to a transform that gets applied
# along the kinematic chain. For example, a planar joint (as in URDF) is 3DOF (x, y, yaw)
# and those 3DOF can be expressed as a transformation matrix, and that transformation
# matrix can be converted back to (x, y, yaw)
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state.
# The goal is to make each of the fields optional. When e.g. your joints have no
# wrench associated with them, you can leave the wrench array empty.
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.

std_msgs/Header header

string[] joint_names
geometry_msgs/Transform[] transforms
geometry_msgs/Twist[] twist
geometry_msgs/Wrench[] wrench
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into its linear and angular parts.

Vector3  force
Vector3  torque
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# Single scan from a multi-echo planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

std_msgs/Header header # timestamp in the header is the acquisition time of
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around
                             # the positive Z axis (counterclockwise, if Z is up)
                             # with zero angle being forward along the x axis

float32 angle_min            # start angle of the scan [rad]
float32 angle_max            # end angle of the scan [rad]
float32 angle_increment      # angular distance between measurements [rad]

float32 time_increment       # time between measurements [seconds] - if your scanner
                             # is moving, this will be used in interpolating position
                             # of 3d points
float32 scan_time            # time between scans [seconds]

float32 range_min            # minimum range value [m]
float32 range_max            # maximum range value [m]

LaserEcho[] ranges           # range data [m]
                             # (Note: NaNs, values < range_min or > range_max should be discarded)
                             # +Inf measurements are out of range
                             # -Inf measurements are too close to determine exact distance.
LaserEcho[] intensities      # intensity data [device-specific units].  If your
                             # device does not provide intensities, please leave
                             # the array empty.
================================================================================
MSG: sensor_msgs/LaserEcho
# This message is a submessage of MultiEchoLaserScan and is not intended
# to be used separately.

float32[] echoes  # Multiple values of ranges or intensities.
                  # Each array represents data from the same angle increment.
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# Navigation Satellite fix for any Global Navigation Satellite System
#
# Specified using the WGS 84 reference ellipsoid

# header.stamp specifies the ROS time for this measurement (the
#        corresponding satellite time may be reported using the
#        sensor_msgs/TimeReference message).
#
# header.frame_id is the frame of reference reported by the satellite
#        receiver, usually the location of the antenna.  This is a
#        Euclidean frame relative to the vehicle, not a reference
#        ellipsoid.
std_msgs/Header header

# Satellite fix status information.
NavSatStatus status

# Latitude [degrees]. Positive is north of equator; negative is south.
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is west.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid
# (quiet NaN if no altitude is available).
float64 altitude

# Position covariance [m^2] defined relative to a tangential plane
# through the reported position. The components are East, North, and
# Up (ENU), in row-major order.
#
# Beware: this coordinate system exhibits singularities at the poles.
float64[9] position_covariance

# If the covariance of the fix is known, fill it in completely. If the
# GPS receiver provides the variance of each measurement, put them
# along the diagonal. If only Dilution of Precision is available,
# estimate an approximate covariance from that.

uint8 COVARIANCE_TYPE_UNKNOWN = 0
uint8 COVARIANCE_TYPE_APPROXIMATED = 1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
uint8 COVARIANCE_TYPE_KNOWN = 3

uint8 position_covariance_type
================================================================================
MSG: sensor_msgs/NavSatStatus
# Navigation Satellite fix status for any Global Navigation Satellite System.
#
# Whether to output an augmented fix is determined by both the fix
# type and the last time differential corrections were received.  A
# fix is valid when status >= STATUS_FIX.

int8 STATUS_NO_FIX =  -1        # unable to fix position
int8 STATUS_FIX =      0        # unaugmented fix
int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation

int8 status

# Bits defining which Global Navigation Satellite System signals were
# used by the receiver.

uint16 SERVICE_GPS =     1
uint16 SERVICE_GLONASS = 2
uint16 SERVICE_COMPASS = 4      # includes BeiDou.
uint16 SERVICE_GALILEO = 8

uint16 service
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(unused)]
    impl NavSatFix {
        pub const r#COVARIANCE_TYPE_UNKNOWN: u8 = 0u8;
        pub const r#COVARIANCE_TYPE_APPROXIMATED: u8 = 1u8;
        pub const r#COVARIANCE_TYPE_DIAGONAL_KNOWN: u8 = 2u8;
        pub const r#COVARIANCE_TYPE_KNOWN: u8 = 3u8;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct NavSatStatus {
        pub r#status: i8,
        pub r#service: u16,
    }
    impl ::roslibrust_codegen::RosMessageType for NavSatStatus {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/NavSatStatus";
        const MD5SUM: &'static str = "331cdbddfa4bc96ffc3b9ad98900a54c";
        const DEFINITION: &'static str = r#"# Navigation Satellite fix status for any Global Navigation Satellite System.
#
# Whether to output an augmented fix is determined by both the fix
# type and the last time differential corrections were received.  A
# fix is valid when status >= STATUS_FIX.

int8 STATUS_NO_FIX =  -1        # unable to fix position
int8 STATUS_FIX =      0        # unaugmented fix
int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation

int8 status

# Bits defining which Global Navigation Satellite System signals were
# used by the receiver.

uint16 SERVICE_GPS =     1
uint16 SERVICE_GLONASS = 2
uint16 SERVICE_COMPASS = 4      # includes BeiDou.
uint16 SERVICE_GALILEO = 8

uint16 service"#;
    }
    #[allow(unused)]
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
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct PointCloud {
        pub r#header: std_msgs::Header,
        pub r#points: ::std::vec::Vec<geometry_msgs::Point32>,
        pub r#channels: ::std::vec::Vec<self::ChannelFloat32>,
    }
    impl ::roslibrust_codegen::RosMessageType for PointCloud {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/PointCloud";
        const MD5SUM: &'static str = "95c9c548e015c235b38b961c79973db7";
        const DEFINITION: &'static str = r#"## THIS MESSAGE IS DEPRECATED AS OF FOXY
## Please use sensor_msgs/PointCloud2

# This message holds a collection of 3d points, plus optional additional
# information about each point.

# Time of sensor data acquisition, coordinate frame ID.
std_msgs/Header header

# Array of 3d points. Each Point32 should be interpreted as a 3d point
# in the frame given in the header.
geometry_msgs/Point32[] points

# Each channel should have the same number of elements as points array,
# and the data in each channel should correspond 1:1 with each point.
# Channel names in common practice are listed in ChannelFloat32.msg.
ChannelFloat32[] channels
================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommended to use Point wherever possible instead of Point32.
#
# This recommendation is to promote interoperability.
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.

float32 x
float32 y
float32 z
================================================================================
MSG: sensor_msgs/ChannelFloat32
# This message is used by the PointCloud message to hold optional data
# associated with each point in the cloud. The length of the values
# array should be the same as the length of the points array in the
# PointCloud, and each value should be associated with the corresponding
# point.
#
# Channel names in existing practice include:
#   "u", "v" - row and column (respectively) in the left stereo image.
#              This is opposite to usual conventions but remains for
#              historical reasons. The newer PointCloud2 message has no
#              such problem.
#   "rgb" - For point clouds produced by color stereo cameras. uint8
#           (R,G,B) values packed into the least significant 24 bits,
#           in order.
#   "intensity" - laser or pixel intensity.
#   "distance"

# The channel name should give semantics of the channel (e.g.
# "intensity" instead of "value").
string name

# The values array should be 1-1 with the elements of the associated
# PointCloud.
float32[] values
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct PointCloud2 {
        pub r#header: std_msgs::Header,
        pub r#height: u32,
        pub r#width: u32,
        pub r#fields: ::std::vec::Vec<self::PointField>,
        pub r#is_bigendian: bool,
        pub r#point_step: u32,
        pub r#row_step: u32,
        #[serde(with = "::roslibrust_codegen::serde_bytes")]
        pub r#data: ::std::vec::Vec<u8>,
        pub r#is_dense: bool,
    }
    impl ::roslibrust_codegen::RosMessageType for PointCloud2 {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/PointCloud2";
        const MD5SUM: &'static str = "c61ffb665fe19735825e4dd31b53913d";
        const DEFINITION: &'static str = r#"# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.
#
# The point cloud data may be organized 2d (image-like) or 1d (unordered).
# Point clouds organized as 2d images may be produced by camera depth sensors
# such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d points).
std_msgs/Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points
================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

# Common PointField names are x, y, z, intensity, rgb, rgba
string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct PointField {
        pub r#name: ::std::string::String,
        pub r#offset: u32,
        pub r#datatype: u8,
        pub r#count: u32,
    }
    impl ::roslibrust_codegen::RosMessageType for PointField {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/PointField";
        const MD5SUM: &'static str = "268eacb2962780ceac86cbd17e328150";
        const DEFINITION: &'static str = r#"# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

# Common PointField names are x, y, z, intensity, rgb, rgba
string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field"#;
    }
    #[allow(unused)]
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
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# Single range reading from an active ranger that emits energy and reports
# one range reading that is valid along an arc at the distance measured.
# This message is  not appropriate for laser scanners. See the LaserScan
# message if you are working with a laser scanner.
#
# This message also can represent a fixed-distance (binary) ranger.  This
# sensor will have min_range===max_range===distance of detection.
# These sensors follow REP 117 and will output -Inf if the object is detected
# and +Inf if the object is outside of the detection range.

std_msgs/Header header # timestamp in the header is the time the ranger
                             # returned the distance reading

# Radiation type enums
# If you want a value added to this list, send an email to the ros-users list
uint8 ULTRASOUND=0
uint8 INFRARED=1

uint8 radiation_type    # the type of radiation used by the sensor
                        # (sound, IR, etc) [enum]

float32 field_of_view   # the size of the arc that the distance reading is
                        # valid for [rad]
                        # the object causing the range reading may have
                        # been anywhere within -field_of_view/2 and
                        # field_of_view/2 at the measured range.
                        # 0 angle corresponds to the x-axis of the sensor.

float32 min_range       # minimum range value [m]
float32 max_range       # maximum range value [m]
                        # Fixed distance rangers require min_range==max_range

float32 range           # range data [m]
                        # (Note: values < range_min or > range_max should be discarded)
                        # Fixed distance rangers only output -Inf or +Inf.
                        # -Inf represents a detection within fixed distance.
                        # (Detection too close to the sensor to quantify)
                        # +Inf represents no detection within the fixed distance.
                        # (Object out of range)
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(unused)]
    impl Range {
        pub const r#ULTRASOUND: u8 = 0u8;
        pub const r#INFRARED: u8 = 1u8;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the "raw"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct RelativeHumidity {
        pub r#header: std_msgs::Header,
        pub r#relative_humidity: f64,
        pub r#variance: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for RelativeHumidity {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/RelativeHumidity";
        const MD5SUM: &'static str = "71cfefa31dcc94f47083b1e89e6fa5c9";
        const DEFINITION: &'static str = r#"# Single reading from a relative humidity sensor.
# Defines the ratio of partial pressure of water vapor to the saturated vapor
# pressure at a temperature.

std_msgs/Header header # timestamp of the measurement
                             # frame_id is the location of the humidity sensor

float64 relative_humidity    # Expression of the relative humidity
                             # from 0.0 to 1.0.
                             # 0.0 is no partial pressure of water vapor
                             # 1.0 represents partial pressure of saturation

float64 variance             # 0 is interpreted as variance unknown
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Temperature {
        pub r#header: std_msgs::Header,
        pub r#temperature: f64,
        pub r#variance: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for Temperature {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/Temperature";
        const MD5SUM: &'static str = "c6df0674fcfebff84a15927a80ebb14b";
        const DEFINITION: &'static str = r#"# Single temperature reading.

std_msgs/Header header # timestamp is the time the temperature was measured
                             # frame_id is the location of the temperature reading

float64 temperature          # Measurement of the Temperature in Degrees Celsius.

float64 variance             # 0 is interpreted as variance unknown.
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct TimeReference {
        pub r#header: std_msgs::Header,
        pub r#time_ref: ::roslibrust_codegen::integral_types::Time,
        pub r#source: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for TimeReference {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/TimeReference";
        const MD5SUM: &'static str = "7cb7ae5aa838323e9028637e304e0ad7";
        const DEFINITION: &'static str = r#"# Measurement from an external time source not actively synchronized with the system clock.

std_msgs/Header header      # stamp is system time for which measurement was valid
                                  # frame_id is not used

builtin_interfaces/Time time_ref  # corresponding time from this external source
string source                     # (optional) name of time source
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct SetCameraInfoRequest {
        pub r#camera_info: self::CameraInfo,
    }
    impl ::roslibrust_codegen::RosMessageType for SetCameraInfoRequest {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/SetCameraInfoRequest";
        const MD5SUM: &'static str = "251c96e357751cc7c699c496178141d5";
        const DEFINITION: &'static str = r#"# This service requests that a camera stores the given CameraInfo as that
# camera's calibration information.
#
# The width and height in the camera_info field should match what the
# camera is currently outputting on its camera_info topic, and the camera
# will assume that the region of the imager that is being referred to is
# the region that the camera is currently capturing.

sensor_msgs/CameraInfo camera_info # The camera_info to store
================================================================================
MSG: sensor_msgs/CameraInfo
# This message defines meta information for a camera. It should be in a
# camera namespace on topic "camera_info" and accompanied by up to five
# image topics named:
#
#   image_raw - raw data from the camera driver, possibly Bayer encoded
#   image            - monochrome, distorted
#   image_color      - color, distorted
#   image_rect       - monochrome, rectified
#   image_rect_color - color, rectified
#
# The image_pipeline contains packages (image_proc, stereo_image_proc)
# for producing the four processed image topics from image_raw and
# camera_info. The meaning of the camera parameters are described in
# detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.
#
# The image_geometry package provides a user-friendly interface to
# common operations using this meta information. If you want to, e.g.,
# project a 3d point into image coordinates, we strongly recommend
# using image_geometry.
#
# If the camera is uncalibrated, the matrices D, K, R, P should be left
# zeroed out. In particular, clients may assume that K[0] == 0.0
# indicates an uncalibrated camera.

#######################################################################
#                     Image acquisition info                          #
#######################################################################

# Time of image acquisition, camera coordinate frame ID
std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of camera
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into the plane of the image


#######################################################################
#                      Calibration Parameters                         #
#######################################################################
# These are fixed during camera calibration. Their values will be the #
# same in all messages until the camera is recalibrated. Note that    #
# self-calibrating systems may "recalibrate" frequently.              #
#                                                                     #
# The internal parameters can be used to warp a raw (distorted) image #
# to:                                                                 #
#   1. An undistorted image (requires D and K)                        #
#   2. A rectified image (requires D, K, R)                           #
# The projection matrix P projects 3D points into the rectified image.#
#######################################################################

# The image dimensions with which the camera was calibrated.
# Normally this will be the full camera resolution in pixels.
uint32 height
uint32 width

# The distortion model used. Supported models are listed in
# sensor_msgs/distortion_models.hpp. For most cameras, "plumb_bob" - a
# simple model of radial and tangential distortion - is sufficent.
string distortion_model

# The distortion parameters, size depending on the distortion model.
# For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
float64[] d

# Intrinsic camera matrix for the raw (distorted) images.
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]
# Projects 3D points in the camera coordinate frame to 2D pixel
# coordinates using the focal lengths (fx, fy) and principal point
# (cx, cy).
float64[9]  k # 3x3 row-major matrix

# Rectification matrix (stereo cameras only)
# A rotation matrix aligning the camera coordinate system to the ideal
# stereo image plane so that epipolar lines in both stereo images are
# parallel.
float64[9]  r # 3x3 row-major matrix

# Projection/camera matrix
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]
# By convention, this matrix specifies the intrinsic (camera) matrix
#  of the processed (rectified) image. That is, the left 3x3 portion
#  is the normal camera intrinsic matrix for the rectified image.
# It projects 3D points in the camera coordinate frame to 2D pixel
#  coordinates using the focal lengths (fx', fy') and principal point
#  (cx', cy') - these may differ from the values in K.
# For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
#  also have R = the identity and P[1:3,1:3] = K.
# For a stereo pair, the fourth column [Tx Ty 0]' is related to the
#  position of the optical center of the second camera in the first
#  camera's frame. We assume Tz = 0 so both cameras are in the same
#  stereo image plane. The first camera always has Tx = Ty = 0. For
#  the right (second) camera of a horizontal stereo pair, Ty = 0 and
#  Tx = -fx' * B, where B is the baseline between the cameras.
# Given a 3D point [X Y Z]', the projection (x, y) of the point onto
#  the rectified image is given by:
#  [u v w]' = P * [X Y Z 1]'
#         x = u / w
#         y = v / w
#  This holds for both images of a stereo pair.
float64[12] p # 3x4 row-major matrix


#######################################################################
#                      Operational Parameters                         #
#######################################################################
# These define the image region actually captured by the camera       #
# driver. Although they affect the geometry of the output image, they #
# may be changed freely without recalibrating the camera.             #
#######################################################################

# Binning refers here to any camera setting which combines rectangular
#  neighborhoods of pixels into larger "super-pixels." It reduces the
#  resolution of the output image to
#  (width / binning_x) x (height / binning_y).
# The default values binning_x = binning_y = 0 is considered the same
#  as binning_x = binning_y = 1 (no subsampling).
uint32 binning_x
uint32 binning_y

# Region of interest (subwindow of full camera resolution), given in
#  full resolution (unbinned) image coordinates. A particular ROI
#  always denotes the same window of pixels on the camera sensor,
#  regardless of binning settings.
# The default setting of roi (all values 0) is considered the same as
#  full resolution (roi.width = width, roi.height = height).
RegionOfInterest roi
================================================================================
MSG: sensor_msgs/RegionOfInterest
# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the "raw"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: sensor_msgs/RegionOfInterest
# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the "raw"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct SetCameraInfoResponse {
        pub r#success: bool,
        pub r#status_message: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for SetCameraInfoResponse {
        const ROS_TYPE_NAME: &'static str = "sensor_msgs/SetCameraInfoResponse";
        const MD5SUM: &'static str = "2ec6f3eff0161f4257b808b12bc830c2";
        const DEFINITION: &'static str = r#"bool success                             # True if the call succeeded
string status_message                    # Used to give details about success"#;
    }
    #[allow(dead_code)]
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
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Mesh {
        pub r#triangles: ::std::vec::Vec<self::MeshTriangle>,
        pub r#vertices: ::std::vec::Vec<geometry_msgs::Point>,
    }
    impl ::roslibrust_codegen::RosMessageType for Mesh {
        const ROS_TYPE_NAME: &'static str = "shape_msgs/Mesh";
        const MD5SUM: &'static str = "1ffdae9486cd3316a121c578b47a85cc";
        const DEFINITION: &'static str = r#"# Definition of a mesh.

# List of triangles; the index values refer to positions in vertices[].
MeshTriangle[] triangles

# The actual vertices that make up the mesh.
geometry_msgs/Point[] vertices
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: shape_msgs/MeshTriangle
# Definition of a triangle's vertices.

uint32[3] vertex_indices"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct MeshTriangle {
        pub r#vertex_indices: [u32; 3],
    }
    impl ::roslibrust_codegen::RosMessageType for MeshTriangle {
        const ROS_TYPE_NAME: &'static str = "shape_msgs/MeshTriangle";
        const MD5SUM: &'static str = "23688b2e6d2de3d32fe8af104a903253";
        const DEFINITION: &'static str = r#"# Definition of a triangle's vertices.

uint32[3] vertex_indices"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Plane {
        pub r#coef: [f64; 4],
    }
    impl ::roslibrust_codegen::RosMessageType for Plane {
        const ROS_TYPE_NAME: &'static str = "shape_msgs/Plane";
        const MD5SUM: &'static str = "2c1b92ed8f31492f8e73f6a4a44ca796";
        const DEFINITION: &'static str = r#"# Representation of a plane, using the plane equation ax + by + cz + d = 0.
#
# a := coef[0]
# b := coef[1]
# c := coef[2]
# d := coef[3]
float64[4] coef"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct SolidPrimitive {
        pub r#type: u8,
        pub r#dimensions: [f64; 0],
        pub r#polygon: geometry_msgs::Polygon,
    }
    impl ::roslibrust_codegen::RosMessageType for SolidPrimitive {
        const ROS_TYPE_NAME: &'static str = "shape_msgs/SolidPrimitive";
        const MD5SUM: &'static str = "0cdf91a0a45ccd7bc1e0deb784cb2958";
        const DEFINITION: &'static str = r#"# Defines box, sphere, cylinder, cone and prism.
# All shapes are defined to have their bounding boxes centered around 0,0,0.

uint8 BOX=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 CONE=4
uint8 PRISM=5

# The type of the shape
uint8 type

# The dimensions of the shape
float64[<=3] dimensions  # At no point will dimensions have a length > 3.

# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array.

# For type BOX, the X, Y, and Z dimensions are the length of the corresponding sides of the box.
uint8 BOX_X=0
uint8 BOX_Y=1
uint8 BOX_Z=2

# For the SPHERE type, only one component is used, and it gives the radius of the sphere.
uint8 SPHERE_RADIUS=0

# For the CYLINDER and CONE types, the center line is oriented along the Z axis.
# Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component of dimensions gives the
# height of the cylinder (cone).
# The CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the radius of
# the base of the cylinder (cone).
# Cone and cylinder primitives are defined to be circular. The tip of the cone
# is pointing up, along +Z axis.

uint8 CYLINDER_HEIGHT=0
uint8 CYLINDER_RADIUS=1

uint8 CONE_HEIGHT=0
uint8 CONE_RADIUS=1

# For the type PRISM, the center line is oriented along Z axis.
# The PRISM_HEIGHT component of dimensions gives the
# height of the prism.
# The polygon defines the Z axis centered base of the prism.
# The prism is constructed by extruding the base in +Z and -Z
# directions by half of the PRISM_HEIGHT
# Only x and y fields of the points are used in the polygon.
# Points of the polygon are ordered counter-clockwise.

uint8 PRISM_HEIGHT=0
geometry_msgs/Polygon polygon
================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommended to use Point wherever possible instead of Point32.
#
# This recommendation is to promote interoperability.
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.

float32 x
float32 y
float32 z
================================================================================
MSG: geometry_msgs/Polygon
# A specification of a polygon where the first and last points are assumed to be connected

Point32[] points
================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommended to use Point wherever possible instead of Point32.
#
# This recommendation is to promote interoperability.
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.

float32 x
float32 y
float32 z"#;
    }
    #[allow(unused)]
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
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Bool {
        pub r#data: bool,
    }
    impl ::roslibrust_codegen::RosMessageType for Bool {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Bool";
        const MD5SUM: &'static str = "8b94c1b53db61fb6aed406028ad6332a";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

bool data"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Byte {
        pub r#data: u8,
    }
    impl ::roslibrust_codegen::RosMessageType for Byte {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Byte";
        const MD5SUM: &'static str = "ad736a2e8818154c487bb80fe42ce43b";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

byte data"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct ByteMultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<u8>,
    }
    impl ::roslibrust_codegen::RosMessageType for ByteMultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/ByteMultiArray";
        const MD5SUM: &'static str = "70ea476cbcfd65ac2f68f3cda1e891fe";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
byte[]            data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Char {
        pub r#data: u8,
    }
    impl ::roslibrust_codegen::RosMessageType for Char {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Char";
        const MD5SUM: &'static str = "1bf77f25acecdedba0e224b162199717";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

char data"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct ColorRGBA {
        pub r#r: f32,
        pub r#g: f32,
        pub r#b: f32,
        pub r#a: f32,
    }
    impl ::roslibrust_codegen::RosMessageType for ColorRGBA {
        const ROS_TYPE_NAME: &'static str = "std_msgs/ColorRGBA";
        const MD5SUM: &'static str = "a29a96539573343b1310c73607334b00";
        const DEFINITION: &'static str = r#"float32 r
float32 g
float32 b
float32 a"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Empty {}
    impl ::roslibrust_codegen::RosMessageType for Empty {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Empty";
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
        const DEFINITION: &'static str = r#""#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Float32 {
        pub r#data: f32,
    }
    impl ::roslibrust_codegen::RosMessageType for Float32 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Float32";
        const MD5SUM: &'static str = "73fcbf46b49191e672908e50842a83d4";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

float32 data"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Float32MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<f32>,
    }
    impl ::roslibrust_codegen::RosMessageType for Float32MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Float32MultiArray";
        const MD5SUM: &'static str = "6a40e0ffa6a17a503ac3f8616991b1f6";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float32[]         data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Float64 {
        pub r#data: f64,
    }
    impl ::roslibrust_codegen::RosMessageType for Float64 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Float64";
        const MD5SUM: &'static str = "fdb28210bfa9d7c91146260178d9a584";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

float64 data"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Float64MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<f64>,
    }
    impl ::roslibrust_codegen::RosMessageType for Float64MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Float64MultiArray";
        const MD5SUM: &'static str = "4b7d974086d4060e7db4613a7e6c3ba4";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Header {
        pub r#stamp: ::roslibrust_codegen::integral_types::Time,
        pub r#frame_id: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for Header {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Header";
        const MD5SUM: &'static str = "5ed6b5dd1ef879ffb9c2ac51bab61a63";
        const DEFINITION: &'static str = r#"# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Int16 {
        pub r#data: i16,
    }
    impl ::roslibrust_codegen::RosMessageType for Int16 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int16";
        const MD5SUM: &'static str = "8524586e34fbd7cb1c08c5f5f1ca0e57";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

int16 data"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Int16MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<i16>,
    }
    impl ::roslibrust_codegen::RosMessageType for Int16MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int16MultiArray";
        const MD5SUM: &'static str = "d9338d7f523fcb692fae9d0a0e9f067c";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
int16[]           data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Int32 {
        pub r#data: i32,
    }
    impl ::roslibrust_codegen::RosMessageType for Int32 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int32";
        const MD5SUM: &'static str = "da5909fbe378aeaf85e547e830cc1bb7";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

int32 data"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Int32MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<i32>,
    }
    impl ::roslibrust_codegen::RosMessageType for Int32MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int32MultiArray";
        const MD5SUM: &'static str = "1d99f79f8b325b44fee908053e9c945b";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
int32[]           data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Int64 {
        pub r#data: i64,
    }
    impl ::roslibrust_codegen::RosMessageType for Int64 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int64";
        const MD5SUM: &'static str = "34add168574510e6e17f5d23ecc077ef";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

int64 data"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Int64MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<i64>,
    }
    impl ::roslibrust_codegen::RosMessageType for Int64MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int64MultiArray";
        const MD5SUM: &'static str = "54865aa6c65be0448113a2afc6a49270";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
int64[]           data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Int8 {
        pub r#data: i8,
    }
    impl ::roslibrust_codegen::RosMessageType for Int8 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int8";
        const MD5SUM: &'static str = "27ffa0c9c4b8fb8492252bcad9e5c57b";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

int8 data"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct Int8MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<i8>,
    }
    impl ::roslibrust_codegen::RosMessageType for Int8MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/Int8MultiArray";
        const MD5SUM: &'static str = "d7c1af35a1b4781bbe79e03dd94b7c13";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
int8[]            data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct MultiArrayDimension {
        pub r#label: ::std::string::String,
        pub r#size: u32,
        pub r#stride: u32,
    }
    impl ::roslibrust_codegen::RosMessageType for MultiArrayDimension {
        const ROS_TYPE_NAME: &'static str = "std_msgs/MultiArrayDimension";
        const MD5SUM: &'static str = "4cd0c83a8683deae40ecdac60e53bfa8";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct MultiArrayLayout {
        pub r#dim: ::std::vec::Vec<self::MultiArrayDimension>,
        pub r#data_offset: u32,
    }
    impl ::roslibrust_codegen::RosMessageType for MultiArrayLayout {
        const ROS_TYPE_NAME: &'static str = "std_msgs/MultiArrayLayout";
        const MD5SUM: &'static str = "0fed2a11c13e11c5571b4e2a995a91a3";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct String {
        pub r#data: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for String {
        const ROS_TYPE_NAME: &'static str = "std_msgs/String";
        const MD5SUM: &'static str = "992ce8a1687cec8c8bd883ec73ca41d1";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string data"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct UInt16 {
        pub r#data: u16,
    }
    impl ::roslibrust_codegen::RosMessageType for UInt16 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt16";
        const MD5SUM: &'static str = "1df79edf208b629fe6b81923a544552d";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

uint16 data"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct UInt16MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<u16>,
    }
    impl ::roslibrust_codegen::RosMessageType for UInt16MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt16MultiArray";
        const MD5SUM: &'static str = "52f264f1c973c4b73790d384c6cb4484";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint16[]            data        # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct UInt32 {
        pub r#data: u32,
    }
    impl ::roslibrust_codegen::RosMessageType for UInt32 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt32";
        const MD5SUM: &'static str = "304a39449588c7f8ce2df6e8001c5fce";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

uint32 data"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct UInt32MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<u32>,
    }
    impl ::roslibrust_codegen::RosMessageType for UInt32MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt32MultiArray";
        const MD5SUM: &'static str = "4d6a180abc9be191b96a7eda6c8a233d";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint32[]          data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct UInt64 {
        pub r#data: u64,
    }
    impl ::roslibrust_codegen::RosMessageType for UInt64 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt64";
        const MD5SUM: &'static str = "1b2a79973e8bf53d7b53acb71299cb57";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

uint64 data"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct UInt64MultiArray {
        pub r#layout: self::MultiArrayLayout,
        pub r#data: ::std::vec::Vec<u64>,
    }
    impl ::roslibrust_codegen::RosMessageType for UInt64MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt64MultiArray";
        const MD5SUM: &'static str = "6088f127afb1d6c72927aa1247e945af";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint64[]          data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct UInt8 {
        pub r#data: u8,
    }
    impl ::roslibrust_codegen::RosMessageType for UInt8 {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt8";
        const MD5SUM: &'static str = "7c8164229e7d2c17eb95e9231617fdee";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

uint8 data"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct UInt8MultiArray {
        pub r#layout: self::MultiArrayLayout,
        #[serde(with = "::roslibrust_codegen::serde_bytes")]
        pub r#data: ::std::vec::Vec<u8>,
    }
    impl ::roslibrust_codegen::RosMessageType for UInt8MultiArray {
        const ROS_TYPE_NAME: &'static str = "std_msgs/UInt8MultiArray";
        const MD5SUM: &'static str = "82373f1612381bb6ee473b5cd6f5d89c";
        const DEFINITION: &'static str = r#"# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint8[]           data          # array of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/MultiArrayLayout
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.
#
# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
#
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding bytes at front of data
================================================================================
MSG: std_msgs/MultiArrayDimension
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension"#;
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
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct EmptyRequest {}
    impl ::roslibrust_codegen::RosMessageType for EmptyRequest {
        const ROS_TYPE_NAME: &'static str = "std_srvs/EmptyRequest";
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
        const DEFINITION: &'static str = r#""#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct EmptyResponse {}
    impl ::roslibrust_codegen::RosMessageType for EmptyResponse {
        const ROS_TYPE_NAME: &'static str = "std_srvs/EmptyResponse";
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
        const DEFINITION: &'static str = r#""#;
    }
    #[allow(dead_code)]
    pub struct Empty {}
    impl ::roslibrust_codegen::RosServiceType for Empty {
        const ROS_SERVICE_NAME: &'static str = "std_srvs/Empty";
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
        type Request = EmptyRequest;
        type Response = EmptyResponse;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct SetBoolRequest {
        pub r#data: bool,
    }
    impl ::roslibrust_codegen::RosMessageType for SetBoolRequest {
        const ROS_TYPE_NAME: &'static str = "std_srvs/SetBoolRequest";
        const MD5SUM: &'static str = "8b94c1b53db61fb6aed406028ad6332a";
        const DEFINITION: &'static str = r#"bool data # e.g. for hardware enabling / disabling"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct SetBoolResponse {
        pub r#success: bool,
        pub r#message: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for SetBoolResponse {
        const ROS_TYPE_NAME: &'static str = "std_srvs/SetBoolResponse";
        const MD5SUM: &'static str = "937c9679a518e3a18d831e57125ea522";
        const DEFINITION: &'static str = r#"bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages"#;
    }
    #[allow(dead_code)]
    pub struct SetBool {}
    impl ::roslibrust_codegen::RosServiceType for SetBool {
        const ROS_SERVICE_NAME: &'static str = "std_srvs/SetBool";
        const MD5SUM: &'static str = "09fb03525b03e7ea1fd3992bafd87e16";
        type Request = SetBoolRequest;
        type Response = SetBoolResponse;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct TriggerRequest {}
    impl ::roslibrust_codegen::RosMessageType for TriggerRequest {
        const ROS_TYPE_NAME: &'static str = "std_srvs/TriggerRequest";
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
        const DEFINITION: &'static str = r#""#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct TriggerResponse {
        pub r#success: bool,
        pub r#message: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for TriggerResponse {
        const ROS_TYPE_NAME: &'static str = "std_srvs/TriggerResponse";
        const MD5SUM: &'static str = "937c9679a518e3a18d831e57125ea522";
        const DEFINITION: &'static str = r#"bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages"#;
    }
    #[allow(dead_code)]
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
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# Separate header for compatibility with current TimeSynchronizer.
# Likely to be removed in a later release, use image.header instead.
std_msgs/Header header

# Floating point disparity image. The disparities are pre-adjusted for any
# x-offset between the principal points of the two cameras (in the case
# that they are verged). That is: d = x_l - x_r - (cx_l - cx_r)
sensor_msgs/Image image

# Stereo geometry. For disparity d, the depth from the camera is Z = fT/d.
float32 f # Focal length, pixels
float32 t # Baseline, world units

# Subwindow of (potentially) valid disparity values.
sensor_msgs/RegionOfInterest valid_window

# The range of disparities searched.
# In the disparity image, any disparity less than min_disparity is invalid.
# The disparity search range defines the horopter, or 3D volume that the
# stereo algorithm can "see". Points with Z outside of:
#     Z_min = fT / max_disparity
#     Z_max = fT / min_disparity
# could not be found.
float32 min_disparity
float32 max_disparity

# Smallest allowed disparity increment. The smallest achievable depth range
# resolution is delta_Z = (Z^2/fT)*delta_d.
float32 delta_d
================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image
                             # If the frame_id here and the frame_id of the CameraInfo
                             # message associated with the image conflict
                             # the behavior is undefined

uint32 height                # image height, that is, number of rows
uint32 width                 # image width, that is, number of columns

# The legal values for encoding are in file include/sensor_msgs/image_encodings.hpp
# If you want to standardize a new string format, join
# ros-users@lists.ros.org and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.hpp

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: sensor_msgs/RegionOfInterest
# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the "raw"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
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
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# This message is specifically for testing generating of default values
# Examples based on https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html
uint8 x 42
int16 y -2000
string full_name "John Doe"
int32[] samples [-200, -100, 0, 100, 200]

# More complicated examples to stress the system, floats with mixed precision
float32[] f_samples [-200, -1.0, 0]
string[] s_vec ["hello", "world"]
# This may or may not be valid ROS, it probably is, but we don't handle yet
# TODO handle this somehow
# string[] s_vec_2 ['hello', 'world']

# TODO ROS says this is valid, but we currently don't handle
#string single_quote 'Jane Doe'"#;
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
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct JointTrajectory {
        pub r#header: std_msgs::Header,
        pub r#joint_names: ::std::vec::Vec<::std::string::String>,
        pub r#points: ::std::vec::Vec<self::JointTrajectoryPoint>,
    }
    impl ::roslibrust_codegen::RosMessageType for JointTrajectory {
        const ROS_TYPE_NAME: &'static str = "trajectory_msgs/JointTrajectory";
        const MD5SUM: &'static str = "d63e3b4556d9dbd9f48b5ab4a03f1fee";
        const DEFINITION: &'static str = r#"# The header is used to specify the coordinate frame and the reference time for
# the trajectory durations
std_msgs/Header header

# The names of the active joints in each trajectory point. These names are
# ordered and must correspond to the values in each trajectory point.
string[] joint_names

# Array of trajectory points, which describe the positions, velocities,
# accelerations and/or efforts of the joints at each time point.
JointTrajectoryPoint[] points
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg.

# Single DOF joint positions for each joint relative to their "0" position.
# The units depend on the specific joint type: radians for revolute or
# continuous joints, and meters for prismatic joints.
float64[] positions

# The rate of change in position of each joint. Units are joint type dependent.
# Radians/second for revolute or continuous joints, and meters/second for
# prismatic joints.
float64[] velocities

# Rate of change in velocity of each joint. Units are joint type dependent.
# Radians/second^2 for revolute or continuous joints, and meters/second^2 for
# prismatic joints.
float64[] accelerations

# The torque or the force to be applied at each joint. For revolute/continuous
# joints effort denotes a torque in newton-meters. For prismatic joints, effort
# denotes a force in newtons.
float64[] effort

# Desired time from the trajectory start to arrive at this trajectory point.
builtin_interfaces/Duration time_from_start"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg.

# Single DOF joint positions for each joint relative to their "0" position.
# The units depend on the specific joint type: radians for revolute or
# continuous joints, and meters for prismatic joints.
float64[] positions

# The rate of change in position of each joint. Units are joint type dependent.
# Radians/second for revolute or continuous joints, and meters/second for
# prismatic joints.
float64[] velocities

# Rate of change in velocity of each joint. Units are joint type dependent.
# Radians/second^2 for revolute or continuous joints, and meters/second^2 for
# prismatic joints.
float64[] accelerations

# The torque or the force to be applied at each joint. For revolute/continuous
# joints effort denotes a torque in newton-meters. For prismatic joints, effort
# denotes a force in newtons.
float64[] effort

# Desired time from the trajectory start to arrive at this trajectory point.
builtin_interfaces/Duration time_from_start"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct MultiDOFJointTrajectory {
        pub r#header: std_msgs::Header,
        pub r#joint_names: ::std::vec::Vec<::std::string::String>,
        pub r#points: ::std::vec::Vec<self::MultiDOFJointTrajectoryPoint>,
    }
    impl ::roslibrust_codegen::RosMessageType for MultiDOFJointTrajectory {
        const ROS_TYPE_NAME: &'static str = "trajectory_msgs/MultiDOFJointTrajectory";
        const MD5SUM: &'static str = "d00d14d97bd70c5eb648278240cfb066";
        const DEFINITION: &'static str = r#"# The header is used to specify the coordinate frame and the reference time for the trajectory durations
std_msgs/Header header

# A representation of a multi-dof joint trajectory (each point is a transformation)
# Each point along the trajectory will include an array of positions/velocities/accelerations
# that has the same length as the array of joint names, and has the same order of joints as 
# the joint names array.

string[] joint_names
MultiDOFJointTrajectoryPoint[] points
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: trajectory_msgs/MultiDOFJointTrajectoryPoint
# Each multi-dof joint can specify a transform (up to 6 DOF).
geometry_msgs/Transform[] transforms

# There can be a velocity specified for the origin of the joint.
geometry_msgs/Twist[] velocities

# There can be an acceleration specified for the origin of the joint.
geometry_msgs/Twist[] accelerations

# Desired time from the trajectory start to arrive at this trajectory point.
builtin_interfaces/Duration time_from_start
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct MultiDOFJointTrajectoryPoint {
        pub r#transforms: ::std::vec::Vec<geometry_msgs::Transform>,
        pub r#velocities: ::std::vec::Vec<geometry_msgs::Twist>,
        pub r#accelerations: ::std::vec::Vec<geometry_msgs::Twist>,
        pub r#time_from_start: ::roslibrust_codegen::integral_types::Duration,
    }
    impl ::roslibrust_codegen::RosMessageType for MultiDOFJointTrajectoryPoint {
        const ROS_TYPE_NAME: &'static str = "trajectory_msgs/MultiDOFJointTrajectoryPoint";
        const MD5SUM: &'static str = "6731945e53cbc0fbc6e93c28f7416a71";
        const DEFINITION: &'static str = r#"# Each multi-dof joint can specify a transform (up to 6 DOF).
geometry_msgs/Transform[] transforms

# There can be a velocity specified for the origin of the joint.
geometry_msgs/Twist[] velocities

# There can be an acceleration specified for the origin of the joint.
geometry_msgs/Twist[] accelerations

# Desired time from the trajectory start to arrive at this trajectory point.
builtin_interfaces/Duration time_from_start
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z"#;
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
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"int32 CIRCLE=0
int32 LINE_STRIP=1
int32 LINE_LIST=2
int32 POLYGON=3
int32 POINTS=4

int32 ADD=0
int32 REMOVE=1

std_msgs/Header header
# Namespace which is used with the id to form a unique id.
string ns
# Unique id within the namespace.
int32 id
# One of the above types, e.g. CIRCLE, LINE_STRIP, etc.
int32 type
# Either ADD or REMOVE.
int32 action
# Two-dimensional coordinate position, in pixel-coordinates.
geometry_msgs/Point position
# The scale of the object, e.g. the diameter for a CIRCLE.
float32 scale
# The outline color of the marker.
std_msgs/ColorRGBA outline_color
# Whether or not to fill in the shape with color.
uint8 filled
# Fill color; in the range: [0.0-1.0]
std_msgs/ColorRGBA fill_color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime

# Coordinates in 2D in pixel coords. Used for LINE_STRIP, LINE_LIST, POINTS, etc.
geometry_msgs/Point[] points
# The color for each line, point, etc. in the points field.
std_msgs/ColorRGBA[] outline_colors
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(unused)]
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
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# Time/frame info.
# If header.time is set to 0, the marker will be retransformed into
# its frame on each timestep. You will receive the pose feedback
# in the same frame.
# Otherwise, you might receive feedback in a different frame.
# For rviz, this will be the current 'fixed frame' set by the user.
std_msgs/Header header

# Initial pose. Also, defines the pivot point for rotations.
geometry_msgs/Pose pose

# Identifying string. Must be globally unique in
# the topic that this message is sent through.
string name

# Short description (< 40 characters).
string description

# Scale to be used for default controls (default=1).
float32 scale

# All menu and submenu entries associated with this marker.
MenuEntry[] menu_entries

# List of controls displayed for this marker.
InteractiveMarkerControl[] controls
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/InteractiveMarkerControl
# Represents a control that is to be displayed together with an interactive marker

# Identifying string for this control.
# You need to assign a unique value to this to receive feedback from the GUI
# on what actions the user performs on this control (e.g. a button click).
string name


# Defines the local coordinate frame (relative to the pose of the parent
# interactive marker) in which is being rotated and translated.
# Default: Identity
geometry_msgs/Quaternion orientation


# Orientation mode: controls how orientation changes.
# INHERIT: Follow orientation of interactive marker
# FIXED: Keep orientation fixed at initial state
# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
uint8 INHERIT = 0
uint8 FIXED = 1
uint8 VIEW_FACING = 2

uint8 orientation_mode

# Interaction mode for this control
#
# NONE: This control is only meant for visualization; no context menu.
# MENU: Like NONE, but right-click menu is active.
# BUTTON: Element can be left-clicked.
# MOVE_AXIS: Translate along local x-axis.
# MOVE_PLANE: Translate in local y-z plane.
# ROTATE_AXIS: Rotate around local x-axis.
# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
uint8 NONE = 0
uint8 MENU = 1
uint8 BUTTON = 2
uint8 MOVE_AXIS = 3
uint8 MOVE_PLANE = 4
uint8 ROTATE_AXIS = 5
uint8 MOVE_ROTATE = 6
# "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
# MOVE_3D: Translate freely in 3D space.
# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
uint8 MOVE_3D = 7
uint8 ROTATE_3D = 8
uint8 MOVE_ROTATE_3D = 9

uint8 interaction_mode


# If true, the contained markers will also be visible
# when the gui is not in interactive mode.
bool always_visible


# Markers to be displayed as custom visual representation.
# Leave this empty to use the default control handles.
#
# Note:
# - The markers can be defined in an arbitrary coordinate frame,
#   but will be transformed into the local frame of the interactive marker.
# - If the header of a marker is empty, its pose will be interpreted as
#   relative to the pose of the parent interactive marker.
Marker[] markers


# In VIEW_FACING mode, set this to true if you don't want the markers
# to be aligned with the camera view point. The markers will show up
# as in INHERIT mode.
bool independent_marker_orientation


# Short description (< 40 characters) of what this control does,
# e.g. "Move the robot".
# Default: A generic description based on the interaction mode
string description
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MenuEntry
# MenuEntry message.
#
# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.
#
# Example:
# - id = 3
#   parent_id = 0
#   title = "fun"
# - id = 2
#   parent_id = 0
#   title = "robot"
# - id = 4
#   parent_id = 2
#   title = "pr2"
# - id = 5
#   parent_id = 2
#   title = "turtle"
#
# Gives a menu tree like this:
#  - fun
#  - robot
#    - pr2
#    - turtle

# ID is a number for each menu entry.  Must be unique within the
# control, and should never be 0.
uint32 id

# ID of the parent of this menu entry, if it is a submenu.  If this
# menu entry is a top-level entry, set parent_id to 0.
uint32 parent_id

# menu / entry title
string title

# Arguments to command indicated by command_type (below)
string command

# Command_type stores the type of response desired when this menu
# entry is clicked.
# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
# ROSRUN: execute "rosrun" with arguments given in the command field (above).
# ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
uint8 FEEDBACK=0
uint8 ROSRUN=1
uint8 ROSLAUNCH=2
uint8 command_type
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# Represents a control that is to be displayed together with an interactive marker

# Identifying string for this control.
# You need to assign a unique value to this to receive feedback from the GUI
# on what actions the user performs on this control (e.g. a button click).
string name


# Defines the local coordinate frame (relative to the pose of the parent
# interactive marker) in which is being rotated and translated.
# Default: Identity
geometry_msgs/Quaternion orientation


# Orientation mode: controls how orientation changes.
# INHERIT: Follow orientation of interactive marker
# FIXED: Keep orientation fixed at initial state
# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
uint8 INHERIT = 0
uint8 FIXED = 1
uint8 VIEW_FACING = 2

uint8 orientation_mode

# Interaction mode for this control
#
# NONE: This control is only meant for visualization; no context menu.
# MENU: Like NONE, but right-click menu is active.
# BUTTON: Element can be left-clicked.
# MOVE_AXIS: Translate along local x-axis.
# MOVE_PLANE: Translate in local y-z plane.
# ROTATE_AXIS: Rotate around local x-axis.
# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
uint8 NONE = 0
uint8 MENU = 1
uint8 BUTTON = 2
uint8 MOVE_AXIS = 3
uint8 MOVE_PLANE = 4
uint8 ROTATE_AXIS = 5
uint8 MOVE_ROTATE = 6
# "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
# MOVE_3D: Translate freely in 3D space.
# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
uint8 MOVE_3D = 7
uint8 ROTATE_3D = 8
uint8 MOVE_ROTATE_3D = 9

uint8 interaction_mode


# If true, the contained markers will also be visible
# when the gui is not in interactive mode.
bool always_visible


# Markers to be displayed as custom visual representation.
# Leave this empty to use the default control handles.
#
# Note:
# - The markers can be defined in an arbitrary coordinate frame,
#   but will be transformed into the local frame of the interactive marker.
# - If the header of a marker is empty, its pose will be interpreted as
#   relative to the pose of the parent interactive marker.
Marker[] markers


# In VIEW_FACING mode, set this to true if you don't want the markers
# to be aligned with the camera view point. The markers will show up
# as in INHERIT mode.
bool independent_marker_orientation


# Short description (< 40 characters) of what this control does,
# e.g. "Move the robot".
# Default: A generic description based on the interaction mode
string description
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v"#;
    }
    #[allow(unused)]
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
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# Time/frame info.
std_msgs/Header header

# Identifying string. Must be unique in the topic namespace.
string client_id

# Feedback message sent back from the GUI, e.g.
# when the status of an interactive marker was modified by the user.

# Specifies which interactive marker and control this message refers to
string marker_name
string control_name

# Type of the event
# KEEP_ALIVE: sent while dragging to keep up control of the marker
# MENU_SELECT: a menu entry has been selected
# BUTTON_CLICK: a button control has been clicked
# POSE_UPDATE: the pose has been changed using one of the controls
uint8 KEEP_ALIVE = 0
uint8 POSE_UPDATE = 1
uint8 MENU_SELECT = 2
uint8 BUTTON_CLICK = 3

uint8 MOUSE_DOWN = 4
uint8 MOUSE_UP = 5

uint8 event_type

# Current pose of the marker
# Note: Has to be valid for all feedback types.
geometry_msgs/Pose pose

# Contains the ID of the selected menu entry
# Only valid for MENU_SELECT events.
uint32 menu_entry_id

# If event_type is BUTTON_CLICK, MOUSE_DOWN, or MOUSE_UP, mouse_point
# may contain the 3 dimensional position of the event on the
# control.  If it does, mouse_point_valid will be true.  mouse_point
# will be relative to the frame listed in the header.
geometry_msgs/Point mouse_point
bool mouse_point_valid
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(unused)]
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
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct InteractiveMarkerInit {
        pub r#server_id: ::std::string::String,
        pub r#seq_num: u64,
        pub r#markers: ::std::vec::Vec<self::InteractiveMarker>,
    }
    impl ::roslibrust_codegen::RosMessageType for InteractiveMarkerInit {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarkerInit";
        const MD5SUM: &'static str = "5d275694a5cb7ea4627f917a9eb1b4cd";
        const DEFINITION: &'static str = r#"# Identifying string. Must be unique in the topic namespace
# that this server works on.
string server_id

# Sequence number.
# The client will use this to detect if it has missed a subsequent
# update.  Every update message will have the same sequence number as
# an init message.  Clients will likely want to unsubscribe from the
# init topic after a successful initialization to avoid receiving
# duplicate data.
uint64 seq_num

# All markers.
InteractiveMarker[] markers
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/InteractiveMarker
# Time/frame info.
# If header.time is set to 0, the marker will be retransformed into
# its frame on each timestep. You will receive the pose feedback
# in the same frame.
# Otherwise, you might receive feedback in a different frame.
# For rviz, this will be the current 'fixed frame' set by the user.
std_msgs/Header header

# Initial pose. Also, defines the pivot point for rotations.
geometry_msgs/Pose pose

# Identifying string. Must be globally unique in
# the topic that this message is sent through.
string name

# Short description (< 40 characters).
string description

# Scale to be used for default controls (default=1).
float32 scale

# All menu and submenu entries associated with this marker.
MenuEntry[] menu_entries

# List of controls displayed for this marker.
InteractiveMarkerControl[] controls
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/InteractiveMarkerControl
# Represents a control that is to be displayed together with an interactive marker

# Identifying string for this control.
# You need to assign a unique value to this to receive feedback from the GUI
# on what actions the user performs on this control (e.g. a button click).
string name


# Defines the local coordinate frame (relative to the pose of the parent
# interactive marker) in which is being rotated and translated.
# Default: Identity
geometry_msgs/Quaternion orientation


# Orientation mode: controls how orientation changes.
# INHERIT: Follow orientation of interactive marker
# FIXED: Keep orientation fixed at initial state
# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
uint8 INHERIT = 0
uint8 FIXED = 1
uint8 VIEW_FACING = 2

uint8 orientation_mode

# Interaction mode for this control
#
# NONE: This control is only meant for visualization; no context menu.
# MENU: Like NONE, but right-click menu is active.
# BUTTON: Element can be left-clicked.
# MOVE_AXIS: Translate along local x-axis.
# MOVE_PLANE: Translate in local y-z plane.
# ROTATE_AXIS: Rotate around local x-axis.
# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
uint8 NONE = 0
uint8 MENU = 1
uint8 BUTTON = 2
uint8 MOVE_AXIS = 3
uint8 MOVE_PLANE = 4
uint8 ROTATE_AXIS = 5
uint8 MOVE_ROTATE = 6
# "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
# MOVE_3D: Translate freely in 3D space.
# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
uint8 MOVE_3D = 7
uint8 ROTATE_3D = 8
uint8 MOVE_ROTATE_3D = 9

uint8 interaction_mode


# If true, the contained markers will also be visible
# when the gui is not in interactive mode.
bool always_visible


# Markers to be displayed as custom visual representation.
# Leave this empty to use the default control handles.
#
# Note:
# - The markers can be defined in an arbitrary coordinate frame,
#   but will be transformed into the local frame of the interactive marker.
# - If the header of a marker is empty, its pose will be interpreted as
#   relative to the pose of the parent interactive marker.
Marker[] markers


# In VIEW_FACING mode, set this to true if you don't want the markers
# to be aligned with the camera view point. The markers will show up
# as in INHERIT mode.
bool independent_marker_orientation


# Short description (< 40 characters) of what this control does,
# e.g. "Move the robot".
# Default: A generic description based on the interaction mode
string description
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MenuEntry
# MenuEntry message.
#
# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.
#
# Example:
# - id = 3
#   parent_id = 0
#   title = "fun"
# - id = 2
#   parent_id = 0
#   title = "robot"
# - id = 4
#   parent_id = 2
#   title = "pr2"
# - id = 5
#   parent_id = 2
#   title = "turtle"
#
# Gives a menu tree like this:
#  - fun
#  - robot
#    - pr2
#    - turtle

# ID is a number for each menu entry.  Must be unique within the
# control, and should never be 0.
uint32 id

# ID of the parent of this menu entry, if it is a submenu.  If this
# menu entry is a top-level entry, set parent_id to 0.
uint32 parent_id

# menu / entry title
string title

# Arguments to command indicated by command_type (below)
string command

# Command_type stores the type of response desired when this menu
# entry is clicked.
# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
# ROSRUN: execute "rosrun" with arguments given in the command field (above).
# ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
uint8 FEEDBACK=0
uint8 ROSRUN=1
uint8 ROSLAUNCH=2
uint8 command_type
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/InteractiveMarkerControl
# Represents a control that is to be displayed together with an interactive marker

# Identifying string for this control.
# You need to assign a unique value to this to receive feedback from the GUI
# on what actions the user performs on this control (e.g. a button click).
string name


# Defines the local coordinate frame (relative to the pose of the parent
# interactive marker) in which is being rotated and translated.
# Default: Identity
geometry_msgs/Quaternion orientation


# Orientation mode: controls how orientation changes.
# INHERIT: Follow orientation of interactive marker
# FIXED: Keep orientation fixed at initial state
# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
uint8 INHERIT = 0
uint8 FIXED = 1
uint8 VIEW_FACING = 2

uint8 orientation_mode

# Interaction mode for this control
#
# NONE: This control is only meant for visualization; no context menu.
# MENU: Like NONE, but right-click menu is active.
# BUTTON: Element can be left-clicked.
# MOVE_AXIS: Translate along local x-axis.
# MOVE_PLANE: Translate in local y-z plane.
# ROTATE_AXIS: Rotate around local x-axis.
# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
uint8 NONE = 0
uint8 MENU = 1
uint8 BUTTON = 2
uint8 MOVE_AXIS = 3
uint8 MOVE_PLANE = 4
uint8 ROTATE_AXIS = 5
uint8 MOVE_ROTATE = 6
# "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
# MOVE_3D: Translate freely in 3D space.
# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
uint8 MOVE_3D = 7
uint8 ROTATE_3D = 8
uint8 MOVE_ROTATE_3D = 9

uint8 interaction_mode


# If true, the contained markers will also be visible
# when the gui is not in interactive mode.
bool always_visible


# Markers to be displayed as custom visual representation.
# Leave this empty to use the default control handles.
#
# Note:
# - The markers can be defined in an arbitrary coordinate frame,
#   but will be transformed into the local frame of the interactive marker.
# - If the header of a marker is empty, its pose will be interpreted as
#   relative to the pose of the parent interactive marker.
Marker[] markers


# In VIEW_FACING mode, set this to true if you don't want the markers
# to be aligned with the camera view point. The markers will show up
# as in INHERIT mode.
bool independent_marker_orientation


# Short description (< 40 characters) of what this control does,
# e.g. "Move the robot".
# Default: A generic description based on the interaction mode
string description
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MenuEntry
# MenuEntry message.
#
# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.
#
# Example:
# - id = 3
#   parent_id = 0
#   title = "fun"
# - id = 2
#   parent_id = 0
#   title = "robot"
# - id = 4
#   parent_id = 2
#   title = "pr2"
# - id = 5
#   parent_id = 2
#   title = "turtle"
#
# Gives a menu tree like this:
#  - fun
#  - robot
#    - pr2
#    - turtle

# ID is a number for each menu entry.  Must be unique within the
# control, and should never be 0.
uint32 id

# ID of the parent of this menu entry, if it is a submenu.  If this
# menu entry is a top-level entry, set parent_id to 0.
uint32 parent_id

# menu / entry title
string title

# Arguments to command indicated by command_type (below)
string command

# Command_type stores the type of response desired when this menu
# entry is clicked.
# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
# ROSRUN: execute "rosrun" with arguments given in the command field (above).
# ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
uint8 FEEDBACK=0
uint8 ROSRUN=1
uint8 ROSLAUNCH=2
uint8 command_type
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct InteractiveMarkerPose {
        pub r#header: std_msgs::Header,
        pub r#pose: geometry_msgs::Pose,
        pub r#name: ::std::string::String,
    }
    impl ::roslibrust_codegen::RosMessageType for InteractiveMarkerPose {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/InteractiveMarkerPose";
        const MD5SUM: &'static str = "b88540594a0f8e3fe46c720be41faa03";
        const DEFINITION: &'static str = r#"# Time/frame info.
std_msgs/Header header

# Initial pose. Also, defines the pivot point for rotations.
geometry_msgs/Pose pose

# Identifying string. Must be globally unique in
# the topic that this message is sent through.
string name
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# Identifying string. Must be unique in the topic namespace
# that this server works on.
string server_id

# Sequence number.
# The client will use this to detect if it has missed an update.
uint64 seq_num

# Type holds the purpose of this message.  It must be one of UPDATE or KEEP_ALIVE.
# UPDATE: Incremental update to previous state.
#         The sequence number must be 1 higher than for
#         the previous update.
# KEEP_ALIVE: Indicates the that the server is still living.
#             The sequence number does not increase.
#             No payload data should be filled out (markers, poses, or erases).
uint8 KEEP_ALIVE = 0
uint8 UPDATE = 1

uint8 type

# Note: No guarantees on the order of processing.
#       Contents must be kept consistent by sender.

# Markers to be added or updated
InteractiveMarker[] markers

# Poses of markers that should be moved
InteractiveMarkerPose[] poses

# Names of markers to be erased
string[] erases
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/InteractiveMarker
# Time/frame info.
# If header.time is set to 0, the marker will be retransformed into
# its frame on each timestep. You will receive the pose feedback
# in the same frame.
# Otherwise, you might receive feedback in a different frame.
# For rviz, this will be the current 'fixed frame' set by the user.
std_msgs/Header header

# Initial pose. Also, defines the pivot point for rotations.
geometry_msgs/Pose pose

# Identifying string. Must be globally unique in
# the topic that this message is sent through.
string name

# Short description (< 40 characters).
string description

# Scale to be used for default controls (default=1).
float32 scale

# All menu and submenu entries associated with this marker.
MenuEntry[] menu_entries

# List of controls displayed for this marker.
InteractiveMarkerControl[] controls
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/InteractiveMarkerControl
# Represents a control that is to be displayed together with an interactive marker

# Identifying string for this control.
# You need to assign a unique value to this to receive feedback from the GUI
# on what actions the user performs on this control (e.g. a button click).
string name


# Defines the local coordinate frame (relative to the pose of the parent
# interactive marker) in which is being rotated and translated.
# Default: Identity
geometry_msgs/Quaternion orientation


# Orientation mode: controls how orientation changes.
# INHERIT: Follow orientation of interactive marker
# FIXED: Keep orientation fixed at initial state
# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
uint8 INHERIT = 0
uint8 FIXED = 1
uint8 VIEW_FACING = 2

uint8 orientation_mode

# Interaction mode for this control
#
# NONE: This control is only meant for visualization; no context menu.
# MENU: Like NONE, but right-click menu is active.
# BUTTON: Element can be left-clicked.
# MOVE_AXIS: Translate along local x-axis.
# MOVE_PLANE: Translate in local y-z plane.
# ROTATE_AXIS: Rotate around local x-axis.
# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
uint8 NONE = 0
uint8 MENU = 1
uint8 BUTTON = 2
uint8 MOVE_AXIS = 3
uint8 MOVE_PLANE = 4
uint8 ROTATE_AXIS = 5
uint8 MOVE_ROTATE = 6
# "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
# MOVE_3D: Translate freely in 3D space.
# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
uint8 MOVE_3D = 7
uint8 ROTATE_3D = 8
uint8 MOVE_ROTATE_3D = 9

uint8 interaction_mode


# If true, the contained markers will also be visible
# when the gui is not in interactive mode.
bool always_visible


# Markers to be displayed as custom visual representation.
# Leave this empty to use the default control handles.
#
# Note:
# - The markers can be defined in an arbitrary coordinate frame,
#   but will be transformed into the local frame of the interactive marker.
# - If the header of a marker is empty, its pose will be interpreted as
#   relative to the pose of the parent interactive marker.
Marker[] markers


# In VIEW_FACING mode, set this to true if you don't want the markers
# to be aligned with the camera view point. The markers will show up
# as in INHERIT mode.
bool independent_marker_orientation


# Short description (< 40 characters) of what this control does,
# e.g. "Move the robot".
# Default: A generic description based on the interaction mode
string description
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MenuEntry
# MenuEntry message.
#
# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.
#
# Example:
# - id = 3
#   parent_id = 0
#   title = "fun"
# - id = 2
#   parent_id = 0
#   title = "robot"
# - id = 4
#   parent_id = 2
#   title = "pr2"
# - id = 5
#   parent_id = 2
#   title = "turtle"
#
# Gives a menu tree like this:
#  - fun
#  - robot
#    - pr2
#    - turtle

# ID is a number for each menu entry.  Must be unique within the
# control, and should never be 0.
uint32 id

# ID of the parent of this menu entry, if it is a submenu.  If this
# menu entry is a top-level entry, set parent_id to 0.
uint32 parent_id

# menu / entry title
string title

# Arguments to command indicated by command_type (below)
string command

# Command_type stores the type of response desired when this menu
# entry is clicked.
# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
# ROSRUN: execute "rosrun" with arguments given in the command field (above).
# ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
uint8 FEEDBACK=0
uint8 ROSRUN=1
uint8 ROSLAUNCH=2
uint8 command_type
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/InteractiveMarkerControl
# Represents a control that is to be displayed together with an interactive marker

# Identifying string for this control.
# You need to assign a unique value to this to receive feedback from the GUI
# on what actions the user performs on this control (e.g. a button click).
string name


# Defines the local coordinate frame (relative to the pose of the parent
# interactive marker) in which is being rotated and translated.
# Default: Identity
geometry_msgs/Quaternion orientation


# Orientation mode: controls how orientation changes.
# INHERIT: Follow orientation of interactive marker
# FIXED: Keep orientation fixed at initial state
# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
uint8 INHERIT = 0
uint8 FIXED = 1
uint8 VIEW_FACING = 2

uint8 orientation_mode

# Interaction mode for this control
#
# NONE: This control is only meant for visualization; no context menu.
# MENU: Like NONE, but right-click menu is active.
# BUTTON: Element can be left-clicked.
# MOVE_AXIS: Translate along local x-axis.
# MOVE_PLANE: Translate in local y-z plane.
# ROTATE_AXIS: Rotate around local x-axis.
# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
uint8 NONE = 0
uint8 MENU = 1
uint8 BUTTON = 2
uint8 MOVE_AXIS = 3
uint8 MOVE_PLANE = 4
uint8 ROTATE_AXIS = 5
uint8 MOVE_ROTATE = 6
# "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
# MOVE_3D: Translate freely in 3D space.
# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
uint8 MOVE_3D = 7
uint8 ROTATE_3D = 8
uint8 MOVE_ROTATE_3D = 9

uint8 interaction_mode


# If true, the contained markers will also be visible
# when the gui is not in interactive mode.
bool always_visible


# Markers to be displayed as custom visual representation.
# Leave this empty to use the default control handles.
#
# Note:
# - The markers can be defined in an arbitrary coordinate frame,
#   but will be transformed into the local frame of the interactive marker.
# - If the header of a marker is empty, its pose will be interpreted as
#   relative to the pose of the parent interactive marker.
Marker[] markers


# In VIEW_FACING mode, set this to true if you don't want the markers
# to be aligned with the camera view point. The markers will show up
# as in INHERIT mode.
bool independent_marker_orientation


# Short description (< 40 characters) of what this control does,
# e.g. "Move the robot".
# Default: A generic description based on the interaction mode
string description
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/InteractiveMarkerPose
# Time/frame info.
std_msgs/Header header

# Initial pose. Also, defines the pivot point for rotations.
geometry_msgs/Pose pose

# Identifying string. Must be globally unique in
# the topic that this message is sent through.
string name
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MenuEntry
# MenuEntry message.
#
# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.
#
# Example:
# - id = 3
#   parent_id = 0
#   title = "fun"
# - id = 2
#   parent_id = 0
#   title = "robot"
# - id = 4
#   parent_id = 2
#   title = "pr2"
# - id = 5
#   parent_id = 2
#   title = "turtle"
#
# Gives a menu tree like this:
#  - fun
#  - robot
#    - pr2
#    - turtle

# ID is a number for each menu entry.  Must be unique within the
# control, and should never be 0.
uint32 id

# ID of the parent of this menu entry, if it is a submenu.  If this
# menu entry is a top-level entry, set parent_id to 0.
uint32 parent_id

# menu / entry title
string title

# Arguments to command indicated by command_type (below)
string command

# Command_type stores the type of response desired when this menu
# entry is clicked.
# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
# ROSRUN: execute "rosrun" with arguments given in the command field (above).
# ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
uint8 FEEDBACK=0
uint8 ROSRUN=1
uint8 ROSLAUNCH=2
uint8 command_type
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v"#;
    }
    #[allow(unused)]
    impl InteractiveMarkerUpdate {
        pub const r#KEEP_ALIVE: u8 = 0u8;
        pub const r#UPDATE: u8 = 1u8;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v"#;
    }
    #[allow(unused)]
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
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct MarkerArray {
        pub r#markers: ::std::vec::Vec<self::Marker>,
    }
    impl ::roslibrust_codegen::RosMessageType for MarkerArray {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/MarkerArray";
        const MD5SUM: &'static str = "11e38f15427197858cf46456867167bd";
        const DEFINITION: &'static str = r#"Marker[] markers
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
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
        const DEFINITION: &'static str = r#"# MenuEntry message.
#
# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.
#
# Example:
# - id = 3
#   parent_id = 0
#   title = "fun"
# - id = 2
#   parent_id = 0
#   title = "robot"
# - id = 4
#   parent_id = 2
#   title = "pr2"
# - id = 5
#   parent_id = 2
#   title = "turtle"
#
# Gives a menu tree like this:
#  - fun
#  - robot
#    - pr2
#    - turtle

# ID is a number for each menu entry.  Must be unique within the
# control, and should never be 0.
uint32 id

# ID of the parent of this menu entry, if it is a submenu.  If this
# menu entry is a top-level entry, set parent_id to 0.
uint32 parent_id

# menu / entry title
string title

# Arguments to command indicated by command_type (below)
string command

# Command_type stores the type of response desired when this menu
# entry is clicked.
# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
# ROSRUN: execute "rosrun" with arguments given in the command field (above).
# ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
uint8 FEEDBACK=0
uint8 ROSRUN=1
uint8 ROSLAUNCH=2
uint8 command_type"#;
    }
    #[allow(unused)]
    impl MenuEntry {
        pub const r#FEEDBACK: u8 = 0u8;
        pub const r#ROSRUN: u8 = 1u8;
        pub const r#ROSLAUNCH: u8 = 2u8;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct MeshFile {
        pub r#filename: ::std::string::String,
        #[serde(with = "::roslibrust_codegen::serde_bytes")]
        pub r#data: ::std::vec::Vec<u8>,
    }
    impl ::roslibrust_codegen::RosMessageType for MeshFile {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/MeshFile";
        const MD5SUM: &'static str = "39f264648e441626a1045a7d9ef1ba17";
        const DEFINITION: &'static str = r#"# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct UVCoordinate {
        pub r#u: f32,
        pub r#v: f32,
    }
    impl ::roslibrust_codegen::RosMessageType for UVCoordinate {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/UVCoordinate";
        const MD5SUM: &'static str = "4f5254e0e12914c461d4b17a0cd07f7f";
        const DEFINITION: &'static str = r#"# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v"#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct GetInteractiveMarkersRequest {}
    impl ::roslibrust_codegen::RosMessageType for GetInteractiveMarkersRequest {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/GetInteractiveMarkersRequest";
        const MD5SUM: &'static str = "d41d8cd98f00b204e9800998ecf8427e";
        const DEFINITION: &'static str = r#""#;
    }
    #[allow(non_snake_case)]
    #[derive(
        :: roslibrust_codegen :: Deserialize,
        :: roslibrust_codegen :: Serialize,
        :: roslibrust_codegen :: SmartDefault,
        Debug,
        Clone,
        PartialEq,
    )]
    #[serde(crate = "::roslibrust_codegen::serde")]
    pub struct GetInteractiveMarkersResponse {
        pub r#sequence_number: u64,
        pub r#markers: ::std::vec::Vec<self::InteractiveMarker>,
    }
    impl ::roslibrust_codegen::RosMessageType for GetInteractiveMarkersResponse {
        const ROS_TYPE_NAME: &'static str = "visualization_msgs/GetInteractiveMarkersResponse";
        const MD5SUM: &'static str = "923b76ef2c497d4ff5f83a061d424d3b";
        const DEFINITION: &'static str = r#"# Sequence number.
# Set to the sequence number of the latest update message
# at the time the server received the request.
# Clients use this to detect if any updates were missed.
uint64 sequence_number

# All interactive markers provided by the server.
InteractiveMarker[] markers
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/InteractiveMarker
# Time/frame info.
# If header.time is set to 0, the marker will be retransformed into
# its frame on each timestep. You will receive the pose feedback
# in the same frame.
# Otherwise, you might receive feedback in a different frame.
# For rviz, this will be the current 'fixed frame' set by the user.
std_msgs/Header header

# Initial pose. Also, defines the pivot point for rotations.
geometry_msgs/Pose pose

# Identifying string. Must be globally unique in
# the topic that this message is sent through.
string name

# Short description (< 40 characters).
string description

# Scale to be used for default controls (default=1).
float32 scale

# All menu and submenu entries associated with this marker.
MenuEntry[] menu_entries

# List of controls displayed for this marker.
InteractiveMarkerControl[] controls
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/InteractiveMarkerControl
# Represents a control that is to be displayed together with an interactive marker

# Identifying string for this control.
# You need to assign a unique value to this to receive feedback from the GUI
# on what actions the user performs on this control (e.g. a button click).
string name


# Defines the local coordinate frame (relative to the pose of the parent
# interactive marker) in which is being rotated and translated.
# Default: Identity
geometry_msgs/Quaternion orientation


# Orientation mode: controls how orientation changes.
# INHERIT: Follow orientation of interactive marker
# FIXED: Keep orientation fixed at initial state
# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
uint8 INHERIT = 0
uint8 FIXED = 1
uint8 VIEW_FACING = 2

uint8 orientation_mode

# Interaction mode for this control
#
# NONE: This control is only meant for visualization; no context menu.
# MENU: Like NONE, but right-click menu is active.
# BUTTON: Element can be left-clicked.
# MOVE_AXIS: Translate along local x-axis.
# MOVE_PLANE: Translate in local y-z plane.
# ROTATE_AXIS: Rotate around local x-axis.
# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
uint8 NONE = 0
uint8 MENU = 1
uint8 BUTTON = 2
uint8 MOVE_AXIS = 3
uint8 MOVE_PLANE = 4
uint8 ROTATE_AXIS = 5
uint8 MOVE_ROTATE = 6
# "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
# MOVE_3D: Translate freely in 3D space.
# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
uint8 MOVE_3D = 7
uint8 ROTATE_3D = 8
uint8 MOVE_ROTATE_3D = 9

uint8 interaction_mode


# If true, the contained markers will also be visible
# when the gui is not in interactive mode.
bool always_visible


# Markers to be displayed as custom visual representation.
# Leave this empty to use the default control handles.
#
# Note:
# - The markers can be defined in an arbitrary coordinate frame,
#   but will be transformed into the local frame of the interactive marker.
# - If the header of a marker is empty, its pose will be interpreted as
#   relative to the pose of the parent interactive marker.
Marker[] markers


# In VIEW_FACING mode, set this to true if you don't want the markers
# to be aligned with the camera view point. The markers will show up
# as in INHERIT mode.
bool independent_marker_orientation


# Short description (< 40 characters) of what this control does,
# e.g. "Move the robot".
# Default: A generic description based on the interaction mode
string description
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MenuEntry
# MenuEntry message.
#
# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.
#
# Example:
# - id = 3
#   parent_id = 0
#   title = "fun"
# - id = 2
#   parent_id = 0
#   title = "robot"
# - id = 4
#   parent_id = 2
#   title = "pr2"
# - id = 5
#   parent_id = 2
#   title = "turtle"
#
# Gives a menu tree like this:
#  - fun
#  - robot
#    - pr2
#    - turtle

# ID is a number for each menu entry.  Must be unique within the
# control, and should never be 0.
uint32 id

# ID of the parent of this menu entry, if it is a submenu.  If this
# menu entry is a top-level entry, set parent_id to 0.
uint32 parent_id

# menu / entry title
string title

# Arguments to command indicated by command_type (below)
string command

# Command_type stores the type of response desired when this menu
# entry is clicked.
# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
# ROSRUN: execute "rosrun" with arguments given in the command field (above).
# ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
uint8 FEEDBACK=0
uint8 ROSRUN=1
uint8 ROSLAUNCH=2
uint8 command_type
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/InteractiveMarkerControl
# Represents a control that is to be displayed together with an interactive marker

# Identifying string for this control.
# You need to assign a unique value to this to receive feedback from the GUI
# on what actions the user performs on this control (e.g. a button click).
string name


# Defines the local coordinate frame (relative to the pose of the parent
# interactive marker) in which is being rotated and translated.
# Default: Identity
geometry_msgs/Quaternion orientation


# Orientation mode: controls how orientation changes.
# INHERIT: Follow orientation of interactive marker
# FIXED: Keep orientation fixed at initial state
# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
uint8 INHERIT = 0
uint8 FIXED = 1
uint8 VIEW_FACING = 2

uint8 orientation_mode

# Interaction mode for this control
#
# NONE: This control is only meant for visualization; no context menu.
# MENU: Like NONE, but right-click menu is active.
# BUTTON: Element can be left-clicked.
# MOVE_AXIS: Translate along local x-axis.
# MOVE_PLANE: Translate in local y-z plane.
# ROTATE_AXIS: Rotate around local x-axis.
# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
uint8 NONE = 0
uint8 MENU = 1
uint8 BUTTON = 2
uint8 MOVE_AXIS = 3
uint8 MOVE_PLANE = 4
uint8 ROTATE_AXIS = 5
uint8 MOVE_ROTATE = 6
# "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
# MOVE_3D: Translate freely in 3D space.
# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
uint8 MOVE_3D = 7
uint8 ROTATE_3D = 8
uint8 MOVE_ROTATE_3D = 9

uint8 interaction_mode


# If true, the contained markers will also be visible
# when the gui is not in interactive mode.
bool always_visible


# Markers to be displayed as custom visual representation.
# Leave this empty to use the default control handles.
#
# Note:
# - The markers can be defined in an arbitrary coordinate frame,
#   but will be transformed into the local frame of the interactive marker.
# - If the header of a marker is empty, its pose will be interpreted as
#   relative to the pose of the parent interactive marker.
Marker[] markers


# In VIEW_FACING mode, set this to true if you don't want the markers
# to be aligned with the camera view point. The markers will show up
# as in INHERIT mode.
bool independent_marker_orientation


# Short description (< 40 characters) of what this control does,
# e.g. "Move the robot".
# Default: A generic description based on the interaction mode
string description
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/Marker
# See:
#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#
# for more information on using this message with rviz.

int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

# Header for timestamp and frame id.
std_msgs/Header header
# Namespace in which to place the object.
# Used in conjunction with id to create a unique name for the object.
string ns
# Object ID used in conjunction with the namespace for manipulating and deleting the object later.
int32 id
# Type of object.
int32 type
# Action to take; one of:
#  - 0 add/modify an object
#  - 1 (deprecated)
#  - 2 deletes an object (with the given ns and id)
#  - 3 deletes all objects (or those with the given ns if any)
int32 action
# Pose of the object with respect the frame_id specified in the header.
geometry_msgs/Pose pose
# Scale of the object; 1,1,1 means default (usually 1 meter square).
geometry_msgs/Vector3 scale
# Color of the object; in the range: [0.0-1.0]
std_msgs/ColorRGBA color
# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime
# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
bool frame_locked

# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
# The number of colors provided must either be 0 or equal to the number of points provided.
# NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# Texture resource is a special URI that can either reference a texture file in
# a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
# or an embedded texture via a string matching the format:
#   "embedded://texture_name"
string texture_resource
# An image to be loaded into the rendering engine as the texture for this marker.
# This will be used iff texture_resource is set to embedded.
sensor_msgs/CompressedImage texture
# Location of each vertex within the texture; in the range: [0.0-1.0]
UVCoordinate[] uv_coordinates

# Only used for text markers
string text

# Only used for MESH_RESOURCE markers.
# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
# use the following format for mesh_resource:
#   "embedded://mesh_name"
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v
================================================================================
MSG: visualization_msgs/MenuEntry
# MenuEntry message.
#
# Each InteractiveMarker message has an array of MenuEntry messages.
# A collection of MenuEntries together describe a
# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
# array.  The tree structure is represented by giving each menu entry
# an ID number and a "parent_id" field.  Top-level entries are the
# ones with parent_id = 0.  Menu entries are ordered within their
# level the same way they are ordered in the containing array.  Parent
# entries must appear before their children.
#
# Example:
# - id = 3
#   parent_id = 0
#   title = "fun"
# - id = 2
#   parent_id = 0
#   title = "robot"
# - id = 4
#   parent_id = 2
#   title = "pr2"
# - id = 5
#   parent_id = 2
#   title = "turtle"
#
# Gives a menu tree like this:
#  - fun
#  - robot
#    - pr2
#    - turtle

# ID is a number for each menu entry.  Must be unique within the
# control, and should never be 0.
uint32 id

# ID of the parent of this menu entry, if it is a submenu.  If this
# menu entry is a top-level entry, set parent_id to 0.
uint32 parent_id

# menu / entry title
string title

# Arguments to command indicated by command_type (below)
string command

# Command_type stores the type of response desired when this menu
# entry is clicked.
# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
# ROSRUN: execute "rosrun" with arguments given in the command field (above).
# ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
uint8 FEEDBACK=0
uint8 ROSRUN=1
uint8 ROSLAUNCH=2
uint8 command_type
================================================================================
MSG: visualization_msgs/MeshFile
# Used to send raw mesh files.

# The filename is used for both debug purposes and to provide a file extension
# for whatever parser is used.
string filename

# This stores the raw text of the mesh file.
uint8[] data
================================================================================
MSG: visualization_msgs/UVCoordinate
# Location of the pixel as a ratio of the width of a 2D texture.
# Values should be in range: [0.0-1.0].
float32 u
float32 v"#;
    }
    #[allow(dead_code)]
    pub struct GetInteractiveMarkers {}
    impl ::roslibrust_codegen::RosServiceType for GetInteractiveMarkers {
        const ROS_SERVICE_NAME: &'static str = "visualization_msgs/GetInteractiveMarkers";
        const MD5SUM: &'static str = "923b76ef2c497d4ff5f83a061d424d3b";
        type Request = GetInteractiveMarkersRequest;
        type Response = GetInteractiveMarkersResponse;
    }
}
