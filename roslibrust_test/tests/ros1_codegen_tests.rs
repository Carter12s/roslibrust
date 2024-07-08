use roslibrust_codegen::{RosMessageType, RosServiceType};
use roslibrust_test::ros1::*;

/// Ensures that associate constants are generated on the test_msgs correctly
/// requires test_msgs gen_code to have been generated.
/// Compilation is passing for this test
#[test]
fn test_associated_constants() {
    let _ = actionlib_msgs::GoalStatus::REJECTED;
}

#[test]
fn test_md5sum_generation() {
    assert_eq!(std_msgs::Header::MD5SUM, "2176decaecbce78abc3b96ef049fabed");
    assert_eq!(
        geometry_msgs::TransformStamped::MD5SUM,
        "b5764a33bfeb3588febc2682852579b0"
    );
    assert_eq!(
        std_srvs::SetBool::MD5SUM,
        "09fb03525b03e7ea1fd3992bafd87e16"
    );
    assert_eq!(
        sensor_msgs::Image::MD5SUM,
        "060021388200f6f0f447d0fcd9c64743"
    );
}

#[test]
fn fixed_sized_arrays() {
    // Prove the default works, compiler failure here is the test
    let x: sensor_msgs::NavSatFix = Default::default();
    // Prove the types here match, compiler failure here is the test
    let _y: [f64; 9] = x.position_covariance;
    // NavSatFix is the easy case because it is <32 items long

    // Harder case that deals with arrays >32
    let x: geometry_msgs::TwistWithCovariance = Default::default();
    let _y: [f64; 36] = x.covariance;
}

#[test]
fn test_gendeps_in_message_definition() {
    // ROS1 requires that the message_definition includes the expanded
    // definitions of all referenced sub-messages.
    // See https://wiki.ros.org/roslib/gentools for example of expected format
    // Confirm here that sub messages are included in the message definition for geometry_msgs::PointStamped
    assert!(geometry_msgs::PointStamped::DEFINITION.contains("MSG: geometry_msgs/Point"));
    assert!(geometry_msgs::PointStamped::DEFINITION.contains("MSG: std_msgs/Header"));
}
