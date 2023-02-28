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
