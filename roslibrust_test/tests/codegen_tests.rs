use roslibrust_test::*;

/// Ensures that associate constants are generated on the test_msgs correctly
/// requires test_msgs gen_code to have been generated.
/// Compliation is passing for this test
#[test]
fn test_associated_contants() {
    let _ = test_msgs::NodeInfo::STATUS_UNINITIALIZED;
    let _ = actionlib_msgs::GoalStatus::REJECTED;
}
