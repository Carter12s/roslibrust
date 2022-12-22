use roslibrust_test::ros1::*;

/// Ensures that associate constants are generated on the test_msgs correctly
/// requires test_msgs gen_code to have been generated.
/// Compilation is passing for this test
#[test]
fn test_associated_constants() {
    let _ = actionlib_msgs::GoalStatus::REJECTED;
}
