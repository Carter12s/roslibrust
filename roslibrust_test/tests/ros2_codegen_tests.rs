use roslibrust_test::ros2::*;

#[test]
fn test_defaults() {
    let x: test_msgs::Deafults = Default::default();
    assert_eq!(x.x, 42);
    assert_eq!(x.y, -2000);
    assert_eq!(x.full_name, "John Doe");
    assert_eq!(x.samples, vec![-200, -100, 0, 100, 200]);
}