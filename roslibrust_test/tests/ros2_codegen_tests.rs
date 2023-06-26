use roslibrust_test::ros2::*;

#[test]
fn test_defaults() {
    let x: test_msgs::Defaults = Default::default();
    assert_eq!(x.x, 42);
    assert_eq!(x.y, -2000);
    assert_eq!(x.full_name, "John Doe");
    assert_eq!(x.samples, vec![-200, -100, 0, 100, 200]);
    assert_eq!(x.s_vec, vec!["hello", "world"]);
    assert_eq!(x.f_samples, vec![-200.0, -1.0, 0.0]);
}
