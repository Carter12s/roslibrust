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
