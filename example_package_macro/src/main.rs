roslibrust_codegen_macro::find_and_generate_ros_messages!(
    "assets/ros1_test_msgs",
    "assets/ros1_common_interfaces"
);

// Example of 'use' pointing to code created by the include! macro
mod submodule {
    #[allow(unused_imports)]
    use crate::std_msgs::Header;
}

// Our actual "main" here doesn't do much, just shows the generate types
// are here and real.
fn main() {
    // Note: within our assets there is a folder named ros1_test_msgs which contains a ros package
    // The ros package in its package.xml refers to the name of that package as test_msgs
    // The module that is generated will use the name in package.xml and not the directory
    let data = test_msgs::Constants::TEST_STR;
    println!("Hello World! {data}");
}
