roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces");

/// This example demonstrates ths usage of the .advertise_any() function
///
/// The intent of the API is to support use cases like play back data from a bag file.
/// Most users are encourage to not use this API and instead rely on generated message types.
/// See ros1_talker.rs for a "normal" example.

#[cfg(feature = "ros1")]
#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    // Note: this example needs a ros master running to work
    let node =
        roslibrust::ros1::NodeHandle::new("http://localhost:11311", "/ros1_publish_any").await?;

    // Definition from: https://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html
    let msg_definition = r#"string data"#;

    let publisher = node
        .advertise_any("/chatter", "std_msgs/String", &msg_definition, 100, false)
        .await?;

    // Data taken from example in:
    // https://wiki.ros.org/ROS/Connection%20Header
    // Note: publish expects the body length field to be present, as well as length of each field
    // - First four bytes are Body length = 9 bytes
    // - Next four bytes are length of data field = 5 bytes
    // - Lass five bytes are the data field as ascii "hello"
    // Note: Byte order!
    let data: Vec<u8> = vec![
        0x09, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x68, 0x65, 0x6c, 0x6c, 0x6f,
    ];

    // This will publish "hello" in a loop at 1Hz
    // `rostopic echo /chatter` will show the message being published
    loop {
        publisher.publish(&data).await?;
        tokio::time::sleep(std::time::Duration::from_secs(1)).await;
    }
}

#[cfg(not(feature = "ros1"))]
fn main() {
    eprintln!("This example does nothing without compiling with the feature 'ros1'");
}
