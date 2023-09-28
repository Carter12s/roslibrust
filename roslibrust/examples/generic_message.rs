//! This example shows off creating a custom generic message, and leveraging serde_json's parsing resolution
//! to decode to the right type.
use log::*;
use roslibrust::ClientHandle;
use roslibrust_codegen::RosMessageType;

/// We place the ros1 generate code into a module to prevent name collisions with the identically
/// named ros2 types.
mod ros1 {
    roslibrust_codegen_macro::find_and_generate_ros_messages!(
        "assets/ros1_common_interfaces/std_msgs"
    );
}

mod ros2 {
    roslibrust_codegen_macro::find_and_generate_ros_messages_without_ros_package_path!(
        "assets/ros2_common_interfaces/std_msgs"
    );
}

/// Here we manually define a custom type with the traits needed to send it as a message
#[derive(serde::Serialize, serde::Deserialize, Clone, Debug)]
/// This serde flag lets the parser know that the incoming type won't say which variant it is
/// instead serde_json will try to deserialize as each variant in order and give whichever one first
/// succeeds
#[serde(untagged)]
enum GenericHeader {
    V1(ros1::std_msgs::Header),
    V2(ros2::std_msgs::Header),
}

/// We need to manually implement this trait for our custom type, normally this is done
/// for us by th code generation
impl RosMessageType for GenericHeader {
    /// Note: this trick only works (well) if the messages we're being generic over have
    /// the same ROS type name.
    const ROS_TYPE_NAME: &'static str = "std_msgs/Header";

    // TODO these fields should be removed and not required for this example see
    // https://github.com/Carter12s/roslibrust/issues/124
    const MD5SUM: &'static str = "";
    const DEFINITION: &'static str = "";
}

/// Sets up a subscriber that could get either of two versions of a message
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .without_timestamps() // required for running in wsl2
        .init()
        .unwrap();

    // An instance of rosbridge needs to be running at this address for the example to work
    let client = ClientHandle::new("ws://localhost:9090").await?;
    info!("ClientHandle connected");

    let rx = client.subscribe::<GenericHeader>("talker").await?;
    info!("Successfully subscribed to topic: talker");

    // Once the code reaches here, we'll need someone to publish to trigger these prints
    // In ros1 this would be `rostopic pub /talker <message data>`
    loop {
        let msg = rx.next().await;
        match msg {
            GenericHeader::V1(ros1_header) => {
                info!("Got ros1: {ros1_header:?}");
            }
            GenericHeader::V2(ros2_header) => {
                info!("Got ros2: {ros2_header:?}");
            }
        }
    }
}
