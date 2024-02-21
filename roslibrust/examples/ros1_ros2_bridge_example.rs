//! This example shows how you can generate message definitions for
//! both ROS1 types and ROS2 types, can use two clients to communicate
//! with both versions of ROS at the same time.

use log::*;
use roslibrust::ClientHandle;

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

/// The goal of this example is to create a "baby bridge" that will listen to a ros1 message,
/// and re-publish it to ros2.
///
/// This is not a recommended use of `roslibrust`, but is just a nice way to show-off the
/// inter-operability.
///
/// To run this example, two instances of rosbridge must be running,
/// a ROS1 instance on port 9090
/// and a ROS2 instance on port 9091
/// With both ros1 and ros2 installed on your system and correctly sourced this can be achieved with:
/// `roslaunch rosbridge_server rosbridge_websocket.launch`
/// `ros2 run rosbridge_server rosbridge_websocket -- --port 9091`
///
/// If you don't have ros1 or ros2 installed, but still want to run this example, the docker images
/// in the docker/ directory may be useful. With docker installed on your system:
///
/// For ros1:
/// ```bash
/// docker run -it -p 9090:9090 carter12s/roslibrust-ci-noetic:latest bash # Runs a docker image with ros1 and rosbridge installed and drops you into a shell
/// source /opt/ros/noetic/setup.bash # Setup the ros environment within the docker image
/// roslaunch rosbridge_server rosbridge_websocket.launch # Starts a ros1 rosbridge inside the docker image
/// ```
///
/// For ros2:
/// ```bash
/// docker run -it -p 9091:9091 carter12s/roslibrust-ci-galactic:latest bash
/// source /opt/ros/noetic/setup.bash # Setup the ros environment within the docker image
/// ros2 run rosbridge_server rosbridge_websocket -- --port 9091 # Starts a ros2 rosbridge inside the docker image
/// ```
///
/// After both bridges are up and running, run this example.
/// With this example running you should then be able to use the ros1 command line tools to publish a message,
/// and see them appear in ros2 with its command line tools
#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<(), anyhow::Error> {
    // Initialize a basic logger useful if anything goes wrong while running the example
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .without_timestamps() // Required for running in wsl2
        .init()
        .unwrap();

    info!("Attempting to connect to ros1...");
    let ros1_client = ClientHandle::new("ws://localhost:9090").await?;
    info!("ros1 client successfully connected!");
    info!("Attempting to connect to ros2...");
    let ros2_client = ClientHandle::new("ws://localhost:9091").await?;
    info!("ros2 client successfully connected!");

    let subscriber = ros1_client
        .subscribe::<ros1::std_msgs::Header>("/bridge_header")
        .await?;

    let publisher = ros2_client.advertise("/bridge_header").await?;

    // We now hit our main loop, which will run for the rest of the program
    loop {
        // Simultaneously await either a user interrupt, or a new message
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                // We got a CTRL+C on command line, exit the example by leaving our loop
                break;
             }
            msg = subscriber.next() => {
                info!("Got a message from ros1 {msg:?}");

                // We got a new message!
                // Covert it to the ros2 format
                let converted_msg = ros2::std_msgs::Header {
                    stamp: msg.stamp,
                    frame_id: msg.frame_id,
                };

                // and re-publish it!
                publisher.publish(converted_msg).await?;
                info!("Message successfully sent to ros2!");
            }
        }
    }

    Ok(())
}
