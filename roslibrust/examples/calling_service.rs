use log::*;
use roslibrust::ClientHandle;

roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces",);

/// This example shows calling a service
/// To run this example rosbridge and a roscore should be running
/// As well as the rosapi node.
/// This node calls a service on the rosapi node to get the current ros time.
#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<(), anyhow::Error> {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .without_timestamps() // Required for running in wsl2
        .init()
        .unwrap();

    let client = ClientHandle::new("ws://localhost:9090").await?;

    let result = client
        .call_service::<rosapi::GetTime>("/rosapi/get_time", rosapi::GetTimeRequest {})
        .await
        .expect("Error while calling get_time service");

    info!("Got time: {:?}", result);
    Ok(())
}
