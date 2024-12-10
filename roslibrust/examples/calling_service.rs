#[cfg(feature = "rosbridge")]
roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces/rosapi");

/// This example shows calling a service
/// To run this example rosbridge and a roscore should be running
/// As well as the rosapi node.
/// This node calls a service on the rosapi node to get the current ros time.
#[cfg(feature = "rosbridge")]
#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use log::*;
    use roslibrust::rosbridge::ClientHandle;

    env_logger::init();

    let client = ClientHandle::new("ws://localhost:9090").await?;

    let result = client
        .call_service::<rosapi::GetTime>("/rosapi/get_time", rosapi::GetTimeRequest {})
        .await
        .expect("Error while calling get_time service");

    info!("Got time: {:?}", result);
    Ok(())
}

#[cfg(not(feature = "rosbridge"))]
fn main() {
    eprintln!("This example does nothing without compiling with the feature 'rosbridge'");
}
