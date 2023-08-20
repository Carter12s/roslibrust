use roslibrust::ClientHandle;

// One way to import message definitions:
roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces");

// A basic service server exampple, that logs the request is recieves and returns
// a canned response.
fn my_service(
    request: std_srvs::SetBoolRequest,
) -> Result<std_srvs::SetBoolResponse, Box<dyn std::error::Error + Send + Sync>> {
    log::info!("Got request to set bool: {request:?}");
    Ok(std_srvs::SetBoolResponse {
        success: true,
        message: "You set my bool!".to_string(),
    })
}

/// This examples shows hosting a service server and calling it to confirm it is working
///
/// This example expects a running rosbridge_server with websockets at port 9090 (the default)
///
/// This example works with either a ROS1 or ROS2 rosbridge_server!
///
/// To run this example: `cargo run --example service_server`
///
/// Note: for this example to work rosbridge has to be able to find the message definition for AddTwoInts
/// Look in the rosbridge logs for "Unable to load manifest for package test_msgs" indicating rosbridge has
/// not found the service definition files this example is using.
///
/// Running this example will cause a service server to start and print something like:
/// ```bash
/// DEBUG [tungstenite::handshake::client] Client handshake done.
/// DEBUG [roslibrust::rosbridge::comm] Sending advertise service on /my_set_bool w/ std_srvs/SetBool
/// DEBUG [roslibrust::rosbridge::client] Starting stubborn_spin
/// ```
///
/// To actually exercise our service we need to call it with something.
/// One option would be to use a ros commaline tool: `rosservice call /my_set_bool "data: false"` or `ros2 service call /my_set_bool std_srvs/srv/SetBool data:\ false\ `.
#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    // Initialize a basic logger useful if anything goes wrong while running the example
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .without_timestamps() // Required for running in wsl2
        .init()
        .unwrap();

    // Create a new client
    let client = ClientHandle::new("ws://localhost:9090").await?;

    // Actually advertise our service
    // The handle returned here establishes the lifetime of our service and dropping it will unadvertise the service
    let _handle = client
        .advertise_service::<std_srvs::SetBool>("/my_set_bool", my_service)
        .await?;

    // Now try manually calling the service with the command line!

    tokio::signal::ctrl_c().await?;
    Ok(())
}
