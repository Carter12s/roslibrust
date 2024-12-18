// One way to import message definitions:
#[cfg(feature = "rosbridge")]
roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces");

// A basic service server exampple, that logs the request is recieves and returns
// a canned response.
#[cfg(feature = "rosbridge")]
fn my_service(
    request: std_srvs::SetBoolRequest,
    my_string: &str,
) -> Result<std_srvs::SetBoolResponse, Box<dyn std::error::Error + Send + Sync>> {
    log::info!("Got request to set bool: {request:?}");
    log::info!("Using my string: {}", my_string); // Use the string here

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
#[cfg(feature = "rosbridge")]
#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    // Create a new client
    let client = roslibrust::rosbridge::ClientHandle::new("ws://localhost:9090").await?;

    // The string you want to pass in to the closure
    let my_string = "Some string";

    // Actually advertise our service
    // The handle returned here establishes the lifetime of our service and dropping it will unadvertise the service
    let _handle = client
        .advertise_service::<std_srvs::SetBool, _>(
            "/my_set_bool",
            move |request: std_srvs::SetBoolRequest| -> Result<
                std_srvs::SetBoolResponse,
                Box<dyn std::error::Error + Send + Sync>,
            > { my_service(request, my_string) },
        )
        .await?;

    // Now try manually calling the service with the command line!

    tokio::signal::ctrl_c().await?;
    Ok(())
}

#[cfg(not(feature = "rosbridge"))]
fn main() {
    eprintln!("This example does nothing without compiling with the feature 'rosbridge'");
}
