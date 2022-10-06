use roslibrust::ClientHandle;

// One way to import message definitions
roslibrust_codegen_macro::find_and_generate_ros_messages!(
    "assets/test_msgs",
    "assets/ros1_common_interfaces",
);

fn my_service(
    request: test_msgs::AddTwoIntsRequest,
) -> Result<test_msgs::AddTwoIntsResponse, Box<dyn std::error::Error + Send + Sync>> {
    let result = request.a.checked_add(request.b);
    match result {
        Some(sum) => Ok(test_msgs::AddTwoIntsResponse { sum }),
        // TODO get the correct error type for overflow here
        None => Err(Box::new(std::io::Error::new(
            std::io::ErrorKind::AddrInUse,
            format!("Overflow!"),
        ))),
    }
}

/// This examples shows hosting a service server and calling it to confirm it is working
///
/// This example expects a running rosbridge_server with websockets at port 9090 (the default)
///
/// To run this example: `cargo run --example service_server`
///
/// Note: for this example to work rosbridge has to be able to find the message definition for AddTwoInts
/// Look in the rosbridge logs for "Unable to load manifest for package test_msgs" indicating rosbridge has
/// not found the service definition files this example is using
///
/// TODO: figure out a better way to explain to people how rosbridge finds this, maybe use a more standard srv file?
#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    // Initialize a basic logger useful if anything goes wrong while running the example
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .without_timestamps() // Required for running in wsl2
        .init()
        .unwrap();

    // Create a new client
    let mut client = ClientHandle::new("ws://localhost:9090").await?;

    // Actually advertise our service
    // The handle returned here establishes the lifetime of our service and dropping it will unadvertise the service
    let _handle = client
        .advertise_service::<test_msgs::AddTwoInts>("/add_two_ints", my_service)
        .await?;

    // Sleep for a small amount to ensure our service is advertised before we try to call it
    // Otherwise the call_service request can arrive at rosbridge before rosbridge has finished processing the
    // advertise_service request.
    tokio::time::sleep(tokio::time::Duration::from_millis(200)).await;

    // Actually call our service with valid inputs and get the result
    let result: test_msgs::AddTwoIntsResponse = client
        .call_service("/add_two_ints", test_msgs::AddTwoIntsRequest { a: 1, b: 2 })
        .await?;

    assert_eq!(result.sum, 3);

    Ok(())
}
