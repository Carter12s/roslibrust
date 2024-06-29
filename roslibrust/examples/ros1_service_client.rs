roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces");

#[cfg(feature = "ros1")]
#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    use roslibrust::ros1::NodeHandle;

    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .without_timestamps()
        .init()
        .unwrap();

    let nh = NodeHandle::new("http://localhost:11311", "service_client_rs").await?;
    log::info!("Connected!");

    let response: rosapi::GetTimeResponse = nh
        .service_client::<rosapi::GetTime>("rosapi/get_time")
        .await?
        .call(&rosapi::GetTimeRequest {})
        .await?;

    log::info!("Got time: {:?}", response);

    Ok(())
}

#[cfg(not(feature = "ros1"))]
fn main() {
    eprintln!("This example does nothing without compiling with the feature 'ros1'");
}
