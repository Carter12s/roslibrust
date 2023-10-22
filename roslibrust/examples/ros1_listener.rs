roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces/std_msgs");

#[cfg(feature = "ros1")]
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    use roslibrust::NodeHandle;

    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .without_timestamps()
        .init()
        .unwrap();

    let nh = NodeHandle::new("http://localhost:11311", "listener_rs").await?;
    let mut subscriber = nh.subscribe::<std_msgs::String>("/chatter", 1).await?;

    while let Ok(msg) = subscriber.next().await {
        log::info!("[/listener_rs] Got message: {}", msg.data);
    }

    Ok(())
}

#[cfg(not(feature = "ros1"))]
fn main() {
    eprintln!("This example does nothing without compiling with the feature 'ros1'");
}
