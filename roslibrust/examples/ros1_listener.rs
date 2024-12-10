#[cfg(feature = "ros1")]
roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces/std_msgs");

#[cfg(feature = "ros1")]
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use roslibrust::ros1::NodeHandle;

    env_logger::init();

    let nh = NodeHandle::new("http://localhost:11311", "listener_rs").await?;
    let mut subscriber = nh.subscribe::<std_msgs::String>("/chatter", 1).await?;

    while let Some(msg) = subscriber.next().await {
        if let Ok(msg) = msg {
            log::info!("[/listener_rs] Got message: {}", msg.data);
        }
    }

    Ok(())
}

#[cfg(not(feature = "ros1"))]
fn main() {
    eprintln!("This example does nothing without compiling with the feature 'ros1'");
}
