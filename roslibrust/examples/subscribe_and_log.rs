#[cfg(feature = "rosbridge")]
roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces");

/// A basic example of connecting and subscribing to data.
/// This example will log received messages if run at the same time as "basic_publisher".
/// A running rosbridge websocket server at the default port (9090) is required to run this example.
#[cfg(feature = "rosbridge")]
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use log::*;
    env_logger::init();

    let client = roslibrust::rosbridge::ClientHandle::new("ws://localhost:9090").await?;
    info!("ClientHandle connected");

    let rx = client.subscribe::<std_msgs::Header>("talker").await?;
    info!("Successfully subscribed to topic: talker");

    loop {
        let msg = rx.next().await;
        info!("Got msg: {:?}", msg);
    }
}

#[cfg(not(feature = "rosbridge"))]
fn main() {
    eprintln!("This example does nothing without compiling with the feature 'rosbridge'");
}
