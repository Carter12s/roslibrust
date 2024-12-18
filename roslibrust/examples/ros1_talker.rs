#[cfg(feature = "ros1")]
roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces");

#[cfg(feature = "ros1")]
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use roslibrust::ros1::NodeHandle;

    env_logger::init();

    let nh = NodeHandle::new("http://localhost:11311", "talker_rs")
        .await
        .map_err(|err| err)?;
    let publisher = nh
        .advertise::<geometry_msgs::PointStamped>("/my_point", 1, false)
        .await?;

    for count in 0..50 {
        let mut msg = geometry_msgs::PointStamped::default();
        msg.point.x = count as f64;
        publisher.publish(&msg).await?;
        tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
    }

    Ok(())
}

#[cfg(not(feature = "ros1"))]
fn main() {
    eprintln!("This example does nothing without compiling with the feature 'ros1'");
}
