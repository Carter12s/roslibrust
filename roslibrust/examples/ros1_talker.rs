roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces/std_msgs");

#[cfg(feature = "ros1")]
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use roslibrust::NodeHandle;

    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .without_timestamps() // required for running wsl2
        .init()
        .unwrap();

    let nh = NodeHandle::new("http://localhost:11311", "talker_rs")
        .await
        .map_err(|err| err)?;
    let publisher = nh.advertise::<std_msgs::String>("/chatter", 1).await?;

    let mut counter = 0u32;

    while counter < 1000 {
        publisher
            .publish(&std_msgs::String {
                data: format!("hello world from rust {counter}"),
            })
            .await?;
        counter = counter.wrapping_add_signed(1);
        tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
    }

    Ok(())
}
