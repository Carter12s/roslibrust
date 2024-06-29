roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces/std_msgs");

#[cfg(feature = "ros1")]
#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    use roslibrust::ros1::NodeHandle;

    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .without_timestamps() // required for running wsl2
        .init()
        .unwrap();

    let nh = NodeHandle::new("http://localhost:11311", "talker_rs")
        .await
        .map_err(|err| err)?;
    let publisher = nh.advertise::<std_msgs::String>("/chatter", 1).await?;

    for count in 0..50 {
        publisher
            .publish(&std_msgs::String {
                data: format!("hello world from rust {count}"),
            })
            .await?;
        tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
    }

    Ok(())
}

#[cfg(not(feature = "ros1"))]
fn main() {
    eprintln!("This example does nothing without compiling with the feature 'ros1'");
}
