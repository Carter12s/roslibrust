//! Goal of this executable is to asses the bandwidths and performance limits of roslibrust.
//! This may turn into a benchmark later.

use log::*;
mod ros1;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::builder()
        .parse_env(env_logger::Env::default().default_filter_or("info"))
        .format_timestamp_millis()
        .init();
    // Goal is to send a large image payload at progressively higher rates until we sta`t to lag
    // Requires running roscore
    let client =
        roslibrust::ros1::NodeHandle::new("http://localhost:11311", "performance_ramp").await?;

    let publisher = client
        .advertise::<ros1::sensor_msgs::Image>("/image_topic", 1, false)
        .await?;
    let mut subscriber = client
        .subscribe::<ros1::sensor_msgs::Image>("/image_topic", 1)
        .await?;
    // Startup delay to make sure pub and sub are connected
    tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

    // Publisher task
    tokio::spawn(async move {
        for data_size_mb in (10..=100).step_by(10) {
            // Creating a big vector here
            let mut data = vec![0; data_size_mb * 1_000_000];
            *data.last_mut().unwrap() = 69;
            let image = ros1::sensor_msgs::Image {
                header: ros1::std_msgs::Header {
                    stamp: roslibrust_codegen::Time { secs: 0, nsecs: 0 },
                    frame_id: "test".to_string(),
                    seq: data_size_mb as u32,
                },
                height: 1080,
                width: 1920,
                encoding: "bgr8".to_string(),
                is_bigendian: false as u8,
                step: 5760,
                data,
            };
            publisher.publish(&image).await.unwrap();
            // Send at 10Hz
            tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
        }
        info!("Test complete");
        // Final bonus sleep to make sure last message sends before shutting down
        tokio::time::sleep(tokio::time::Duration::from_millis(200)).await;
        std::process::exit(0);
    });

    // Subscriber task
    loop {
        if let Some(msg) = subscriber.next().await {
            match msg {
                Ok(msg) => {
                    info!("Got message @ {:?}", msg.header.seq);
                }
                Err(e) => {
                    error!("Error: {e}");
                    break;
                }
            }
        } else {
            // Shutting down
            error!("Channel dropped?");
            break;
        }
    }

    Ok(())
}
