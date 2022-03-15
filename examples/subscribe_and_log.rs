use log::*;
use roslibrust::{test_msgs::*, Client};

/// A basic example of connecting and subscribing to data
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .init()
        .unwrap();

    let mut client = Client::new("ws://localhost:9090").await?;
    info!("Client connected");

    let mut rx = client
        .subscribe::<roslibrust::test_msgs::Header>("talker")
        .await?;
    info!("Successfully subscribed to topic: talker");

    loop {
        client.spin();
        rx.changed().await;
        let msg = rx.borrow_and_update();
        info!("Got msg: {:?}", msg);
    }

    Ok(())
}
