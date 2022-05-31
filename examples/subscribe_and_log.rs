use log::*;
use roslibrust::Client;

/// A basic example of connecting and subscribing to data.
/// This example will log received messages if run at the same time as "basic_publisher".
/// A running rosbridge websocket server at the default port (9090) is required to run this example.
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .without_timestamps() // required for running in wsl2
        .init()
        .unwrap();

    let mut client = Client::new("ws://localhost:9090").await?;
    info!("Client connected");

    let mut rx = client
        .subscribe::<roslibrust::test_msgs::Header>("talker")
        .await?;
    info!("Successfully subscribed to topic: talker");

    loop {
        let _ = rx.changed().await;
        let msg = rx.borrow();
        info!("Got msg: {:?}", msg);
    }
}
