use log::*;
use roslibrust::test_msgs::Header;
use roslibrust::Client;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .init()
        .unwrap();

    let mut client = Client::new("ws://localhost:9090").await?;

    client.advertise::<Header, &str>("talker").await?;

    loop {
        let msg = Header::default();
        client.publish("talker", msg).await;
        info!("Published msg...");
        tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
    }

    Ok(())
}
