use tokio_tungstenite::*;

pub fn main() -> Result<(), String> {
    let mut client = roslibrust::Client::new("ws://localhost:9090");
    client.subscribe("/NodeStatus/configatron", |msg| {});
    client.subscribe("/NodeStatus/camera_file", |msg|{});
    client.spin();

    Ok(())
}
