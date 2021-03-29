use log::*;
use simple_logger::SimpleLogger;
use roslibrust::{NodeStatusMsg};


pub fn main() -> Result<(), String> {
    SimpleLogger::new().with_level(log::LevelFilter::Debug).init().unwrap();
    let mut client = roslibrust::Client::new("ws://localhost:9090");
    client.subscribe("/NodeStatus/configatron", |msg: NodeStatusMsg| {
        info!("Got configatron status: {:?}", msg);
    });
    client.subscribe("/NodeStatus/camera_file", |msg: NodeStatusMsg|{
        info!("Got camera_file status: {:?}", msg);
    });
    client.spin();

    Ok(())
}
