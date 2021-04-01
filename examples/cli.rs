use log::*;
use simple_logger::SimpleLogger;

use roslibrust::gen_msgs_amp::*;

pub fn main() -> Result<(), String> {
    SimpleLogger::new().with_level(log::LevelFilter::Debug).init().unwrap();
    let mut client = roslibrust::Client::new("ws://localhost:9090");
    client.subscribe("/NodeStatus/configatron", |msg: NodeInfo| {
        info!("Got configatron status: {:?}", msg);
    });
    client.subscribe("/NodeStatus/camera_file", |msg: NodeInfo|{
        info!("Got camera_file status: {:?}", msg);
    });
    client.subscribe("/system_monitor/memory", |msg: Memory| {
        info!("Got Memory: {:?}", msg);
    });
    client.spin();

    Ok(())
}
