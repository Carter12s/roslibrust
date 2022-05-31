use std::path::{Path, PathBuf};

use log::*;
use roslibrust::test_msgs::{self, Header, TimeI};
use roslibrust::{message_gen, util, Client, RosMessageType};
use serde::{Deserialize, Serialize};

#[derive(Deserialize, Serialize, Debug, Default, Clone, PartialEq)]
pub struct GetTimeResponse {
    pub time: TimeI,
}

impl RosMessageType for GetTimeResponse {
    const ROS_TYPE_NAME: &'static str = "rosapi/GetTimeResponse";
}

#[derive(Deserialize, Serialize, Debug, Default, Clone, PartialEq)]
struct Empty {}

// TODO this is hack for now
impl RosMessageType for Empty {
    const ROS_TYPE_NAME: &'static str = "";
}

/// This example shows calling a service
/// To run this example rosbridge and a roscore should be running
/// As well as the rosapi node.
/// This node calls a service on the rosapi node to get the current ros time.

#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .without_timestamps()
        .init()
        .unwrap();

    // Uncomment this line if you want to test message generation for the rosapi package
    // gen_rosapi_msgs();

    let mut client = Client::new("ws://localhost:9090").await?;

    let result = client
        .call_service::<Empty, GetTimeResponse>("/rosapi/get_time", Empty {})
        .await
        .expect("Error while calling get_time service");

    info!("Got time: {:?}", result);
    Ok(())
}

/// This function can be used to generate the types place manually in the source above
/// Running this requires ros installed (and sourced) and rosapi installed
fn gen_rosapi_msgs() {
    let rosapi_path = std::process::Command::new("rospack")
        .args(["find", "rosapi"])
        .output()
        .unwrap();
    let rosapi_path = PathBuf::from(std::str::from_utf8(&rosapi_path.stdout).unwrap().trim());
    debug!("Looking in {:?} for srv files", &rosapi_path);

    let files = util::recursive_find_srv_files(&rosapi_path)
        .into_iter()
        .map(|f| f.path)
        .collect();

    debug!("Generating rust code for: {:?}", &files);
    let dest_path = Path::new("/tmp/rosapi.rs");

    let opts = message_gen::MessageGenOpts::new(files)
        .destination(dest_path)
        .format(true);
    message_gen::generate_messages(&opts).unwrap();
}
