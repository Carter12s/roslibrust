/**
 * Testing ground for hitting on rosmaster directly
 */

#[cfg(feature = "ros1")]
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    use roslibrust::NodeHandle;

    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .without_timestamps() // required for running wsl2
        .init()
        .unwrap();

    let nh = NodeHandle::new("http://localhost:11311", "native_ros1").await?;

    // Work to do:
    // * [DONE] Take the ros MasterApi and create valid in/out types for each api call
    // * [DONE] Build out a test suite for the materapi
    // * Build out a host for the slaveapi (maybe skip some features if we can? stats?)
    // * Build out a test against our own slaveapi (maybe skip some features if we can? stats?)
    // * Actually make a connection with TCPROS

    Ok(())
}

#[cfg(not(feature = "ros1"))]
fn main() {
    // Provide a dummy main for this example when ros1 is disabled
}
