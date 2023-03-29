/**
 * Testing ground for hitting on rosmaster directly
 */

#[cfg(feature = "ros1")]
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .without_timestamps() // required for running wsl2
        .init()
        .unwrap();
    let client = roslibrust::MasterClient::new(
        "http://localhost:11311",
        "http://localhost:11312",
        "/native_ros1_test",
    )
    .await?;
    println!("Got master uri: {}", client.get_uri().await?);
    println!("Got topic types: {:?}", client.get_topic_types().await?);
    println!(
        "Register service: {:?}",
        client
            .register_service("/my_service", "http://localhost:11313")
            .await?
    );
    println!(
        "lookup service: {:?}",
        client.lookup_service("/my_service").await?
    );
    println!(
        "Unregister service: {:?}",
        client
            .unregister_service("/my_service", "http://localhost:11313")
            .await?
    );
    println!(
        "Register subscriber {:?}",
        client
            .register_subscriber("/my_topic", "std_msgs/String")
            .await?
    );
    println!(
        "Lookup self: {:?}",
        client.lookup_node("/native_ros1_test").await?
    );
    println!(
        "Unregister subscriber {:?}",
        client.unregister_subscriber("/my_topic").await?
    );
    println!(
        "Register publisher {:?}",
        client
            .register_publisher("/my_topic", "std_msgs/String")
            .await?
    );
    println!(
        "Get published topics: {:?}",
        client.get_published_topics("").await?
    );
    // TODO not working
    // println!("get system state: {:?}", client.get_system_state().await?);
    println!(
        "Unregister publisher: {:?}",
        client.unregister_publisher("/my_topic").await?
    );

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
