/**
 * Testing ground for hitting on rosmaster directly
 */

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .without_timestamps() // required for running wsl2
        .init()
        .unwrap();

    // XMLrpc crate looks archived and doesn't support async, but this worked out of the box
    // let request = xmlrpc::Request::new("getTopicTypes").arg("my_id");
    // let result = request.call_url("http://localhost:11311");
    // println!("Result: {result:?}");

    // rosrust uses a crate xml_rpc, but again no async
    // Skipped testing it out

    // 3rd try is the charm, serde_xmlrpc looks like it'll do the job, but we need to bring our
    // own http client. Shouldn't be an issue just use hyper (nevermind use reqwest)
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

    // Okay above is working which shows at least in theory something is possible here...

    // Work to do:
    // * Take the ros MasterApi and create valid in/out types for each api call
    // * Build out a test suite for the materapi
    // * Build out a host for the slaveapi (maybe skip some features if we can? stats?)
    // * Build out a test against our own slaveapi (maybe skip some features if we can? stats?)
    // * Actually make a connection with TCPROS

    Ok(())
}
