use roslibrust_zenoh::ZenohClient;

/// IMPORTANT to bring this trait into scope so we can access the functions it provides
/// This trait provides the [next][roslibrust::Subscribe::next] function on ZenohSubscriber
use roslibrust::Publish;
/// IMPORTANT to bring this trait into scope so we can access the functions it provides
/// This trait provides the [subscribe][roslibrust::TopicProvider::subscribe] and [advertise][roslibrust::TopicProvider::advertise] functions on ZenohCilent
use roslibrust::TopicProvider;

// Generate rust definitions for our messages
roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces");

// The example expects a zenoh-ros1-bridge to be running see [here](https://github.com/eclipse-zenoh/zenoh-plugin-ros1)
// for details on running the bridge.

// While the bridge is running (with a rosmaster either internally or externally), the following command can be used
// to test the functionality: `rostopic echo /chatter`
// Or run the subscriber example `cargo run --example subscriber`
#[tokio::main]
async fn main() {
    // Setup a logger for debugging purposes
    // use RUST_LOG=debug for more information if this doesn't work for you
    env_logger::init();

    let session = zenoh::open(zenoh::Config::default()).await.unwrap();
    let client = ZenohClient::new(session);

    // Create a zenoh subscriber to the ros topic /chatter
    // Internally this handles the "topic mangling" that zenoh-ros1-plugin / zenoh-ros1-bridge performs
    // and sets up deserialization of the ROS1 type into our Rust type
    let publisher = client
        .advertise::<std_msgs::String>("/chatter")
        .await
        .unwrap();

    // Run at 1Hz
    let mut interval = tokio::time::interval(std::time::Duration::from_secs(1));

    loop {
        let msg = std_msgs::String {
            data: "Hello world".to_string(),
        };
        publisher.publish(&msg).await.unwrap();
        println!("Published!");
        interval.tick().await;
    }
}
