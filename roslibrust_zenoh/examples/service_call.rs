//! Purpose of this example is to show how to call a ROS1 service from a Zenoh client.

// IMPORTANT to bring this trait into scope so we can access the functions it provides
// [Service] is what allows us to actually access [call]
use roslibrust::topic_provider::Service;

// IMPORTANT to bring this trait into scope so we can access the functions it provides
// [ServiceProvider] is what allows us to actually access .service_client()
use roslibrust::topic_provider::ServiceProvider;

// Bring our client type into scope
use roslibrust_zenoh::ZenohClient;

// Generate rust definitions for our messages compatible with roslibrust
roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces");

#[tokio::main]
async fn main() {
    // Initialize a logger for debug information while running the example.
    // Setting the RUST_LOG environment variable can be used to get debug
    // information if you have issues with this example.
    env_logger::init();

    // Start a zenoh session, depending on how you are running zenoh / the bridge you may
    // need to pass specific configuration into zenoh::open().
    let session = zenoh::open(zenoh::Config::default()).await.unwrap();
    // Wrap the zenoh session into a roslibrust client
    let client = ZenohClient::new(session);

    // Create a service client we'll use to call the service
    let client = client
        .service_client::<std_srvs::SetBool>("/my_set_bool")
        .await
        .unwrap();

    // Actually makes the call, sending in the request data, and getting back the response
    let response = client
        .call(&std_srvs::SetBoolRequest { data: true })
        .await
        .unwrap();

    println!("Got response: {response:?}");
}
