//! Purpose of this example is to show how to call a ROS1 service from a Zenoh client.

// IMPORTANT to bring this trait into scope so we can access the functions it provides
// [Service] is what allows us to actually access [call]
use roslibrust::topic_provider::Service;

// IMPORTANT to bring this trait into scope so we can access the functions it provides
// [ServiceProvider] is what allows us to actually access .service_client()
use roslibrust::topic_provider::ServiceProvider;

use roslibrust_zenoh::ZenohClient;

roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces");

#[tokio::main]
async fn main() {
    env_logger::init();

    let session = zenoh::open(zenoh::Config::default()).await.unwrap();
    let client = ZenohClient::new(session);

    let client = client
        .service_client::<std_srvs::SetBool>("/my_set_bool")
        .await
        .unwrap();

    let response = client
        .call(&std_srvs::SetBoolRequest { data: true })
        .await
        .unwrap();

    println!("Got response: {response:?}");
}
