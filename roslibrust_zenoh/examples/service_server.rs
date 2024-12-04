//! Purpose of this example is to show how to host service visible to ROS1 from a Zenoh client.

// IMPORTANT to bring this trait into scope so we can access the functions it provides
// [ServiceProvider] is what allows us to actually access .service_client()
use roslibrust::topic_provider::ServiceProvider;
use roslibrust_zenoh::ZenohClient;

// IMPORTANT this example will not work with the default zenoh-ros1-bridge settings!
// It requires the flag `--client_bridging_mode auto` to be set when the bridge is started
// Otherwise the service will not be advertised to ROS1 master.

// Generate rust definitions for our messages
roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces");

#[tokio::main]
async fn main() {
    // Setup a logger for debugging purposes
    // Run this example with RUST_LOG=debug for more information if this doesn't work for you
    env_logger::init();

    // Create our zenoh client, depending on how you are running zenoh / the bridge you may
    // need to pass specific configuration in here.
    let session = zenoh::open(zenoh::Config::default()).await.unwrap();
    let client = ZenohClient::new(session);

    // Service will remain alive until handle is dropped
    let _handle = client
        .advertise_service::<std_srvs::SetBool, _>(
            "/my_set_bool",
            |request: std_srvs::SetBoolRequest| {
                log::info!("Got request to set bool: {request:?}");
                Ok(std_srvs::SetBoolResponse {
                    success: true,
                    message: "You set my bool!".to_string(),
                })
            },
        )
        .await
        .unwrap();

    // Wait for ctrl_c to kill this process
    tokio::signal::ctrl_c().await.unwrap();

    // While this example is running:
    // `rosservice list` should show /my_set_bool
    // `rosservice call /my_set_bool "data: true"` should work
}
