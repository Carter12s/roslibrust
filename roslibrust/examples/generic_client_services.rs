//! Purpose of this example is to show how the ServiceProvider trait can be use
//! to create code that is generic of which communication backend it will use.

#[cfg(all(feature = "rosbridge", feature = "ros1"))]
roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces");

#[cfg(all(feature = "rosbridge", feature = "ros1"))]
#[tokio::main]
async fn main() {
    // Important to bring these traits into scope so we can use them
    use roslibrust::{Service, ServiceProvider};
    env_logger::init();

    // TopicProvider cannot be an "Object Safe Trait" due to its generic parameters
    // This means we can't do:

    // Which specific TopicProvider you are going to use must be known at
    // compile time! We can use features to build multiple copies of our
    // executable with different backends. Or mix and match within a
    // single application. The critical part is to make TopicProvider a
    // generic type on you Node.

    struct MyNode<T: ServiceProvider + 'static> {
        ros: T,
    }

    // Basic example of a node that publishes and subscribes to itself
    impl<T: ServiceProvider> MyNode<T> {
        fn handle_service(
            _request: std_srvs::SetBoolRequest,
        ) -> Result<std_srvs::SetBoolResponse, Box<dyn std::error::Error + Send + Sync>> {
            // Not actually doing anything here just example
            // Note: if we did want to set a bool, we'd probably want to use Arc<Mutex<bool>>
            Ok(std_srvs::SetBoolResponse {
                success: true,
                message: "You set my bool!".to_string(),
            })
        }

        async fn run(self) {
            let _handle = self
                .ros
                .advertise_service::<std_srvs::SetBool, _>("/my_set_bool", Self::handle_service)
                .await
                .unwrap();

            let client = self
                .ros
                .service_client::<std_srvs::SetBool>("/my_set_bool")
                .await
                .unwrap();

            loop {
                tokio::time::sleep(std::time::Duration::from_millis(500)).await;
                println!("sleeping");

                let response = client
                    .call(&std_srvs::SetBoolRequest { data: true })
                    .await
                    .unwrap();
                println!("Got response: {response:?}");
            }
        }
    }

    // create a rosbridge handle and start node
    let ros = roslibrust::rosbridge::ClientHandle::new("ws://localhost:9090")
        .await
        .unwrap();
    let node = MyNode { ros };
    tokio::spawn(async move { node.run().await });

    // create a ros1 handle and start node
    let ros = roslibrust::ros1::NodeHandle::new("http://localhost:11311", "/my_node")
        .await
        .unwrap();
    let node = MyNode { ros };
    tokio::spawn(async move { node.run().await });

    loop {
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        println!("sleeping");
    }

    // With this executable running
    // RUST_LOG=debug cargo run --features ros1,topic_provider --example generic_client_services
    // You should see log output from both nodes
}

#[cfg(not(all(feature = "rosbridge", feature = "ros1")))]
fn main() {
    eprintln!(
        "This example does nothing without compiling with the feature 'rosbridge' and 'ros1'"
    );
}
