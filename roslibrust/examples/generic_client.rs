//! Purpose of this example is to show how the TopicProvider trait can be use
//! to create code that is generic of which communication backend it will use.

#[cfg(feature = "topic_provider")]
fn main() {
    use roslibrust::topic_provider::TopicProvider;

    roslibrust_codegen_macro::find_and_generate_ros_messages!(
        "assets/ros1_common_interfaces/std_msgs"
    );
    // TopicProvider cannot be an "Object Safe Trait" due to its generic parameters
    // This means we can't do:
    // let x: Box<dyn TopicProvider> = ros1::NodeHandle;

    // Which specific TopicProvider you are going to use must be known at
    // compile time! We can use features to build multiple copies of our
    // executable with different backends. Or mix and match within a
    // single application. The critical part is to make TopicProvider a
    // generic type on you Node.

    struct MyNode<T: TopicProvider> {
        ros: T,
    }

    impl<T: TopicProvider> MyNode<T> {
        async fn run(ros: T) {
            let publisher = ros.advertise::<std_msgs::String>("/chatter").await.unwrap();

            loop {}
        }
    }

    // Start the node
    
}

#[cfg(not(feature = "topic_provider"))]
fn main() {}
