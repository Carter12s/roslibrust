//! A crate for interfacing to ROS (the Robot Operating System) via the [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)
//!
//! # Introduction
//! This crate is designed to provide a convenient API for interfacing between Rust and ROS.
//! This crate prioritizes ergonomics and ease-of-use over performance, while leveraging Rust's
//! exceptional type system and memory guarantees to ensure correctness.
//!
//! See the README.md to learn about the future of this crate.
//!
//! ## Why Use This Crate?
//! Do you have a fleet of systems distributed around a factory, city, nation, or solar system? Do you want to write a system
//! for managing that fleet? Problems this crate solves for you:
//!   - Rust is better than C++, Easier to test than Python, and well suited for cloud services
//!   - Native ROS communication is PITA for firewalls / VPNs, rosbridge is much easier to deal with when remote
//!   - Don't write your own REST API or leave your comfortable ROS style abstractions. Listen to the same messages, and call the same
//!     services you would if you were local
//!   - Deal with multiple versions of a message definition by utilizing Rust enums, optional fields, and other serde directives
//!   - Run a single process on a single server that can slurp data from 100's of your systems efficiently and without bottlenecks
//!  
//! ### Why not use rosrust?
//! rosrust is a completely valid alternative to this crate! The reasons to use this crate are the same reasons why people use rosbridge
//! in general:
//!   - rosbridge is served on a single port meaning it can more easily be exposed through docker or firewalls
//!   - rosbirdge's JSON encoding makes it possible to work with multiple versions of a ROS message at the same time. A custom written
//!     message type that utilizes enums / optional fields allows a rust client to deal with version differences amongst a fleet of
//!     systems
//!   - async. rosrust was written before async/.await was standardized in rust an is fundamentally a synchronous paradigm. This makes
//!     it much more difficult to leverage Rust's "fearless concurrency" than with `roslibrust`
//!
//! ### Is rosbridge slower?
//! Yes! Your mileage may vary. This crate is much more designed for low bandwidth communication with a large fleet of systems than
//! writing high performance nodes designed to run locally.
//!
//! ## Message Generation
//! Message generation is provided in two APIs. The first, which is visible in `roslibrust/examples`, is a proc-macro which can be invoked to generate ROS message structs in place:
//! ```ignore
//! use roslibrust_codegen_macro::find_and_generate_ros_messages;
//! find_and_generate_ros_messages!();
//! ```
//! If you have ROS installed, this macro will search for message files under paths in the `ROS_PACKAGE_PATH`. If you do not have ROS installed in your environment, you can specify search paths explicitly:
//! ```ignore
//! use roslibrust_codegen_macro::find_and_generate_ros_messages;
//! find_and_generate_ros_messages!("/path/to/noetic/packages", "/path/to/my/packages");
//! ```
//! It's important to note that these macros have no way to know when the messages in the search paths change and changes to your msg and srv files won't trigger a re-compile. These macros are most useful if they're embedded directly in code that needs to use it (i.e. examples, one-off nodes that need to talk to an external ROS system).
//! If you want to commit the generated code or create a crate that contains the generated messages, you should use the second mechanism; a library under the optional `codegen` feature of `roslibrust`. The proc-macro is a thin wrapper around this function so the results will be the same.
//! An example of invoking it can be found in `roslibrust_test/src/main.rs`, but it's very similar to the macro example:
//! ```
//! use roslibrust_codegen::find_and_generate_ros_messages;
//! let output = find_and_generate_ros_messages(vec![]);
//! // OR
//! let output = find_and_generate_ros_messages(vec!["/path/to/noetic/packages".into()]);
//! ```
//! An example of the output based on message and service files in this repo can be found under `roslibrust_test/src/lib.rs`.
//!
//! A full example of incorporating code generation into a project with a build.rs file can be found in example_project.
//! This is the recommended method of incorporating code generation, as this allows automatic detection and re-building
//! when message files are edited which is not possible with the proc_macro approach.
//!
//! ## How Does It Work?
//! When you create a new client via ClientHandle::new() or ClientHandle::new_with_options() a new connection to rosbridge is created.
//! This new connection is literally opening a new Websocket. A specific "stubborn spin" tokio task is created which handles reading
//! from the websocket, and automatically connecting / reconnecting if communication is interrupted.
//!
//! This central stubborn_spin task has the only access to the -read- half of the websocket and is constantly .await'ing any new messages
//! on the websocket connection. When a new message is received from rosbridge it does its best to dispatch it to the relevant entities
//! that are registered with the Client.
//!
//! It is completely valid and recommended to clone() ClientHandle to pass access around your application, in fact many of the types
//! returned by roslibrust include clones of the client handle within them to enable functionality like automatically de-registering
//! when dropped. It is not recommended, although completely valid, to create multiple new clients to the same rosbridge server as
//! the additional websockets created add overhead to both roslibrust and rosbridge.
//!
//! ### How Subscribers Work
//! Each time subscribe is called, a new queue for that subscriber is created. When a `publish` message is received from rosbridge,
//! the message is duplicated and inserted into the queue for *each* subscriber. This means if you call subscribe multiple times
//! on the same topic, each of the returned subscribers will receive a copy of every message. Currently, the size of the queue for
//! every publisher is hardcoded to 1_000 messages control of this will be provided in a future version.
//!
//! If the queue for a subscriber is full and a new message arrives the central spin task will not block but simply drop that message
//! for that subscriber (warnings will be logged).
//!
//! Internally roslibrust type-erases the type that `subscribe` is called with an stores a callback where the deserialization
//! is embedded within the callback. Roslibrust does not check that the type that subscribe is called with matches the type of the topic.
//! If an incorrect type is used, each time a message is received on the topic it will fail to de-serialize and an error will be emitted
//! by the subscriber. This can be useful when building client designed to work with multiple different versions of a message definition.
//!
//! When the subscriber returned from the subscribe call is dropped it removes is queue from the client. When the last subscriber
//! on a given topic dropped the client will automatically unsubscribe from the topic with rosbridge.
//!
//! ### How Publishers Work
//! When advertise is called a publisher an advertise message is sent to rosbridge_server and a publisher returned.
//! Dropping the publisher will automatically unadvertise the topic.
//! roslibrust currently does not support multiple publishers / multiple advertises for a single topic.
//!
//! ### How Service Servers Work
//! When advertise service is called you must pass into it a callback conforming to the libraries requirements.
//! Specifically, roslibrust attempts to follow "good" ros error handling convention and be as compatible as possible
//! with various error types; however, due to the async nature of the crate `Box<dyn Error + Send + Sync>` is needed.

// Re-export common types and traits under the roslibrust namespace
pub use roslibrust_common::*;

// If the rosapi feature is enabled, export the roslibrust_rosapi crate under rosapi
#[cfg(feature = "rosapi")]
pub use roslibrust_rosapi as rosapi;

// If the ros1 feature is enabled, export the roslibrust_ros1 crate under ros1
#[cfg(feature = "ros1")]
pub use roslibrust_ros1 as ros1;

// If the rosbridge feature is enabled, export the roslibrust_rosbridge crate under rosbridge
#[cfg(feature = "rosbridge")]
pub use roslibrust_rosbridge as rosbridge;

// If the zenoh feature is enabled, export the roslibrust_zenoh crate under zenoh
#[cfg(feature = "zenoh")]
pub use roslibrust_zenoh as zenoh;

// If the mock feature is enabled, export the roslibrust_mock crate under mock
#[cfg(feature = "mock")]
pub use roslibrust_mock as mock;
