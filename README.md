# RosLibRust
[![Noetic](https://github.com/Carter12s/roslibrust/actions/workflows/noetic.yml/badge.svg)](https://github.com/Carter12s/roslibrust/actions/workflows/noetic.yml)
[![Galactic](https://github.com/Carter12s/roslibrust/actions/workflows/galactic.yml/badge.svg)](https://github.com/Carter12s/roslibrust/actions/workflows/galactic.yml)
[![Humble](https://github.com/Carter12s/roslibrust/actions/workflows/humble.yml/badge.svg)](https://github.com/Carter12s/roslibrust/actions/workflows/humble.yml)
[![License:MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This package aims to provide a convenient intermediary between ROS1's rosbridge and Rust similar to roslibpy and roslibjs.

Information about the protocol can be found [here](https://github.com/RobotWebTools/rosbridge_suite).

Note on documentation:
All information about the crate itself (examples, documentation, tutorials, etc.) lives in the source code and can be viewed on [docs.rs](https://docs.rs/roslibrust).
This readme is for "Meta" information about developing for the crate.

Fully Supported via rosbridge: Noetic, Galactic, Humble.

## Code Generation of ROS Messages

The crates `roslibrust_codegen` and `roslibrust_codegen_macro` support code generation in Rust for ROS1 and ROS2 message, service, and action files. Many of the examples use the macro for convenience. `find_and_generate_ros_messages` is a macro which accepts an optional list of paths relative to the project workspace directory and will additionally check the `ROS_PACKAGE_PATH` environment variable for paths to ROS packages.

It's used like this:
```rust
roslibrust_codegen_macro::find_and_generate_ros_messages!("assets/ros1_common_interfaces/std_msgs");
```

Code generation can also be done with a script using the same code generation backend called by the macro. See `roslibrust_test/src/main.rs` for a demonstration of how this can be done.

## Experimental Support for ROS1 Native

If built with the `ros1` feature, `roslibrust` exports some experimental support for implementing nodes which talk to other ROS1 nodes using the TCPROS protocol without the need for the rosbridge as an intermediary. See `ros1_talker.rs` and `ros1_listener.rs` under `roslibrust/examples` to see usage. This implementation is relatively new, incomplete, and untested. Filing issues on bugs encountered is very appreciated!

See this issue filter for known issues: https://github.com/Carter12s/roslibrust/labels/ros1

## Roadmap

Rough overview of the features planned to built for this crate in what order:

| Feature                      | rosbridge                                                   | ROS1 | ROS2 |
|------------------------------|-------------------------------------------------------------|------|------|
| examples                     | ✅                                                         | ✅   | x    |
| message_gen                  | ✅                                                         | ✅   | ✅  |
| advertise / publish          | ✅                                                         | ✅   | x    |
| unadvertise                  | ✅                                                         | x    | x    |
| subscribe                    | ✅                                                         | ✅   | x    |
| unsubscribe                  | ✅                                                         | x    | x    |
| services                     | ✅                                                         | x    | x    |
| rosapi                       | ✅ (ROS1 only for now)                                     | N/A  | N/A  |
| TLS / wss://                 | Should be working, untested                                 | N/A  | N/A  |
| ROS2 msgs length limits      | Planned                                                     | N/A  | x    |
| cbor                         | Planned                                                     | N/A  | N/A  |
| rosbridge status access      | Planned                                                     | N/A  | N/A  |
| rosout logger                | Planned                                                     | x    | x    |
| auth                         | Planned                                                     | N/A  | N/A  |
| fragment / png               | Uncertain if this package will support                      | x    | N/A  |
| cbor-raw                     | Uncertain if this package will support                      | N/A  | N/A  |


## Contributing

Contribution through reporting of issues encountered and implementation in PRs is welcome! Before landing a large PR with lots of code implemented, please open an issue if there isn't a relevant one already available and chat with a maintainer to make sure the design fits well with all supported platforms and any in-progress implementation efforts.

### Running Tests

There are various unit tests and integration tests behind feature flags. For tests with ROS1, both through rosbridge and native clients, you'll need a locally running `rosbridge_websocket` node and `rosmaster`. Then run with `cargo test --features "ros1_test ros1"`. For tests with ROS2, you'll need a running rosbridge server, then run with `cargo test --features "ros2_test"`. You can find relevant `Dockerfile`s and docker compose configurations udner the `docker` directory.