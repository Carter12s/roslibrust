# RosLibRust
[![Rust](https://github.com/Carter12s/roslibrust/actions/workflows/rust.yml/badge.svg)](https://github.com/Carter12s/roslibrust/actions/workflows/rust.yml)
[![License:MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This package aims to provide a convenient intermediary between ROS1's rosbridge and Rust similar to roslibpy and roslibjs.

Information about the protocol can be found [here](https://github.com/RobotWebTools/rosbridge_suite).

Current Status: Ready for Beta Testing / Early Access
Intended Support: Noetic, Galactic, Rolling. Development currently focused on Noetic

| Feature                      | Status                                                      |
|------------------------------|-------------------------------------------------------------|
| tutorials                    | Planned and upcoming                                        |
| message_gen                  | Working and tested, some non-compliance with ROS likely     |
| advertise                    | Working and tested, needs documentation and error reporting |
| unadvertise                  | Working untested                                            |
| publish                      | Working and tested, needs documentation and error reporting |
| subscribe                    | Working and tested, needs documentation and error reporting |
| unsubscribe                  | Working and tested, needs documentation and error reporting | 
| services                     | Working and tested                                          |
| fragment / png / cbor        | No support planned                                          |
| cbor-raw                     | Planned                                                     |
| ros1 TCPROS / raw            | Planned                                                     |
| automatic integration tests  | Started, basic coverage                                     |
| rosbridge status access      | Planned                                                     |
| ros2                         | Planned                                                     |
| ros2 DDS / raw               | Uncertain if this package will support                      |
| rosapi                       | Planned                                                     |

## Message Generation
Message generation is provided in two APIs. The first, which is visible in `roslibrust/examples`, is a proc-macro which can be invoked to generate ROS message structs in place:

```rust
use roslibrust_codegen_macro::find_and_generate_ros_messages;

find_and_generate_ros_messages!();
```

If you have ROS installed, this macro will search for message files under paths in the `ROS_PACKAGE_PATH`. If you do not have ROS installed in your environment, you can specify search paths explicitly:

```rust
use roslibrust_codegen_macro::find_and_generate_ros_messages;

find_and_generate_ros_messages!("/path/to/noetic/packages", "/path/to/my/packages");
```

It's important to note that these macros have no way to know when the messages in the search paths change and changes to your msg and srv files won't trigger a re-compile. These macros are most useful if they're embedded directly in code that needs to use it (i.e. examples, one-off nodes that need to talk to an external ROS system).

If you want to commit the generated code or create a crate that contains the generated messages, you should use the second mechanism; a library under `roslibrust_codegen`. The proc-macro is a thin wrapper around this macro so the results will be the same.

An example of invoking it can be found in `roslibrust_test/src/main.rs`, but it's very similar to the macro example:

```rust
use roslibrust_codegen::find_and_generate_ros_messages;

let output = find_and_generate_ros_messages(vec![]);
// OR
let output = find_and_generate_ros_messages(vec!["/path/to/noetic/packages"]);
```

An example of the output based on message and service files in this repo can be found under `roslibrust_test/src/lib.rs`.
