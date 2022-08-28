# roslibrust_codegen_macro
This crate defines the proc-macro for generating ROS message types. It invokes APIs from the `roslibrust_codegen` crate.

## Warning
This macro cannot detect if the message files it generates from have changed. If you are creating a crate which just contains your message definitions and won't be recompiled otherwise, you'll likely want to use `roslibrust_codegen` with a `build.rs` script.

## Usage
If you're generating messages in an environment with ROS installed, no arguments need to be passed.

```rust
use roslibrust_codegen_macro::find_and_generate_ros_messages;

find_and_generate_ros_messages!();
```

If you're generating without ROS installed or your environment can't depend on the `ROS_PACKAGE_PATH` variable, you can specify additional paths to search:

```rust
use roslibrust_codegen_macro::find_and_generate_ros_messages;

find_and_generate_ros_messages!("/path/to/my/msg/package", "/opt/ros/noetic");
```