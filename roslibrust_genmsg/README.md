# roslibrust_genmsg
A CLI application and library for generating ROS C++ headers using `roslibrust_codegen` as the backend.

Running the command line application:
```bash
roslibrust$ mkdir -p /tmp/sensor_msgs && \ 
cargo run --bin gencpp -- \
--msg assets/ros1_common_interfaces/common_msgs/sensor_msgs/msg/BatteryState.msg \
--package sensor_msgs \
-I std_msgs:assets/ros1_common_interfaces/std_msgs \
-I geometry_msgs:assets/ros1_common_interfaces/common_msgs/geometry_msgs \
-I sensor_msgs:assets/ros1_common_interfaces/common_msgs/sensor_msgs \
--output /tmp/sensor_msgs
```

## Key Differences with the official CLI
* The include paths must be to the top-level of the message package directory as `roslibrust_codegen` uses this to determine the ROS version.
* The output is to a path instead of a file. Filenames are assumed based on message or service file name.
* The include paths must include the package that the input message comes from as `roslibrust_codegen` does not currently expose an API for parsing a message given a message file path. Consequently, you'll want to include any dependencies of other messages in the package so that `roslibrust_codegen` can solve the dependency graph.