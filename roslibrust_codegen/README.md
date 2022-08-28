# roslibrust_codegen
This crate contains the implementation of the code generation tools for ROS messages. 

It's primary method is `find_and_generate_ros_messages` which takes a vector of paths that it will search in addition to the `ROS_PACKAGE_PATH` environment variable. The additional paths are most useful in environments where ROS has not been installed.