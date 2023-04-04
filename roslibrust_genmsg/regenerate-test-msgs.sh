#!/usr/bin/env bash

start_dir=$(pwd)

# cd to the directory containing this script
cd -- "$(dirname -- "${BASH_SOURCE[0]}" )"

# Try to get to the repository root
while [ ! -e .git ];
do
  cd ..
  if [[ "$(pwd)" == "/" ]]; then
    echo "Executed from outside roslibrust repo..."
    exit 1
  fi
done

echo "Generating test message headers..."

cargo run --bin gencpp -- \
--msg assets/ros1_common_interfaces/std_msgs/msg/Header.msg \
--package std_msgs \
-I std_msgs:assets/ros1_common_interfaces/std_msgs \
--output roslibrust_genmsg/test_package/include/std_msgs

cargo run --bin gencpp -- \
--msg assets/ros1_common_interfaces/common_msgs/sensor_msgs/msg/BatteryState.msg \
--package sensor_msgs \
-I std_msgs:assets/ros1_common_interfaces/std_msgs \
-I geometry_msgs:assets/ros1_common_interfaces/common_msgs/geometry_msgs \
-I sensor_msgs:assets/ros1_common_interfaces/common_msgs/sensor_msgs \
--output roslibrust_genmsg/test_package/include/sensor_msgs

cargo run --bin gencpp -- \
--msg assets/ros1_common_interfaces/ros_comm_msgs/std_srvs/srv/Trigger.srv \
--package std_srvs \
-I std_srvs:assets/ros1_common_interfaces/ros_comm_msgs/std_srvs \
--output roslibrust_genmsg/test_package/include/std_srvs

echo "Done"

cd $start_dir