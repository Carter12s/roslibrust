#include <ros/ros.h>
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Polygon.h"
#include "test_package/Test.h"

int main() {
    sensor_msgs::BatteryState battery_state;
    std_srvs::Trigger trigger;

    geometry_msgs::Point32 point;
    point.x = 1.0;
    point.y = 2.0;
    point.z = 3.0;
    geometry_msgs::Polygon polygon;
    polygon.points.push_back(point);

    std::cout << "Polygon: " << polygon << std::endl;
    std::cout << "Test State: " << test_package::Test::RUNNING_STATE << std::endl;

    static_assert("c9a58c1b0b154e0e6da7578cb991d214" == ros::message_traits::MD5Sum<sensor_msgs::CameraInfo>().value());

    return 0;
}