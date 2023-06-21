#include <ros/ros.h>
#include "sensor_msgs/BatteryState.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Polygon.h"

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

    return 0;
}