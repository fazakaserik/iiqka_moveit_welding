#include "geometry_msgs/msg/pose.hpp"
#include <vector>

class Waypoints
{
public:
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // Constructor
    Waypoints() {}

    // Function to add a waypoint
    void addWaypoint(const geometry_msgs::msg::Pose& waypoint) {
        waypoints.push_back(waypoint);
    }

    // Function to clear all waypoints
    void clear() {
        waypoints.clear();
    }
};