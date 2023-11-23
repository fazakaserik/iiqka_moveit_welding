#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include <vector>

class Waypoints
{
public:
    std::vector<geometry_msgs::msg::Pose> vector; // TODO initialize with relative zero elements (0 or current robot pose) so the builder can add to it

    // Constructor
    Waypoints() {}

    // Function to add a waypoint
    void addWaypoint(const geometry_msgs::msg::Pose& waypoint) {
        vector.push_back(waypoint);
    }

    // Function to clear all waypoints
    void clear() {
        vector.clear();
    }
};