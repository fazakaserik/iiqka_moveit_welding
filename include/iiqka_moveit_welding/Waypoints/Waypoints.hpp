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
    void addWaypoint(const geometry_msgs::msg::Pose& waypoint) 
    {
        vector.push_back(waypoint);
    }

    // Function to clear all waypoints
    void clear() 
    {
        vector.clear();
    }

    // Function to intialize waypoints with zeros
    void set_zeros(uint steps) 
    {
        vector.clear();

        // Initialize 0 pose
        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.0;
        pose.position.y = 0.0;
        pose.position.z = 0.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0; // Quaternion representing no rotation

        for (uint i = 0; i < steps; i++) {
            vector.push_back(pose);
        }
    }
};