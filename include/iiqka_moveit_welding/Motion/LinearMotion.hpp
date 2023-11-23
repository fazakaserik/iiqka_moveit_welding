#pragma once

#include "IMotion.hpp"
#include <vector>
#include <math.h>
#include "geometry_msgs/msg/pose.hpp"

class LinearMotion : public IMotion
{
private:
    geometry_msgs::msg::Pose goal_;

public:
    LinearMotion(const geometry_msgs::msg::Pose& goal)
        : goal_(goal) {}

    void apply(std::vector<geometry_msgs::msg::Pose>& waypoints, uint steps) override 
    {
        waypoints.clear();
        for (uint i = 0; i <= steps; i++) {
            double t = static_cast<double>(i) / steps;
            geometry_msgs::msg::Pose waypoint;
            waypoint.position.x = t * goal_.position.x;
            waypoint.position.y = t * goal_.position.y;
            waypoint.position.z = t * goal_.position.z;
            // Add orientation interpolation if needed
            waypoints.push_back(waypoint);
        }
    }
};
