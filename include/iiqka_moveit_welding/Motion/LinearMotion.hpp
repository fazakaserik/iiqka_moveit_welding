#pragma once

#include "IMotion.hpp"
#include <vector>
#include <math.h>
#include "geometry_msgs/msg/pose.hpp"
#include "../Waypoints/Waypoints.hpp"

class LinearMotion : public IMotion
{
private:
    geometry_msgs::msg::Pose goal_;

public:
    LinearMotion(const geometry_msgs::msg::Pose& goal)
        : goal_(goal) {}

    void apply(Waypoints& waypoints) override 
    {
        size_t vector_size = waypoints.vector.size();

        for (uint i = 0; i < vector_size; i++) {
            double t = static_cast<double>(i+1) / vector_size; // Motion should begin at i=0, thus using i+1
            waypoints.vector.at(i).position.x += t * goal_.position.x;
            waypoints.vector.at(i).position.y += t * goal_.position.y;
            waypoints.vector.at(i).position.z += t * goal_.position.z;
        }
    }
};
