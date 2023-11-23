#pragma once

#include "IMotion.hpp"
#include <vector>
#include <math.h>
#include "../Waypoints/Waypoints.hpp"

class SinusoidalMotion : public IMotion
{
private:
    double amplitude_;
    double frequency_;

public:
    SinusoidalMotion(double amplitude, double frequency)
        : amplitude_(amplitude), frequency_(frequency) {}

    void apply(Waypoints& waypoints, uint steps) override 
    {
        double time_step = 1.0 / frequency_;
        for (uint i = 0; i <= steps; i++) {
            geometry_msgs::msg::Pose waypoint;
            waypoint.position.z += amplitude_ * sin(frequency_ * time_step * i);
            waypoints.vector.push_back(waypoint);
        }
    }
};
