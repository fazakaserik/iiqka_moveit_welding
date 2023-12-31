#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include <vector>
#include "../Waypoints/Waypoints.hpp"

// IMotion interface
class IMotion
{
public:
    virtual void apply(Waypoints& waypoints) = 0;
    virtual ~IMotion() {}
};
