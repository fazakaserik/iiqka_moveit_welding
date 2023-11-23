#pragma once

#include "Waypoints.hpp"
#include "../Motion/IMotion.hpp"

class WaypointsBuilder
{
private:
    Waypoints waypoints_;

public:
    WaypointsBuilder(uint steps)
    {
        waypoints_.set_zeros(steps);
    }

    WaypointsBuilder& addMotion(IMotion& motion) 
    {
        motion.apply(waypoints_);
        return *this;
    }

    Waypoints build() 
    {
        return waypoints_;
    }
};
