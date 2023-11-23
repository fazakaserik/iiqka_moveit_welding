#pragma once

#include "Waypoints.hpp"
#include "../Motion/IMotion.hpp"

class WaypointsBuilder
{
private:
    Waypoints waypoints_;
    uint steps_;

public:
    WaypointsBuilder(uint steps) : steps_(steps) {}

    WaypointsBuilder& addMotion(IMotion& motion) 
    {
        motion.apply(waypoints_, steps_);
        return *this;
    }

    Waypoints build() 
    {
        return waypoints_;
    }
};
