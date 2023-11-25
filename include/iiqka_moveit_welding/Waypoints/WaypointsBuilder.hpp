#pragma once

#include "Waypoints.hpp"
#include "../Motion/IMotion.hpp"

class WaypointsBuilder
{
private:
    Waypoints waypoints_;

public:

    WaypointsBuilder(uint steps, const Eigen::Isometry3d& origin)
    {
        waypoints_.set_zeros(steps);
        waypoints_.set_origin(origin);
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
