#include "Waypoints.hpp"
#include "IMotion.hpp"

class WaypointsBuilder
{
private:
    Waypoints waypoints_;

public:
    WaypointsBuilder& addMotion(IMotion& motion) {
        motion.apply(waypoints_.waypoints);
        return *this;
    }

    Waypoints build() {
        return waypoints_;
    }
};
