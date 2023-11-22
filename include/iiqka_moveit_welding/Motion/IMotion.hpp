#include "geometry_msgs/msg/pose.hpp"
#include <vector>

// IMotion interface
class IMotion
{
public:
    virtual void apply(std::vector<geometry_msgs::msg::Pose>& waypoints) = 0;
    virtual ~IMotion() {}
};
