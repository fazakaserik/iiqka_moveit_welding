#include "IMotion.hpp"
#include <vector>
#include <math.h>
#include "geometry_msgs/msg/pose.hpp"

class LinearMotion : public IMotion
{
private:
    geometry_msgs::msg::Pose start_;
    geometry_msgs::msg::Pose end_;
    int steps_;

public:
    LinearMotion(const geometry_msgs::msg::Pose& start, const geometry_msgs::msg::Pose& end, int steps)
        : start_(start), end_(end), steps_(steps) {}

    void apply(std::vector<geometry_msgs::msg::Pose>& waypoints) override {
        waypoints.clear();
        for (int i = 0; i <= steps_; ++i) {
            double t = static_cast<double>(i) / steps_;
            geometry_msgs::msg::Pose waypoint;
            waypoint.position.x = (1 - t) * start_.position.x + t * end_.position.x;
            waypoint.position.y = (1 - t) * start_.position.y + t * end_.position.y;
            waypoint.position.z = (1 - t) * start_.position.z + t * end_.position.z;
            // Add orientation interpolation if needed
            waypoints.push_back(waypoint);
        }
    }
};
