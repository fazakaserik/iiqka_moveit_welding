#include "IMotion.hpp"
#include <vector>
#include <math.h>

class SinusoidalMotion : public IMotion
{
private:
    double amplitude_;
    double frequency_;
    int steps_;

public:
    SinusoidalMotion(double amplitude, double frequency, int steps)
        : amplitude_(amplitude), frequency_(frequency), steps_(steps) {}

    void apply(std::vector<geometry_msgs::msg::Pose>& waypoints) override {
        if (waypoints.empty()) return;

        double time_step = 1.0 / frequency_;
        int current_step = 0;
        for (auto& waypoint : waypoints) {
            waypoint.position.z += amplitude_ * sin(frequency_ * time_step * current_step);
            ++current_step;
        }
    }
};