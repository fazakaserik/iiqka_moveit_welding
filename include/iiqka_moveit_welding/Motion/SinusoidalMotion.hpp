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

    void apply(Waypoints& waypoints) override 
    {
        size_t vector_size = waypoints.vector.size();

        double time_step = 1.0 / frequency_;
        for (uint i = 0; i < vector_size; i++) {
            waypoints.vector.at(i).position.y += amplitude_ * sin(frequency_ * time_step * (i+1)); // In the i=0 step the sin motion should already start
        }
    }
};
