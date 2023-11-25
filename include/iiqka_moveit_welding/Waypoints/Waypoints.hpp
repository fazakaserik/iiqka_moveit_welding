#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include <vector>

class Waypoints
{
public:
    std::vector<geometry_msgs::msg::Pose> vector; // TODO initialize with relative zero elements (0 or current robot pose) so the builder can add to it

    // Constructor
    Waypoints() {}

    // Function to add a waypoint
    void addWaypoint(const geometry_msgs::msg::Pose& waypoint) 
    {
        vector.push_back(waypoint);
    }
    void addWaypoint(const Eigen::Isometry3d& waypoint) 
    {
        vector.push_back(isometryToPose(waypoint));
    }

    // Function to clear all waypoints
    void clear() 
    {
        vector.clear();
    }

    // Function to intialize waypoints with zeros
    void set_zeros(uint steps) 
    {
        auto zero = Eigen::Isometry3d(
            Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Quaterniond( 0.0, 0.0, 0.0, 0.0)
        );
        set_vector(zero, steps);
    }

    void set_origin(const geometry_msgs::msg::Pose& origin)
    {
        set_vector(origin, vector.size());
    }
    void set_origin(const Eigen::Isometry3d& origin)
    {
        set_vector(origin, vector.size());
    }

private:
    geometry_msgs::msg::Pose isometryToPose(const Eigen::Isometry3d& isometry)
    {
        geometry_msgs::msg::Pose pose;

        // Assign the translation
        Eigen::Translation3d trans(isometry.translation());
        pose.position.x = trans.x();
        pose.position.y = trans.y();
        pose.position.z = trans.z();

        // Convert the rotation (which is a Quaternion in Eigen) and assign
        Eigen::Quaterniond quat(isometry.rotation());
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();

        return pose;
    }

    void set_vector(const geometry_msgs::msg::Pose& pose, size_t size)
    {
        vector.clear();

        for (uint i = 0; i < size; i++) {
            vector.push_back(pose);
        }
    }
    void set_vector(const Eigen::Isometry3d& isometry, size_t size)
    {
        vector.clear();

        for (uint i = 0; i < size; i++) {
            vector.push_back(isometryToPose(isometry));
        }
    }
};