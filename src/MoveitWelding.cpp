// Copyright 2022 Áron Svastits
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <math.h>

#include <memory>

#include "iiqka_moveit_welding/moveit_welding.hpp"
#include "iiqka_moveit_welding/Waypoints/Waypoints.hpp"
#include "iiqka_moveit_welding/Waypoints/WaypointsBuilder.hpp"
#include "iiqka_moveit_welding/Motion/LinearMotion.hpp"
#include "iiqka_moveit_welding/Motion/SinusoidalMotion.hpp"

int main(int argc, char * argv[])
{
  // Setup
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const welding_node = std::make_shared<MoveitWelding>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(welding_node);
  std::thread(
    [&executor]()
    {executor.spin();})
  .detach();

  welding_node->initialize();
  welding_node->addBreakPoint();

  // Welding

  // Initialize the start pose
  geometry_msgs::msg::Pose start;
  start.position.x = 1.0;  // X coordinate
  start.position.y = 0.0;  // Y coordinate
  start.position.z = 0.5;  // Z coordinate

  // For simplicity, we'll use a default orientation (no rotation)
  start.orientation.x = 0.0;
  start.orientation.y = 0.0;
  start.orientation.z = 0.0;
  start.orientation.w = 1.0; // Represents no rotation

  // Initialize the end pose
  geometry_msgs::msg::Pose end;
  end.position.x = 2.0; // X coordinate
  end.position.y = 1.0; // Y coordinate
  end.position.z = 0.5; // Z coordinate

  // Let's say the end pose also has no rotation
  end.orientation.x = 0.0;
  end.orientation.y = 0.0;
  end.orientation.z = 0.0;
  end.orientation.w = 1.0; // Represents no rotation

  WaypointsBuilder builder;

  LinearMotion linearMotion(start, end, 10);
  SinusoidalMotion sinusoidalMotion(0.5, 1.0, 10);

  builder.addMotion(linearMotion)
          .addMotion(sinusoidalMotion);

  Waypoints waypoints = builder.build();

  moveit_msgs::msg::RobotTrajectory combined_trajectory;

  for (const auto& waypoint : waypoints.waypoints) {
      auto sub_trajectory_ptr = welding_node->planToPoint(waypoint, "pilz_industrial_motion_planner", "PTP");

      if (sub_trajectory_ptr) {
          const auto& sub_trajectory = *sub_trajectory_ptr;

          // If combined_trajectory is empty, initialize it with the first sub_trajectory's joint names
          if (combined_trajectory.joint_trajectory.joint_names.empty()) {
              combined_trajectory.joint_trajectory.joint_names = sub_trajectory.joint_trajectory.joint_names;
          }

          // Append each point from sub_trajectory to combined_trajectory
          for (const auto& point : sub_trajectory.joint_trajectory.points) {
              auto adjusted_point = point;

              // Adjust time_from_start by adding the duration of the combined_trajectory
              if (!combined_trajectory.joint_trajectory.points.empty()) {
                  adjusted_point.time_from_start += combined_trajectory.joint_trajectory.points.back().time_from_start;
              }

              combined_trajectory.joint_trajectory.points.push_back(adjusted_point);
          }
      } else {
          RCLCPP_INFO(LOGGER, "Sub-trajectory planning failed for a waypoint");
          // Handle the failure case (e.g., stop the loop, throw an exception, etc.)
      }
  }

  if (combined_trajectory != nullptr) {
    welding_node->drawTrajectory(*combined_trajectory);
    welding_node->addBreakPoint();
    welding_node->moveGroupInterface()->execute(*combined_trajectory);
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;

  // Rest of it is part of the example

  // Add robot platform
  welding_node->addRobotPlatform();

  // Pilz PTP planner
  auto standing_pose = Eigen::Isometry3d(
    Eigen::Translation3d(
      0.1, 0.2,  0.8) *
    Eigen::Quaterniond::Identity());

  auto planned_trajectory = welding_node->planToPoint(
    standing_pose,
    "pilz_industrial_motion_planner", "PTP");
  if (planned_trajectory != nullptr) {
    welding_node->drawTrajectory(*planned_trajectory);
    welding_node->addBreakPoint();
    welding_node->moveGroupInterface()->execute(*planned_trajectory);
  }
  welding_node->addBreakPoint();

  // Pilz LIN planner
  auto cart_goal = Eigen::Isometry3d(
    Eigen::Translation3d(
      0.4, -0.15,
      0.55) *
    Eigen::Quaterniond::Identity());
  planned_trajectory =
    welding_node->planToPoint(cart_goal, "pilz_industrial_motion_planner", "LIN");
  if (planned_trajectory != nullptr) {
    welding_node->drawTrajectory(*planned_trajectory);
    welding_node->addBreakPoint();
    welding_node->moveGroupInterface()->execute(*planned_trajectory);
  }

  // Add collision object
  welding_node->addCollisionBox(
    geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.25).y(-0.075).z(0.675),
    geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.1).y(0.4).z(0.1));
  welding_node->addBreakPoint();

  // Try moving back with Pilz LIN
  planned_trajectory = welding_node->planToPoint(
    standing_pose, "pilz_industrial_motion_planner",
    "LIN");
  if (planned_trajectory != nullptr) {
    welding_node->drawTrajectory(*planned_trajectory);
  } else {
    welding_node->drawTitle("Failed planning with Pilz LIN");
  }
  welding_node->addBreakPoint();

  // Try moving back with Pilz PTP
  planned_trajectory = welding_node->planToPoint(
    standing_pose, "pilz_industrial_motion_planner",
    "PTP");
  if (planned_trajectory != nullptr) {
    welding_node->drawTrajectory(*planned_trajectory);
  } else {
    welding_node->drawTitle("Failed planning with Pilz PTP");
  }
  welding_node->addBreakPoint();

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
