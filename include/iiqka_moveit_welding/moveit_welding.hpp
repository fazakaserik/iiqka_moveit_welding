// Copyright 2023 Erik Fazakas & Áron Papp
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

#ifndef IIQKA_MOVEIT_WELDING__MOVEIT_WELDING_HPP_
#define IIQKA_MOVEIT_WELDING__MOVEIT_WELDING_HPP_

#include <math.h>

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_visual_tools/moveit_visual_tools.h"
#include "geometry_msgs/msg/vector3.hpp"
#include "Waypoints/Waypoints.hpp"

class MoveitWelding : public rclcpp::Node
{
public:
  MoveitWelding()
  : rclcpp::Node("iiqka_moveit_welding")
  {
  }

  void initialize()
  {
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(),
      PLANNING_GROUP);

    moveit_visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
      shared_from_this(), "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface_->getRobotModel());

    moveit_visual_tools_->deleteAllMarkers();
    moveit_visual_tools_->loadRemoteControl();
    moveit_visual_tools_->trigger();

    planning_scene_diff_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
      "planning_scene", 10);

    move_group_interface_->setMaxVelocityScalingFactor(0.1);
    move_group_interface_->setMaxAccelerationScalingFactor(0.1);
  }

  moveit_msgs::msg::RobotTrajectory::SharedPtr planFromWaypoints(
    Waypoints& waypoints
    )
  {
    moveit_msgs::msg::RobotTrajectory trajectory;

    RCLCPP_INFO(LOGGER, "Start planning");
    
    int i = 0;
    for (auto waypoint : waypoints.vector)
    {
        RCLCPP_DEBUG(LOGGER, "Waypoint %d - x: %f y: %f z: %f", i, waypoint.position.x, waypoint.position.y, waypoint.position.z);
        i++;
    }

    double fraction = move_group_interface_->computeCartesianPath(waypoints.vector, 0.005, 0.0, trajectory);

    if (fraction < 1) {
      RCLCPP_ERROR(LOGGER, "Could not compute trajectory through all waypoints!");
      return nullptr;
    } else {
      RCLCPP_INFO(LOGGER, "Planning done!");
      return std::make_shared<moveit_msgs::msg::RobotTrajectory>(trajectory);
    }
  }

  moveit_msgs::msg::RobotTrajectory::SharedPtr planToPoint(
    const Eigen::Isometry3d & pose,
    const std::string & planning_pipeline = "pilz_industrial_motion_planner",
    const std::string & planner_id = "PTP")
  {
    // Create planning request using pilz industrial motion planner
    move_group_interface_->setPlanningPipelineId(planning_pipeline);
    move_group_interface_->setPlannerId(planner_id);
    move_group_interface_->setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(LOGGER, "Sending planning request");
    if (!move_group_interface_->plan(plan)) {
      RCLCPP_INFO(LOGGER, "Planning failed");
      return nullptr;
    } else {
      RCLCPP_INFO(LOGGER, "Planning successful");
      return std::make_shared<moveit_msgs::msg::RobotTrajectory>(plan.trajectory_);
    }
  }

  void drawTrajectory(const moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    moveit_visual_tools_->deleteAllMarkers();
    moveit_visual_tools_->publishTrajectoryLine(
      trajectory,
      moveit_visual_tools_->getRobotModel()->getJointModelGroup(PLANNING_GROUP));
  }

  void drawTitle(const std::string & text)
  {
    auto const text_pose = []
      {
        auto msg = Eigen::Isometry3d::Identity();
        msg.translation().z() = 1.0;
        return msg;
      } ();
    moveit_visual_tools_->publishText(
      text_pose, text, rviz_visual_tools::RED,
      rviz_visual_tools::XXLARGE);
  }

  void addBreakPoint()
  {
    moveit_visual_tools_->trigger();
    moveit_visual_tools_->prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroupInterface()
  {
    return move_group_interface_;
  }

protected:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_;
  const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_basic_plan");
  const std::string PLANNING_GROUP = "manipulator";
};

#endif  // IIQKA_MOVEIT_WELDING__MOVEIT_WELDING_HPP_
