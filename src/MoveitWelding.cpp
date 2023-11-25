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

#include <math.h>

#include <memory>

#include "iiqka_moveit_welding/moveit_welding.hpp"
#include "iiqka_moveit_welding/Waypoints/Waypoints.hpp"
#include "iiqka_moveit_welding/Waypoints/WaypointsBuilder.hpp"
#include "iiqka_moveit_welding/Motion/LinearMotion.hpp"
#include "iiqka_moveit_welding/Motion/SinusoidalMotion.hpp"

int main(int argc, char * argv[])
{
  std::shared_ptr<moveit_msgs::msg::RobotTrajectory> trajectory;

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

  auto starting_point = Eigen::Isometry3d(
    Eigen::Translation3d(0.3, 0.0, 0.2) * Eigen::Quaterniond( 0.0, 0.0, 1.0, 0.0)
  );

  trajectory = welding_node->planToPoint(starting_point, "pilz_industrial_motion_planner", "PTP");

  if (trajectory == nullptr) {
    return 1;
  }
  welding_node->drawTrajectory(*trajectory);
  welding_node->addBreakPoint();
  welding_node->moveGroupInterface()->execute(*trajectory);
  welding_node->addBreakPoint();

  // Welding
  auto origin = welding_node->moveGroupInterface()->getCurrentPose().pose;

  auto lin = Eigen::Isometry3d(
    Eigen::Translation3d(0.5, 0.0, 0.0) * Eigen::Quaterniond( 0.0, 0.0, 1.0, 0.0)
  );

  LinearMotion linearMotion(lin);
  SinusoidalMotion sinusoidalMotion(0.1, 1.0);

  WaypointsBuilder builder(100, origin);
  
  builder.addMotion(linearMotion)
         .addMotion(sinusoidalMotion);

  Waypoints waypoints = builder.build();

  trajectory = welding_node->planFromWaypoints(waypoints);

  if (trajectory == nullptr) {
    return 1;
  }
  welding_node->drawTrajectory(*trajectory);
  welding_node->addBreakPoint();
  welding_node->moveGroupInterface()->execute(*trajectory);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
