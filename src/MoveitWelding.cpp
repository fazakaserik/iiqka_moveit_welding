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
  // SETUP START
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
  //welding_node->addBreakPoint();

  // Add robot platform
  welding_node->addRobotPlatform();

  // SETUP END
  // Eigen::Quaterniond( 0,1,0,0)); W X Y Z
  // Pilz PTP planner START \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\qqqqq
  // LEFELE NEZ           Eigen::Quaterniond( 0,0,1,0)
  // LEFELE NÉZ butyok a tul oldalon Eigen::Quaterniond( 0,1,0,0));
   // FELFELE NEZ JOBBROL Eigen::Quaterniond( 0,0,0,1)
   // FELFELE NEZ DE BALROL Eigen::Quaterniond( 1,0,0,0));
   // FElfele néz Eigen::Quaterniond( 0,-1,0,0));
   
  auto pos1 = Eigen::Isometry3d(
    Eigen::Translation3d(-0.5, 0, 0.8) * Eigen::Quaterniond( 0,0,1,0));
  
  auto planned_trajectory = 
          welding_node->planToPoint(pos1,
                                    "pilz_industrial_motion_planner", 
                                    "PTP");

  if (planned_trajectory != nullptr) {
    welding_node->drawTrajectory(*planned_trajectory);
    //welding_node->addBreakPoint();
    welding_node->moveGroupInterface()->execute(*planned_trajectory);
  }
 // welding_node->addBreakPoint();
  
 
  
 
  // Pilz PTP planner END //////////////////////////////////////////////////////////////////////////
   //welding_node->addBreakPoint();

  // MOVE TO LOCATION 2 START \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\qqqqq
   auto pos_2 = Eigen::Isometry3d(
    Eigen::Translation3d(-0.5, 0.7,0.3) *  Eigen::Quaterniond(0,0,1,0));
  planned_trajectory = 
          welding_node->planToPoint(pos_2,
                                    "pilz_industrial_motion_planner", 
                                    "LIN");

  if (planned_trajectory != nullptr) {
    welding_node->drawTrajectory(*planned_trajectory);
    //welding_node->addBreakPoint();
    welding_node->moveGroupInterface()->execute(*planned_trajectory);
  }
  // MOVE TO LOCATION 2 END //////////////////////////////////////////////////////////////////////////
  //welding_node->addBreakPoint();

  // DRAW A LINE  START \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\qqqqq
  
  auto pos_A = Eigen::Isometry3d(
    Eigen::Translation3d(-0.5, 0.7,0.15) *  Eigen::Quaterniond( 0.707,0,-0.707,0)*Eigen::Quaterniond( 0.707,-0.707,0,0));
  auto pos_B = Eigen::Isometry3d(
    Eigen::Translation3d(0.2, 0.7,0.15) *  Eigen::Quaterniond( 0.707,0,-0.707,0)*Eigen::Quaterniond( 0.707,-0.707,0,0));
  double freq_d       = 50;
  double amplitude_d  = 0.1;
  double minimum_step = 0.01;

  planned_trajectory = welding_node->draw_line_from_A2B_with_sin(pos_A,pos_B,freq_d,amplitude_d,minimum_step);
  //welding_node->addBreakPoint();
  if (planned_trajectory != nullptr) {
    welding_node->drawTrajectory(*planned_trajectory);
    //welding_node->addBreakPoint();
    welding_node->moveGroupInterface()->execute(*planned_trajectory);
  }

   pos_A = Eigen::Isometry3d(
    Eigen::Translation3d(0.2, 0.5,0.3) *  Eigen::Quaterniond( 0,0,1,0));
   pos_B = Eigen::Isometry3d(
    Eigen::Translation3d(-0.5, 0.7,0.3) *  Eigen::Quaterniond( 0,0,1,0));
   freq_d       = 30;
   amplitude_d  = 0.01;
   minimum_step = 0.01;

  planned_trajectory = welding_node->draw_line_from_A2B_with_sin(pos_A,pos_B,freq_d,amplitude_d,minimum_step);
  //welding_node->addBreakPoint();
  if (planned_trajectory != nullptr) {
    welding_node->drawTrajectory(*planned_trajectory);
    //welding_node->addBreakPoint();
    welding_node->moveGroupInterface()->execute(*planned_trajectory);
  }

  
  // DRAW A LINE  END //////////////////////////////////////////////////////////////////////////
  
  // MOVE BACK START \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
  
  auto pos_3 = Eigen::Isometry3d(
    Eigen::Translation3d(0.2, 0.5,0.5) *  Eigen::Quaterniond(  0,0,1,0 ));
  planned_trajectory = 
          welding_node->planToPoint(pos_3,
                                    "pilz_industrial_motion_planner", 
                                    "LIN");

  if (planned_trajectory != nullptr) {
    welding_node->drawTrajectory(*planned_trajectory);
    //welding_node->addBreakPoint();
    welding_node->moveGroupInterface()->execute(*planned_trajectory);
  }

   auto pos_4 = Eigen::Isometry3d(
    Eigen::Translation3d(-0.5, 0.5,0.7) *  Eigen::Quaterniond(  0,0,1,0 ));
  planned_trajectory = 
          welding_node->planToPoint(pos_4,
                                    "pilz_industrial_motion_planner", 
                                    "PTP");

  if (planned_trajectory != nullptr) {
    welding_node->drawTrajectory(*planned_trajectory);
    //welding_node->addBreakPoint();
    welding_node->moveGroupInterface()->execute(*planned_trajectory);
  }
  // MOVE BACK END //////////////////////////////////////////////////////////////

  // MOVE TO LOCATION 1   START \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


  planned_trajectory = 
          welding_node->planToPoint(pos1,
                                    "pilz_industrial_motion_planner", 
                                    "LIN");

  if (planned_trajectory != nullptr) {
    welding_node->drawTrajectory(*planned_trajectory);
    //welding_node->addBreakPoint();
    welding_node->moveGroupInterface()->execute(*planned_trajectory);
  }
  // MOVE TO LOCATION 1 CIRCLE  END //////////////////////////////////////////////////////////////////////////


  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}