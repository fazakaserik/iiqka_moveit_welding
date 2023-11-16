# Dependencies
This ROS2 package is built to work with [ROS2 KUKA drivers](https://github.com/kroshu/kuka_drivers/wiki) based on [examples/iiqka_moveit_example](https://github.com/kroshu/kuka_drivers/tree/master/examples/iiqka_moveit_example).

# System Informations
| Information      | Value |
| :---        |    :----:   |
| ROS2 Version | Humble |
| Robot Model | [LBR iisy 11](https://www.kuka.com/event/media?itemId=3EE29492F391404491777BF684749DE2) |
| SRDF | [kuka_lbr_iisy_moveit_config/srdf/lbr_iisy11_r1300_arm.srdf.xacro](https://github.com/kroshu/kuka_robot_descriptions/blob/master/kuka_lbr_iisy_moveit_config/srdf/lbr_iisy11_r1300_arm.srdf.xacro) |

# Build
`colcon build`