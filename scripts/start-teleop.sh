!#/bin/bash

. /opt/ros/${ROS_DISTRO}/setup.bash
. /ssd/ros2_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard & 

