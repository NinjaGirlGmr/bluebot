#!/bin/bash

source /opt/ros/humble/setup.bash

ros2 run robot_state_publisher robot_state_publisher /ssd/ros2_ws/scripts/my_robot_parsed.urdf & \
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{name: ['joint1','joint2','joint3'], position: [0.0,0.0,0.0]}" --rate 10 &

sleep 2

ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 &
