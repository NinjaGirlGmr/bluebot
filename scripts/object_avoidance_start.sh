#!/bin/bash

export ROS_HUMBLE=/opt/ros/humble
export ROS_WS=/ssd/ros2_ws


source ${ROS_HUMBLE}/setup.bash

cd ${ROS_WS}

source ./install/setup.bash

# Start Lidar
ros2 launch lidar_launch lidar_with_tf.launch.py frame_id:=laser &

sleep 5
ros2 topic list

sleep 2

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser &


#Start IMU
ros2 run yb_a471_driver imu_node &

sleep 5

ros2 topic list

sleep 2

#Start Motor Drive Service
ros2 launch arduino_drive serial_motor_bridge.launch.py &

sleep 5

ros2 topic list

sleep 2

#Start Realsense Camera
#ros2 launch realsense2_camera rs_launch.py \
#  enable_color:=false \
#  enable_infra1:=true enable_infra2:=true \
#  infra1_profile:=848x480x30 infra2_profile:=848x480x30 \
#  enable_depth:=true depth_module.profile:=848x480x30 \
#  enable_sync:=true \
#  initial_reset:=true &

#ros2 launch realsense2_camera rs_launch.py \
#  enable_color:=false \
#  enable_infra1:=true enable_infra2:=true \
#  infra1_profile:=848x480x30 infra2_profile:=848x480x30 \
#  enable_depth:=false \
#  enable_gyro:=false enable_accel:=false \
#  initial_reset:=true &

sleep 5

ros2 topic list

sleep 2

#Start Ojbect Avoidance

ros2 launch avoid_objects avoid_objects.launch.py &

