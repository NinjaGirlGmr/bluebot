#!/bin/bash
# start_lidar_jetson.sh
# Launch Slamtec A2M8-R2 LIDAR on Jetson with ROS 2 Humble

# --- CONFIG VARIABLES ---
ROS_DOMAIN_ID=30
LIDAR_LAUNCH_PKG=lidar_launch
LIDAR_LAUNCH_FILE=lidar_with_tf.launch.py
JETSON_INTERFACE=wlP1p1s0

# --- EXPORT ENVIRONMENT VARIABLES ---
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
export ROS_INTERFACE_OVERRIDE=$JETSON_INTERFACE
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///ssd/cyclonedds/cyclonedds_jetson.xml


# --- SOURCE ROS 2 ENVIRONMENT ---
source /opt/ros/humble/setup.bash

# --- SOURCE WORKSPACE IF EXISTS ---
if [ -f /ssd/ros2_ws/install/setup.bash ]; then
    source /ssd/ros2_ws/install/setup.bash
fi

# --- ALLOW ROS 2 UDP PORTS THROUGH FIREWALL ---
# Ports 7400-7410 are used by DDS discovery and traffic
sudo ufw allow proto udp from any to any port 7400:7410

# --- LAUNCH LIDAR NODE ---
echo "Starting LIDAR node..."
ros2 launch $LIDAR_LAUNCH_PKG $LIDAR_LAUNCH_FILE
