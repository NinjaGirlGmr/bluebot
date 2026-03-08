#!/usr/bin/env bash
set -e

echo "ROS_WS is: $ROS_WS"

source /opt/ros/humble/setup.bash

# Overlay (your mounted workspace)
ROS_WS="${ROS_WS:-/ssd/ros2_ws}"
if [ -f "$ROS_WS/install/setup.bash" ]; then
  source "$ROS_WS/install/setup.bash"
else
  echo "WARN: $ROS_WS/install/setup.bash not found. Did you colcon build on the host?"
  echo "Mounted workspace contents:"
  ls -la "$ROS_WS" || true
fi


exec "$@"

