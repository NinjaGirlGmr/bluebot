#!/bin/bash
# start_vslam_container.sh
# Launch Isaac ROS VSLAM container on Jetson Orin Nano (headless)

# --- CONFIG VARIABLES ---
CONTAINER_IMAGE=nvcr.io/nvidia/isaac/ros:noble-ros2_jazzy_d3e84470d576702a380478a513fb3fc6-arm64
WORKSPACE=/ssd/isaac_ros2_ws
LOG_DIR=$WORKSPACE/logs
LOG_FILE=$LOG_DIR/vslam_container_$(date +%Y%m%d_%H%M%S).log

# --- CREATE LOG DIRECTORY IF IT DOESN'T EXIST ---
mkdir -p $LOG_DIR

# --- START CONTAINER ---
echo "[INFO] Starting Isaac ROS VSLAM container..."
sudo docker run --rm -it --runtime nvidia \
    --gpus all \
    --network host \
    -e "ROS_DOMAIN_ID=30" \
    -v $WORKSPACE:/ssd/isaac_ros2_ws \
    -v /dev:/dev \
    -v /run/udev:/run/udev:ro \
    --privileged \
    $CONTAINER_IMAGE \
    bash 

echo "[INFO] Container exited. Logs saved to $LOG_FILE"
