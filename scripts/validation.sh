#!/bin/bash
set -e

ROS2_WS=/ssd/ros2_ws
ISAAC_ROS2_WS=/ssd/isaac_ros2_ws

source ~/.bashrc
source $ROS2_WS/install/setup.bash
source $ISAAC_ROS2_WS/install/setup.bash

export CYCLONEDDS_URI=file:///ssd/cyclonedds/cyclonedds_jetson.xml

# Cleanup function
cleanup() {
    echo "[INFO] Cleaning up background processes..."
    jobs -p | xargs -r kill -TERM
    wait
    echo "[INFO] Cleanup done."
}
trap cleanup EXIT

pause_for_verification() {
    echo
    echo "======================================"
    echo "Verify nodes and topics above."
    echo "Press ENTER to continue..."
    read
    echo "======================================"
    echo
}

# ------------------------------
# Step 1: LIDAR
# ------------------------------
echo "[STEP 1] Starting LIDAR node..."
ros2 launch lidar_launch lidar_with_tf.launch.py \
    serial_port:=/dev/ttyUSB0 \
    frame_id:=laser &

sleep 2
echo "[INFO] LIDAR nodes:"
ros2 node list
echo "[INFO] LIDAR topics:"
ros2 topic list
pause_for_verification

# Publish static transform
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser &
sleep 0.5


# ------------------------------
# STEP 1B: IMU
# -----------------------------
echo "[STEP 1B] Starting IMU
ros2 run yb_a471_driver imu_node &


sleep 2
echo "[INFO] IMU nodes:"
ros2 node list
echo "[INFO] IMU topics:"
ros2 topic list
pause_for_verification

# ------------------------------
# Step 2: Camera
# ------------------------------
echo "[STEP 2] Starting D435 camera node..."
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.profile:=640x480x15 \
  depth_module.profile:=640x480x15 \
  align_depth.enable:=true \
  pointcloud.enable:=false &

sleep 3
echo "[INFO] Camera nodes:"
ros2 node list
echo "[INFO] Camera topics:"
ros2 topic list
pause_for_verification

echo "[Info] Locking Exposure:"
ros2 param set /camera/camera infra1_auto_exposure false
ros2 param set /camera/camera infra2_auto_exposure false

ros2 param set /camera/camera infra1_exposure 8500
ros2 param set /camera/camera infra2_exposure 8500

echo "[Info] Setting Gain:"
ros2 param set /camera/camera infra1_gain 16
ros2 param set /camera/camera infra2_gain 16

echo "[Info] Disable IR Emitter:"
ros2 param set /camera/camera emitter_enabled false

echo "[Info] Tuning for latency:"
ros2 param set /camera/camera enable_frame_sync true
ros2 param set /camera/camera publish_tf false
ros2 param set /camera/camera enable_rgbd false



# Publish static transforms
ros2 run tf2_ros static_transform_publisher 0.1 0 0.2 0 0 0 base_link camera_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5708 0 -1.5708 camera_link camera_infra1_optical_frame &
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5708 0 -1.5708 camera_link camera_infra2_optical_frame &
sleep 0.5

# ------------------------------
# Step 3: Nitros Bridge
# ------------------------------
echo "[STEP 3] Starting Nitros bridge..."
ros2 launch isaac_ros_nitros_bridge_ros2 \
  isaac_ros_nitros_bridge_image_converter.launch.py \
  sub_image_name:=/camera/infra1/image_rect_raw \
  pub_image_name:=/visual_slam/image_0_raw \
  node_name:=image_0_converter \
  container_name:=ros2_converter_container &

ros2 launch isaac_ros_nitros_bridge_ros2 \
  isaac_ros_nitros_bridge_image_converter.launch.py \
  sub_image_name:=/camera/infra2/image_rect_raw \
  pub_image_name:=/visual_slam/image_1_raw \
  node_name:=image_1_converter \
  container_name:=ros2_converter_container &

sleep 2
echo "[INFO] Nitros nodes:"
ros2 node list
echo "[INFO] Nitros topics:"
ros2 topic list
pause_for_verification

# ------------------------------
# Step 4: Visual SLAM
# ------------------------------
echo "[STEP 4] Starting Visual SLAM node..."
ros2 launch $ISAAC_ROS2_WS/install/isaac_ros_visual_slam/share/isaac_ros_visual_slam/launch/isaac_ros_visual_slam.launch.py &

sleep 3
echo "[INFO] Visual SLAM nodes:"
ros2 node list
echo "[INFO] Visual SLAM topics:"
ros2 topic list
pause_for_verification

echo "[INFO] All nodes started successfully."
