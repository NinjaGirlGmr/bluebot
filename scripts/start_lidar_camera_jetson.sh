#!/bin/bash
# start_lidar_camera_jetson.sh
# Launch Slamtec A2M8-R2 LIDAR and Astra Camera on Jetson with ROS 2 Humble

# --- CONFIG VARIABLES ---
ROS_DOMAIN_ID=30
JETSON_INTERFACE=wlP1p1s0

# LIDAR launch
LIDAR_LAUNCH_PKG=lidar_launch
LIDAR_LAUNCH_FILE=lidar_with_tf.launch.py

# CAMERA launch
CAMERA_LAUNCH_PKG=astra_camera
CAMERA_LAUNCH_FILE=dabai_pro.launch.py

# CycloneDDS XML
CYCLONE_XML=/ssd/cyclonedds/cyclonedds_jetson.xml

# --- EXPORT ENVIRONMENT VARIABLES ---
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
export ROS_INTERFACE_OVERRIDE=$JETSON_INTERFACE
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$CYCLONE_XML

# --- SOURCE ROS 2 ENVIRONMENT ---
source /opt/ros/humble/setup.bash

# --- SOURCE WORKSPACE IF EXISTS ---
if [ -f /ssd/ros2_ws/install/setup.bash ]; then
    source /ssd/ros2_ws/install/setup.bash
fi

# --- ALLOW ROS 2 UDP PORTS THROUGH FIREWALL ---
# Ports 7400-7410 are used by DDS discovery and traffic
sudo ufw allow proto udp from any to any port 7400:7410

# --- ENSURE CYCLONEDDS XML EXISTS ---
if [ ! -f $CYCLONE_XML ]; then
    echo "[INFO] Creating CycloneDDS XML at $CYCLONE_XML"
    cat <<EOF > $CYCLONE_XML
<?xml version="1.0"?>
<CycloneDDS>
  <Domain Id="30">
    <!-- Networking settings -->
    <General>
      <AllowMulticast>true</AllowMulticast> <!-- unicast-only -->
    </General>

    <!-- Static discovery: only add peers you need -->
    <Discovery>
      <Peers>
        <Peer address="10.0.0.2"/> <!-- VM IP -->
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>40</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>

EOF
fi

# --- START NODES IN BACKGROUND ---
echo "[INFO] Starting LIDAR node..."
ros2 launch $LIDAR_LAUNCH_PKG $LIDAR_LAUNCH_FILE &

sleep 5

echo "[INFO] Starting Astra camera node..."
ros2 launch $CAMERA_LAUNCH_PKG $CAMERA_LAUNCH_FILE &

echo "[INFO] LIDAR and camera nodes launched in background."
echo "[INFO] Use 'ros2 node list' and 'ros2 topic list' to verify topics."
