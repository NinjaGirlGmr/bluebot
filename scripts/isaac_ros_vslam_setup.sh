#!/bin/bash
set -e

# ==========================
# Isaac ROS vSLAM Setup Script
# Jetson Orin Nano / ROS 2 Humble / CUDA 12.6
# ==========================

# --- Variables ---
WORKSPACE="$HOME/isaac_ros2_ws"
SRC="$WORKSPACE/src"

echo "=== Creating workspace directories ==="
mkdir -p "$SRC"
cd "$SRC"

# --- 1️⃣ Clone Isaac ROS repositories ---
echo "=== Cloning required Isaac ROS repos (release-3.2) ==="

# isaac_ros_common
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# isaac_ros_visual_slam
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

# isaac_ros_nitros (core + examples)
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git

# OSRF negotiated (required for Nitros headers)
git clone https://github.com/osrf/negotiated.git

# ==========================
# 2️⃣ Setup NVTX symlink for CUDA 12.6
# ==========================
echo "=== Setting up CUDA NVTX symlink ==="
if [ ! -f /usr/lib/aarch64-linux-gnu/libnvToolsExt.so ]; then
    sudo ln -s /usr/local/cuda-12.6/targets/aarch64-linux/lib/libnvToolsExt.so /usr/lib/aarch64-linux-gnu/libnvToolsExt.so
fi

# Export CUDA paths (persistent)
echo 'export CUDA_HOME=/usr/local/cuda' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/targets/aarch64-linux/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# ==========================
# 3️⃣ Install Python dependencies
# ==========================
echo "=== Installing Python dependencies ==="
python3 -m pip install --user posix_ipc

# ==========================
# 4️⃣ Clean previous builds
# ==========================
echo "=== Cleaning old build artifacts ==="
cd "$WORKSPACE"
rm -rf build install log

# ==========================
# 5️⃣ Build workspace (skip problematic example packages)
# ==========================
echo "=== Building Isaac ROS workspace ==="
colcon build \
  --symlink-install \
  --packages-ignore custom_nitros_dnn_image_encoder \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# ==========================
# 6️⃣ Setup environment
# ==========================
echo "=== Sourcing workspace setup ==="
source "$WORKSPACE/install/setup.bash"

# ==========================
# ✅ Finished
# ==========================
echo "=== Isaac ROS vSLAM environment setup complete ==="
echo "You can now run:"
echo "ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py"
echo "Verify topics with: ros2 topic list | grep visual_slam"
