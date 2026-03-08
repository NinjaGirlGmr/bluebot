#!/usr/bin/env bash
set -Eeuo pipefail

# ----- config -----
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
LIDAR_DEV="${LIDAR_DEV:-/dev/lidar}"
IMU_DEV="${IMU_DEV:-/dev/myimu}"
FRAME_ID="${FRAME_ID:-laser}"
BASE_FRAME="${BASE_FRAME:-base_link}"
LASER_FRAME="${LASER_FRAME:-laser}"

export ROS_DOMAIN_ID

log() { echo "[$(date -Is)] $*"; }

require_dev() {
  local dev="$1"
  if [[ ! -e "$dev" ]]; then
    log "ERROR: Required device missing: $dev"
    ls -la /dev | sed -n '1,200p' || true
    exit 1
  fi
  if [[ ! -r "$dev" || ! -w "$dev" ]]; then
    log "WARN: Device exists but may have permission issues: $dev"
    ls -la "$dev" || true
  fi
}

# Kill whole process group on stop (so background jobs die cleanly)
shutdown() {
  local code=$?
  log "Shutdown requested (exit code: $code). Stopping child processes..."
  # Send SIGTERM to our process group
  kill -TERM -- -$$ 2>/dev/null || true
  # Give them a moment, then SIGKILL if needed
  sleep 2
  kill -KILL -- -$$ 2>/dev/null || true
  log "Shutdown complete."
  exit $code
}
trap shutdown INT TERM EXIT

log "Starting ROS stack (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
require_dev "$LIDAR_DEV"
require_dev "$IMU_DEV"

# --- Start processes ---
log "Starting lidar launch..."
ros2 launch lidar_launch lidar_with_tf.launch.py "frame_id:=$FRAME_ID" &

log "Starting static TF base_link -> laser..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 "$BASE_FRAME" "$LASER_FRAME" &

log "Starting IMU node..."
ros2 run yb_a471_driver imu_node &

log "All processes started. Waiting..."
wait
