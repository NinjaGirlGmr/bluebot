#!/bin/bash

set -euo pipefail

ROS_WS=/ssd/ros2_ws
PID_FILE=/tmp/serial_diff_drive_pids.txt
SHUTDOWN_TIMEOUT_SEC=8
VSLAM_MODE="${VSLAM_MODE:-}"
MAP_DIR="${MAP_DIR:-/ssd/maps}"
MAP_TOPIC="${MAP_TOPIC:-/map}"
MAP_SAVE_TIMEOUT_SEC="${MAP_SAVE_TIMEOUT_SEC:-15.0}"
MAP_OCCUPIED_THRESH="${MAP_OCCUPIED_THRESH:-0.65}"
MAP_FREE_THRESH="${MAP_FREE_THRESH:-0.25}"
MAP_IMAGE_FORMAT="${MAP_IMAGE_FORMAT:-pgm}"
LIDAR_SCAN_FREQUENCY="${LIDAR_SCAN_FREQUENCY:-6.0}"
LIDAR_ANGLE_COMPENSATE="${LIDAR_ANGLE_COMPENSATE:-false}"
LIDAR_SCAN_MODE="${LIDAR_SCAN_MODE:-}"
NAV2_USE_SIM_TIME="${NAV2_USE_SIM_TIME:-false}"
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-/ssd/ros2_ws/src/serial_diff_drive_hw/config/nav2_navigation_params.yaml}"
NAV2_MAP_EXPLORE_PARAMS_FILE="${NAV2_MAP_EXPLORE_PARAMS_FILE:-/ssd/ros2_ws/src/serial_diff_drive_hw/config/nav2_map_explore_params.yaml}"
NAV2_AUTOSTART="${NAV2_AUTOSTART:-true}"
NAV2_USE_COMPOSITION="${NAV2_USE_COMPOSITION:-__AUTO__}"
NAV2_USE_RESPAWN="${NAV2_USE_RESPAWN:-False}"
NAV2_LOG_LEVEL="${NAV2_LOG_LEVEL:-warn}"
NAV2_READY_TIMEOUT_SEC="${NAV2_READY_TIMEOUT_SEC:-60}"
NAV2_READY_CHECK_INTERVAL_SEC="${NAV2_READY_CHECK_INTERVAL_SEC:-1}"
NAV2_STARTUP_RETRY_SEC="${NAV2_STARTUP_RETRY_SEC:-5}"
NAV2_STARTUP_GRACE_SEC="${NAV2_STARTUP_GRACE_SEC:-20}"
REALSENSE_LIGHT_MODE_FOR_MAP="${REALSENSE_LIGHT_MODE_FOR_MAP:-true}"
REALSENSE_ENABLE_INFRA1="${REALSENSE_ENABLE_INFRA1:-true}"
REALSENSE_ENABLE_INFRA2="${REALSENSE_ENABLE_INFRA2:-true}"
REALSENSE_ENABLE_GYRO="${REALSENSE_ENABLE_GYRO:-true}"
REALSENSE_ENABLE_ACCEL="${REALSENSE_ENABLE_ACCEL:-true}"
REALSENSE_ENABLE_COLOR="${REALSENSE_ENABLE_COLOR:-true}"
REALSENSE_ENABLE_DEPTH="${REALSENSE_ENABLE_DEPTH:-true}"
REALSENSE_INFRA_PROFILE="${REALSENSE_INFRA_PROFILE:-640x480x30}"
REALSENSE_DEPTH_PROFILE="${REALSENSE_DEPTH_PROFILE:-640x480x30}"
REALSENSE_COLOR_PROFILE="${REALSENSE_COLOR_PROFILE:-640x480x30}"
REALSENSE_ENABLE_COLOR_LIGHT="${REALSENSE_ENABLE_COLOR_LIGHT:-false}"
REALSENSE_ENABLE_DEPTH_LIGHT="${REALSENSE_ENABLE_DEPTH_LIGHT:-false}"
REALSENSE_INFRA_PROFILE_LIGHT="${REALSENSE_INFRA_PROFILE_LIGHT:-640x480x15}"
REALSENSE_DEPTH_PROFILE_LIGHT="${REALSENSE_DEPTH_PROFILE_LIGHT:-640x480x15}"
REALSENSE_COLOR_PROFILE_LIGHT="${REALSENSE_COLOR_PROFILE_LIGHT:-640x480x15}"
FOXGLOVE_BRIDGE_ENABLED="${FOXGLOVE_BRIDGE_ENABLED:-false}"
FOXGLOVE_WAYPOINT_BRIDGE="${FOXGLOVE_WAYPOINT_BRIDGE:-false}"
FOXGLOVE_WAYPOINT_TOPIC="${FOXGLOVE_WAYPOINT_TOPIC:-/foxglove/waypoints}"
FOXGLOVE_WAYPOINT_FRAME="${FOXGLOVE_WAYPOINT_FRAME:-map}"
FOXGLOVE_WAYPOINT_ACTION_NAME="${FOXGLOVE_WAYPOINT_ACTION_NAME:-/navigate_through_poses}"
WAYPOINTS_FILE="${WAYPOINTS_FILE:-$MAP_DIR/waypoints.yaml}"
NVBLOX_ENABLED="${NVBLOX_ENABLED:-false}"
NVBLOX_LAUNCH_PKG="${NVBLOX_LAUNCH_PKG:-serial_diff_drive_hw}"
NVBLOX_LAUNCH_FILE="${NVBLOX_LAUNCH_FILE:-nvblox_bridge.launch.py}"
NVBLOX_USE_SIM_TIME="${NVBLOX_USE_SIM_TIME:-$NAV2_USE_SIM_TIME}"
EXPLORE_LITE_ENABLED="${EXPLORE_LITE_ENABLED:-false}"
EXPLORE_LITE_NAMESPACE="${EXPLORE_LITE_NAMESPACE:-}"
EXPLORE_LITE_USE_SIM_TIME="${EXPLORE_LITE_USE_SIM_TIME:-$NAV2_USE_SIM_TIME}"
EXPLORE_LITE_PARAMS_FILE="${EXPLORE_LITE_PARAMS_FILE:-/ssd/ros2_ws/src/serial_diff_drive_hw/config/explore_lite_params.yaml}"
MAP_EXPLORE_LITE_START_DELAY_SEC="${MAP_EXPLORE_LITE_START_DELAY_SEC:-10}"
ISAAC_GRID_LOCALIZATION_ENABLED="${ISAAC_GRID_LOCALIZATION_ENABLED:-true}"
ISAAC_GRID_LOCALIZER_LAUNCH_PKG="${ISAAC_GRID_LOCALIZER_LAUNCH_PKG:-isaac_nav2_pose_bridge}"
ISAAC_GRID_LOCALIZER_LAUNCH_FILE="${ISAAC_GRID_LOCALIZER_LAUNCH_FILE:-isaac_grid_localization_to_nav2.launch.py}"
ISAAC_GRID_LOCALIZER_SCAN_TOPIC="${ISAAC_GRID_LOCALIZER_SCAN_TOPIC:-/scan}"
ISAAC_GRID_LOCALIZER_FLATSCAN_TOPIC="${ISAAC_GRID_LOCALIZER_FLATSCAN_TOPIC:-/flatscan}"
ISAAC_GRID_LOCALIZER_RESULT_TOPIC="${ISAAC_GRID_LOCALIZER_RESULT_TOPIC:-/localization_result}"
ISAAC_GRID_LOCALIZER_TRIGGER_SERVICE="${ISAAC_GRID_LOCALIZER_TRIGGER_SERVICE:-/trigger_grid_search_localization}"
ISAAC_GRID_LOCALIZER_TRIGGER_RETRIES="${ISAAC_GRID_LOCALIZER_TRIGGER_RETRIES:-12}"
ISAAC_GRID_LOCALIZER_TRIGGER_WAIT_SEC="${ISAAC_GRID_LOCALIZER_TRIGGER_WAIT_SEC:-1}"
ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_ENABLED="${ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_ENABLED:-true}"
ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_WAIT_SEC="${ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_WAIT_SEC:-6.0}"
ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_PUBLISH_COUNT="${ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_PUBLISH_COUNT:-5}"
ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_PUBLISH_PERIOD_SEC="${ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_PUBLISH_PERIOD_SEC:-0.5}"
ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_X="${ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_X:-0.0}"
ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_Y="${ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_Y:-0.0}"
ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_YAW="${ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_YAW:-0.0}"
SELF_PGID="$(ps -o pgid= -p $$ 2>/dev/null | tr -d ' ' || true)"

usage() {
  echo "Usage: $0 [start|start-map|start-map-explore|start-nav <map>|save-map [name]|capture-waypoint [name]|send-waypoints [name ...]|slam-health|nav-health|stop|restart|restart-map|restart-map-explore|restart-nav <map>|status]"
}

is_true() {
  case "${1,,}" in
    1|true|yes|on|y|enabled)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

is_jetson_platform() {
  local model
  if [[ -f /proc/device-tree/model ]]; then
    model="$(tr -d '\0' < /proc/device-tree/model)"
    case "${model,,}" in
      *jetson*|*tegra*)
        return 0
        ;;
    esac
  fi
  return 1
}

filter_colon_paths() {
  local input="${1:-}"
  local exclude_regex="${2:-}"
  local out=""
  local part
  local IFS=':'
  local -a parts=()

  read -r -a parts <<< "$input"
  for part in "${parts[@]}"; do
    [[ -z "$part" ]] && continue
    if [[ -n "$exclude_regex" ]] && [[ "$part" =~ $exclude_regex ]]; then
      continue
    fi
    if [[ -n "$out" ]]; then
      out="${out}:$part"
    else
      out="$part"
    fi
  done

  echo "$out"
}

if [[ -z "$VSLAM_MODE" ]]; then
  if is_jetson_platform; then
    VSLAM_MODE="nitros"
  else
    VSLAM_MODE="direct"
  fi
fi

VSLAM_MODE="${VSLAM_MODE,,}"

source_ros() {
  setup_isaac_gxf_runtime() {
    local ros_distro="${ROS_DISTRO:-humble}"
    local path
    local -a gxf_paths=()
    local -a globbed=()
    local pattern

    gxf_paths+=(
      "$ROS_WS/install/isaac_ros_gxf/share/isaac_ros_gxf/gxf/lib"
      "/opt/ros/$ros_distro/share/isaac_ros_gxf/gxf/lib"
    )

    shopt -s nullglob
    for pattern in \
      "$ROS_WS/install/isaac_ros_gxf/share/isaac_ros_gxf/gxf/lib/"* \
      "$ROS_WS/install"/gxf_isaac_*/share/gxf_isaac_*/gxf/lib \
      "/opt/ros/$ros_distro/share/isaac_ros_gxf/gxf/lib/"* \
      "/opt/ros/$ros_distro"/share/gxf_isaac_*/gxf/lib; do
      globbed+=("$pattern")
    done
    shopt -u nullglob

    gxf_paths+=("${globbed[@]}")

    for path in "${gxf_paths[@]}"; do
      [[ -d "$path" ]] || continue
      case ":${LD_LIBRARY_PATH:-}:" in
        *":$path:"*) ;;
        *)
          if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
            LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:$path"
          else
            LD_LIBRARY_PATH="$path"
          fi
          ;;
      esac
    done
    export LD_LIBRARY_PATH
  }

  local had_nounset=0
  if [[ $- == *u* ]]; then
    had_nounset=1
    set +u
  fi
  source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
  source "$ROS_WS/install/setup.bash"
  setup_isaac_gxf_runtime
  if [[ $had_nounset -eq 1 ]]; then
    set -u
  fi
}

is_pid_running() {
  local pid="$1"
  kill -0 "$pid" 2>/dev/null
}

running_pids_from_file() {
  if [[ ! -f "$PID_FILE" ]]; then
    return 0
  fi
  while IFS= read -r pid; do
    [[ -z "${pid}" ]] && continue
    if is_pid_running "$pid"; then
      echo "$pid"
    fi
  done < "$PID_FILE"
}

record_pid() {
  local pid="$1"
  echo "$pid" >> "$PID_FILE"
}

launch_bg() {
  local label="$1"
  shift
  echo "Starting ${label}..."
  if command -v setsid >/dev/null 2>&1; then
    setsid "$@" &
  else
    "$@" &
  fi
  record_pid "$!"
}

descendant_pids() {
  local root="$1"
  local -a queue=("$root")
  local current
  local -a children=()
  local child
  local -A seen=()

  while [[ ${#queue[@]} -gt 0 ]]; do
    current="${queue[0]}"
    queue=("${queue[@]:1}")
    mapfile -t children < <(pgrep -P "$current" 2>/dev/null || true)
    for child in "${children[@]}"; do
      if [[ -z "${seen[$child]+x}" ]]; then
        seen["$child"]=1
        echo "$child"
        queue+=("$child")
      fi
    done
  done
}

signal_pid_family() {
  local pid="$1"
  local signal="$2"
  local pgid
  local -a descendants=()
  local child

  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
  if [[ -n "$pgid" && "$pgid" != "$SELF_PGID" ]]; then
    kill -s "$signal" -- "-$pgid" 2>/dev/null || true
  fi

  kill -s "$signal" "$pid" 2>/dev/null || true

  mapfile -t descendants < <(descendant_pids "$pid" || true)
  for child in "${descendants[@]}"; do
    kill -s "$signal" "$child" 2>/dev/null || true
  done
}

leaked_stack_pids() {
  ps -eo pid=,cmd= | awk '
    /ros2_serial_diff_drive_bridge\/serial_diff_drive_bridge/ ||
    /realsense2_camera\/realsense2_camera_node/ ||
    /teleop_twist_joy\/teleop_node/ ||
    /foxglove_bridge/ ||
    /rclcpp_components\/component_container/ ||
    /controller_manager\/ros2_control_node/ ||
    /controller_manager\/spawner/ ||
    /robot_state_publisher\/robot_state_publisher/ ||
    /yb_a471_driver\/imu_node/ ||
    /tf2_ros\/static_transform_publisher/ ||
    /serial_diff_drive_hw\/drop_detector_node/ ||
    /serial_diff_drive_hw\/cmd_vel_safety_gate/ ||
    /serial_diff_drive_hw\/foxglove_waypoint_bridge/ ||
    /isaac_nav2_pose_bridge\/isaac_to_nav2_pose/ ||
    /slam_toolbox\/sync_slam_toolbox_node/ ||
    /nvblox_node/ ||
    /explore_node/ ||
    /nav2_map_server\/map_saver_server/ ||
    /nav2_lifecycle_manager\/lifecycle_manager/ ||
    /rplidar|sllidar/ {
      print $1
    }'
}

start_nvblox() {
  local enabled="${1:-$NVBLOX_ENABLED}"
  if ! is_true "$enabled"; then
    return 0
  fi

  launch_bg "nvblox bridge" \
    ros2 launch "$NVBLOX_LAUNCH_PKG" "$NVBLOX_LAUNCH_FILE" \
      use_sim_time:="$NVBLOX_USE_SIM_TIME"
}

start_explore_lite() {
  local mode_name="$1"
  local enabled="${2:-$EXPLORE_LITE_ENABLED}"
  local -a explore_cmd
  if ! is_true "$enabled"; then
    return 0
  fi

  explore_cmd=(
    ros2 run explore_lite explore
    --ros-args
    --params-file "$EXPLORE_LITE_PARAMS_FILE"
    -p "use_sim_time:=$EXPLORE_LITE_USE_SIM_TIME"
    -r /tf:=tf
    -r /tf_static:=tf_static
  )
  if [[ -n "$EXPLORE_LITE_NAMESPACE" ]]; then
    explore_cmd+=(-r "__ns:=$EXPLORE_LITE_NAMESPACE")
  fi

  launch_bg "Explore Lite (${mode_name})" "${explore_cmd[@]}"
}

nav2_is_active() {
  local out
  local out_lower
  out="$(
    timeout 5s ros2 service call \
      /lifecycle_manager_navigation/is_active \
      std_srvs/srv/Trigger "{}" 2>/dev/null || true
  )"
  out_lower="${out,,}"
  [[ "$out_lower" == *"success=true"* || "$out_lower" == *"success: true"* ]]
}

nav2_request_startup() {
  timeout 5s ros2 service call \
    /lifecycle_manager_navigation/manage_nodes \
    nav2_msgs/srv/ManageLifecycleNodes "{command: 0}" >/dev/null 2>&1 || true
}

nav2_navigate_to_pose_server_count() {
  local servers
  servers="$(ros2 action info /navigate_to_pose 2>/dev/null | awk '/Action servers:/ {print $3; exit}')"
  if [[ "$servers" =~ ^[0-9]+$ ]]; then
    echo "$servers"
  else
    echo 0
  fi
}

wait_for_nav2_ready() {
  local timeout_sec="${1:-$NAV2_READY_TIMEOUT_SEC}"
  local check_interval_sec="${NAV2_READY_CHECK_INTERVAL_SEC:-1}"
  local startup_retry_sec="${NAV2_STARTUP_RETRY_SEC:-5}"
  local startup_grace_sec="${NAV2_STARTUP_GRACE_SEC:-20}"
  local start_ts
  local now_ts
  local next_startup_ts
  local servers
  local seen_active=0
  local startup_requested=0

  [[ "$timeout_sec" =~ ^[0-9]+$ ]] || timeout_sec=60
  [[ "$check_interval_sec" =~ ^[0-9]+$ ]] || check_interval_sec=1
  [[ "$startup_retry_sec" =~ ^[0-9]+$ ]] || startup_retry_sec=5
  [[ "$startup_grace_sec" =~ ^[0-9]+$ ]] || startup_grace_sec=20

  start_ts="$(date +%s)"
  next_startup_ts=$((start_ts + startup_grace_sec))
  echo "Waiting for Nav2 lifecycle manager and action servers..."

  while true; do
    now_ts="$(date +%s)"
    if (( now_ts - start_ts >= timeout_sec )); then
      echo "Nav2 readiness check timed out after ${timeout_sec}s." >&2
      return 1
    fi

    servers="$(nav2_navigate_to_pose_server_count)"
    if (( servers > 0 )); then
      return 0
    fi

    if nav2_is_active; then
      seen_active=1
    else
      if (( seen_active == 0 && startup_requested == 0 && now_ts >= next_startup_ts )); then
        echo "Nav2 not active yet; requesting lifecycle startup..."
        nav2_request_startup
        startup_requested=1
        next_startup_ts=$((now_ts + startup_retry_sec))
      fi
    fi

    sleep "$check_interval_sec"
  done
}

cleanup_leaked_stack() {
  local -a leaked=()
  local pid
  local -a remaining=()

  mapfile -t leaked < <(leaked_stack_pids || true)
  if [[ ${#leaked[@]} -eq 0 ]]; then
    return 0
  fi

  echo "Found leaked stack processes without PID-file tracking: ${leaked[*]}"
  echo "Attempting cleanup with SIGTERM..."
  for pid in "${leaked[@]}"; do
    signal_pid_family "$pid" TERM
  done
  sleep 1

  for pid in "${leaked[@]}"; do
    if is_pid_running "$pid"; then
      remaining+=("$pid")
    fi
  done

  if [[ ${#remaining[@]} -gt 0 ]]; then
    echo "Some leaked processes still running: ${remaining[*]}"
    echo "Sending SIGKILL to leaked processes."
    for pid in "${remaining[@]}"; do
      signal_pid_family "$pid" KILL
    done
  fi
}

start_stack() {
  local job_mode="${1:-normal}"
  local existing
  local rs_enable_color="$REALSENSE_ENABLE_COLOR"
  local rs_enable_depth="$REALSENSE_ENABLE_DEPTH"
  local rs_infra_profile="$REALSENSE_INFRA_PROFILE"
  local rs_depth_profile="$REALSENSE_DEPTH_PROFILE"
  local rs_color_profile="$REALSENSE_COLOR_PROFILE"
  local launch_realsense="true"
  local launch_vslam="true"
  local launch_nvblox="$NVBLOX_ENABLED"

  if [[ "$job_mode" == "map-explore" ]]; then
    launch_realsense="false"
    launch_vslam="false"
    launch_nvblox="false"
    echo "Map-explore mode: using lidar + odom only (RealSense, vSLAM, and nvblox disabled)."
  fi
  existing="$(running_pids_from_file || true)"
  if [[ -n "$existing" ]]; then
    echo "Stack appears to already be running (PIDs: $existing)"
    echo "Use '$0 stop' first, or '$0 restart'."
    exit 1
  fi

  source_ros
  : > "$PID_FILE"

  local bridge_cmd_vel_topic="/cmd_vel"
  if [[ "$job_mode" == "map" ]]; then
    # Mapping job launches cmd_vel_safety_gate (/cmd_vel -> /cmd_vel_safe).
    bridge_cmd_vel_topic="/cmd_vel_safe"
  fi

  launch_bg "Serial Diff Drive Bridge" \
    ros2 launch ros2_serial_diff_drive_bridge ros2_serial_diff_drive_bridge.launch.py \
      port:=/dev/arduino \
      cmd_vel_topic:="$bridge_cmd_vel_topic" \
      odom_topic:=/odom_raw \
      publish_tf:=false
  sleep 2

  launch_bg "ros2_control controller manager and controllers" \
    ros2 launch serial_diff_drive_hw bringup.launch.py
  sleep 3

  local -a lidar_cmd
  lidar_cmd=(
    ros2 launch lidar_launch lidar_with_tf.launch.py
    serial_port:=/dev/lidar
    frame_id:=laser
    "scan_frequency:=$LIDAR_SCAN_FREQUENCY"
    "angle_compensate:=$LIDAR_ANGLE_COMPENSATE"
  )
  if [[ -n "$LIDAR_SCAN_MODE" ]]; then
    lidar_cmd+=("scan_mode:=$LIDAR_SCAN_MODE")
  fi

  launch_bg "Lidar with TF" "${lidar_cmd[@]}"
  sleep 1

  launch_bg "IMU node" \
    ros2 run yb_a471_driver imu_node
  sleep 1

  launch_bg "State estimation (odom + IMU fusion)" \
    ros2 launch serial_diff_drive_hw state_estimation.launch.py \
      odom_topic:=/odom_raw \
      fused_odom_topic:=/odom \
      imu_orientation_topic:=/imu/orientation \
      imu_raw_topic:=/imu/data_raw \
      use_sim_time:="$NAV2_USE_SIM_TIME"
  sleep 2

  if is_true "$launch_realsense" && [[ "$job_mode" == "map" || "$job_mode" == "map-explore" ]]; then
    if is_true "$REALSENSE_LIGHT_MODE_FOR_MAP"; then
      rs_enable_color="$REALSENSE_ENABLE_COLOR_LIGHT"
      rs_enable_depth="$REALSENSE_ENABLE_DEPTH_LIGHT"
      rs_infra_profile="$REALSENSE_INFRA_PROFILE_LIGHT"
      rs_depth_profile="$REALSENSE_DEPTH_PROFILE_LIGHT"
      rs_color_profile="$REALSENSE_COLOR_PROFILE_LIGHT"
      echo "Using low-load RealSense profile for ${job_mode} mode."
    fi
  fi

  if is_true "$launch_nvblox"; then
    rs_enable_color=true
    rs_enable_depth=true
  fi

  if is_true "$launch_realsense"; then
    launch_bg "RealSense Camera" \
      ros2 launch realsense2_camera rs_launch.py \
        rgb_camera.color_profile:="$rs_color_profile" \
        depth_module.depth_profile:="$rs_depth_profile" \
        depth_module.infra_profile:="$rs_infra_profile" \
        enable_infra1:="$REALSENSE_ENABLE_INFRA1" \
        enable_infra2:="$REALSENSE_ENABLE_INFRA2" \
        enable_gyro:="$REALSENSE_ENABLE_GYRO" \
        enable_accel:="$REALSENSE_ENABLE_ACCEL" \
        enable_color:="$rs_enable_color" \
        enable_depth:="$rs_enable_depth" \
        unite_imu_method:=2 \
        align_depth.enable:=false \
        pointcloud.enable:=false
    sleep 3

    launch_bg "static TF between base_link and camera_link" \
      ros2 run tf2_ros static_transform_publisher 0.097 0 0.155 0 0 0 base_link camera_link
    sleep 1
  fi

  if [[ "$NAV2_USE_COMPOSITION" == "__AUTO__" ]]; then
    # Prefer non-composed Nav2 by default for reliability under heavy perception load.
    NAV2_USE_COMPOSITION="False"
  fi

  if is_true "$launch_vslam"; then
    local vslam_launch_pkg="serial_diff_drive_hw"
    local vslam_launch_file="isaac_ros_visual_slam_existing_realsense.launch.py"
    if [[ "$VSLAM_MODE" == "nitros" ]]; then
      vslam_launch_file="isaac_ros_visual_slam_existing_realsense_nitros.launch.py"
    elif [[ "$VSLAM_MODE" != "direct" ]]; then
      echo "Invalid VSLAM_MODE='$VSLAM_MODE' (expected: direct|nitros)"
      exit 1
    fi

    local isaac_overlay_regex='^/ssd/ros2_ws/install/(isaac_ros_nitros($|_)|isaac_ros_gxf($|_)|gxf_isaac_|custom_nitros_|isaac_common)'
    local vslam_ament_prefix
    local vslam_cmake_prefix
    local vslam_colcon_prefix
    local vslam_ld_library_path

    vslam_ament_prefix="$(filter_colon_paths "${AMENT_PREFIX_PATH:-}" "$isaac_overlay_regex")"
    vslam_cmake_prefix="$(filter_colon_paths "${CMAKE_PREFIX_PATH:-}" "$isaac_overlay_regex")"
    vslam_colcon_prefix="$(filter_colon_paths "${COLCON_PREFIX_PATH:-}" "$isaac_overlay_regex")"
    vslam_ld_library_path="$(filter_colon_paths "${LD_LIBRARY_PATH:-}" "$isaac_overlay_regex")"

    launch_bg "Isaac ROS Visual SLAM (mode: $VSLAM_MODE)" \
      env \
        AMENT_PREFIX_PATH="$vslam_ament_prefix" \
        CMAKE_PREFIX_PATH="$vslam_cmake_prefix" \
        COLCON_PREFIX_PATH="$vslam_colcon_prefix" \
        LD_LIBRARY_PATH="$vslam_ld_library_path" \
        ros2 launch "$vslam_launch_pkg" "$vslam_launch_file"
    sleep 2
  fi

  if is_true "$FOXGLOVE_BRIDGE_ENABLED"; then
    launch_bg "Foxglove Bridge" \
      ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
    sleep 1
  fi

  start_nvblox "$launch_nvblox"
  sleep 1

  local teleop_linear_scale="0.5"
  local teleop_angular_scale="1.5"
  if [[ "$job_mode" == "map" ]]; then
    # Keep mapping speeds conservative to reduce slip-induced odom/map artifacts.
    teleop_linear_scale="${MAP_TELEOP_LINEAR_SCALE:-0.25}"
    teleop_angular_scale="${MAP_TELEOP_ANGULAR_SCALE:-1.0}"
  fi

  launch_bg "teleop_twist_joy for Foxglove teleop (/joy -> /cmd_vel)" \
    ros2 run teleop_twist_joy teleop_node --ros-args \
      -p require_enable_button:=false \
      -p axis_linear.x:=1 \
      -p axis_angular.yaw:=0 \
      -p scale_linear.x:="$teleop_linear_scale" \
      -p scale_angular.yaw:="$teleop_angular_scale" \
      -r /joy:=/joy \
      -r /cmd_vel:=/cmd_vel
  sleep 1

  if [[ "$job_mode" == "map" ]]; then
    mkdir -p "$MAP_DIR"
    launch_bg "Nav2 mapping + perception safety job" \
      ros2 launch serial_diff_drive_hw mapping_job.launch.py \
        use_sim_time:=false \
        use_composition:="$NAV2_USE_COMPOSITION"
    sleep 2
    start_explore_lite "start-map"
  elif [[ "$job_mode" == "map-explore" ]]; then
    mkdir -p "$MAP_DIR"
    launch_bg "Nav2 SLAM + exploration + navigation" \
      ros2 launch nav2_bringup bringup_launch.py \
        slam:=True \
        map:=/tmp/bluebot-map-explore-seed.yaml \
        use_sim_time:="$NAV2_USE_SIM_TIME" \
        params_file:="$NAV2_MAP_EXPLORE_PARAMS_FILE" \
        autostart:="$NAV2_AUTOSTART" \
        use_composition:="$NAV2_USE_COMPOSITION" \
        use_respawn:="$NAV2_USE_RESPAWN" \
        log_level:="$NAV2_LOG_LEVEL"
    sleep 2
    if ! wait_for_nav2_ready "$NAV2_READY_TIMEOUT_SEC"; then
      echo "Nav2 did not become ready in map-explore mode; not starting Explore Lite." >&2
      return 1
    fi
    local map_explore_delay_sec="$MAP_EXPLORE_LITE_START_DELAY_SEC"
    if ! [[ "$map_explore_delay_sec" =~ ^[0-9]+$ ]]; then
      map_explore_delay_sec=10
    fi
    if (( map_explore_delay_sec > 0 )); then
      echo "Waiting ${map_explore_delay_sec}s for SLAM map growth before starting Explore Lite..."
      sleep "$map_explore_delay_sec"
    fi
    start_explore_lite "start-map-explore" true
  fi

  echo "All nodes started."
  echo "PIDs saved to $PID_FILE"
  if [[ "$job_mode" == "map" ]]; then
    echo "Maps will be saved under: $MAP_DIR"
    echo "Run '$0 save-map <name>' to save (example: $0 save-map office_a)."
  elif [[ "$job_mode" == "map-explore" ]]; then
    echo "Map-and-explore mode active with lidar + odom stack (vSLAM and nvblox disabled)."
    echo "Maps will be saved under: $MAP_DIR"
    echo "Run '$0 save-map <name>' to save (example: $0 save-map office_a)."
  fi
  echo "Run '$0 stop' for graceful shutdown."
}

resolve_map_yaml() {
  local map_arg="$1"
  local candidate
  local -a candidates=()

  if [[ -z "$map_arg" ]]; then
    echo "Usage: $0 start-nav <map_name|map_yaml_path>" >&2
    return 1
  fi

  candidates+=("$map_arg")
  if [[ "$map_arg" != *.yaml ]]; then
    candidates+=("${map_arg}.yaml")
  fi
  if [[ "$map_arg" != /* ]]; then
    candidates+=("$MAP_DIR/$map_arg")
    if [[ "$map_arg" != *.yaml ]]; then
      candidates+=("$MAP_DIR/${map_arg}.yaml")
    fi
  fi

  for candidate in "${candidates[@]}"; do
    if [[ -f "$candidate" ]]; then
      echo "$candidate"
      return 0
    fi
  done

  echo "Map file not found for '$map_arg'." >&2
  echo "Try '$0 start-nav <name>' for $MAP_DIR/<name>.yaml or pass a full .yaml path." >&2
  return 1
}

map_yaml_image_path() {
  local map_yaml="$1"
  local image_entry=""

  image_entry="$(
    awk '
      /^[[:space:]]*#/ { next }
      /^[[:space:]]*image:[[:space:]]*/ {
        sub(/^[[:space:]]*image:[[:space:]]*/, "", $0)
        print
        exit
      }' "$map_yaml"
  )"

  image_entry="${image_entry#"${image_entry%%[![:space:]]*}"}"
  image_entry="${image_entry%"${image_entry##*[![:space:]]}"}"
  image_entry="${image_entry%\"}"
  image_entry="${image_entry#\"}"
  image_entry="${image_entry%\'}"
  image_entry="${image_entry#\'}"

  if [[ -z "$image_entry" ]]; then
    echo "Unable to parse map image path from: $map_yaml" >&2
    return 1
  fi

  if [[ "$image_entry" == /* ]]; then
    echo "$image_entry"
  else
    echo "$(dirname "$map_yaml")/$image_entry"
  fi
}

prepare_isaac_localizer_map_yaml() {
  local map_yaml="$1"
  local map_dir
  local map_name
  local source_image
  local source_ext
  local localizer_image
  local localizer_yaml
  local tmp_yaml

  map_dir="$(dirname "$map_yaml")"
  map_name="$(basename "$map_yaml" .yaml)"
  source_image="$(map_yaml_image_path "$map_yaml")" || return 1

  if [[ ! -f "$source_image" ]]; then
    echo "Map image does not exist: $source_image" >&2
    return 1
  fi

  source_ext="${source_image##*.}"
  source_ext="${source_ext,,}"
  case "$source_ext" in
    png|jpg|jpeg)
      echo "$map_yaml"
      return 0
      ;;
  esac

  localizer_image="${map_dir}/${map_name}_isaac_localizer.png"
  localizer_yaml="${map_dir}/${map_name}_isaac_localizer.yaml"
  tmp_yaml="${localizer_yaml}.tmp"

  if [[ ! -f "$localizer_image" || "$source_image" -nt "$localizer_image" ]]; then
    echo "Converting map image for Isaac localizer: $source_image -> $localizer_image"
    if ! python3 -c \
      "from PIL import Image; import sys; Image.open(sys.argv[1]).save(sys.argv[2])" \
      "$source_image" "$localizer_image"; then
      echo "Failed converting map image for Isaac localizer." >&2
      return 1
    fi
  fi

  if [[ ! -f "$localizer_yaml" || "$map_yaml" -nt "$localizer_yaml" || "$localizer_image" -nt "$localizer_yaml" ]]; then
    if ! awk -v new_image="$(basename "$localizer_image")" '
      BEGIN { replaced = 0 }
      /^[[:space:]]*image:[[:space:]]*/ && replaced == 0 {
        print "image: " new_image
        replaced = 1
        next
      }
      { print }
      END {
        if (replaced == 0) {
          print "image: " new_image
        }
      }' "$map_yaml" > "$tmp_yaml"; then
      echo "Failed creating localizer-specific map YAML: $localizer_yaml" >&2
      rm -f "$tmp_yaml"
      return 1
    fi
    mv "$tmp_yaml" "$localizer_yaml"
  fi

  echo "$localizer_yaml"
}

trigger_grid_localization() {
  local service_name="${1:-$ISAAC_GRID_LOCALIZER_TRIGGER_SERVICE}"
  local retries="$ISAAC_GRID_LOCALIZER_TRIGGER_RETRIES"
  local wait_sec="$ISAAC_GRID_LOCALIZER_TRIGGER_WAIT_SEC"
  local attempt

  for ((attempt = 1; attempt <= retries; attempt++)); do
    if timeout 3s ros2 service call "$service_name" std_srvs/srv/Empty "{}" \
      >/dev/null 2>&1; then
      echo "Triggered Isaac grid localization via ${service_name}."
      return 0
    fi
    sleep "$wait_sec"
  done

  echo "Warning: failed to trigger Isaac grid localization service '${service_name}'."
  echo "Initial pose may need manual publication on /initialpose."
  return 1
}

start_nav() {
  local map_arg="${1:-}"
  local map_yaml
  local localizer_map_yaml
  local isaac_localizer_started=0

  map_yaml="$(resolve_map_yaml "$map_arg")" || return 1
  localizer_map_yaml="$map_yaml"
  start_stack

  launch_bg "Nav2 localization + navigation (map: $map_yaml)" \
    ros2 launch nav2_bringup bringup_launch.py \
      slam:=False \
      map:="$map_yaml" \
      use_sim_time:="$NAV2_USE_SIM_TIME" \
      params_file:="$NAV2_PARAMS_FILE" \
      autostart:="$NAV2_AUTOSTART" \
      use_composition:="$NAV2_USE_COMPOSITION" \
      use_respawn:="$NAV2_USE_RESPAWN" \
      log_level:="$NAV2_LOG_LEVEL"
  sleep 2

  if is_true "$ISAAC_GRID_LOCALIZATION_ENABLED"; then
    if ! localizer_map_yaml="$(prepare_isaac_localizer_map_yaml "$map_yaml")"; then
      echo "Warning: Isaac localizer map preparation failed; skipping occupancy-grid localizer launch."
      echo "Set initial pose manually on /initialpose."
    else
      launch_bg "Isaac occupancy-grid localization + initial pose bridge" \
        ros2 launch "$ISAAC_GRID_LOCALIZER_LAUNCH_PKG" \
          "$ISAAC_GRID_LOCALIZER_LAUNCH_FILE" \
          map_yaml_path:="$localizer_map_yaml" \
          use_sim_time:="$NAV2_USE_SIM_TIME" \
          scan_topic:="$ISAAC_GRID_LOCALIZER_SCAN_TOPIC" \
          flatscan_topic:="$ISAAC_GRID_LOCALIZER_FLATSCAN_TOPIC" \
          localization_result_topic:="$ISAAC_GRID_LOCALIZER_RESULT_TOPIC" \
          pose_with_covariance_topic:="$ISAAC_GRID_LOCALIZER_RESULT_TOPIC" \
          output_topic:=/initialpose \
          output_frame_id:=map \
          fallback_initial_pose_enabled:="$ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_ENABLED" \
          fallback_initial_pose_wait_sec:="$ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_WAIT_SEC" \
          fallback_initial_pose_publish_count:="$ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_PUBLISH_COUNT" \
          fallback_initial_pose_publish_period_sec:="$ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_PUBLISH_PERIOD_SEC" \
          fallback_initial_pose_x:="$ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_X" \
          fallback_initial_pose_y:="$ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_Y" \
          fallback_initial_pose_yaw:="$ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_YAW"
      isaac_localizer_started=1
      sleep 2
      trigger_grid_localization "$ISAAC_GRID_LOCALIZER_TRIGGER_SERVICE" || true
    fi
  fi

  if is_true "$FOXGLOVE_WAYPOINT_BRIDGE"; then
    launch_bg "Foxglove waypoint bridge (PoseArray -> NavigateThroughPoses)" \
      ros2 run serial_diff_drive_hw foxglove_waypoint_bridge --ros-args \
        -p input_topic:="$FOXGLOVE_WAYPOINT_TOPIC" \
        -p default_frame_id:="$FOXGLOVE_WAYPOINT_FRAME" \
        -p action_name:="$FOXGLOVE_WAYPOINT_ACTION_NAME"
    sleep 1
  fi

  if ! wait_for_nav2_ready "$NAV2_READY_TIMEOUT_SEC"; then
    echo "Nav2 did not become ready in navigation mode; skipping Explore Lite startup." >&2
    return 1
  fi
  start_explore_lite "start-nav"

  echo "Navigation stack started with map: $map_yaml"
  if [[ "$isaac_localizer_started" -eq 1 ]]; then
    echo "Initial pose auto-triggered via Isaac grid localization on startup."
  else
    echo "Set initial pose on /initialpose and goals on /goal_pose."
  fi
  if is_true "$FOXGLOVE_WAYPOINT_BRIDGE"; then
    echo "Foxglove waypoints: publish geometry_msgs/PoseArray to $FOXGLOVE_WAYPOINT_TOPIC (frame: $FOXGLOVE_WAYPOINT_FRAME)."
  fi
}

save_map() {
  local map_name="${1:-map_$(date +%Y%m%d_%H%M%S)}"
  local map_path="$MAP_DIR/$map_name"
  local topics
  local map_topic_found=0
  source_ros
  mkdir -p "$MAP_DIR"

  topics="$(timeout 4s ros2 topic list 2>/dev/null || true)"
  while IFS= read -r topic; do
    if [[ "$topic" == "$MAP_TOPIC" ]]; then
      map_topic_found=1
      break
    fi
  done <<< "$topics"
  if [[ "$map_topic_found" -eq 0 ]]; then
    echo "Unable to save map: topic '$MAP_TOPIC' is not available."
    echo "Start mapping first with '$0 start-map', then drive briefly and retry."
    return 1
  fi

  echo "Saving map to ${map_path}.{yaml,pgm} ..."
  if ! ros2 run nav2_map_server map_saver_cli -f "$map_path" \
      --occ "$MAP_OCCUPIED_THRESH" \
      --free "$MAP_FREE_THRESH" \
      --fmt "$MAP_IMAGE_FORMAT" \
      --ros-args \
      -p save_map_timeout:="$MAP_SAVE_TIMEOUT_SEC" \
      -p map_subscribe_transient_local:=true \
      -r map:="$MAP_TOPIC"; then
    echo "Map save failed: no map message received from '$MAP_TOPIC' before timeout (${MAP_SAVE_TIMEOUT_SEC}s)."
    echo "Ensure mapping is active and SLAM is publishing map updates, then retry."
    return 1
  fi
}

capture_waypoint() {
  local waypoint_name="${1:-wp_$(date +%Y%m%d_%H%M%S)}"
  local tf_sample
  local x y z roll pitch yaw
  local captured_at
  local tmp_file

  source_ros
  mkdir -p "$MAP_DIR"

  tf_sample="$(
    timeout 5s ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | awk '
      /Translation:/ {
        if (match($0, /\[[^]]+\]/)) {
          s = substr($0, RSTART + 1, RLENGTH - 2)
          gsub(/,/, " ", s)
          split(s, a, /[[:space:]]+/)
          tx = a[1]; ty = a[2]; tz = a[3]
        }
      }
      /Rotation: in RPY/ {
        if (match($0, /\[[^]]+\]/)) {
          s = substr($0, RSTART + 1, RLENGTH - 2)
          gsub(/,/, " ", s)
          split(s, a, /[[:space:]]+/)
          rr = a[1]; rp = a[2]; ry = a[3]
          printf "%s %s %s %s %s %s\n", tx, ty, tz, rr, rp, ry
          exit
        }
      }' || true
  )"

  if [[ -z "$tf_sample" ]]; then
    echo "Failed to capture waypoint: transform map -> base_link unavailable."
    echo "Make sure mapping or navigation is running and localization has initialized."
    return 1
  fi

  read -r x y z roll pitch yaw <<< "$tf_sample"
  # Keep waypoints planar for Nav2: ignore vertical/tilt components from 3D odometry drift.
  z="0.000"
  roll="0.000"
  pitch="0.000"
  captured_at="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

  if [[ ! -f "$WAYPOINTS_FILE" ]]; then
    cat > "$WAYPOINTS_FILE" <<EOF
waypoints:
EOF
  else
    # Replace existing waypoint with the same name (upsert behavior).
    tmp_file="$(mktemp)"
    awk -v target="$waypoint_name" '
      function flush_record() {
        if (count == 0) {
          return
        }
        if (!skip_record) {
          for (i = 1; i <= count; i++) {
            print record[i]
          }
        }
        count = 0
        skip_record = 0
      }
      {
        if ($0 ~ /^  - name:[[:space:]]*/) {
          flush_record()
          name = $0
          sub(/^  - name:[[:space:]]*/, "", name)
          gsub(/"/, "", name)
          skip_record = (name == target)
        }
        count++
        record[count] = $0
      }
      END {
        flush_record()
      }
    ' "$WAYPOINTS_FILE" > "$tmp_file"
    mv "$tmp_file" "$WAYPOINTS_FILE"
  fi

  cat >> "$WAYPOINTS_FILE" <<EOF
  - name: "$waypoint_name"
    frame_id: "map"
    x: $x
    y: $y
    z: $z
    roll: $roll
    pitch: $pitch
    yaw: $yaw
    captured_at: "$captured_at"
EOF

  echo "Captured waypoint '$waypoint_name': x=$x y=$y yaw=$yaw"
  echo "Saved to $WAYPOINTS_FILE"
}

send_waypoints() {
  local -a requested_names=("$@")
  local waypoint_data
  local selected_count=0
  local frame_id=""
  local poses_json=""
  local sep=""

  source_ros

  if [[ ! -f "$WAYPOINTS_FILE" ]]; then
    echo "Waypoint file not found: $WAYPOINTS_FILE"
    echo "Capture waypoints first with '$0 capture-waypoint <name>'."
    return 1
  fi

  waypoint_data="$(awk '
    function emit_record() {
      if (name != "") {
        print name "\t" frame "\t" x "\t" y "\t" z "\t" roll "\t" pitch "\t" yaw
      }
    }
    /^  - name:/ {
      emit_record()
      name = $0
      sub(/^  - name:[[:space:]]*/, "", name)
      gsub(/"/, "", name)
      frame = "map"
      x = y = z = roll = pitch = yaw = "0"
      next
    }
    /^[[:space:]]+frame_id:/ {
      frame = $0
      sub(/^[[:space:]]*frame_id:[[:space:]]*/, "", frame)
      gsub(/"/, "", frame)
      next
    }
    /^[[:space:]]+x:/ {
      x = $0
      sub(/^[[:space:]]*x:[[:space:]]*/, "", x)
      next
    }
    /^[[:space:]]+y:/ {
      y = $0
      sub(/^[[:space:]]*y:[[:space:]]*/, "", y)
      next
    }
    /^[[:space:]]+z:/ {
      z = $0
      sub(/^[[:space:]]*z:[[:space:]]*/, "", z)
      next
    }
    /^[[:space:]]+roll:/ {
      roll = $0
      sub(/^[[:space:]]*roll:[[:space:]]*/, "", roll)
      next
    }
    /^[[:space:]]+pitch:/ {
      pitch = $0
      sub(/^[[:space:]]*pitch:[[:space:]]*/, "", pitch)
      next
    }
    /^[[:space:]]+yaw:/ {
      yaw = $0
      sub(/^[[:space:]]*yaw:[[:space:]]*/, "", yaw)
      next
    }
    END { emit_record() }
  ' "$WAYPOINTS_FILE")"

  if [[ -z "$waypoint_data" ]]; then
    echo "No waypoint entries found in $WAYPOINTS_FILE"
    return 1
  fi

  while IFS=$'\t' read -r name wp_frame x y z roll pitch yaw; do
    [[ -z "${name:-}" ]] && continue

    if [[ ${#requested_names[@]} -gt 0 ]]; then
      local matched=0
      local req
      for req in "${requested_names[@]}"; do
        if [[ "$name" == "$req" ]]; then
          matched=1
          break
        fi
      done
      if [[ "$matched" -eq 0 ]]; then
        continue
      fi
    fi

    if [[ -z "$frame_id" ]]; then
      frame_id="$wp_frame"
    elif [[ "$frame_id" != "$wp_frame" ]]; then
      echo "Waypoint frame mismatch: '$name' has frame '$wp_frame' but expected '$frame_id'."
      echo "All waypoints in one PoseArray must share a frame."
      return 1
    fi

    local qx qy qz qw
    # Publish planar orientation for Nav2 (yaw only).
    read -r qx qy qz qw <<< "$(awk -v y="$yaw" '
      BEGIN {
        cy = cos(y / 2.0); sy = sin(y / 2.0)
        qx = 0.0
        qy = 0.0
        qz = sy
        qw = cy
        printf "%.9f %.9f %.9f %.9f\n", qx, qy, qz, qw
      }')"

    poses_json+="${sep}{\"position\":{\"x\":${x},\"y\":${y},\"z\":0.0},\"orientation\":{\"x\":${qx},\"y\":${qy},\"z\":${qz},\"w\":${qw}}}"
    sep=","
    selected_count=$((selected_count + 1))
  done <<< "$waypoint_data"

  if [[ "$selected_count" -eq 0 ]]; then
    if [[ ${#requested_names[@]} -gt 0 ]]; then
      echo "No matching waypoint names found in $WAYPOINTS_FILE"
      echo "Requested: ${requested_names[*]}"
    else
      echo "No waypoints selected from $WAYPOINTS_FILE"
    fi
    return 1
  fi

  local pose_array_msg
  pose_array_msg="{\"header\":{\"frame_id\":\"${frame_id}\"},\"poses\":[${poses_json}]}"

  echo "Publishing ${selected_count} waypoint(s) to ${FOXGLOVE_WAYPOINT_TOPIC} (frame: ${frame_id})..."
  ros2 topic pub --once "$FOXGLOVE_WAYPOINT_TOPIC" geometry_msgs/msg/PoseArray "$pose_array_msg"
}

stop_stack() {
  if [[ ! -f "$PID_FILE" ]]; then
    echo "No PID file found at $PID_FILE."
    cleanup_leaked_stack
    return 0
  fi

  mapfile -t pids < <(running_pids_from_file || true)
  if [[ ${#pids[@]} -eq 0 ]]; then
    echo "No running PIDs found in $PID_FILE."
    cleanup_leaked_stack
    rm -f "$PID_FILE"
    return 0
  fi

  echo "Stopping nodes gracefully (SIGINT): ${pids[*]}"
  for pid in "${pids[@]}"; do
    signal_pid_family "$pid" INT
  done

  local end_time
  end_time=$((SECONDS + SHUTDOWN_TIMEOUT_SEC))
  while (( SECONDS < end_time )); do
    local remaining=()
    for pid in "${pids[@]}"; do
      if is_pid_running "$pid"; then
        remaining+=("$pid")
      fi
    done
    if [[ ${#remaining[@]} -eq 0 ]]; then
      break
    fi
    sleep 0.5
  done

  local still_running=()
  for pid in "${pids[@]}"; do
    if is_pid_running "$pid"; then
      still_running+=("$pid")
    fi
  done

  if [[ ${#still_running[@]} -gt 0 ]]; then
    echo "Some processes still alive after SIGINT; sending SIGTERM."
    for pid in "${still_running[@]}"; do
      signal_pid_family "$pid" TERM
    done
    sleep 1
  fi

  still_running=()
  for pid in "${pids[@]}"; do
    if is_pid_running "$pid"; then
      still_running+=("$pid")
    fi
  done

  if [[ ${#still_running[@]} -gt 0 ]]; then
    echo "Some processes did not exit after ${SHUTDOWN_TIMEOUT_SEC}s: ${still_running[*]}"
    echo "Sending SIGKILL to remaining processes."
    for pid in "${still_running[@]}"; do
      signal_pid_family "$pid" KILL
    done
  fi

  rm -f "$PID_FILE"
  cleanup_leaked_stack
  echo "Shutdown complete."
}

status_stack() {
  mapfile -t pids < <(running_pids_from_file || true)
  if [[ ${#pids[@]} -eq 0 ]]; then
    echo "Stack status: stopped"
  else
    echo "Stack status: running (PIDs: ${pids[*]})"
  fi
}

nav_health() {
  local node_list=""
  local -a required_nodes=("/controller_server" "/planner_server" "/behavior_server" "/bt_navigator")
  local -a missing_nodes=()
  local node=""
  local explore_status=""
  local action_snapshot=""
  local active_count=0
  local succeeded_count=0
  local canceled_count=0
  local aborted_count=0
  local other_count=0
  local cmd_sample=""
  local odom_x_1=""
  local odom_x_2=""
  local odom_delta=""
  local rosout_tmp=""
  local progress_fail_count=0
  local collision_ahead_count=0
  local control_missed_count=0
  local planner_missed_count=0
  local frontier_done_count=0
  local no_frontier_count=0
  local goal_failed_count=0
  local relevant_lines=""
  local bridge_cmd_topic=""
  local bridge_cmd_publishers=""

  source_ros

  node_list="$(timeout 4s ros2 node list 2>/dev/null || true)"
  if [[ -z "$node_list" ]]; then
    echo "Nav status: unable to query ROS graph (is the stack running?)."
    return 1
  fi

  for node in "${required_nodes[@]}"; do
    if ! grep -qx "$node" <<< "$node_list"; then
      missing_nodes+=("$node")
    fi
  done

  if [[ ${#missing_nodes[@]} -eq 0 ]]; then
    echo "Nav nodes: OK (controller/planner/behavior/bt_navigator present)."
  else
    echo "Nav nodes: missing ${missing_nodes[*]}"
  fi

  bridge_cmd_topic="$(timeout 4s ros2 param get /serial_diff_drive_bridge cmd_vel_topic 2>/dev/null | awk '/value is:/ {print $NF; exit}' || true)"
  if [[ -n "$bridge_cmd_topic" ]]; then
    bridge_cmd_publishers="$(timeout 4s ros2 topic info "$bridge_cmd_topic" 2>/dev/null | awk '/Publisher count:/ {print $3; exit}' || true)"
    echo "Bridge cmd topic: $bridge_cmd_topic (publishers=${bridge_cmd_publishers:-unknown})"
    if [[ "${bridge_cmd_publishers:-}" == "0" ]]; then
      echo "Bridge cmd path warning: no publishers on $bridge_cmd_topic"
    fi
  else
    echo "Bridge cmd topic: unknown (serial_diff_drive_bridge parameter unavailable)"
  fi

  if grep -qx "/explore_node" <<< "$node_list"; then
    explore_status="$(timeout 4s ros2 topic echo /explore/status --once 2>/dev/null | awk '/^status:/ {print $2; exit}' || true)"
    if [[ -n "$explore_status" ]]; then
      echo "Explore status: $explore_status"
    else
      echo "Explore status: unknown (no /explore/status sample)."
    fi
  else
    echo "Explore status: explore_node not running."
  fi

  action_snapshot="$(timeout 4s ros2 topic echo /navigate_to_pose/_action/status --once 2>/dev/null || true)"
  if [[ -n "$action_snapshot" ]]; then
    read -r active_count succeeded_count canceled_count aborted_count other_count <<< "$(
      awk '
        /^  status:/ {
          s = $2 + 0
          if (s == 1 || s == 2 || s == 3) active++
          else if (s == 4) succeeded++
          else if (s == 5) canceled++
          else if (s == 6) aborted++
          else other++
        }
        END { print active + 0, succeeded + 0, canceled + 0, aborted + 0, other + 0 }
      ' <<< "$action_snapshot"
    )"
    echo "NavigateToPose statuses: active=$active_count succeeded=$succeeded_count canceled=$canceled_count aborted=$aborted_count other=$other_count"
  else
    echo "NavigateToPose statuses: unavailable (no /navigate_to_pose/_action/status sample)."
  fi

  cmd_sample="$(
    timeout 4s ros2 topic echo /cmd_vel_nav --once --field linear.x 2>/dev/null \
      | awk '/^-?[0-9]+([.][0-9]+)?$/ {print; exit}' || true
  )"
  if [[ -n "$cmd_sample" ]]; then
    echo "cmd_vel_nav: present (sample linear.x=$cmd_sample)"
  else
    echo "cmd_vel_nav: no numeric sample received in 4s"
  fi

  odom_x_1="$(
    timeout 4s ros2 topic echo /odom --once --field pose.pose.position.x 2>/dev/null \
      | awk '/^-?[0-9]+([.][0-9]+)?$/ {print; exit}' || true
  )"
  sleep 1
  odom_x_2="$(
    timeout 4s ros2 topic echo /odom --once --field pose.pose.position.x 2>/dev/null \
      | awk '/^-?[0-9]+([.][0-9]+)?$/ {print; exit}' || true
  )"
  if [[ "$odom_x_1" =~ ^-?[0-9]+([.][0-9]+)?$ && "$odom_x_2" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
    odom_delta="$(awk -v a="$odom_x_1" -v b="$odom_x_2" 'BEGIN { d = b - a; if (d < 0) d = -d; printf "%.4f", d }')"
    echo "odom: x=$odom_x_1 -> $odom_x_2 (|delta|=${odom_delta} m over ~1s)"
  else
    echo "odom: unable to sample pose.pose.position.x twice"
  fi

  rosout_tmp="$(mktemp)"
  timeout 8s ros2 topic echo /rosout > "$rosout_tmp" 2>/dev/null || true

  progress_fail_count="$( (rg -n "Failed to make progress" "$rosout_tmp" 2>/dev/null || true) | wc -l | tr -d ' ' )"
  collision_ahead_count="$( (rg -n "Collision Ahead" "$rosout_tmp" 2>/dev/null || true) | wc -l | tr -d ' ' )"
  control_missed_count="$( (rg -n "Control loop missed" "$rosout_tmp" 2>/dev/null || true) | wc -l | tr -d ' ' )"
  planner_missed_count="$( (rg -n "Planner loop missed" "$rosout_tmp" 2>/dev/null || true) | wc -l | tr -d ' ' )"
  frontier_done_count="$( (rg -n "All frontiers traversed/tried out" "$rosout_tmp" 2>/dev/null || true) | wc -l | tr -d ' ' )"
  no_frontier_count="$( (rg -n "No frontiers found" "$rosout_tmp" 2>/dev/null || true) | wc -l | tr -d ' ' )"
  goal_failed_count="$( (rg -n "Goal failed" "$rosout_tmp" 2>/dev/null || true) | wc -l | tr -d ' ' )"

  echo "Recent rosout (8s): progress_fail=$progress_fail_count collision_ahead=$collision_ahead_count control_missed=$control_missed_count planner_missed=$planner_missed_count frontiers_exhausted=$frontier_done_count no_frontiers=$no_frontier_count goal_failed=$goal_failed_count"

  relevant_lines="$(rg -n "Failed to make progress|Collision Ahead|Control loop missed|Planner loop missed|All frontiers traversed/tried out|No frontiers found|Goal failed" "$rosout_tmp" 2>/dev/null || true)"
  if [[ -n "$relevant_lines" ]]; then
    echo "Recent warning/error lines:"
    echo "$relevant_lines" | tail -n 8
  fi

  rm -f "$rosout_tmp"
}

slam_health() {
  local latest_launch=""
  local start_line=""
  local finish_line=""
  local died_line=""
  local start_pid=""
  local slam_log=""
  local signal_seen=0
  local tf_err_count=0
  local queue_max=""
  local running_now=0
  local companion_failures=""
  local launch_candidate=""

  if pgrep -fa "slam_toolbox.*sync_slam_toolbox_node" >/dev/null 2>&1; then
    running_now=1
  fi

  while IFS= read -r launch_candidate; do
    if rg -q "sync_slam_toolbox_node-.*process started with pid" "$launch_candidate"; then
      latest_launch="$launch_candidate"
      break
    fi
  done < <(ls -1dt /home/hailey/.ros/log/*/launch.log 2>/dev/null || true)

  if [[ -z "$latest_launch" ]]; then
    if [[ "$running_now" -eq 1 ]]; then
      echo "SLAM status: running, but no historical launch log with sync_slam_toolbox_node was found."
      return 0
    fi
    echo "SLAM status: unknown (no sync_slam_toolbox_node launch logs found)."
    return 1
  fi

  start_line="$(rg "sync_slam_toolbox_node-.*process started with pid" "$latest_launch" | tail -n 1 || true)"
  finish_line="$(rg "sync_slam_toolbox_node-.*process has finished cleanly" "$latest_launch" | tail -n 1 || true)"
  died_line="$(rg "sync_slam_toolbox_node-.*process has died" "$latest_launch" | tail -n 1 || true)"
  companion_failures="$(rg "(cmd_vel_safety_gate|drop_detector).*process has died" "$latest_launch" || true)"

  if [[ -n "$start_line" ]]; then
    start_pid="$(echo "$start_line" | sed -n 's/.*pid \[\([0-9]\+\)\].*/\1/p')"
    if [[ -n "$start_pid" ]]; then
      slam_log="$(ls -1t /home/hailey/.ros/log/sync_slam_toolbox_node_"$start_pid"_*.log 2>/dev/null | head -n 1 || true)"
    fi
  fi

  if [[ -n "$slam_log" ]]; then
    if rg -q "signal_handler\\(SIGINT/SIGTERM\\)" "$slam_log"; then
      signal_seen=1
    fi
    tf_err_count="$( (rg -n "Transform from base_link to odom failed" "$slam_log" 2>/dev/null || true) | wc -l | tr -d ' ' )"
    queue_max="$( (rg -o "Queue size has grown to: [0-9]+" "$slam_log" 2>/dev/null || true) | awk '{print $NF}' | sort -nr | head -n 1 )"
  fi

  echo "Latest SLAM launch log: $latest_launch"
  if [[ -n "$slam_log" ]]; then
    echo "Latest SLAM node log: $slam_log"
  fi

  if [[ "$running_now" -eq 1 ]]; then
    echo "SLAM status: RUNNING (/slam_toolbox present)."
  elif [[ -n "$died_line" ]]; then
    echo "SLAM status: CRASHED (sync_slam_toolbox_node reported process has died)."
    echo "$died_line"
    return 1
  elif [[ -n "$finish_line" ]]; then
    if [[ "$signal_seen" -eq 1 ]]; then
      echo "SLAM status: CLEAN EXIT (SIGINT/SIGTERM handled)."
    else
      echo "SLAM status: CLEAN EXIT (process has finished cleanly)."
    fi
  else
    echo "SLAM status: UNKNOWN (no clean-exit or crash line found)."
  fi

  if [[ -n "$queue_max" ]]; then
    echo "SLAM queue max observed: $queue_max"
  fi
  if [[ "$tf_err_count" -gt 0 ]]; then
    echo "SLAM TF extrapolation errors (base_link->odom): $tf_err_count"
  fi
  if [[ -n "$companion_failures" ]]; then
    echo "Companion node failures in same launch (may trigger overall shutdown):"
    echo "$companion_failures"
  fi
}

command="${1:-start}"
case "$command" in
  start)
    start_stack
    ;;
  start-map)
    start_stack map
    ;;
  start-nav)
    start_nav "${2:-}"
    ;;
  start-map-explore)
    start_stack map-explore
    ;;
  save-map)
    save_map "${2:-}"
    ;;
  capture-waypoint)
    capture_waypoint "${2:-}"
    ;;
  send-waypoints)
    shift
    send_waypoints "$@"
    ;;
  slam-health)
    slam_health
    ;;
  nav-health)
    nav_health
    ;;
  stop)
    stop_stack
    ;;
  restart)
    stop_stack
    start_stack
    ;;
  restart-map)
    stop_stack
    start_stack map
    ;;
  restart-map-explore)
    stop_stack
    start_stack map-explore
    ;;
  restart-nav)
    stop_stack
    start_nav "${2:-}"
    ;;
  status)
    status_stack
    ;;
  *)
    usage
    exit 1
    ;;
esac
