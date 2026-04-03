#!/bin/bash

set -euo pipefail

ROS_WS=/ssd/ros2_ws
PID_FILE=/tmp/bluebot_mapper.pid
LOG_FILE=/tmp/bluebot_mapper.log
FOXGLOVE_PID_FILE=/tmp/bluebot_mapper_foxglove.pid
FOXGLOVE_LOG_FILE=/tmp/bluebot_mapper_foxglove.log
UDP_CMD_VEL_BRIDGE_PID_FILE=/tmp/bluebot_mapper_udp_cmd_vel_bridge.pid
UDP_CMD_VEL_BRIDGE_LOG_FILE=/tmp/bluebot_mapper_udp_cmd_vel_bridge.log
SERIAL_BRIDGE_EXEC="${ROS_WS}/install/ros2_serial_diff_drive_bridge/lib/ros2_serial_diff_drive_bridge/serial_diff_drive_bridge"
SHUTDOWN_TIMEOUT_SEC="${SHUTDOWN_TIMEOUT_SEC:-8}"

MAP_DIR="${MAP_DIR:-/ssd/maps}"
MAP_TOPIC="${MAP_TOPIC:-/map}"
MAP_SAVE_TIMEOUT_SEC="${MAP_SAVE_TIMEOUT_SEC:-15.0}"
MAP_OCCUPIED_THRESH="${MAP_OCCUPIED_THRESH:-0.65}"
MAP_FREE_THRESH="${MAP_FREE_THRESH:-0.25}"
MAP_IMAGE_FORMAT="${MAP_IMAGE_FORMAT:-pgm}"
APRILTAG_LANDMARKS_FILE="${APRILTAG_LANDMARKS_FILE:-/tmp/apriltag_map_landmarks.yaml}"
APRILTAG_LANDMARKS_SAVE_ENABLED="${APRILTAG_LANDMARKS_SAVE_ENABLED:-true}"

ARDUINO_PORT="${ARDUINO_PORT:-/dev/arduino}"
ARDUINO_BAUD="${ARDUINO_BAUD:-115200}"
MAP_FRAME="${MAP_FRAME:-map}"
ODOM_FRAME="${ODOM_FRAME:-odom}"
BASE_FRAME="${BASE_FRAME:-base_link}"
WHEEL_RADIUS_M="${WHEEL_RADIUS_M:-0.03354}"
WHEEL_SEPARATION_M="${WHEEL_SEPARATION_M:-0.195}"
LIDAR_PORT="${LIDAR_PORT:-/dev/lidar}"
LIDAR_FRAME="${LIDAR_FRAME:-laser}"
LIDAR_SCAN_FREQUENCY="${LIDAR_SCAN_FREQUENCY:-8.0}"
LIDAR_ANGLE_COMPENSATE="${LIDAR_ANGLE_COMPENSATE:-true}"
LIDAR_SCAN_MODE="${LIDAR_SCAN_MODE:-}"
LIDAR_TF_X="${LIDAR_TF_X:-0.0}"
LIDAR_TF_Y="${LIDAR_TF_Y:-0.0}"
LIDAR_TF_Z="${LIDAR_TF_Z:-0.199}"
LIDAR_TF_ROLL="${LIDAR_TF_ROLL:-0.0}"
LIDAR_TF_PITCH="${LIDAR_TF_PITCH:-0.0}"
LIDAR_TF_YAW="${LIDAR_TF_YAW:-0.0}"
IMU_FRAME="${IMU_FRAME:-imu_link}"
IMU_PORT="${IMU_PORT:-/dev/myimu}"
IMU_BAUD="${IMU_BAUD:-115200}"
IMU_TF_X="${IMU_TF_X:-0.0}"
IMU_TF_Y="${IMU_TF_Y:-0.0}"
IMU_TF_Z="${IMU_TF_Z:-0.0}"
IMU_TF_ROLL="${IMU_TF_ROLL:-0.0}"
IMU_TF_PITCH="${IMU_TF_PITCH:-0.0}"
IMU_TF_YAW="${IMU_TF_YAW:-0.0}"

USE_RVIZ="${USE_RVIZ:-false}"
NAV2_USE_SIM_TIME="${NAV2_USE_SIM_TIME:-false}"
SLAM_PARAMS_FILE="${SLAM_PARAMS_FILE:-/ssd/ros2_ws/src/bluebot_v3/config/slam_toolbox_mapping_nav2.yaml}"
ODOM_SOURCE_MODE="${ODOM_SOURCE_MODE:-ekf}"
FOXGLOVE_BRIDGE_ENABLED="${FOXGLOVE_BRIDGE_ENABLED:-true}"
FOXGLOVE_BRIDGE_PORT="${FOXGLOVE_BRIDGE_PORT:-8765}"
FOXGLOVE_BRIDGE_ADDRESS="${FOXGLOVE_BRIDGE_ADDRESS:-0.0.0.0}"
FOXGLOVE_BRIDGE_CAPABILITIES="${FOXGLOVE_BRIDGE_CAPABILITIES:-[clientPublish,services,connectionGraph,assets]}"
UDP_CMD_VEL_BRIDGE_ENABLED="${UDP_CMD_VEL_BRIDGE_ENABLED:-true}"
UDP_CMD_VEL_BRIDGE_LISTEN_HOST="${UDP_CMD_VEL_BRIDGE_LISTEN_HOST:-0.0.0.0}"
UDP_CMD_VEL_BRIDGE_LISTEN_PORT="${UDP_CMD_VEL_BRIDGE_LISTEN_PORT:-8766}"
UDP_CMD_VEL_BRIDGE_CMD_TOPIC="${UDP_CMD_VEL_BRIDGE_CMD_TOPIC:-/cmd_vel}"
UDP_CMD_VEL_BRIDGE_TIMEOUT_S="${UDP_CMD_VEL_BRIDGE_TIMEOUT_S:-0.5}"
STRAIGHT_COMP_ENABLED="${STRAIGHT_COMP_ENABLED:-true}"
STRAIGHT_COMP_ODOM_TOPIC="${STRAIGHT_COMP_ODOM_TOPIC:-__AUTO__}"
STRAIGHT_COMP_LINEAR_MIN="${STRAIGHT_COMP_LINEAR_MIN:-0.03}"
STRAIGHT_COMP_ANGULAR_DEADBAND="${STRAIGHT_COMP_ANGULAR_DEADBAND:-0.05}"
STRAIGHT_COMP_TURN_MEMORY_ANGULAR_THRESHOLD="${STRAIGHT_COMP_TURN_MEMORY_ANGULAR_THRESHOLD:-0.20}"
STRAIGHT_COMP_KP="${STRAIGHT_COMP_KP:-1.8}"
STRAIGHT_COMP_KI="${STRAIGHT_COMP_KI:-0.0}"
STRAIGHT_COMP_KD="${STRAIGHT_COMP_KD:-0.20}"
STRAIGHT_COMP_MAX_ANGULAR="${STRAIGHT_COMP_MAX_ANGULAR:-0.50}"
STRAIGHT_COMP_INTEGRAL_LIMIT="${STRAIGHT_COMP_INTEGRAL_LIMIT:-0.40}"
STRAIGHT_COMP_CASTER_ENABLED="${STRAIGHT_COMP_CASTER_ENABLED:-true}"
STRAIGHT_COMP_CASTER_GAIN="${STRAIGHT_COMP_CASTER_GAIN:-0.12}"
STRAIGHT_COMP_CASTER_DECAY_SEC="${STRAIGHT_COMP_CASTER_DECAY_SEC:-0.60}"
STRAIGHT_COMP_CASTER_MAX="${STRAIGHT_COMP_CASTER_MAX:-0.25}"
STRAIGHT_COMP_CASTER_MAX_AGE_SEC="${STRAIGHT_COMP_CASTER_MAX_AGE_SEC:-2.0}"
STRAIGHT_COMP_CASTER_FORWARD_ONLY="${STRAIGHT_COMP_CASTER_FORWARD_ONLY:-true}"
STRAIGHT_COMP_ODOM_TIMEOUT_SEC="${STRAIGHT_COMP_ODOM_TIMEOUT_SEC:-0.40}"
STRAIGHT_COMP_RESET_ON_REVERSE="${STRAIGHT_COMP_RESET_ON_REVERSE:-true}"

usage() {
  echo "Usage: $0 [start|stop|restart|status|tf-check|save-map [name]]"
}

robot_ip_hint() {
  local ip
  ip="$(hostname -I 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i !~ /^127\./) {print $i; exit}}')"
  if [[ -n "$ip" ]]; then
    echo "$ip"
  else
    echo "<robot-ip>"
  fi
}

normalize_foxglove_capabilities() {
  local raw="$1"
  local stripped="${raw//[[:space:]]/}"

  if [[ -z "$stripped" ]]; then
    echo "[clientPublish,services,connectionGraph,assets]"
    return
  fi

  if [[ "$stripped" == \[*\] ]]; then
    echo "$stripped"
    return
  fi

  # Backward compatible with comma-separated env format.
  if [[ "$stripped" == *","* ]]; then
    echo "[$stripped]"
    return
  fi

  echo "[$stripped]"
}

source_ros() {
  local had_nounset=0
  if [[ $- == *u* ]]; then
    had_nounset=1
    set +u
  fi
  source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
  source "$ROS_WS/install/setup.bash"
  if [[ $had_nounset -eq 1 ]]; then
    set -u
  fi
}

is_running() {
  [[ -f "$PID_FILE" ]] || return 1
  local pid
  pid="$(cat "$PID_FILE" 2>/dev/null || true)"
  [[ -n "$pid" ]] || return 1
  kill -0 "$pid" 2>/dev/null
}

cleanup_orphan_serial_bridge() {
  local current_launch_pid=""
  local pid
  local ppid
  local parent_cmd
  local -a stale_pids=()

  if is_running; then
    current_launch_pid="$(cat "$PID_FILE" 2>/dev/null || true)"
  fi

  while IFS= read -r pid; do
    [[ -n "$pid" ]] || continue
    ppid="$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"

    if [[ -n "$current_launch_pid" && "$ppid" == "$current_launch_pid" ]]; then
      continue
    fi

    if [[ -z "$ppid" || "$ppid" == "1" ]]; then
      stale_pids+=("$pid")
      continue
    fi

    parent_cmd="$(ps -o args= -p "$ppid" 2>/dev/null || true)"
    if [[ -z "$parent_cmd" ]]; then
      stale_pids+=("$pid")
    fi
  done < <(pgrep -f "$SERIAL_BRIDGE_EXEC" 2>/dev/null || true)

  if [[ "${#stale_pids[@]}" -eq 0 ]]; then
    return 0
  fi

  echo "Cleaning up stale serial_diff_drive_bridge process(es): ${stale_pids[*]}"
  for pid in "${stale_pids[@]}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  sleep 0.4
  for pid in "${stale_pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  sleep 0.4
  for pid in "${stale_pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
}

is_foxglove_running() {
  [[ -f "$FOXGLOVE_PID_FILE" ]] || return 1
  local pid
  pid="$(cat "$FOXGLOVE_PID_FILE" 2>/dev/null || true)"
  [[ -n "$pid" ]] || return 1
  kill -0 "$pid" 2>/dev/null
}

is_udp_cmd_vel_bridge_running() {
  [[ -f "$UDP_CMD_VEL_BRIDGE_PID_FILE" ]] || return 1
  local pid
  pid="$(cat "$UDP_CMD_VEL_BRIDGE_PID_FILE" 2>/dev/null || true)"
  [[ -n "$pid" ]] || return 1
  kill -0 "$pid" 2>/dev/null
}

start_udp_cmd_vel_bridge() {
  if [[ "${UDP_CMD_VEL_BRIDGE_ENABLED,,}" != "true" ]]; then
    rm -f "$UDP_CMD_VEL_BRIDGE_PID_FILE"
    return 0
  fi

  if is_udp_cmd_vel_bridge_running; then
    echo "udp_cmd_vel_bridge already running (PID: $(cat "$UDP_CMD_VEL_BRIDGE_PID_FILE"))."
    return 0
  fi

  if command -v setsid >/dev/null 2>&1; then
    setsid ros2 launch udp_cmd_vel_bridge udp_cmd_vel_bridge.launch.py \
      listen_host:="$UDP_CMD_VEL_BRIDGE_LISTEN_HOST" \
      listen_port:="$UDP_CMD_VEL_BRIDGE_LISTEN_PORT" \
      cmd_vel_topic:="$UDP_CMD_VEL_BRIDGE_CMD_TOPIC" \
      timeout_s:="$UDP_CMD_VEL_BRIDGE_TIMEOUT_S" >"$UDP_CMD_VEL_BRIDGE_LOG_FILE" 2>&1 &
  else
    ros2 launch udp_cmd_vel_bridge udp_cmd_vel_bridge.launch.py \
      listen_host:="$UDP_CMD_VEL_BRIDGE_LISTEN_HOST" \
      listen_port:="$UDP_CMD_VEL_BRIDGE_LISTEN_PORT" \
      cmd_vel_topic:="$UDP_CMD_VEL_BRIDGE_CMD_TOPIC" \
      timeout_s:="$UDP_CMD_VEL_BRIDGE_TIMEOUT_S" >"$UDP_CMD_VEL_BRIDGE_LOG_FILE" 2>&1 &
  fi

  echo "$!" > "$UDP_CMD_VEL_BRIDGE_PID_FILE"
  sleep 1

  if ! is_udp_cmd_vel_bridge_running; then
    echo "Warning: udp_cmd_vel_bridge failed to stay running. Check log: $UDP_CMD_VEL_BRIDGE_LOG_FILE"
    tail -n 30 "$UDP_CMD_VEL_BRIDGE_LOG_FILE" 2>/dev/null || true
    rm -f "$UDP_CMD_VEL_BRIDGE_PID_FILE"
    return 1
  fi

  echo "udp_cmd_vel_bridge started (PID: $(cat "$UDP_CMD_VEL_BRIDGE_PID_FILE"))."
  echo "udp_cmd_vel_bridge log: $UDP_CMD_VEL_BRIDGE_LOG_FILE"
  echo "udp_cmd_vel_bridge: udp://${UDP_CMD_VEL_BRIDGE_LISTEN_HOST}:${UDP_CMD_VEL_BRIDGE_LISTEN_PORT} -> ${UDP_CMD_VEL_BRIDGE_CMD_TOPIC}"
}

stop_udp_cmd_vel_bridge() {
  if ! is_udp_cmd_vel_bridge_running; then
    rm -f "$UDP_CMD_VEL_BRIDGE_PID_FILE"
    return 0
  fi

  local pid
  local pgid
  local end_time
  pid="$(cat "$UDP_CMD_VEL_BRIDGE_PID_FILE")"
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"

  if [[ -n "$pgid" ]]; then
    kill -INT -- "-$pgid" 2>/dev/null || true
  fi
  kill -INT "$pid" 2>/dev/null || true

  end_time=$((SECONDS + SHUTDOWN_TIMEOUT_SEC))
  while (( SECONDS < end_time )); do
    if ! kill -0 "$pid" 2>/dev/null; then
      break
    fi
    sleep 0.3
  done

  if kill -0 "$pid" 2>/dev/null; then
    if [[ -n "$pgid" ]]; then
      kill -TERM -- "-$pgid" 2>/dev/null || true
    fi
    kill -TERM "$pid" 2>/dev/null || true
  fi

  rm -f "$UDP_CMD_VEL_BRIDGE_PID_FILE"
  echo "udp_cmd_vel_bridge stopped."
}

start_foxglove_bridge() {
  if [[ "${FOXGLOVE_BRIDGE_ENABLED,,}" != "true" ]]; then
    rm -f "$FOXGLOVE_PID_FILE"
    return 0
  fi

  if is_foxglove_running; then
    echo "Foxglove Bridge already running (PID: $(cat "$FOXGLOVE_PID_FILE"))."
    return 0
  fi

  local foxglove_capabilities
  foxglove_capabilities="$(normalize_foxglove_capabilities "$FOXGLOVE_BRIDGE_CAPABILITIES")"

  if command -v setsid >/dev/null 2>&1; then
    setsid ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
      port:="$FOXGLOVE_BRIDGE_PORT" \
      address:="$FOXGLOVE_BRIDGE_ADDRESS" \
      capabilities:="$foxglove_capabilities" >"$FOXGLOVE_LOG_FILE" 2>&1 &
  else
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
      port:="$FOXGLOVE_BRIDGE_PORT" \
      address:="$FOXGLOVE_BRIDGE_ADDRESS" \
      capabilities:="$foxglove_capabilities" >"$FOXGLOVE_LOG_FILE" 2>&1 &
  fi
  echo "$!" > "$FOXGLOVE_PID_FILE"
  sleep 1

  if ! is_foxglove_running; then
    echo "Warning: Foxglove Bridge failed to stay running. Check log: $FOXGLOVE_LOG_FILE"
    tail -n 30 "$FOXGLOVE_LOG_FILE" 2>/dev/null || true
    rm -f "$FOXGLOVE_PID_FILE"
    return 1
  fi

  echo "Foxglove Bridge started (PID: $(cat "$FOXGLOVE_PID_FILE"))."
  echo "Foxglove websocket: ws://$(robot_ip_hint):${FOXGLOVE_BRIDGE_PORT}"
}

stop_foxglove_bridge() {
  if ! is_foxglove_running; then
    rm -f "$FOXGLOVE_PID_FILE"
    return 0
  fi

  local pid
  local pgid
  local end_time
  pid="$(cat "$FOXGLOVE_PID_FILE")"
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"

  if [[ -n "$pgid" ]]; then
    kill -INT -- "-$pgid" 2>/dev/null || true
  fi
  kill -INT "$pid" 2>/dev/null || true

  end_time=$((SECONDS + SHUTDOWN_TIMEOUT_SEC))
  while (( SECONDS < end_time )); do
    if ! kill -0 "$pid" 2>/dev/null; then
      break
    fi
    sleep 0.3
  done

  if kill -0 "$pid" 2>/dev/null; then
    if [[ -n "$pgid" ]]; then
      kill -TERM -- "-$pgid" 2>/dev/null || true
    fi
    kill -TERM "$pid" 2>/dev/null || true
  fi

  rm -f "$FOXGLOVE_PID_FILE"
  echo "Foxglove Bridge stopped."
}

start_stack() {
  cleanup_orphan_serial_bridge

  if is_running; then
    echo "bluebot_mapper is already running (PID: $(cat "$PID_FILE"))."
    echo "Log: $LOG_FILE"
    source_ros
    start_udp_cmd_vel_bridge || true
    start_foxglove_bridge || true
    return 0
  fi

  source_ros

  if [[ "${FOXGLOVE_BRIDGE_ENABLED,,}" == "true" && "${UDP_CMD_VEL_BRIDGE_ENABLED,,}" == "true" ]]; then
    if [[ "$FOXGLOVE_BRIDGE_PORT" == "$UDP_CMD_VEL_BRIDGE_LISTEN_PORT" ]]; then
      echo "Port conflict: FOXGLOVE_BRIDGE_PORT ($FOXGLOVE_BRIDGE_PORT) equals UDP_CMD_VEL_BRIDGE_LISTEN_PORT ($UDP_CMD_VEL_BRIDGE_LISTEN_PORT)."
      echo "Set one of them to a different port (recommended UDP_CMD_VEL_BRIDGE_LISTEN_PORT=8766)."
      return 1
    fi
  fi

  local -a launch_cmd
  local use_ekf="true"
  local publish_bridge_tf="false"
  local straight_comp_odom_topic="$STRAIGHT_COMP_ODOM_TOPIC"
  case "${ODOM_SOURCE_MODE,,}" in
    ekf)
      use_ekf="true"
      publish_bridge_tf="false"
      if [[ "$straight_comp_odom_topic" == "__AUTO__" ]]; then
        straight_comp_odom_topic="/odom"
      fi
      ;;
    raw)
      # Bypass EKF and map directly from wheel-odom TF to isolate IMU/EKF issues.
      use_ekf="false"
      publish_bridge_tf="true"
      if [[ "$straight_comp_odom_topic" == "__AUTO__" ]]; then
        straight_comp_odom_topic="/odom_raw"
      fi
      ;;
    *)
      echo "Invalid ODOM_SOURCE_MODE='$ODOM_SOURCE_MODE' (expected: ekf|raw)"
      return 1
      ;;
  esac

  launch_cmd=(
    ros2 launch bluebot_v3 mapping_stack.launch.py
    "arduino_port:=$ARDUINO_PORT"
    "arduino_baud:=$ARDUINO_BAUD"
    "map_frame:=$MAP_FRAME"
    "odom_frame:=$ODOM_FRAME"
    "base_frame:=$BASE_FRAME"
    "wheel_radius_m:=$WHEEL_RADIUS_M"
    "wheel_separation_m:=$WHEEL_SEPARATION_M"
    "publish_bridge_tf:=$publish_bridge_tf"
    "lidar_port:=$LIDAR_PORT"
    "lidar_frame:=$LIDAR_FRAME"
    "lidar_scan_frequency:=$LIDAR_SCAN_FREQUENCY"
    "lidar_angle_compensate:=$LIDAR_ANGLE_COMPENSATE"
    "lidar_tf_x:=$LIDAR_TF_X"
    "lidar_tf_y:=$LIDAR_TF_Y"
    "lidar_tf_z:=$LIDAR_TF_Z"
    "lidar_tf_roll:=$LIDAR_TF_ROLL"
    "lidar_tf_pitch:=$LIDAR_TF_PITCH"
    "lidar_tf_yaw:=$LIDAR_TF_YAW"
    "imu_frame:=$IMU_FRAME"
    "imu_serial_port:=$IMU_PORT"
    "imu_serial_baud:=$IMU_BAUD"
    "imu_tf_x:=$IMU_TF_X"
    "imu_tf_y:=$IMU_TF_Y"
    "imu_tf_z:=$IMU_TF_Z"
    "imu_tf_roll:=$IMU_TF_ROLL"
    "imu_tf_pitch:=$IMU_TF_PITCH"
    "imu_tf_yaw:=$IMU_TF_YAW"
    "use_sim_time:=$NAV2_USE_SIM_TIME"
    "use_rviz:=$USE_RVIZ"
    "use_ekf:=$use_ekf"
    "straight_comp_enabled:=$STRAIGHT_COMP_ENABLED"
    "straight_comp_odom_topic:=$straight_comp_odom_topic"
    "straight_comp_linear_speed_min:=$STRAIGHT_COMP_LINEAR_MIN"
    "straight_comp_angular_deadband:=$STRAIGHT_COMP_ANGULAR_DEADBAND"
    "straight_comp_turn_memory_angular_threshold:=$STRAIGHT_COMP_TURN_MEMORY_ANGULAR_THRESHOLD"
    "straight_comp_kp:=$STRAIGHT_COMP_KP"
    "straight_comp_ki:=$STRAIGHT_COMP_KI"
    "straight_comp_kd:=$STRAIGHT_COMP_KD"
    "straight_comp_max_angular:=$STRAIGHT_COMP_MAX_ANGULAR"
    "straight_comp_integral_limit:=$STRAIGHT_COMP_INTEGRAL_LIMIT"
    "straight_comp_caster_enabled:=$STRAIGHT_COMP_CASTER_ENABLED"
    "straight_comp_caster_gain:=$STRAIGHT_COMP_CASTER_GAIN"
    "straight_comp_caster_decay_sec:=$STRAIGHT_COMP_CASTER_DECAY_SEC"
    "straight_comp_caster_max:=$STRAIGHT_COMP_CASTER_MAX"
    "straight_comp_caster_max_age_sec:=$STRAIGHT_COMP_CASTER_MAX_AGE_SEC"
    "straight_comp_caster_forward_only:=$STRAIGHT_COMP_CASTER_FORWARD_ONLY"
    "straight_comp_odom_timeout_sec:=$STRAIGHT_COMP_ODOM_TIMEOUT_SEC"
    "straight_comp_reset_on_reverse:=$STRAIGHT_COMP_RESET_ON_REVERSE"
    "slam_params_file:=$SLAM_PARAMS_FILE"
  )
  if [[ -n "$LIDAR_SCAN_MODE" ]]; then
    launch_cmd+=("lidar_scan_mode:=$LIDAR_SCAN_MODE")
  fi

  if command -v setsid >/dev/null 2>&1; then
    setsid "${launch_cmd[@]}" >"$LOG_FILE" 2>&1 &
  else
    "${launch_cmd[@]}" >"$LOG_FILE" 2>&1 &
  fi

  echo "$!" > "$PID_FILE"
  sleep 2
  if ! is_running; then
    echo "bluebot_mapper failed to stay running. Check log: $LOG_FILE"
    tail -n 40 "$LOG_FILE" 2>/dev/null || true
    rm -f "$PID_FILE"
    return 1
  fi

  echo "bluebot_mapper started (bluebot_v3 mapping stack, PID: $(cat "$PID_FILE"))."
  echo "Log: $LOG_FILE"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
  echo "ODOM source mode: ${ODOM_SOURCE_MODE,,} (use_ekf=$use_ekf, bridge_tf=$publish_bridge_tf)"
  echo "A471 serial IMU: port=${IMU_PORT}, baud=${IMU_BAUD}"
  echo "Wheel kinematics: radius=${WHEEL_RADIUS_M} m, separation=${WHEEL_SEPARATION_M} m"
  echo "Straight compensation: enabled=${STRAIGHT_COMP_ENABLED}, odom=${straight_comp_odom_topic}, kp=${STRAIGHT_COMP_KP}, kd=${STRAIGHT_COMP_KD}"
  echo "Caster compensation: enabled=${STRAIGHT_COMP_CASTER_ENABLED}, gain=${STRAIGHT_COMP_CASTER_GAIN}, decay=${STRAIGHT_COMP_CASTER_DECAY_SEC}s"
  echo "TF chain target: ${MAP_FRAME} -> ${ODOM_FRAME} -> ${BASE_FRAME} -> {${LIDAR_FRAME},${IMU_FRAME}}"
  echo "Drive using your remote joystick publisher on /cmd_vel."
  echo "When done, run: $0 save-map <name>"
  echo "Run '$0 tf-check' to verify TF links."

  start_udp_cmd_vel_bridge || true
  start_foxglove_bridge || true
}

stop_stack() {
  if ! is_running; then
    echo "bluebot_mapper is not running."
    rm -f "$PID_FILE"
    cleanup_orphan_serial_bridge
    stop_udp_cmd_vel_bridge
    stop_foxglove_bridge
    return 0
  fi

  local pid
  local pgid
  local end_time

  pid="$(cat "$PID_FILE")"
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"

  if [[ -n "$pgid" ]]; then
    kill -INT -- "-$pgid" 2>/dev/null || true
  fi
  kill -INT "$pid" 2>/dev/null || true

  end_time=$((SECONDS + SHUTDOWN_TIMEOUT_SEC))
  while (( SECONDS < end_time )); do
    if ! kill -0 "$pid" 2>/dev/null; then
      break
    fi
    sleep 0.3
  done

  if kill -0 "$pid" 2>/dev/null; then
    if [[ -n "$pgid" ]]; then
      kill -TERM -- "-$pgid" 2>/dev/null || true
    fi
    kill -TERM "$pid" 2>/dev/null || true
  fi

  rm -f "$PID_FILE"
  cleanup_orphan_serial_bridge
  echo "bluebot_mapper stopped."
  stop_udp_cmd_vel_bridge
  stop_foxglove_bridge
}

status_stack() {
  if is_running; then
    echo "bluebot_mapper is running (PID: $(cat "$PID_FILE"))."
    echo "Log: $LOG_FILE"
  else
    echo "bluebot_mapper is not running."
  fi

  if is_foxglove_running; then
    echo "Foxglove Bridge is running (PID: $(cat "$FOXGLOVE_PID_FILE"))."
    echo "Foxglove log: $FOXGLOVE_LOG_FILE"
    echo "Foxglove websocket: ws://$(robot_ip_hint):${FOXGLOVE_BRIDGE_PORT}"
  elif [[ "${FOXGLOVE_BRIDGE_ENABLED,,}" == "true" ]]; then
    echo "Foxglove Bridge is not running."
    echo "Foxglove log: $FOXGLOVE_LOG_FILE"
  else
    echo "Foxglove Bridge is disabled (FOXGLOVE_BRIDGE_ENABLED=$FOXGLOVE_BRIDGE_ENABLED)."
  fi

  if is_udp_cmd_vel_bridge_running; then
    echo "udp_cmd_vel_bridge is running (PID: $(cat "$UDP_CMD_VEL_BRIDGE_PID_FILE"))."
    echo "udp_cmd_vel_bridge log: $UDP_CMD_VEL_BRIDGE_LOG_FILE"
    echo "udp_cmd_vel_bridge: udp://${UDP_CMD_VEL_BRIDGE_LISTEN_HOST}:${UDP_CMD_VEL_BRIDGE_LISTEN_PORT} -> ${UDP_CMD_VEL_BRIDGE_CMD_TOPIC}"
  elif [[ "${UDP_CMD_VEL_BRIDGE_ENABLED,,}" == "true" ]]; then
    echo "udp_cmd_vel_bridge is not running."
    echo "udp_cmd_vel_bridge log: $UDP_CMD_VEL_BRIDGE_LOG_FILE"
  else
    echo "udp_cmd_vel_bridge is disabled (UDP_CMD_VEL_BRIDGE_ENABLED=$UDP_CMD_VEL_BRIDGE_ENABLED)."
  fi
}

check_tf_pair() {
  local target="$1"
  local source="$2"
  local out

  out="$(timeout 6s ros2 run tf2_ros tf2_echo "$target" "$source" 2>&1 || true)"
  if [[ "$out" == *"At time"* ]]; then
    echo "OK: ${target} <- ${source}"
    return 0
  fi

  echo "MISSING: ${target} <- ${source}"
  local last_line
  last_line="$(printf '%s\n' "$out" | tail -n 1)"
  if [[ -n "$last_line" ]]; then
    echo "  tf2_echo: $last_line"
  fi
  return 1
}

tf_check() {
  local failures=0
  source_ros

  if ! is_running; then
    echo "Warning: bluebot_mapper is not running. TF check may fail unless stack was started manually."
  fi

  echo "Checking required TF links for mapping..."
  check_tf_pair "$BASE_FRAME" "$LIDAR_FRAME" || failures=$((failures + 1))
  check_tf_pair "$BASE_FRAME" "$IMU_FRAME" || failures=$((failures + 1))
  check_tf_pair "$ODOM_FRAME" "$BASE_FRAME" || failures=$((failures + 1))
  check_tf_pair "$MAP_FRAME" "$ODOM_FRAME" || failures=$((failures + 1))

  if [[ "$failures" -gt 0 ]]; then
    echo "TF check failed (${failures} missing links)."
    return 1
  fi

  echo "TF check passed."
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
    echo "Start mapping first with '$0 start', then drive briefly and retry."
    return 1
  fi

  echo "Saving map to ${map_path}.{yaml,${MAP_IMAGE_FORMAT}} ..."
  ros2 run nav2_map_server map_saver_cli -f "$map_path" \
    --occ "$MAP_OCCUPIED_THRESH" \
    --free "$MAP_FREE_THRESH" \
    --fmt "$MAP_IMAGE_FORMAT" \
    --ros-args \
    -p save_map_timeout:="$MAP_SAVE_TIMEOUT_SEC" \
    -p map_subscribe_transient_local:=true \
    -r map:="$MAP_TOPIC"

  if [[ "${APRILTAG_LANDMARKS_SAVE_ENABLED,,}" == "true" ]]; then
    local apriltag_dst="${map_path}.apriltags.yaml"
    if [[ -s "$APRILTAG_LANDMARKS_FILE" ]]; then
      if cp "$APRILTAG_LANDMARKS_FILE" "$apriltag_dst"; then
        echo "Saved AprilTag landmarks to ${apriltag_dst}"
      else
        echo "Warning: failed to copy AprilTag landmarks to ${apriltag_dst}"
      fi
    else
      echo "AprilTag landmarks not copied (missing or empty): $APRILTAG_LANDMARKS_FILE"
      echo "Enable apriltag recorder during mapping or set APRILTAG_LANDMARKS_FILE."
    fi
  fi
}

case "${1:-}" in
  start)
    start_stack
    ;;
  stop)
    stop_stack
    ;;
  restart)
    stop_stack
    start_stack
    ;;
  status)
    status_stack
    ;;
  tf-check)
    tf_check
    ;;
  save-map)
    save_map "${2:-}"
    ;;
  *)
    usage
    exit 1
    ;;
esac
