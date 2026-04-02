#!/bin/bash

set -euo pipefail

ROS_WS=/ssd/ros2_ws
PID_FILE=/tmp/bluebot_v2.pid
LOG_FILE=/tmp/bluebot_v2.log
MAP_DIR="${MAP_DIR:-/ssd/maps}"
MAP_TOPIC="${MAP_TOPIC:-/map}"
ARDUINO_PORT="${ARDUINO_PORT:-/dev/arduino}"
LIDAR_PORT="${LIDAR_PORT:-/dev/lidar}"
USE_RVIZ="${USE_RVIZ:-false}"
CMD_TOPIC="${CMD_TOPIC:-/bluebot_v2/drive_command}"

usage() {
  echo "Usage: $0 [start|stop|restart|status|cmd <forward|backward|left|right|stop> [duration_sec]|save-map [name]]"
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

start_stack() {
  if is_running; then
    echo "bluebot_v2 is already running (PID: $(cat "$PID_FILE"))."
    echo "Log: $LOG_FILE"
    return 1
  fi

  source_ros

  if command -v setsid >/dev/null 2>&1; then
    setsid ros2 launch bluebot_v2 minimal_mapping.launch.py \
      arduino_port:="$ARDUINO_PORT" \
      lidar_port:="$LIDAR_PORT" \
      use_rviz:="$USE_RVIZ" \
      foxglove_bridge_enabled:=false >"$LOG_FILE" 2>&1 &
  else
    ros2 launch bluebot_v2 minimal_mapping.launch.py \
      arduino_port:="$ARDUINO_PORT" \
      lidar_port:="$LIDAR_PORT" \
      use_rviz:="$USE_RVIZ" \
      foxglove_bridge_enabled:=false >"$LOG_FILE" 2>&1 &
  fi

  echo "$!" > "$PID_FILE"
  echo "bluebot_v2 started (PID: $(cat "$PID_FILE"))."
  echo "Log: $LOG_FILE"
  echo "Send commands with: $0 cmd forward"
}

stop_stack() {
  if ! is_running; then
    echo "bluebot_v2 is not running."
    rm -f "$PID_FILE"
    return 0
  fi

  local pid
  local pgid
  pid="$(cat "$PID_FILE")"
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"

  if [[ -n "$pgid" ]]; then
    kill -INT -- "-$pgid" 2>/dev/null || true
  fi
  kill -INT "$pid" 2>/dev/null || true

  sleep 2
  if kill -0 "$pid" 2>/dev/null; then
    if [[ -n "$pgid" ]]; then
      kill -TERM -- "-$pgid" 2>/dev/null || true
    fi
    kill -TERM "$pid" 2>/dev/null || true
  fi

  rm -f "$PID_FILE"
  echo "bluebot_v2 stopped."
}

status_stack() {
  if is_running; then
    echo "bluebot_v2 is running (PID: $(cat "$PID_FILE"))."
    echo "Log: $LOG_FILE"
  else
    echo "bluebot_v2 is not running."
  fi
}

send_command() {
  local command="${1:-}"
  local duration="${2:-}"

  case "$command" in
    forward|backward|left|right|stop)
      ;;
    *)
      echo "Invalid command '$command'."
      usage
      return 1
      ;;
  esac

  source_ros

  local payload="$command"
  if [[ -n "$duration" ]]; then
    payload="$command $duration"
  fi

  ros2 topic pub --once "$CMD_TOPIC" std_msgs/msg/String "{data: '$payload'}"
}

save_map() {
  local map_name="${1:-map_$(date +%Y%m%d_%H%M%S)}"
  local map_path="$MAP_DIR/$map_name"

  source_ros
  mkdir -p "$MAP_DIR"

  echo "Saving map to ${map_path}.{yaml,pgm}"
  ros2 run nav2_map_server map_saver_cli -f "$map_path" --ros-args -r map:="$MAP_TOPIC"
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
  cmd)
    send_command "${2:-}" "${3:-}"
    ;;
  save-map)
    save_map "${2:-}"
    ;;
  *)
    usage
    exit 1
    ;;
esac
