#!/bin/bash

set -euo pipefail

ROS_WS="${ROS_WS:-/ssd/ros2_ws}"
BRINGUP_SCRIPT="${BRINGUP_SCRIPT:-$ROS_WS/scripts/bluebot_bringup.sh}"
RVIZ_START_SCRIPT="${RVIZ_START_SCRIPT:-$ROS_WS/rviz/start-rviz}"
RVIZ_CONFIG="${RVIZ_CONFIG:-$ROS_WS/rviz/my_house_nav.rviz}"
FOXGLOVE_LAYOUT="${FOXGLOVE_LAYOUT:-$ROS_WS/foxglove/bluebot_nav_layout.json}"
FOXGLOVE_BT_LAYOUT="${FOXGLOVE_BT_LAYOUT:-$ROS_WS/foxglove/bluebot_nav_bt_layout.json}"
FOXGLOVE_EXTENSION_DIR="${FOXGLOVE_EXTENSION_DIR:-$ROS_WS/foxglove/bluebot_nav2_extension}"

MAP_DIR="${MAP_DIR:-/ssd/maps}"
AUTO_RVIZ="${AUTO_RVIZ:-true}"
NAV_UI="${NAV_UI:-foxglove}"
FOXGLOVE_BRIDGE_PORT="${FOXGLOVE_BRIDGE_PORT:-8765}"
FOXGLOVE_WAYPOINT_TOPIC="${FOXGLOVE_WAYPOINT_TOPIC:-/foxglove/waypoints}"
FOXGLOVE_WAYPOINT_STATUS_TOPIC="${FOXGLOVE_WAYPOINT_STATUS_TOPIC:-/foxglove/waypoints/status}"
SHUTDOWN_TIMEOUT_SEC="${SHUTDOWN_TIMEOUT_SEC:-8}"

RVIZ_PID_FILE=/tmp/bluebot_nav_rviz.pid
RVIZ_LOG_FILE=/tmp/bluebot_nav_rviz.log

usage() {
  echo "Usage: $0 [start [map]|stop|restart [map]|status|maps|goal <x> <y> [yaw_deg]|waypoints [name ...]|foxglove]"
  echo "  start [map]          Start navigation using map from /ssd/maps (or explicit map name/path)."
  echo "  stop                 Stop navigation stack and RViz (if launched by this script)."
  echo "  restart [map]        Stop then start navigation."
  echo "  status               Show navigation and UI status."
  echo "  maps                 List saved maps in $MAP_DIR."
  echo "  goal x y [yaw_deg]   Publish one navigation goal to /goal_pose in map frame."
  echo "  waypoints [name ...] Publish saved waypoints to the waypoint follower."
  echo "  foxglove             Print Foxglove connection and layout info."
  echo
  echo "Environment:"
  echo "  NAV_UI=foxglove|rviz|none (default: foxglove)"
  echo "  FOXGLOVE_LAYOUT=$FOXGLOVE_LAYOUT"
  echo "  FOXGLOVE_BT_LAYOUT=$FOXGLOVE_BT_LAYOUT"
  echo "  FOXGLOVE_EXTENSION_DIR=$FOXGLOVE_EXTENSION_DIR"
}

is_true() {
  case "${1,,}" in
    1|true|yes|on)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

is_number() {
  [[ "${1:-}" =~ ^-?[0-9]+([.][0-9]+)?$ ]]
}

is_supported_nav_ui() {
  case "${NAV_UI,,}" in
    foxglove|rviz|none)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
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

robot_ip_hint() {
  local ip
  ip="$(hostname -I 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i !~ /^127\./) {print $i; exit}}')"
  if [[ -n "$ip" ]]; then
    echo "$ip"
  else
    echo "<robot-ip>"
  fi
}

is_rviz_running() {
  [[ -f "$RVIZ_PID_FILE" ]] || return 1
  local pid
  pid="$(cat "$RVIZ_PID_FILE" 2>/dev/null || true)"
  [[ -n "$pid" ]] || return 1
  kill -0 "$pid" 2>/dev/null
}

resolve_map_yaml() {
  local map_arg="${1:-}"
  local candidate
  local -a candidates=()

  if [[ -n "$map_arg" ]]; then
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
    echo "Try '$0 start <name>' for $MAP_DIR/<name>.yaml or pass a full .yaml path." >&2
    return 1
  fi

  candidate="$(
    find "$MAP_DIR" -maxdepth 1 -type f -name "*.yaml" \
      ! -name "*_isaac_localizer.yaml" \
      ! -name "waypoints.yaml" \
      -printf "%T@ %p\n" 2>/dev/null | sort -nr | awk 'NR==1 {print $2}'
  )"

  if [[ -z "$candidate" ]]; then
    echo "No saved maps found in $MAP_DIR." >&2
    echo "Create one first with: /ssd/ros2_ws/scripts/bluebot_bringup.sh save-map <name>" >&2
    return 1
  fi

  echo "$candidate"
}

list_maps() {
  local maps
  maps="$(
    find "$MAP_DIR" -maxdepth 1 -type f -name "*.yaml" \
      ! -name "*_isaac_localizer.yaml" \
      ! -name "waypoints.yaml" \
      -printf "%f\n" 2>/dev/null | sort
  )"

  if [[ -z "$maps" ]]; then
    echo "No saved maps found in $MAP_DIR."
    echo "Create one with: /ssd/ros2_ws/scripts/bluebot_bringup.sh save-map <name>"
    return 1
  fi

  echo "Saved maps in $MAP_DIR:"
  while IFS= read -r map_name; do
    [[ -z "$map_name" ]] && continue
    echo "  - ${map_name%.yaml}"
  done <<< "$maps"
}

publish_goal() {
  local x="${1:-}"
  local y="${2:-}"
  local yaw_deg="${3:-0}"
  local yaw_rad
  local qz
  local qw

  if [[ -z "$x" || -z "$y" ]]; then
    echo "Usage: $0 goal <x> <y> [yaw_deg]" >&2
    return 1
  fi

  if ! is_number "$x" || ! is_number "$y" || ! is_number "$yaw_deg"; then
    echo "Goal values must be numeric. Received: x='$x' y='$y' yaw_deg='$yaw_deg'" >&2
    return 1
  fi

  yaw_rad="$(awk -v d="$yaw_deg" 'BEGIN { pi = atan2(0, -1); printf "%.12f", d * pi / 180.0 }')"
  qz="$(awk -v y="$yaw_rad" 'BEGIN { printf "%.12f", sin(y / 2.0) }')"
  qw="$(awk -v y="$yaw_rad" 'BEGIN { printf "%.12f", cos(y / 2.0) }')"

  source_ros
  ros2 topic pub --once -w 0 /goal_pose geometry_msgs/msg/PoseStamped "{
header: {frame_id: map},
pose: {
  position: {x: $x, y: $y, z: 0.0},
  orientation: {x: 0.0, y: 0.0, z: $qz, w: $qw}
}
}"

  echo "Published goal to /goal_pose: x=$x y=$y yaw_deg=$yaw_deg"
  if [[ "${NAV_UI,,}" == "foxglove" ]]; then
    echo "Tip: in Foxglove 3D, use Publish -> Pose on /goal_pose for click-to-go."
  else
    echo "Tip: in RViz use 2D Goal Pose on /goal_pose."
  fi
}

show_foxglove_info() {
  echo "Foxglove websocket: ws://$(robot_ip_hint):${FOXGLOVE_BRIDGE_PORT}"
  if [[ -f "$FOXGLOVE_LAYOUT" ]]; then
    echo "Foxglove nav layout: $FOXGLOVE_LAYOUT"
  else
    echo "Foxglove nav layout not found: $FOXGLOVE_LAYOUT"
  fi
  if [[ -f "$FOXGLOVE_BT_LAYOUT" ]]; then
    echo "Foxglove BT layout: $FOXGLOVE_BT_LAYOUT"
  else
    echo "Foxglove BT layout not found: $FOXGLOVE_BT_LAYOUT"
  fi
  if [[ -d "$FOXGLOVE_EXTENSION_DIR" ]]; then
    echo "Foxglove Nav2 extension: $FOXGLOVE_EXTENSION_DIR"
  fi
  echo "Goal topic: /goal_pose (geometry_msgs/PoseStamped)"
  echo "Waypoint topic: $FOXGLOVE_WAYPOINT_TOPIC (geometry_msgs/PoseArray)"
  echo "Waypoint status: $FOXGLOVE_WAYPOINT_STATUS_TOPIC (std_msgs/String)"
}

send_waypoints() {
  echo "The 'waypoints' CLI command has been removed." >&2
  echo "Use the BlueBot Nav2 Controls panel in Foxglove to publish waypoint routes." >&2
  return 1
}

start_rviz() {
  if ! is_true "$AUTO_RVIZ"; then
    rm -f "$RVIZ_PID_FILE"
    echo "RViz auto-start disabled (AUTO_RVIZ=$AUTO_RVIZ)."
    return 0
  fi

  if is_rviz_running; then
    echo "Navigation RViz already running (PID: $(cat "$RVIZ_PID_FILE"))."
    echo "RViz log: $RVIZ_LOG_FILE"
    return 0
  fi

  if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    rm -f "$RVIZ_PID_FILE"
    echo "Skipping RViz launch: DISPLAY/WAYLAND_DISPLAY is not set."
    echo "Start RViz manually from a desktop session: /ssd/ros2_ws/rviz/start-rviz nav"
    return 0
  fi

  local -a launch_cmd=()
  if [[ -x "$RVIZ_START_SCRIPT" ]]; then
    launch_cmd=("$RVIZ_START_SCRIPT" nav)
  else
    if ! command -v rviz2 >/dev/null 2>&1; then
      echo "rviz2 is not available and RVIZ_START_SCRIPT was not found: $RVIZ_START_SCRIPT"
      return 1
    fi
    source_ros
    launch_cmd=(rviz2 -d "$RVIZ_CONFIG")
  fi

  if command -v setsid >/dev/null 2>&1; then
    setsid "${launch_cmd[@]}" >"$RVIZ_LOG_FILE" 2>&1 &
  else
    "${launch_cmd[@]}" >"$RVIZ_LOG_FILE" 2>&1 &
  fi
  echo "$!" > "$RVIZ_PID_FILE"
  sleep 1

  if ! is_rviz_running; then
    echo "RViz failed to stay running. Check log: $RVIZ_LOG_FILE"
    tail -n 40 "$RVIZ_LOG_FILE" 2>/dev/null || true
    rm -f "$RVIZ_PID_FILE"
    return 1
  fi

  echo "Navigation RViz started (PID: $(cat "$RVIZ_PID_FILE"))."
  echo "RViz log: $RVIZ_LOG_FILE"
}

stop_rviz() {
  if ! is_rviz_running; then
    rm -f "$RVIZ_PID_FILE"
    return 0
  fi

  local pid
  local pgid
  local end_time
  pid="$(cat "$RVIZ_PID_FILE")"
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

  rm -f "$RVIZ_PID_FILE"
  echo "Navigation RViz stopped."
}

start_nav() {
  local map_yaml

  if ! is_supported_nav_ui; then
    echo "Unsupported NAV_UI='$NAV_UI'. Use one of: foxglove, rviz, none." >&2
    return 1
  fi

  map_yaml="$(resolve_map_yaml "${1:-}")" || return 1
  "$BRINGUP_SCRIPT" start navigation "$map_yaml"

  case "${NAV_UI,,}" in
    rviz)
      start_rviz
      echo "Select goals in RViz with '2D Goal Pose' (topic: /goal_pose)."
      ;;
    foxglove)
      rm -f "$RVIZ_PID_FILE"
      echo "NAV_UI=foxglove: skipping RViz auto-start."
      show_foxglove_info
      echo "In Foxglove 3D panel, use Publish -> Pose to click goals on the map."
      ;;
    none)
      rm -f "$RVIZ_PID_FILE"
      echo "NAV_UI=none: UI launch disabled."
      echo "Send goals with '$0 goal <x> <y> [yaw_deg]'."
      ;;
  esac

  echo "Navigation stack started with map: $map_yaml"
}

stop_nav() {
  "$BRINGUP_SCRIPT" stop || true
  stop_rviz
}

status_nav() {
  "$BRINGUP_SCRIPT" status || true

  echo "Navigation UI mode: ${NAV_UI,,}"
  if is_rviz_running; then
    echo "Navigation RViz is running (PID: $(cat "$RVIZ_PID_FILE"))."
    echo "RViz log: $RVIZ_LOG_FILE"
  else
    echo "Navigation RViz is not running."
  fi

  if [[ "${NAV_UI,,}" == "foxglove" ]]; then
    show_foxglove_info
  fi
}

command="${1:-start}"
case "$command" in
  start)
    start_nav "${2:-}"
    ;;
  stop)
    stop_nav
    ;;
  restart)
    stop_nav
    start_nav "${2:-}"
    ;;
  status)
    status_nav
    ;;
  maps)
    list_maps
    ;;
  goal)
    publish_goal "${2:-}" "${3:-}" "${4:-0}"
    ;;
  waypoints)
    shift
    send_waypoints "$@"
    ;;
  foxglove|foxglove-info)
    show_foxglove_info
    ;;
  *)
    usage
    exit 1
    ;;
esac
