# bluebot.sh

`bluebot.sh` is the main bringup script for Bluebot.

Path: `/ssd/ros2_ws/scripts/bluebot.sh`

## Prerequisites

- ROS 2 Humble installed at `/opt/ros/humble`
- Workspace built at `/ssd/ros2_ws/install`
- Robot hardware available on expected ports (`/dev/arduino`, `/dev/lidar`, camera)

## Usage

```bash
/ssd/ros2_ws/scripts/bluebot.sh [start|start-map|start-map-explore|start-nav <map>|save-map [name]|capture-waypoint [name]|send-waypoints [name ...]|slam-health|nav-health|stop|restart|restart-map|restart-map-explore|restart-nav <map>|status]
```

## Commands

- `start`: Start base robot stack (bridge, control, lidar, camera, vSLAM, Foxglove, teleop).
- `start-map`: Start base stack plus SLAM mapping job.
- `start-map-explore`: Start lidar + odom stack plus SLAM + navigation + active explore (vSLAM and nvblox are disabled in this mode).
- `start-nav <map>`: Start base stack plus Nav2 localization/navigation using a saved map. Use this mode (with `NVBLOX_ENABLED=true`) for perception-based local obstacle avoidance.
- `save-map [name]`: Save current map to `/ssd/maps`.
- `capture-waypoint [name]`: Capture current `map -> base_link` pose into waypoint YAML.
- `send-waypoints [name ...]`: Publish all or selected waypoints as `PoseArray` to Foxglove waypoint topic.
- `slam-health`: Inspect latest SLAM launch/logs and classify `RUNNING`, `CLEAN EXIT`, or `CRASHED`, including TF/queue warnings.
- `nav-health`: Snapshot Nav2/explore health (required nodes, explore status, NavigateToPose status counts, cmd/odom samples, and recent rosout warning/error counters).
- `stop`: Graceful shutdown of stack and cleanup.
- `restart`: `stop` then `start`.
- `restart-map`: `stop` then `start-map`.
- `restart-map-explore`: `stop` then `start-map-explore`.
- `restart-nav <map>`: `stop` then `start-nav <map>`.
- `status`: Show whether tracked stack processes are running.

## Optional Modules (env toggles)

Set these variables before starting if you want additional stack components:

```bash
export NVBLOX_ENABLED=true
export NVBLOX_LAUNCH_PKG=serial_diff_drive_hw
export NVBLOX_LAUNCH_FILE=nvblox_bridge.launch.py
export BRIDGE_STALL_COMPENSATION_ENABLED=true
export BRIDGE_MIN_EFFECTIVE_LINEAR_MPS=0.14
export BRIDGE_MIN_EFFECTIVE_ANGULAR_RAD_S=0.0
export BRIDGE_ZERO_CMD_EPSILON=0.0001
export FOXGLOVE_BRIDGE_ENABLED=true
export FOXGLOVE_BRIDGE_PORT=8765
export FOXGLOVE_BRIDGE_ADDRESS=0.0.0.0
export FOXGLOVE_BRIDGE_CAPABILITIES='[clientPublish,services,connectionGraph,assets]'
export EXPLORE_LITE_ENABLED=true
export EXPLORE_LITE_NAMESPACE=
export EXPLORE_LITE_USE_SIM_TIME=false
```

Notes:
- `NVBLOX_ENABLED` launches an nvblox perception node alongside the existing stack using the existing RealSense streams.
- `BRIDGE_STALL_COMPENSATION_ENABLED` enforces minimum non-zero drive commands to help overcome drivetrain static friction/stall deadband.
- `BRIDGE_MIN_EFFECTIVE_LINEAR_MPS` and `BRIDGE_MIN_EFFECTIVE_ANGULAR_RAD_S` set those minimum effective non-zero command magnitudes. Keep angular at `0.0` unless rotation stall is proven.
- `BRIDGE_ZERO_CMD_EPSILON` defines what counts as a true zero command (to avoid creeping).
- `FOXGLOVE_BRIDGE_ENABLED` controls whether `foxglove_bridge` is started (default `true`).
- `FOXGLOVE_BRIDGE_PORT` and `FOXGLOVE_BRIDGE_ADDRESS` control websocket bind settings (defaults `8765` and `0.0.0.0`).
- `FOXGLOVE_BRIDGE_CAPABILITIES` controls bridge capabilities; default omits parameter browsing to avoid noisy parameter-service timeout logs under load.
- `EXPLORE_LITE_ENABLED` launches frontier exploration in `start-map` and `start-nav`.
- `NVBLOX_USE_SIM_TIME` and `EXPLORE_LITE_USE_SIM_TIME` default to the current `NAV2_USE_SIM_TIME` value.
- `start-map-explore` forces Explore Lite on and disables RealSense/vSLAM/nvblox to run on lidar + odom only.
- `start-map-explore` now waits `MAP_EXPLORE_LITE_START_DELAY_SEC` (default `20`) after Nav2 is ready before launching Explore Lite.
- `start-map-explore` waits up to `MAP_EXPLORE_TF_READY_TIMEOUT_SEC` (default `90`) for `map -> base_link` TF before launching Explore Lite.
- `start-map-explore` aborts Explore Lite startup if `map -> base_link` TF never appears (`MAP_EXPLORE_REQUIRE_TF_READY=true` by default).
- `start-map-explore` routes drive commands directly from Nav2 (`/cmd_vel`) to the serial bridge (no `/cmd_vel_safe` gate in this mode).
- `start-map-explore` uses tuned Nav2 map-explore parameters (more tolerant progress checker and lower controller/planner load targets) to reduce false `Failed to make progress` aborts on Jetson.

## Quick Workflows

### Mapping

```bash
cd /ssd
/ssd/ros2_ws/scripts/bluebot.sh start-map
# drive robot to build map
/ssd/ros2_ws/scripts/bluebot.sh save-map office_a
```

### Mapping + Explore

```bash
cd /ssd
/ssd/ros2_ws/scripts/bluebot.sh start-map-explore
# robot explores while building map using lidar + odom only
/ssd/ros2_ws/scripts/bluebot.sh save-map office_a
```

Foxglove helper layout for this mode:

- Import: `/ssd/ros2_ws/foxglove/start_map_explore_layout.json`
- Connect: `ws://<robot-ip>:8765` (or your overridden `FOXGLOVE_BRIDGE_PORT`)

### Navigation with Saved Map

```bash
cd /ssd
/ssd/ros2_ws/scripts/bluebot.sh start-nav office_a
```

After `start-nav`, Bluebot now auto-triggers Isaac occupancy-grid localization and publishes an initial pose on `/initialpose`. Then send goals on `/goal_pose` or waypoint arrays on `/foxglove/waypoints`.

## Waypoints

Capture named waypoints while mapping or navigating:

```bash
/ssd/ros2_ws/scripts/bluebot.sh capture-waypoint dock
/ssd/ros2_ws/scripts/bluebot.sh capture-waypoint hallway
```

Waypoint file default:

`/ssd/maps/waypoints.yaml`

Send all stored waypoints:

```bash
/ssd/ros2_ws/scripts/bluebot.sh send-waypoints
```

Send only selected waypoints:

```bash
/ssd/ros2_ws/scripts/bluebot.sh send-waypoints dock hallway
```

## Key Environment Variables

- `VSLAM_MODE` (default `direct`, options `direct|nitros`)
- `MAP_DIR` (default `/ssd/maps`)
- `MAP_TOPIC` (default `/map`)
- `BRIDGE_STALL_COMPENSATION_ENABLED` (default `true`)
- `BRIDGE_MIN_EFFECTIVE_LINEAR_MPS` (default `0.14`)
- `BRIDGE_MIN_EFFECTIVE_ANGULAR_RAD_S` (default `0.0`)
- `BRIDGE_ZERO_CMD_EPSILON` (default `0.0001`)
- `NAV2_PARAMS_FILE` (default `/ssd/ros2_ws/src/serial_diff_drive_hw/config/nav2_navigation_params.yaml`)
- `NAV2_MAP_EXPLORE_PARAMS_FILE` (default `/ssd/ros2_ws/src/serial_diff_drive_hw/config/nav2_map_explore_params.yaml`)
- `FOXGLOVE_BRIDGE_ENABLED` (default `true`)
- `FOXGLOVE_BRIDGE_PORT` (default `8765`)
- `FOXGLOVE_BRIDGE_ADDRESS` (default `0.0.0.0`)
- `FOXGLOVE_BRIDGE_CAPABILITIES` (default `[clientPublish,services,connectionGraph,assets]`)
- `FOXGLOVE_WAYPOINT_BRIDGE` (default `true`)
- `FOXGLOVE_WAYPOINT_TOPIC` (default `/foxglove/waypoints`)
- `FOXGLOVE_WAYPOINT_FRAME` (default `map`)
- `FOXGLOVE_WAYPOINT_ACTION_NAME` (default `/navigate_through_poses`)
- `WAYPOINTS_FILE` (default `$MAP_DIR/waypoints.yaml`)
- `NVBLOX_ENABLED` (default `false`)
- `NVBLOX_LAUNCH_PKG` (default `serial_diff_drive_hw`)
- `NVBLOX_LAUNCH_FILE` (default `nvblox_bridge.launch.py`)
- `NVBLOX_USE_SIM_TIME` (default `$NAV2_USE_SIM_TIME`)
- `EXPLORE_LITE_ENABLED` (default `false`)
- `EXPLORE_LITE_NAMESPACE` (default empty)
- `EXPLORE_LITE_USE_SIM_TIME` (default `$NAV2_USE_SIM_TIME`)
- `MAP_EXPLORE_LITE_START_DELAY_SEC` (default `20`)
- `MAP_EXPLORE_TF_READY_TIMEOUT_SEC` (default `90`)
- `MAP_EXPLORE_REQUIRE_TF_READY` (default `true`)
- `ISAAC_GRID_LOCALIZATION_ENABLED` (default `true`)
- `ISAAC_GRID_LOCALIZER_LAUNCH_PKG` (default `isaac_nav2_pose_bridge`)
- `ISAAC_GRID_LOCALIZER_LAUNCH_FILE` (default `isaac_grid_localization_to_nav2.launch.py`)
- `ISAAC_GRID_LOCALIZER_SCAN_TOPIC` (default `/scan`)
- `ISAAC_GRID_LOCALIZER_FLATSCAN_TOPIC` (default `/flatscan`)
- `ISAAC_GRID_LOCALIZER_RESULT_TOPIC` (default `/localization_result`)
- `ISAAC_GRID_LOCALIZER_TRIGGER_SERVICE` (default `/trigger_grid_search_localization`)
- `ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_ENABLED` (default `true`)
- `ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_WAIT_SEC` (default `6.0`)
- `ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_X` (default `0.0`)
- `ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_Y` (default `0.0`)
- `ISAAC_GRID_LOCALIZER_FALLBACK_INITIAL_POSE_YAW` (default `0.0`, radians)

Example override:

```bash
FOXGLOVE_WAYPOINT_TOPIC=/my_waypoints /ssd/ros2_ws/scripts/bluebot.sh restart-nav office_a
```

## Troubleshooting

- `AMCL cannot publish a pose...`: ensure `ISAAC_GRID_LOCALIZATION_ENABLED=true` and that `/trigger_grid_search_localization` service is available; otherwise publish initial pose on `/initialpose`.
- `Map file not found`: pass map name in `/ssd/maps/<name>.yaml` or full `.yaml` path.
- `Waypoint file not found`: run `capture-waypoint` first.
- `Stack appears to already be running`: run `/ssd/ros2_ws/scripts/bluebot.sh stop` before starting a different mode.
- `controller_server Failed to make progress` repeats: run `/ssd/ros2_ws/scripts/bluebot.sh nav-health` and check `progress_fail`, `collision_ahead`, and `control_missed` counters.
