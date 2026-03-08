# bluebot.sh

`bluebot.sh` is the main bringup script for Bluebot.

Path: `/ssd/ros2_ws/scripts/bluebot.sh`

## Prerequisites

- ROS 2 Humble installed at `/opt/ros/humble`
- Workspace built at `/ssd/ros2_ws/install`
- Robot hardware available on expected ports (`/dev/arduino`, `/dev/lidar`, camera)

## Usage

```bash
/ssd/ros2_ws/scripts/bluebot.sh [start|start-map|start-map-explore|start-nav <map>|save-map [name]|capture-waypoint [name]|send-waypoints [name ...]|slam-health|stop|restart|restart-map|restart-map-explore|restart-nav <map>|status]
```

## Commands

- `start`: Start base robot stack (bridge, control, lidar, camera, vSLAM, Foxglove, teleop).
- `start-map`: Start base stack plus SLAM mapping job.
- `start-map-explore`: Start base stack plus SLAM + navigation + active explore and local nvblox obstacle avoidance.
- `start-nav <map>`: Start base stack plus Nav2 localization/navigation using a saved map.
- `save-map [name]`: Save current map to `/ssd/maps`.
- `capture-waypoint [name]`: Capture current `map -> base_link` pose into waypoint YAML.
- `send-waypoints [name ...]`: Publish all or selected waypoints as `PoseArray` to Foxglove waypoint topic.
- `slam-health`: Inspect latest SLAM launch/logs and classify `RUNNING`, `CLEAN EXIT`, or `CRASHED`, including TF/queue warnings.
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
export EXPLORE_LITE_ENABLED=true
export EXPLORE_LITE_NAMESPACE=
export EXPLORE_LITE_USE_SIM_TIME=false
```

Notes:
- `NVBLOX_ENABLED` launches an nvblox perception node alongside the existing stack using the existing RealSense streams.
- `EXPLORE_LITE_ENABLED` launches frontier exploration in `start-map` and `start-nav`.
- `NVBLOX_USE_SIM_TIME` and `EXPLORE_LITE_USE_SIM_TIME` default to the current `NAV2_USE_SIM_TIME` value.
- `start-map-explore` forces both NVBLOX and Explore Lite on for that mode.

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
# robot explores while building map and using nvblox local obstacle layer
/ssd/ros2_ws/scripts/bluebot.sh save-map office_a
```

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
- `NAV2_PARAMS_FILE` (default `/ssd/ros2_ws/src/serial_diff_drive_hw/config/nav2_navigation_params.yaml`)
- `NAV2_MAP_EXPLORE_PARAMS_FILE` (default `/ssd/ros2_ws/src/serial_diff_drive_hw/config/nav2_map_explore_params.yaml`)
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
