# robot_bringup

ROS 2 bringup package for Bluebot runtime stacks.

This package provides launch/config for:
- Sensors (LiDAR + IMU + static sensor TFs)
- Mapping (SLAM toolbox + drivetrain + EKF + UDP teleop)
- Navigation (Nav2 + drivetrain + EKF + UDP teleop)
- AprilTag detection (Realsense + Isaac ROS AprilTag)
- AprilTag-driven navigation behavior (rules, docking, relocalization)
- Health/diagnostics publishing to `/diagnostics` and transition logging to `/rosout`
- Foxglove observability

## Package Type

- Build type: `ament_python`
- Console executables:
  - `health_monitor`
  - `apriltag_nav_behavior_tree`
  - `apriltag_map_recorder`

## Directory Layout

- `launch/`: entrypoints (`sensors`, `mapping`, `nav2`, `navigation`, `apriltag_realsense`, `localization`, `health`, `observability`)
- `config/`: dedicated YAML config per stack/component
- `robot_bringup/`: Python nodes (`health_monitor`, `apriltag_nav_behavior_tree`, `apriltag_map_recorder`)

## Build

```bash
cd /ssd/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_bringup --symlink-install
source /ssd/ros2_ws/install/setup.bash
```

## Launch Files

| Launch file | Purpose |
| --- | --- |
| `sensors.launch.py` | LiDAR, IMU serial bridge, IMU bridge, sensor static TFs, health monitor |
| `apriltag_realsense.launch.py` | Realsense + Isaac rectification + AprilTag detection + health monitor |
| `mapping.launch.py` | Sensors + drive stack + EKF (drift-tuned) + straight-line compensator + SLAM + UDP teleop + optional AprilTag recorder + health monitor |
| `nav2.launch.py` | Nav2 + drive stack + EKF (drift-tuned) + straight-line compensator + UDP teleop + health monitor |
| `navigation.launch.py` | Top-level nav mode: sensors + `nav2.launch.py` + optional AprilTag camera + AprilTag behavior tree |
| `localization.launch.py` | Standalone `slam_toolbox` localization + health monitor |
| `health.launch.py` | Standalone health monitor |
| `observability.launch.py` | Foxglove bridge + optional health monitor |

All launch files default to:

```bash
use_sim_time:=true
```

Inspect launch arguments:

```bash
ros2 launch robot_bringup navigation.launch.py --show-args
```

## Consolidated Runtime Script

Use `/ssd/ros2_ws/scripts/bluebot_bringup.sh` for mode management:

```bash
/ssd/ros2_ws/scripts/bluebot_bringup.sh list-modes
/ssd/ros2_ws/scripts/bluebot_bringup.sh start mapping
/ssd/ros2_ws/scripts/bluebot_bringup.sh start navigation office_a
/ssd/ros2_ws/scripts/bluebot_bringup.sh status
/ssd/ros2_ws/scripts/bluebot_bringup.sh stop
```

Available modes:
- `sensors`
- `apriltag`
- `mapping`
- `nav2 <map>`
- `navigation <map>`
- `localization`
- `health`
- `observability`

## Foxglove Templates

New tabbed Foxglove layouts for `robot_bringup`:

- Mapping template:
  - `/ssd/ros2_ws/foxglove/robot_bringup_mapping_layout.json`
  - Tabs:
    - `Mapping + AprilTag` (map creation, scan, odom, AprilTag detections, pose updates)
    - `Health + Logs` (diagnostics summary/detail, drive metrics, `/rosout`)

- Navigation template:
  - `/ssd/ros2_ws/foxglove/robot_bringup_navigation_layout.json`
  - Tabs:
    - `Navigation + Landmarks` (map/plans, AMCL, AprilTag detections and landmark topic if published)
    - `Behavior Tree` (behavior-focused logs, Nav2 action status, motion response metrics)
    - `Health + Logs` (diagnostics summary/detail, drive metrics, `/rosout`)

## Common Workflows

### Mapping (with optional AprilTag landmark recording)

```bash
source /ssd/ros2_ws/install/setup.bash
ros2 launch robot_bringup mapping.launch.py \
  apriltag_realsense_enabled:=true \
  apriltag_map_recorder_enabled:=true \
  apriltag_map_output_yaml:=/tmp/apriltag_map_landmarks.yaml
```

Save map and copy landmark snapshot beside it:

```bash
/ssd/ros2_ws/scripts/bluebot_bringup.sh save-map office_a
```

This writes:
- map: `/ssd/maps/office_a.yaml` (+ `.pgm`)
- landmarks: `/ssd/maps/office_a.apriltags.yaml` (copied from `APRILTAG_LANDMARKS_FILE`, default `/tmp/apriltag_map_landmarks.yaml`)

### Navigation

```bash
source /ssd/ros2_ws/install/setup.bash
ros2 launch robot_bringup navigation.launch.py map:=/ssd/maps/office_a.yaml
```

Enable AprilTag camera pipeline in nav mode:

```bash
ros2 launch robot_bringup navigation.launch.py \
  map:=/ssd/maps/office_a.yaml \
  apriltag_realsense_enabled:=true
```

Disable behavior tree temporarily:

```bash
ros2 launch robot_bringup navigation.launch.py \
  map:=/ssd/maps/office_a.yaml \
  apriltag_behavior_enabled:=false
```

## Drift Management

`robot_bringup` now includes two drift-management layers used by `mapping`, `nav2`, and `navigation`:
- Odom drift mitigation in EKF: wheel odom + IMU yaw-rate focused fusion
- Yaw drift mitigation while driving straight: `straight_line_compensator`

Drive command path:
- `/cmd_vel` -> `straight_line_compensator` -> `/cmd_vel_compensated` -> `serial_diff_drive_bridge`

Main tuning files:
- `config/state_estimation_drift.yaml`
- `config/straight_line_compensator.yaml`

## AprilTag Behavior Tree

`navigation.launch.py` starts `apriltag_nav_behavior_tree` by default.

Important launch args:
- `apriltag_behavior_enabled` (default `true`)
- `apriltag_behavior_params_file` (default `config/apriltag_nav_behavior.yaml`)
- `apriltag_behavior_rules_file` (default `config/apriltag_nav_rules.yaml`)
- `apriltag_landmarks_file` (default empty)
- `map` (required for normal navigation)
- `docking_params_file` (default `config/docking_server.yaml`)

### How `rules_file`, `landmarks_file`, and `map_yaml` are passed

In `navigation.launch.py`, these are injected into the behavior node parameters:
- `rules_file` <- launch arg `apriltag_behavior_rules_file`
- `landmarks_file` <- launch arg `apriltag_landmarks_file`
- `map_yaml` <- launch arg `map`

The empty values in `config/apriltag_nav_behavior.yaml` are intentional placeholders.  
At runtime, launch arguments override them.

If `landmarks_file` is left empty and `auto_landmarks_from_map: true`, the node auto-resolves:
- map `/path/to/my_map.yaml` -> landmarks `/path/to/my_map.apriltags.yaml`

### Supported rule actions

| Action | Description | Common fields |
| --- | --- | --- |
| `log` | Log message when tag is seen | `message`, `cooldown_sec`, `priority` |
| `stop` | Publish zero `cmd_vel` for duration | `duration_sec`, `cooldown_sec`, `priority` |
| `cmd_vel` | Publish custom velocity for duration | `linear_x`, `angular_z`, `duration_sec` |
| `cancel_navigation` | Cancel active Nav2 goal | `cooldown_sec`, `priority` |
| `navigate_to_pose` | Send Nav2 goal | `frame_id`, `x`, `y`, `yaw_deg`, `cancel_active_goal` |
| `set_initial_pose` | Publish localization pose correction | `frame_id`, `x`, `y`, `yaw_deg`, `covariance_xy`, `covariance_yaw`, `use_landmark_pose` |
| `dock_then_sleep` | Reverse dock, optional Nav2 pause, LiDAR motor stop/start during sleep, resume | `linear_x`, `angular_z`, `dock_target_range_m`, `dock_timeout_sec`, `sleep_duration_sec`, `pause_nav2_during_sleep` |

General rule fields:
- `name`
- `tag_id`
- `tag_family` (optional)
- `action`
- `priority`
- `cooldown_sec`
- `once`
- `min_range_m` / `max_range_m`

Current rules file already includes:
- docking on tags `25` and `26`
- relocalization on tags `6`, `9`, `11`, `12` via `set_initial_pose` + `use_landmark_pose: true`

### Docking tuning

`config/docking_server.yaml` controls defaults used by `dock_then_sleep`, including:
- `docking_default_linear_x` (negative = reverse docking)
- `docking_default_target_range_m`
- `docking_default_timeout_sec`
- `docking_sleep_duration_sec`
- `docking_pause_nav2`
- `docking_lidar_power_control_enabled`
- `lidar_stop_motor_service`
- `lidar_start_motor_service`
- `lifecycle_manager_service`

## Diagnostics and Health

`health_monitor` publishes:
- `diagnostic_msgs/DiagnosticArray` on `/diagnostics`
- state transition logs on `/rosout`

Health profile configs:
- `config/health_monitor.yaml`
- `config/sensors_health_monitor.yaml`
- `config/apriltag_health_monitor.yaml`
- `config/localization_health_monitor.yaml`
- `config/mapping_health_monitor.yaml`
- `config/nav2_health_monitor.yaml`
- `config/navigation_health_monitor.yaml`
- `config/observability_health_monitor.yaml`

## Key Config Files

- Sensors:
  - `config/lidar.yaml`
  - `config/imu_serial.yaml`
  - `config/imu_bridge.yaml`
  - `config/sensor_transforms.yaml`
- Mapping:
  - `config/mapping_slam_toolbox.yaml`
  - `config/mapping_serial_diff_drive_bridge.yaml`
  - `config/mapping_udp_cmd_vel_bridge.yaml`
  - `config/state_estimation_drift.yaml`
  - `config/straight_line_compensator.yaml`
  - `config/mapping_health_monitor.yaml`
- Navigation:
  - `config/nav2.yaml`
  - `config/nav2_serial_diff_drive_bridge.yaml`
  - `config/nav2_udp_cmd_vel_bridge.yaml`
  - `config/state_estimation_drift.yaml`
  - `config/straight_line_compensator.yaml`
  - `config/nav2_health_monitor.yaml`
  - `config/navigation_health_monitor.yaml`
- AprilTag:
  - `config/apriltag.yaml`
  - `config/realsense.yaml`
  - `config/apriltag_nav_behavior.yaml`
  - `config/apriltag_nav_rules.yaml`
  - `config/apriltag_map_recorder.yaml`
  - `config/docking_server.yaml`
  - `config/apriltag_health_monitor.yaml`
- Observability:
  - `config/foxglove_bridge.yaml`

## Troubleshooting

- Show launch args:
  - `ros2 launch robot_bringup <file>.launch.py --show-args`
- Confirm install prefix:
  - `ros2 pkg prefix robot_bringup`
- Check topics:
  - `ros2 topic list`
- Check diagnostics:
  - `ros2 topic echo /diagnostics`
- If you see `file 'X.launch.py' was not found`, verify spelling and `setup.py` includes `launch/*.launch.py` in `data_files`.
- `PkgResourcesDeprecationWarning` during `colcon build` is non-fatal in this workspace.
