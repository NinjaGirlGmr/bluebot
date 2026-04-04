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
  - `apriltag_landmark_tf_publisher`
  - `apriltag_map_localization`

## Directory Layout

- `launch/`: entrypoints (`sensors`, `mapping`, `nav2`, `navigation`, `apriltag_realsense`, `localization`, `health`, `observability`)
- `config/`: dedicated YAML config per stack/component
- `robot_bringup/`: Python nodes (`health_monitor`, `apriltag_nav_behavior_tree`, `apriltag_map_recorder`, `apriltag_landmark_tf_publisher`, `apriltag_map_localization`)

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
| `nav2.launch.py` | Nav2 + dedicated smoother params + drive stack + local/global EKF + straight-line compensator + UDP teleop + health monitor |
| `navigation.launch.py` | Top-level nav mode: sensors + `nav2.launch.py` + optional AprilTag camera + landmark TF publisher + AprilTag map-localization pose publisher + AprilTag behavior tree |
| `localization.launch.py` | Standalone `slam_toolbox` localization + health monitor |
| `health.launch.py` | Standalone health monitor |
| `observability.launch.py` | Foxglove bridge + optional health monitor |

All launch files default to:

```bash
use_sim_time:=false
```

Hardware note:
- Keep `use_sim_time:=false` unless you actively publish `/clock`.
- If `use_sim_time:=true` and `/clock` has no publishers, timer-driven nodes can appear alive but stop publishing data (for example `/odom`).

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

Foxglove layouts for `robot_bringup`:

- Mapping template:
  - `/ssd/ros2_ws/foxglove/robot_bringup_mapping_layout.json`
  - Tabs:
    - `Mapping + AprilTag` (map creation, scan, odom, AprilTag detections, pose updates)
    - `Health + Logs` (diagnostics summary/detail, drive metrics, `/rosout`)

- Navigation template (single-pane):
  - `/ssd/ros2_ws/foxglove/robot_bringup_navigation_layout.json`
  - One 3D pane with:
    - `/map`, `/scan`, `/odom`, `/odom_raw`, `/tag_detections`
    - pose publish tools:
      - goal pose: `/goal_pose`
      - initial pose: `/initialpose`

## Common Workflows

### Mapping (with optional AprilTag landmark recording)

```bash
source /ssd/ros2_ws/install/setup.bash
ros2 launch robot_bringup mapping.launch.py \
  apriltag_realsense_enabled:=true \
  apriltag_map_recorder_enabled:=true \
  apriltag_map_output_yaml:=/tmp/apriltag_map_landmarks.yaml
```

AprilTag recorder requirements:
- `apriltag_map_recorder_enabled:=true`
- a valid TF chain from `map` to camera optical frame (for example `camera_color_optical_frame`)
- sufficient observations (`min_observations` in `config/apriltag_map_recorder.yaml`, default `5`)

Recommended mapping capture pattern (best landmark quality):
- stop-and-scan at each tag (robot stationary before sampling)
- keep tags inside configured range gates (`0.10m` to `2.50m` defaults)
- capture multiple angles per tag
- let the recorder filters work:
  - `min_consecutive_detections` (default `12`)
  - stationary gate from odom (`require_stationary: true`)
  - covariance gate (`use_covariance_filter: true`)
  - sample spacing (`min_sample_interval_sec: 0.10`)

`apriltag_realsense.launch.py` now publishes a static TF from `base_link` to `camera_link` by default:
- `publish_base_to_camera_tf:=true`
- default mount transform: `x=0.097`, `y=0.0`, `z=0.155`, `roll/pitch/yaw=0.0`

Override camera mount if needed:

```bash
ros2 launch robot_bringup apriltag_realsense.launch.py \
  camera_tf_x:=0.097 camera_tf_y:=0.0 camera_tf_z:=0.155 \
  camera_tf_roll:=0.0 camera_tf_pitch:=0.0 camera_tf_yaw:=0.0
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

Recommended AprilTag-enabled navigation run:

```bash
source /ssd/ros2_ws/install/setup.bash
ros2 launch robot_bringup navigation.launch.py \
  map:=/ssd/maps/office_a.yaml \
  apriltag_realsense_enabled:=true
```

Runtime frame/estimation pipeline in nav mode:
- static landmark anchors are loaded from `<map>.apriltags.yaml` (or `apriltag_landmarks_file`) and published as `map -> apriltag_landmark/*`
- detector provides `camera -> tag_i`
- `apriltag_map_localization` computes robot pose in `map` and publishes `/apriltag/map_pose`
- `robot_localization_global_filter` fuses:
  - `odom0`: local fused odom (`/odom`)
  - `pose0`: AprilTag global pose (`/apriltag/map_pose`)
  - `pose1`: fallback/global initial pose (`/initialpose`)
- global EKF publishes `map -> odom`; Nav2 consumes TF automatically

Important defaults for this pipeline:
- `state_estimation_global_enabled:=true`
- `apriltag_landmark_tf_enabled:=true`
- `apriltag_map_localization_enabled:=true`
- AMCL `tf_broadcast: false` in `config/nav2.yaml` (single owner for `map -> odom`)

Quick validation checks:

```bash
ros2 topic echo /apriltag/map_pose --once
ros2 run tf2_ros tf2_echo map odom
ros2 node list | rg "apriltag_landmark_tf_publisher|apriltag_map_localization|robot_localization_global_filter|smoother_server"
```

Enable AprilTag camera pipeline in nav mode:

```bash
ros2 launch robot_bringup navigation.launch.py \
  map:=/ssd/maps/office_a.yaml \
  apriltag_realsense_enabled:=true
```

Isaac occupancy-grid localization (auto initial pose to Nav2) is enabled by default in nav mode:

```bash
ros2 launch robot_bringup navigation.launch.py \
  map:=/ssd/maps/office_a.yaml \
  grid_localization_enabled:=true
```

Key args:
- `grid_localization_enabled` (default `true`)
- `grid_localization_output_topic` (default `/initialpose`)
- `grid_localization_fallback_enabled` (default `true`)
- `grid_localization_fallback_wait_sec` (default `6.0`)
- `occupancy_grid_localizer_enabled` (default `true`)

Grid-localizer map image note:
- `isaac_ros_occupancy_grid_localizer` supports `.png/.jpg/.jpeg` map images.
- If your Nav2 map YAML references `.pgm`, `isaac_grid_localization_to_nav2.launch.py` now auto-creates a localizer copy in `/tmp/isaac_grid_localizer_maps/` with a converted `.png` image.

Disable behavior tree temporarily:

```bash
ros2 launch robot_bringup navigation.launch.py \
  map:=/ssd/maps/office_a.yaml \
  apriltag_behavior_enabled:=false
```

## Navigation Troubleshooting

### Map missing in Foxglove (`/map` not visible)

Check:
- `map_server` reached Active lifecycle state.
- `lifecycle_manager_localization` reports managed nodes active.

Useful command:

```bash
ros2 topic echo /map --once
```

If no map message appears:
- verify `map:=/absolute/path/to/map.yaml` is passed to `navigation.launch.py`
- verify Nav2 params are loaded from `config/nav2.yaml` (not overridden by unrelated params)

### No initial pose seen (`/initialpose`)

Expected behavior in this stack:
- `isaac_to_nav2_pose` publishes `/initialpose` from:
  - occupancy-grid localizer output (`/localization_result`) when available, or
  - fallback initial pose after `grid_localization_fallback_wait_sec` (default `6.0`)

Checks:

```bash
ros2 topic echo /initialpose --once
ros2 topic echo /localization_result --once
```

Notes:
- `apriltag_nav_behavior_tree` may log QoS incompatibility warnings on `/initialpose`; this does not block AMCL initialization.
- Occupancy localizer only emits on trigger. Trigger one global-search pass with:

```bash
ros2 service call /trigger_grid_search_localization std_srvs/srv/Empty {}
```

- If occupancy localizer is unstable, run with fallback only:

```bash
ros2 launch robot_bringup navigation.launch.py \
  map:=/ssd/maps/office_a.yaml \
  occupancy_grid_localizer_enabled:=false
```

## Drift Management

`robot_bringup` now includes two drift-management layers used by `mapping`, `nav2`, and `navigation`:
- Odom drift mitigation in EKF: wheel odom + IMU yaw-rate focused fusion
- Yaw drift mitigation while driving straight: `straight_line_compensator`
- Low-command stall compensation in serial bridge (helps prevent turn buzz/stall)

Drive command path:
- `/cmd_vel` -> `straight_line_compensator` -> `/cmd_vel_compensated` -> `serial_diff_drive_bridge`

Main tuning files:
- `config/state_estimation_drift.yaml`
- `config/straight_line_compensator.yaml`
- `config/nav2_serial_diff_drive_bridge.yaml`
- `config/mapping_serial_diff_drive_bridge.yaml`

Current straight-line defaults are tuned for startup stability and right-pull correction:
- `angular_deadband: 0.08`
- `kp: 2.2`
- `ki: 0.04`
- `kd: 0.20`
- `max_angular_correction: 0.65`
- `startup_holdoff_sec: 1.5`
- `min_odom_samples_before_comp: 10`

Current serial bridge stall-comp defaults:
- `enable_stall_compensation: true`
- `min_effective_linear_mps: 0.14`
- `min_effective_angular_rad_s: 0.25`
- `min_effective_turn_wheel_mps: 0.10` (enforces turning wheel breakaway speed floor)

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
  - `config/nav2_smoother.yaml`
  - `config/nav2_serial_diff_drive_bridge.yaml`
  - `config/nav2_udp_cmd_vel_bridge.yaml`
  - `config/state_estimation_drift.yaml`
  - `config/state_estimation_map_fusion.yaml`
  - `config/straight_line_compensator.yaml`
  - `config/nav2_health_monitor.yaml`
  - `config/navigation_health_monitor.yaml`
- AprilTag:
  - `config/apriltag.yaml`
  - `config/realsense.yaml`
  - `config/apriltag_nav_behavior.yaml`
  - `config/apriltag_nav_rules.yaml`
  - `config/apriltag_map_recorder.yaml`
  - `config/apriltag_landmark_tf.yaml`
  - `config/apriltag_map_localization.yaml`
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
- Check simulation time wiring:
  - `ros2 topic info -v /clock` (publisher count should be `> 0` when `use_sim_time:=true`)
  - `ros2 param get /serial_diff_drive_bridge use_sim_time`
  - `ros2 param get /robot_localization_filter use_sim_time`
- If `/odom` is empty but nodes exist:
  - verify `/clock` publisher status
  - verify `/odom_raw` and `/odom` rates: `ros2 topic hz /odom_raw` and `ros2 topic hz /odom`
- If you are not seeing `/initialpose` in navigation:
  - confirm `grid_localization_enabled:=true`
  - check nodes: `ros2 node list | rg "occupancy_grid_localizer|isaac_to_nav2_pose"`
  - check topic once: `ros2 topic echo /initialpose --once`
  - trigger occupancy-grid localization once: `ros2 service call /trigger_grid_search_localization std_srvs/srv/Empty {}`
  - verify map path is valid and passed to launch with `map:=/ssd/maps/<map>.yaml`
  - `isaac_nav2_pose_bridge` now auto-converts `.pgm` map images to a temporary png-backed yaml under `/tmp/isaac_grid_localizer_maps/` for `isaac_ros_occupancy_grid_localizer`
  - to force fallback-only initial pose (skip occupancy localizer), run with `occupancy_grid_localizer_enabled:=false`
- If AprilTag map save writes `tags: []`:
  - ensure mapping ran with `apriltag_realsense_enabled:=true` and `apriltag_map_recorder_enabled:=true`
  - check recorder logs for TF errors like `Failed transform camera_color_optical_frame->map`
  - verify TF chain: `ros2 run tf2_ros tf2_echo map camera_color_optical_frame`
  - lower `min_observations` in `config/apriltag_map_recorder.yaml` for sparse passes
- If navigation launch fails with `name 'true' is not defined`:
  - this comes from Nav2 `PythonExpression(['not ', use_composition])` when `use_composition` is lowercase `true/false`
  - `robot_bringup/launch/nav2.launch.py` now normalizes booleans before including Nav2 bringup
  - rebuild and relaunch:
    - `colcon build --packages-select robot_bringup`
    - `source /ssd/ros2_ws/install/setup.bash`
    - `ros2 launch robot_bringup navigation.launch.py map:=/ssd/maps/<map>.yaml apriltag_realsense_enabled:=true`
- If you see `file 'X.launch.py' was not found`, verify spelling and `setup.py` includes `launch/*.launch.py` in `data_files`.
- `PkgResourcesDeprecationWarning` during `colcon build` is non-fatal in this workspace.
