# Project Parameter Baselines (Current)

This document tracks project-specific parameter baselines that are currently active in this workspace.

## Primary Runtime: `robot_bringup` (`bluebot_bringup.sh`)

### Serial Bridge Profile (`mapping` and `navigation`)

Sources:
- `src/robot_bringup/config/mapping_serial_diff_drive_bridge.yaml`
- `src/robot_bringup/config/nav2_serial_diff_drive_bridge.yaml`

| Parameter | Current Value | What It Does |
|---|---:|---|
| `cmd_vel_topic` | `/cmd_vel_compensated` | Uses compensated velocity output (straight-line compensator) as the bridge input. |
| `odom_topic` | `/odom_raw` | Publishes raw wheel odometry topic used by local EKF. |
| `publish_tf` | `false` | Prevents duplicate odom TF from bridge; TF authority is handled by EKF stack. |
| `enable_stall_compensation` | `true` | Enforces minimum effective commands to overcome drivetrain deadband. |
| `min_effective_linear_mps` | `0.14` | Minimum non-zero linear command magnitude when stall compensation is active. |
| `min_effective_angular_rad_s` | `0.25` | Minimum non-zero angular command magnitude when stall compensation is active. |
| `min_effective_turn_wheel_mps` | `0.10` | Per-wheel floor to ensure in-place turns break static friction. |
| `zero_cmd_epsilon` | `0.0001` | Values at/below threshold are treated as exact zero before clamping. |

### Straight-Line Compensator (`robot_bringup` node)

Source:
- `src/robot_bringup/config/straight_line_compensator.yaml`

| Parameter | Current Value | What It Does |
|---|---:|---|
| `input_cmd_topic` | `/cmd_vel` | Raw command input topic. |
| `output_cmd_topic` | `/cmd_vel_compensated` | Compensated command output sent to serial bridge profile. |
| `odom_topic` | `/odom_raw` | Uses raw odometry for heading-hold correction. |
| `linear_speed_min` | `0.03` | Minimum linear speed required before straight-line compensation engages. |
| `angular_deadband` | `0.08` | Commands within deadband are treated as straight-driving intent. |
| `kp` / `ki` / `kd` | `2.2 / 0.04 / 0.20` | PID terms for heading correction. |
| `max_angular_correction` | `0.65` | Hard limit on correction magnitude. |
| `startup_holdoff_sec` | `1.5` | Wait period before compensation begins after startup. |
| `min_odom_samples_before_comp` | `10` | Requires odom sample history before enabling correction. |

### Local State Estimation EKF

Source:
- `src/robot_bringup/config/state_estimation_drift.yaml`

| Parameter | Current Value | What It Does |
|---|---:|---|
| `frequency` | `30.0` | Local EKF update rate. |
| `sensor_timeout` | `0.2` | Measurement timeout for local fusion. |
| `world_frame` | `odom` | Local EKF publishes in odom world frame. |
| `publish_tf` | `true` | Publishes `odom -> base_link` TF. |
| `odom0` | `/odom_raw` | Primary wheel-odometry source. |
| `imu0` | `/imu/orientation` | Wired but disabled (`imu0_config` all false). |
| `imu1` | `/imu/data_raw` | Yaw-rate-only fusion path for drift mitigation. |

### Global Map Fusion EKF

Source:
- `src/robot_bringup/config/state_estimation_map_fusion.yaml`

| Parameter | Current Value | What It Does |
|---|---:|---|
| `frequency` | `20.0` | Global EKF update rate. |
| `sensor_timeout` | `0.4` | Measurement timeout for global fusion. |
| `world_frame` | `map` | Global EKF solves in map frame. |
| `publish_tf` | `true` | Publishes authoritative `map -> odom` TF. |
| `odom0` | `/odom` | Local EKF fused odometry prior. |
| `pose0` | `/apriltag/map_pose` | Absolute map pose from AprilTag localization node. |
| `pose1` | `/initialpose` | Global fallback source (grid localizer / RViz / behavior actions). |

### Nav2 + Smoother Profile

Sources:
- `src/robot_bringup/config/nav2.yaml`
- `src/robot_bringup/config/nav2_smoother.yaml`

| Parameter | Current Value | What It Does |
|---|---:|---|
| `amcl.tf_broadcast` | `false` | Disables AMCL TF ownership so global EKF owns `map -> odom`. |
| `controller_server.controller_frequency` | `10.0` | Main local-control loop frequency. |
| `controller_server.progress_checker.required_movement_radius` | `0.15` | Minimum displacement considered progress. |
| `controller_server.progress_checker.movement_time_allowance` | `20.0` | Allowed time window to make progress before failure. |
| `controller_server.FollowPath.max_vel_x` | `0.26` | Max forward speed. |
| `controller_server.FollowPath.max_speed_xy` | `0.26` | Max planar translational speed. |
| `controller_server.FollowPath.trans_stopped_velocity` | `0.25` | Velocity threshold treated as stopped by controller logic. |
| `smoother_server.smoother_plugins` | `['simple_smoother']` | Enables Nav2 path smoothing server plugin. |
| `smoother_server.simple_smoother.max_its` | `1000` | Iterations for smoothing convergence. |

### AprilTag Mapping + Localization

Sources:
- `src/robot_bringup/config/apriltag_map_recorder.yaml`
- `src/robot_bringup/config/apriltag_map_localization.yaml`
- `src/robot_bringup/config/apriltag_landmark_tf.yaml`
- `src/robot_bringup/config/apriltag_nav_behavior.yaml`
- `src/robot_bringup/config/docking_server.yaml`

| Parameter | Current Value | What It Does |
|---|---:|---|
| `apriltag_map_recorder.min_observations` | `5` | Requires multi-sample support before writing a tag landmark. |
| `apriltag_map_recorder.min_consecutive_detections` | `12` | Stabilizes capture by requiring sustained detections. |
| `apriltag_map_recorder.require_stationary` | `true` | Rejects samples while robot motion exceeds thresholds. |
| `apriltag_map_recorder.min_detection_range_m / max_detection_range_m` | `0.10 / 2.50` | Distance gate for mapping-time landmark recording. |
| `apriltag_map_recorder.use_covariance_filter` | `true` | Rejects low-confidence detections based on covariance thresholds. |
| `apriltag_map_recorder.publish_landmarks_topic` | `true` | Publishes registered map landmarks as `/apriltag/landmarks` PoseArray. |
| `apriltag_map_recorder.publish_landmark_tf` | `true` | Publishes `map -> apriltag_landmark/*` TF for registered tags during mapping. |
| `apriltag_map_recorder.landmark_tf_child_frame_prefix` | `apriltag_landmark` | Prefix for mapping-time landmark TF child frames. |
| `apriltag_map_localization.output_pose_topic` | `/apriltag/map_pose` | Global pose output fused by global EKF. |
| `apriltag_map_localization.auto_landmarks_from_map` | `true` | Auto-resolves `<map>.apriltags.yaml` when `landmarks_file` is empty. |
| `apriltag_map_localization.use_landmark_yaw` | `true` | Uses saved landmark orientation when solving map pose. |
| `apriltag_landmark_tf_publisher.child_frame_prefix` | `apriltag_landmark` | Prefix for static landmark TF child frames. |
| `apriltag_nav_behavior_tree.docking_default_target_range_m` | `0.30` | Default reverse-docking stop range. |
| `apriltag_nav_behavior_tree.docking_default_linear_x` | `-0.08` | Default docking approach speed (reverse). |
| `apriltag_nav_behavior_tree.docking_pause_nav2` | `true` | Pauses Nav2 around docking/sleep action flow. |

### Launch Defaults That Gate Behavior

Sources:
- `src/robot_bringup/launch/navigation.launch.py`
- `src/robot_bringup/launch/nav2.launch.py`
- `src/robot_bringup/launch/mapping.launch.py`

| Launch Arg | Default | Effect |
|---|---:|---|
| `state_estimation_global_enabled` | `true` | Enables global EKF map fusion path. |
| `apriltag_landmark_tf_enabled` | `true` | Publishes static `map -> apriltag_landmark/*` TFs in nav mode. |
| `apriltag_map_localization_enabled` | `true` | Enables AprilTag-based global pose publishing. |
| `grid_localization_enabled` | `true` | Enables Isaac occupancy-grid localization bridge to `/initialpose`. |
| `apriltag_realsense_enabled` | `false` | Leaves camera/tag detector pipeline off unless explicitly enabled. |
| `apriltag_map_recorder_enabled` | `true` | Enables mapping recorder by default (can be disabled per launch). |

## Optional / Legacy Stack (Used by `bluebot.sh` Map-Explore Flows)

### Explore Lite (`m-explore-ros2`)

Sources:
- `src/serial_diff_drive_hw/config/explore_lite_params.yaml`
- `src/m-explore-ros2/explore/src/explore.cpp`

| Parameter | Current Value | What It Does |
|---|---:|---|
| `costmap_topic` | `/global_costmap/costmap` | Uses Nav2 global costmap for frontier extraction. |
| `costmap_updates_topic` | `/global_costmap/costmap_updates` | Incremental updates paired with selected costmap topic. |
| `planner_frequency` | `0.5` | Frontier planning loop rate. |
| `progress_timeout` | `55.0` | Time without progress before blacklisting current frontier. |
| `same_goal_refresh_sec` | `8.0` | Re-sends active goal periodically to avoid stale goal timestamp issues. |
| `min_frontier_size` | `0.35` | Filters tiny/noisy frontiers. |
| `blacklist_tolerance_cells` | `4.0` | Blacklist proximity tolerance in costmap cells. |
| `no_frontier_wait_sec` | `45.0` | Grace period before declaring no-frontier completion. |
| `max_blacklist_resets` | `2` | Number of blacklist-clearing recovery attempts before stop. |
| `return_to_init` | `false` | Keeps exploration in-place instead of returning to origin. |

### Nav2 Map-Explore Profile (`serial_diff_drive_hw`)

Source:
- `src/serial_diff_drive_hw/config/nav2_map_explore_params.yaml`

| Parameter | Current Value | What It Does |
|---|---:|---|
| `controller_server.controller_frequency` | `5.0` | Lower control loop target for Jetson map-explore workload. |
| `controller_server.progress_checker.required_movement_radius` | `0.03` | More tolerant progress threshold for slow/stop-go exploration motion. |
| `controller_server.progress_checker.movement_time_allowance` | `90.0` | Allows more time before progress failure in cluttered areas. |
| `controller_server.FollowPath.min_speed_xy` | `0.1` | Keeps commanded speed above drivetrain deadband. |
| `controller_server.FollowPath.trans_stopped_velocity` | `0.03` | Lower stop threshold to reduce false stalled detections. |
| `controller_server.FollowPath.max_vel_x` | `0.24` | Forward speed cap for map-explore mode. |
| `controller_server.FollowPath.max_speed_xy` | `0.24` | Planar translational speed cap for map-explore mode. |
| `planner_server.GridBased.tolerance` | `1.2` | Planner tolerance for reachable-end fallback in partially blocked goals. |

## Base Package Defaults vs Project Overrides (`ros2_serial_diff_drive_bridge`)

Sources:
- `src/ros2_serial_diff_drive_bridge/config/ros2_serial_diff_drive_bridge.params.yaml` (package defaults)
- `src/robot_bringup/config/nav2_serial_diff_drive_bridge.yaml` (project profile)

| Parameter | Package Default | Project Override (`robot_bringup`) |
|---|---:|---:|
| `cmd_vel_topic` | `/cmd_vel` | `/cmd_vel_compensated` |
| `odom_topic` | `/odom` | `/odom_raw` |
| `publish_tf` | `true` | `false` |
| `enable_stall_compensation` | `false` | `true` |
| `min_effective_linear_mps` | `0.0` | `0.14` |
| `min_effective_angular_rad_s` | `0.0` | `0.25` |
| `min_effective_turn_wheel_mps` | `0.0` | `0.10` |
| `zero_cmd_epsilon` | `0.0001` | `0.0001` |
