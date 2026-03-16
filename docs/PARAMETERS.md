# Project Parameter Overrides

This document tracks ROS parameters we have modified for this project, grouped by package.

## Package: `explore` (`m-explore-ros2`)

Sources:
- `src/serial_diff_drive_hw/config/explore_lite_params.yaml`
- `src/m-explore-ros2/explore/src/explore.cpp` (new parameter support)

| Parameter | Current Value | What It Does |
|---|---:|---|
| `costmap_topic` | `/global_costmap/costmap` | Selects which occupancy/cost map is used for frontier extraction. Using the Nav2 global costmap biases goals toward planner-reachable space. |
| `costmap_updates_topic` | `/global_costmap/costmap_updates` | Topic for incremental map updates paired with `costmap_topic`. |
| `planner_frequency` | `0.5` | How often frontier planning runs (Hz). Lower values reduce planning load and replan less aggressively. |
| `progress_timeout` | `55.0` | Seconds without meaningful progress before the current frontier goal is blacklisted. |
| `same_goal_refresh_sec` | `8.0` | Re-sends the same active goal after this many seconds to keep goal timestamps fresh during long retries. `0` disables refresh. |
| `min_frontier_size` | `0.35` | Filters very small frontiers to reduce noise while keeping sensitivity high enough to keep exploration moving. |
| `blacklist_tolerance_cells` | `4.0` | Radius (in costmap cells) used when deciding whether a frontier is "too close" to a previously failed goal. |

## Package: `nav2` profile in `serial_diff_drive_hw`

Source:
- `src/serial_diff_drive_hw/config/nav2_map_explore_params.yaml`

| Parameter | Current Value | What It Does |
|---|---:|---|
| `controller_server.current_goal_checker` | `"general_goal_checker"` | Explicitly selects the goal-checker plugin used by the controller server. |
| `controller_server.progress_checker.required_movement_radius` | `0.03` | Minimum displacement considered valid progress. Lowering this accepts slower/shorter real-world motion. |
| `controller_server.progress_checker.movement_time_allowance` | `90.0` | Time window allowed to make progress before Nav2 reports failure. Higher values are more tolerant in clutter/slow zones. |
| `controller_server.FollowPath.min_speed_xy` | `0.1` | Minimum commanded translational speed magnitude while moving to avoid deadband stall behavior. |
| `controller_server.FollowPath.trans_stopped_velocity` | `0.03` | Velocity threshold below which the robot is treated as stopped for controller logic. |
| `controller_server.FollowPath.max_vel_x` | `0.24` | Maximum forward linear velocity for the DWB local planner. |
| `controller_server.FollowPath.max_speed_xy` | `0.24` | Maximum translational speed magnitude for the DWB local planner. |
| `planner_server.GridBased.tolerance` | `1.2` | Planner goal tolerance (meters) for valid end states when exact goal cells are blocked/unreachable. |

## Package: `ros2_serial_diff_drive_bridge`

Sources:
- `src/ros2_serial_diff_drive_bridge/config/ros2_serial_diff_drive_bridge.params.yaml`
- `src/ros2_serial_diff_drive_bridge/ros2_serial_diff_drive_bridge/bridge_node.py`
- `src/ros2_serial_diff_drive_bridge/launch/ros2_serial_diff_drive_bridge.launch.py`

| Parameter | Current Value | What It Does |
|---|---:|---|
| `enable_stall_compensation` | `false` | Enables minimum-effective command enforcement before sending `CMD_VEL` to firmware. |
| `min_effective_linear_mps` | `0.0` | Minimum absolute non-zero linear command when stall compensation is enabled. |
| `min_effective_angular_rad_s` | `0.0` | Minimum absolute non-zero angular command when stall compensation is enabled. |
| `zero_cmd_epsilon` | `0.0001` | Magnitude at or below this threshold is treated as exact zero before minimum-effective clamping. |

Launch override support was also added for these existing node parameters:
- `cmd_vel_topic`
- `odom_topic`
- `publish_tf`
