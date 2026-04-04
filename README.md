# Bluebot ROS 2 Workspace

Main ROS 2 Humble workspace for Bluebot bringup, mapping, navigation, and hardware integration.

## Current Runtime State

The active runtime stack is centered on `robot_bringup`.

- Mapping: `slam_toolbox` + drivetrain + local EKF + optional AprilTag landmark recording.
- Navigation: Nav2 + dedicated smoother config + local/global EKF + optional AprilTag camera pipeline.
- AprilTag global localization path in nav mode:
  - static landmarks from `<map>.apriltags.yaml` -> `map -> apriltag_landmark/*`
  - detections `camera -> tag_i`
  - `apriltag_map_localization` publishes `/apriltag/map_pose`
  - `robot_localization_global_filter` fuses `/odom`, `/apriltag/map_pose`, `/initialpose`
  - global EKF publishes `map -> odom` (consumed by Nav2 via TF)

## Workspace Layout

- `src/`: ROS 2 packages and submodules (Isaac ROS, RealSense, lidar, navigation, hardware drivers, bringup).
- `scripts/`: runtime helper scripts.
- `foxglove/`: layouts and custom panel extension.
- `3d_stl/`: printable robot part models.
- `docs/`: parameter notes and stack diagrams.

## Hardware

- Jetson Orin Nano Super Dev Kit
- Arduino Nano
- Slamtec A2R8 LiDAR
- Intel RealSense D435
- Yahboom A471 10-axis IMU

## Prerequisites

- Ubuntu + ROS 2 Humble (`/opt/ros/humble`)
- `colcon`
- Robot devices available when running on hardware (`/dev/arduino`, `/dev/lidar`, camera)

## Setup

Set workspace path:

```bash
export ROS_WS=/path/to/your/ros2_ws
```

Initialize submodules:

```bash
cd "$ROS_WS"
git submodule update --init --recursive
```

Build workspace:

```bash
cd "$ROS_WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source "$ROS_WS/install/setup.bash"
```

## Primary Runtime Entry Points

Preferred script for `robot_bringup` modes:

```bash
"$ROS_WS/scripts/bluebot_bringup.sh" list-modes
"$ROS_WS/scripts/bluebot_bringup.sh" start mapping
"$ROS_WS/scripts/bluebot_bringup.sh" start navigation office_a
"$ROS_WS/scripts/bluebot_bringup.sh" status
"$ROS_WS/scripts/bluebot_bringup.sh" stop
```

Full `bluebot_bringup.sh` command/function reference:

- `scripts/README_bluebot_bringup.md`

## `bluebot_bringup.sh` Quick Reference

Script path:

```bash
"$ROS_WS/scripts/bluebot_bringup.sh"
```

Core usage:

```bash
bluebot_bringup.sh start <mode> [mode args] [extra ros2 launch args...]
bluebot_bringup.sh stop
bluebot_bringup.sh restart <mode> [mode args] [extra ros2 launch args...]
bluebot_bringup.sh status
bluebot_bringup.sh save-map [name]
```

Mode list:

```bash
"$ROS_WS/scripts/bluebot_bringup.sh" list-modes
```

Full command, argument, environment-variable, and function-level docs:

- `scripts/README_bluebot_bringup.md`

Direct launch usage:

```bash
source "$ROS_WS/install/setup.bash"
ros2 launch robot_bringup navigation.launch.py --show-args
```

Legacy orchestration script still available for non-`robot_bringup` flows:

```bash
"$ROS_WS/scripts/bluebot.sh" --help
```

## Mapping Workflow (SLAM + AprilTag Landmarks)

Start mapping with AprilTag detection and recorder enabled:

```bash
source "$ROS_WS/install/setup.bash"
ros2 launch robot_bringup mapping.launch.py \
  apriltag_realsense_enabled:=true \
  apriltag_map_recorder_enabled:=true \
  apriltag_map_output_yaml:=/tmp/apriltag_map_landmarks.yaml
```

Best-practice capture during mapping:

- Stop-and-scan per tag (stationary robot before sampling).
- Capture multiple views/angles per tag.
- Keep detections in reasonable range.
- Let recorder filters reject unstable samples (consecutive detections, stationary gate, covariance gate).

Save map and copy landmarks alongside it:

```bash
"$ROS_WS/scripts/bluebot_bringup.sh" save-map office_a
```

Outputs:

- map: `/ssd/maps/office_a.yaml` (+ image)
- landmarks: `/ssd/maps/office_a.apriltags.yaml`

## Navigation Workflow (Nav2 + Global Fusion)

Recommended nav launch:

```bash
source "$ROS_WS/install/setup.bash"
ros2 launch robot_bringup navigation.launch.py \
  map:=/ssd/maps/office_a.yaml \
  apriltag_realsense_enabled:=true
```

Quick validation:

```bash
ros2 topic echo /apriltag/map_pose --once
ros2 run tf2_ros tf2_echo map odom
ros2 node list | rg "apriltag_landmark_tf_publisher|apriltag_map_localization|robot_localization_global_filter|smoother_server"
```

## Active Core Packages

- `src/robot_bringup`
- `src/isaac_nav2_pose_bridge`
- `src/ros2_serial_diff_drive_bridge`
- `src/serial_diff_drive_hw`
- `src/udp_cmd_vel_bridge`
- `src/yb_a471_driver`

## Cleanup Notes

Legacy packages `bluebot_v2` and `bluebot_v3` were removed from this workspace.
`straight_line_compensator_node` is now provided by `robot_bringup`.

## Related Documentation

- `src/robot_bringup/README.md`: full package-level launch/config reference.
- `scripts/README_bluebot_bringup.md`: full `bluebot_bringup.sh` command/function/arg reference.
- `scripts/README_bluebot.md`: detailed `bluebot.sh` command reference.
- `scripts/README_bluebot_nav.md`: Foxglove-first nav helper workflow.
- `docs/PARAMETERS.md`: project parameter overrides.
- `docs/stack_diagrams/README.md`: architecture diagrams.
