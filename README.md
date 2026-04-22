# Bluebot

<div align="center">
  <table><tr>
    <td><img width="252" height="336" alt="IMG_5673" src="https://github.com/user-attachments/assets/7a28949f-eb91-4703-aa26-80fc5caea16e"></td>
    <td><img width="252" height="336" alt="IMG_5674" src="https://github.com/user-attachments/assets/98038e4b-68a5-4cc1-9ce0-097e8eb675c4"></td>
  </tr></table>
</div>

## Introduction

Bluebot is a personal autonomous mobile robot project built from the ground up on **ROS 2 Humble**, running on a **NVIDIA Jetson Orin Nano**. The goal was to build a fully self-contained indoor navigation platform — from hardware integration through sensor fusion, mapping, and autonomous goal-seeking — using the same tools and frameworks used in professional and research robotics.

The robot integrates a **Slamtec A2R8 LiDAR**, **Intel RealSense D435** depth camera, **Yahboom A471 10-axis IMU**, and an **Arduino Nano** motor controller into a unified ROS 2 stack. Navigation is built on **Nav2** with a multi-sensor localization pipeline: wheel odometry fused with IMU via `robot_localization` EKF, **AprilTag**-based global pose correction via a secondary EKF, and **Isaac ROS occupancy-grid localization** for initial pose estimation. Maps are built using **SLAM Toolbox** and annotated with AprilTag landmarks for repeatable localization across sessions.

Key areas of custom development include:

- **AprilTag landmark pipeline** — recording, static TF publishing, and map-frame pose estimation nodes written in Python, integrated with the Nav2 localization graph
- **Custom Nav2 Behavior Tree plugin** (C++) — rule-based BT node for AprilTag-triggered actions including docking, relocalizing, and navigation
- **Autonomous docking** — `opennav_docking` integration with a custom detector node bridging AprilTag detections to the docking server
- **Straight-line drift compensation** — PID yaw-correction node to counteract drivetrain asymmetry on open-loop straight drives
- **Foxglove** telemetry, custom panel extension, and layout files for real-time visualization and remote operation

This repository is the full ROS 2 workspace — all bringup launch files, configuration, custom nodes, and runtime scripts are here.

## Goals

### Why This Project

I've been interested in coding since middle school, starting with building my own games. In high school that interest found a new direction when I joined my school's FIRST Robotics team as a programmer. Working on a competition robot gave me my first real exposure to semi-autonomous systems — building software to achieve specific, defined objectives under real hardware constraints — and I was immediately drawn to the problem space.

FIRST is a great environment for collaborative development and creative problem solving, but competition goals are predefined and the season is fixed. I wanted the freedom to set my own objectives, explore directions I hadn't planned for, and iterate without a deadline. Bluebot was the answer to that. It gave me a platform where I could decide what "success" meant and keep redefining it as I learned more.

It's also been a two-way exchange. Things I learned building Bluebot — particularly around vision-based perception and robot localization — I was able to bring back to our competition robot and apply directly. That feedback loop between personal exploration and team work has been one of the most valuable parts of the project.

More broadly, I started Bluebot because I wanted more than a theoretical understanding of autonomous systems — I wanted to build one end to end. The goal from the beginning was a project that touched every layer of the problem: selecting and integrating hardware, designing the physical structure, wiring sensors into a software stack, and implementing the algorithms that make a robot capable of understanding and acting in its environment. Robotics is uniquely demanding in that way, and that's exactly what drew me to it.

### What I Set Out to Build

The initial technical goal was straightforward: a robot that could map an unknown environment using sensor data and then navigate it autonomously. Getting there required working through localization, sensor fusion, motion control, and path planning — none of which are simple in practice.

Along the way I explored Visual SLAM as a localization strategy, but after working through the implementation decided it was more infrastructure than the project needed. That decision led to something more interesting: using what I had learned about perception to instead detect AprilTag landmarks during mapping and register them as fixed reference points for navigation. Rather than relying on dense visual features, the robot builds a sparse semantic map it can re-localize against reliably. That pivot taught me more about the real engineering tradeoffs in perception than following the original plan would have.

From there the scope grew naturally — reliable localization raised the question of what the robot should actually *do* once it knew where it was, which led to behavior trees, goal management, and ultimately autonomous docking.

### Where It's Going

The near-term focus is continued navigation tuning and developing richer behavior trees that allow the robot to handle goals in dynamic, changing environments — not just static ones it has seen before.

Longer term, I want to explore autonomous exploration: giving the robot the ability to extend its own map into unmapped frontiers without teleop, blending navigation and mapping into a single continuous capability. I'm also interested in dynamic system management — having the robot start and stop compute-intensive services based on what it's currently doing, which is both a practical constraint on the Jetson and an interesting systems design problem in its own right.

## Current Runtime State

Main ROS 2 Humble workspace for Bluebot bringup, mapping, navigation, and hardware integration.

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
- `docks_db`: docks_database yaml
- `3d_stl/`: printable robot part models.
- `docs/`: parameter notes and stack diagrams.

## Hardware

- Jetson Orin Nano Super Dev Kit
- Arduino Nano
- Slamtec A2R8 LiDAR
- Intel RealSense D435
- Yahboom A471 10-axis IMU
- 2x BTS7960 H-Bridge
- 2x DC 12v Encoder Gear Motor

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

## Mapping Workflow (SLAM + AprilTag Landmarks)

Start mapping with AprilTag detection and recorder enabled:

```bash
source "$ROS_WS/install/setup.bash"
ros2 launch robot_bringup mapping.launch.py \
  apriltag_realsense_enabled:=true \
  apriltag_map_recorder_enabled:=true \
  apriltag_map_output_yaml:=/tmp/apriltag_map_landmarks.yaml
```

During mapping, registered landmarks are published in `map` on `/apriltag/landmarks`
and as TF `map -> apriltag_landmark/*`. Raw `/tag_detections` remains camera-relative.

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
- `scripts/README_bluebot_nav.md`: Foxglove-first nav helper workflow.
- `docs/PARAMETERS.md`: project parameter overrides.
- `docs/stack_diagrams/README.md`: architecture diagrams.
