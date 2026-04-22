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

I've been interested in coding since middle school, starting with building my own games. In high school that interest found a new direction when I joined my school's FIRST Robotics team as a programmer. Working on a competition robot gave me my first real exposure to semi-autonomous systems — writing software to achieve defined objectives under real hardware constraints — and I wanted to go deeper than the competition season allowed.

FIRST is excellent for collaborative development, but the goals are set for you. I wanted to set my own, fail at them, revise them, and follow the problem wherever it led. That freedom is what Bluebot is built on.

Before Bluebot, I built *Le Roomba* — a differential drive robot cannibalized from a robotic vacuum, controlled via teleop with a basic collision avoidance function. It was my first self-built robot and it worked, but finishing it made the gap obvious: reactive avoidance is not the same problem as autonomous navigation, and I didn't yet have the tools to close that gap. Bluebot started where Le Roomba left off.

The knowledge transfer has also run in both directions. Techniques I developed on Bluebot — particularly around vision-based perception and landmark-based localization — I was able to apply directly to our FIRST competition robot. Understanding the underlying problem well enough to port a solution to a different platform, under competition constraints, is a different kind of test than building it the first time. That translation was one of the more satisfying parts of the project.

### What I Set Out to Build

The initial goal was concrete: a robot that could map an unknown environment from sensor data and then navigate it autonomously. Getting there meant working through localization, sensor fusion, motion control, and path planning — problems that are straightforward to describe and genuinely hard to get right on physical hardware.

Not all of those challenges were software. Each component in the stack has specific power requirements, and getting them wrong means damaged hardware or unstable behavior. I had to learn electrical design and power distribution from scratch — understanding voltage regulators, current budgets, and how to wire a system where the motor drivers, Jetson, LiDAR, camera, and microcontroller all coexist safely. I also learned to solder to build reliable connections rather than depending on breadboard prototypes. That hands-on electrical work was a different kind of problem-solving than software, and it made the overall build more grounded. The full power distribution design is documented in [`docs/electrical/power_distribution.md`](docs/electrical/power_distribution.md).

Early on I explored Visual SLAM as the localization backbone. After working through the implementation I concluded it was solving a harder problem than I needed — dense visual feature tracking across a full scene, when what I actually needed was reliable re-localization to a small set of known positions. I stepped back from the implementation and asked what the real requirement was, which led to a different approach entirely: detect AprilTag landmarks during the mapping pass, register their positions in the map frame, and use them as fixed reference points for localization during navigation. The robot builds a sparse semantic map rather than a dense visual one, fuses AprilTag pose corrections into an EKF alongside wheel odometry and IMU, and re-localizes reliably against landmarks it has seen before.

That pivot was instructive in a way the original plan wouldn't have been. Knowing *why* Visual SLAM was the wrong tool — not just that it was hard — required understanding both the technique and the actual problem well enough to compare them honestly.

From there the scope grew in a natural sequence: reliable localization raised the question of what the robot should *do* once it knew where it was, which led to behavior trees, goal management, and ultimately autonomous docking using AprilTag-identified dock targets.

### Where It's Going

The near-term focus is navigation tuning and developing richer behavior trees that allow the robot to handle goals in dynamic environments — not just spaces it has already fully mapped and committed to memory.

Longer term, I want to tackle autonomous exploration: giving the robot the ability to extend its own map into unmapped space without teleop, blending active navigation and SLAM into a single capability. The interesting design challenge there isn't the exploration algorithm itself — frontier-based methods are well understood — it's integrating exploration mode cleanly with the existing localization and docking pipeline so the robot can transition between them without losing state.

I'm also interested in dynamic system management: having the robot start and stop compute-intensive services based on what it's currently doing. On a Jetson Orin Nano this is partly a practical resource constraint, but it's also a more general problem about how an autonomous system should reason about its own computational budget alongside its task budget.

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
- 24V / 6A Battery
- 3x Buck Converters (24V → 16V, 12V, 5V)

Power distribution schematic: [`docs/electrical/power_distribution.md`](docs/electrical/power_distribution.md)

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
