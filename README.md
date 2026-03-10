# Bluebot ROS 2 Workspace

This repository is the main ROS 2 Humble workspace for Bluebot bringup, mapping, navigation, and hardware integration.

## What Is Here

- `src/`: ROS 2 packages and git submodules (Isaac ROS, RealSense, lidar, navigation, hardware drivers).
- `scripts/`: runtime helper scripts (primary entrypoint: `bluebot.sh`).
- `3d_stl/`: printable robot part models (`.stl`) and part notes.
- `docs/`: project documentation and diagrams.

## Prerequisites

- Ubuntu with ROS 2 Humble installed (`/opt/ros/humble`).
- Colcon toolchain.
- Robot hardware connected as expected (`/dev/arduino`, `/dev/lidar`, camera), when running on the robot.

## Setup

Initialize submodules:

```bash
cd /ssd/ros2_ws
git submodule update --init --recursive
```

Build workspace:

```bash
cd /ssd/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Source workspace:

```bash
source /ssd/ros2_ws/install/setup.bash
```

## Run

Primary bringup script:

```bash
/ssd/ros2_ws/scripts/bluebot.sh --help
```

Common flows:

```bash
/ssd/ros2_ws/scripts/bluebot.sh start-map
/ssd/ros2_ws/scripts/bluebot.sh save-map office_a
/ssd/ros2_ws/scripts/bluebot.sh start-nav office_a
```

## Related Docs

- `scripts/README_bluebot.md`: full command reference and operational workflows.
- `3d_stl/README.md`: STL part inventory.
- `docs/`: additional notes and diagrams.
