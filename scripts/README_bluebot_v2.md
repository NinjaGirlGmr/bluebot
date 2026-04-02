# bluebot_v2.sh

Path: `/ssd/ros2_ws/scripts/bluebot_v2.sh`

A minimal runtime helper for Bluebot v2 wheel-odometry + LiDAR mapping.
Foxglove bridge is included by default for remote control and visualization.

## Commands

```bash
/ssd/ros2_ws/scripts/bluebot_v2.sh start
/ssd/ros2_ws/scripts/bluebot_v2.sh stop
/ssd/ros2_ws/scripts/bluebot_v2.sh restart
/ssd/ros2_ws/scripts/bluebot_v2.sh status
/ssd/ros2_ws/scripts/bluebot_v2.sh cmd forward
/ssd/ros2_ws/scripts/bluebot_v2.sh cmd backward
/ssd/ros2_ws/scripts/bluebot_v2.sh cmd left
/ssd/ros2_ws/scripts/bluebot_v2.sh cmd right
/ssd/ros2_ws/scripts/bluebot_v2.sh cmd stop
/ssd/ros2_ws/scripts/bluebot_v2.sh cmd forward 1.5
/ssd/ros2_ws/scripts/bluebot_v2.sh save-map office_a
```

## Environment Overrides

- `ARDUINO_PORT` (default `/dev/arduino`)
- `LIDAR_PORT` (default `/dev/lidar`)
- `USE_RVIZ` (default `false`)
- `MAP_DIR` (default `/ssd/maps`)
- `MAP_TOPIC` (default `/map`)
- `CMD_TOPIC` (default `/bluebot_v2/drive_command`)
- `FOXGLOVE_BRIDGE_ENABLED` (default `true`)
- `FOXGLOVE_BRIDGE_PORT` (default `8765`)
- `FOXGLOVE_BRIDGE_ADDRESS` (default `0.0.0.0`)
- `FOXGLOVE_BRIDGE_CAPABILITIES` (default `[clientPublish,services,connectionGraph,assets]`)

Example:

```bash
ARDUINO_PORT=/dev/ttyUSB0 LIDAR_PORT=/dev/ttyUSB1 /ssd/ros2_ws/scripts/bluebot_v2.sh start
```
