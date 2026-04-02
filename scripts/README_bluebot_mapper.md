# bluebot_mapper.sh

Path: `/ssd/ros2_ws/scripts/bluebot_mapper.sh`

Bluebot v3 mapping workflow for generating Nav2-ready maps using:

- Wheel odometry (`/odom_raw`)
- IMU + wheel fusion (`/odom`)
- LiDAR scans (`/scan`)
- SLAM Toolbox mapping (`/map`)

## Commands

```bash
/ssd/ros2_ws/scripts/bluebot_mapper.sh start
/ssd/ros2_ws/scripts/bluebot_mapper.sh stop
/ssd/ros2_ws/scripts/bluebot_mapper.sh restart
/ssd/ros2_ws/scripts/bluebot_mapper.sh status
/ssd/ros2_ws/scripts/bluebot_mapper.sh tf-check
/ssd/ros2_ws/scripts/bluebot_mapper.sh save-map office_a
```

`tf-check` validates these links: `map <- odom`, `odom <- base_link`, `base_link <- laser`, `base_link <- imu_link`.

## Environment Overrides

- `ARDUINO_PORT` (default `/dev/arduino`)
- `ARDUINO_BAUD` (default `115200`)
- `MAP_FRAME` (default `map`)
- `ODOM_FRAME` (default `odom`)
- `BASE_FRAME` (default `base_link`)
- `WHEEL_RADIUS_M` (default `0.03354`)
- `WHEEL_SEPARATION_M` (default `0.195`)
- `LIDAR_PORT` (default `/dev/lidar`)
- `LIDAR_FRAME` (default `laser`)
- `LIDAR_SCAN_FREQUENCY` (default `8.0`)
- `LIDAR_ANGLE_COMPENSATE` (default `true`)
- `LIDAR_SCAN_MODE` (default empty)
- `LIDAR_TF_X/Y/Z` (defaults `0.0/0.0/0.199`)
- `LIDAR_TF_ROLL/PITCH/YAW` (defaults `0.0/0.0/0.0`)
- `IMU_FRAME` (default `imu_link`)
- `IMU_PORT` (default `/dev/myimu`)
- `IMU_BAUD` (default `115200`)
- `IMU_TF_X/Y/Z` (defaults `0.0/0.0/0.0`)
- `IMU_TF_ROLL/PITCH/YAW` (defaults `0.0/0.0/0.0`)
- `USE_RVIZ` (default `false`)
- `NAV2_USE_SIM_TIME` (default `false`)
- `FOXGLOVE_BRIDGE_ENABLED` (default `true`)
- `FOXGLOVE_BRIDGE_PORT` (default `8765`)
- `FOXGLOVE_BRIDGE_ADDRESS` (default `0.0.0.0`)
- `FOXGLOVE_BRIDGE_CAPABILITIES` (default `[clientPublish,services,connectionGraph,assets]`)
- `UDP_CMD_VEL_BRIDGE_ENABLED` (default `true`)
- `UDP_CMD_VEL_BRIDGE_LISTEN_HOST` (default `0.0.0.0`)
- `UDP_CMD_VEL_BRIDGE_LISTEN_PORT` (default `8766`)
- `UDP_CMD_VEL_BRIDGE_CMD_TOPIC` (default `/cmd_vel`)
- `UDP_CMD_VEL_BRIDGE_TIMEOUT_S` (default `0.5`)
- `ODOM_SOURCE_MODE` (default `ekf`, options: `ekf` or `raw`)
- `STRAIGHT_COMP_ENABLED` (default `true`)
- `STRAIGHT_COMP_ODOM_TOPIC` (default `__AUTO__`, auto-selects `/odom_raw` in `raw` mode, `/odom` in `ekf` mode)
- `STRAIGHT_COMP_LINEAR_MIN` (default `0.03`)
- `STRAIGHT_COMP_ANGULAR_DEADBAND` (default `0.05`)
- `STRAIGHT_COMP_TURN_MEMORY_ANGULAR_THRESHOLD` (default `0.20`)
- `STRAIGHT_COMP_KP` (default `1.8`)
- `STRAIGHT_COMP_KI` (default `0.0`)
- `STRAIGHT_COMP_KD` (default `0.20`)
- `STRAIGHT_COMP_MAX_ANGULAR` (default `0.50`)
- `STRAIGHT_COMP_INTEGRAL_LIMIT` (default `0.40`)
- `STRAIGHT_COMP_CASTER_ENABLED` (default `true`)
- `STRAIGHT_COMP_CASTER_GAIN` (default `0.12`)
- `STRAIGHT_COMP_CASTER_DECAY_SEC` (default `0.60`)
- `STRAIGHT_COMP_CASTER_MAX` (default `0.25`)
- `STRAIGHT_COMP_CASTER_MAX_AGE_SEC` (default `2.0`)
- `STRAIGHT_COMP_CASTER_FORWARD_ONLY` (default `true`)
- `STRAIGHT_COMP_ODOM_TIMEOUT_SEC` (default `0.40`)
- `STRAIGHT_COMP_RESET_ON_REVERSE` (default `true`)
- `SLAM_PARAMS_FILE` (default `/ssd/ros2_ws/src/bluebot_v3/config/slam_toolbox_mapping_nav2.yaml`)
- `EKF params` (default `/ssd/ros2_ws/src/bluebot_v3/config/ekf_mapping.yaml` via launch default)
- `MAP_DIR` (default `/ssd/maps`)
- `MAP_TOPIC` (default `/map`)

Example:

```bash
LIDAR_SCAN_FREQUENCY=10.0 /ssd/ros2_ws/scripts/bluebot_mapper.sh start
```

## RViz (Laptop)

Use the mapping-focused RViz profile:

```bash
/ssd/ros2_ws/rviz/start-rviz mapper
```

Equivalent direct command:

```bash
rviz2 -d /ssd/ros2_ws/rviz/bluebot_mapper.rviz
```

## Foxglove (Optional)

`bluebot_mapper.sh start` now also starts `foxglove_bridge` by default.

Note: `foxglove_bridge` does not add a dedicated ROS topic name. Verify with:

```bash
ros2 node list | grep foxglove_bridge
```

Websocket:

```bash
ws://<robot-ip>:8765
```

Use your Jetson LAN IP for `<robot-ip>` (for example `192.168.1.42`), not the literal text.

Recommended mapping layout to import:

```bash
/ssd/ros2_ws/foxglove/bluebot_mapper_mapping_layout.json
```

Disable bridge if needed:

```bash
FOXGLOVE_BRIDGE_ENABLED=false /ssd/ros2_ws/scripts/bluebot_mapper.sh restart
```

## UDP cmd_vel Bridge (Optional)

`bluebot_mapper.sh start` now also starts:

```bash
ros2 launch udp_cmd_vel_bridge udp_cmd_vel_bridge.launch.py
```

By default this bridge publishes directly to `/cmd_vel`, so you may not see a new topic name.
Verify with:

```bash
ros2 node list | grep udp_cmd_vel_bridge
ros2 node info /udp_cmd_vel_bridge
```

To force a visible separate topic:

```bash
UDP_CMD_VEL_BRIDGE_CMD_TOPIC=/cmd_vel_udp /ssd/ros2_ws/scripts/bluebot_mapper.sh restart
```

Keep UDP and Foxglove ports different:
- Foxglove websocket (TCP): `FOXGLOVE_BRIDGE_PORT` (default `8765`)
- UDP teleop receiver (UDP): `UDP_CMD_VEL_BRIDGE_LISTEN_PORT` (default `8766`)

Disable it if needed:

```bash
UDP_CMD_VEL_BRIDGE_ENABLED=false /ssd/ros2_ws/scripts/bluebot_mapper.sh restart
```

## Mapping Stability Notes

- `bluebot_v3` uses a conservative EKF tuned for mapping:
  wheel odometry pose + IMU yaw rate only.
- IMU fusion in mapper uses only the 10-axis gyro topics (`/imu/data` then `/imu`).
- `a471_serial_node` reads Yahboom IMU from `IMU_PORT` and publishes `/imu/data`.
- Wheel odometry integration is derived from wheel encoder positions in
  `ros2_serial_diff_drive_bridge` (not firmware linear/angular fields).
- Straight-line drift compensation is enabled by default:
  `/cmd_vel` -> `straight_line_compensator` -> `/cmd_vel_compensated` -> motor bridge.
  The compensator holds heading when linear command is active and commanded turn rate is near zero.
- Caster memory compensation is enabled by default:
  after a turn, a short decaying counter-steer term is added to reduce bias drift.
- Fake IMU fallback data is disabled in mapper launch.
- If map quality degrades into blobs:
  1. Verify TF: `/ssd/ros2_ws/scripts/bluebot_mapper.sh tf-check`
  2. Verify LiDAR rate: `ros2 topic hz /scan`
  3. Verify IMU stream exists (optional but recommended): `ros2 topic hz /imu/data_raw`
  4. Drive slower, especially turns, during mapping.
  5. Isolate EKF/IMU by mapping with raw wheel odom:
     `ODOM_SOURCE_MODE=raw /ssd/ros2_ws/scripts/bluebot_mapper.sh restart`

## Straight-Line Drift Tuning

Start with defaults, then tune in this order:

1. Increase `STRAIGHT_COMP_KP` in small steps (`+0.2`) until veer is corrected.
2. If you see oscillation (wiggle), reduce `STRAIGHT_COMP_KP` or increase `STRAIGHT_COMP_KD`.
3. Keep `STRAIGHT_COMP_MAX_ANGULAR` modest (`0.3` to `0.6`) so correction cannot over-steer.
4. Leave `STRAIGHT_COMP_KI=0.0` initially; add a small value only if there is persistent bias.

Example:

```bash
STRAIGHT_COMP_KP=2.2 STRAIGHT_COMP_KD=0.25 /ssd/ros2_ws/scripts/bluebot_mapper.sh restart
```

For front-caster bias after turns:

1. Increase `STRAIGHT_COMP_CASTER_GAIN` (`0.12` -> `0.16` -> `0.20`).
2. Increase `STRAIGHT_COMP_CASTER_DECAY_SEC` if drift lasts longer (`0.60` -> `0.80`).
3. Limit spike with `STRAIGHT_COMP_CASTER_MAX` (`0.20` to `0.30`).

Example:

```bash
STRAIGHT_COMP_CASTER_GAIN=0.18 STRAIGHT_COMP_CASTER_DECAY_SEC=0.80 /ssd/ros2_ws/scripts/bluebot_mapper.sh restart
```
