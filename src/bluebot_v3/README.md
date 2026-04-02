# bluebot_v3

Bluebot v3 mapping stack optimized for creating Nav2-ready 2D occupancy maps.

Pipeline:

- Wheel odometry (`ros2_serial_diff_drive_bridge`) -> `/odom_raw`
- IMU (`yb_a471_driver`) + wheel odom fusion (`robot_localization`) -> `/odom`
- LiDAR scan source (`lidar_launch`) -> `/scan`
- SLAM (`slam_toolbox`) -> `/map`, `map -> odom`

For mapping stability, EKF in `bluebot_v3` is tuned to use:

- wheel odometry pose (`/odom_raw`)
- IMU yaw rate (`/imu/data_raw` angular `z`) only

This avoids acceleration-driven drift that can cause map blob artifacts indoors.

Bridge odometry integration uses wheel encoder kinematics
(`wheel_radius_m`, `wheel_separation_m`) for robust yaw during turns.

Required TF chain:

- `map -> odom` (from `slam_toolbox`)
- `odom -> base_link` (from `robot_localization`)
- `base_link -> laser` (static transform from `lidar_launch`)
- `base_link -> imu_link` (static transform in `bluebot_v3` launch)

## Build

```bash
cd /ssd/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select bluebot_v3 --symlink-install
source /ssd/ros2_ws/install/setup.bash
```

## Launch

```bash
ros2 launch bluebot_v3 mapping_stack.launch.py
```

If using the helper script, run TF verification before mapping:

```bash
/ssd/ros2_ws/scripts/bluebot_mapper.sh tf-check
```

## Save Map

```bash
ros2 run nav2_map_server map_saver_cli -f /ssd/maps/my_map
```

Result:

- `/ssd/maps/my_map.yaml`
- `/ssd/maps/my_map.pgm`

These map files can be used later with Nav2 localization/navigation.
