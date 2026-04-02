# bluebot_v2

Minimal ROS 2 stack for Bluebot v2 mapping with:

- Wheel-only odometry from `ros2_serial_diff_drive_bridge`
- LiDAR scans from `lidar_launch`
- 2D map building with `slam_toolbox`
- Built-in `foxglove_bridge` for remote control/monitoring
- Simple movement commands: `forward`, `backward`, `left`, `right`, `stop`

## Build

```bash
cd /ssd/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select bluebot_v2
source /ssd/ros2_ws/install/setup.bash
```

## Start Minimal Mapping Stack

```bash
ros2 launch bluebot_v2 minimal_mapping.launch.py
```

Foxglove bridge is enabled by default on `ws://<robot-ip>:8765`.

## Remote Xbox Teleop (Foxglove)

From your remote laptop:

1. Connect Foxglove Desktop to `ws://<robot-ip>:8765`
2. Add a joystick/gamepad panel
3. Publish `geometry_msgs/msg/Twist` to `/cmd_vel`

## Send Drive Commands

Publish command words to `/bluebot_v2/drive_command`:

```bash
ros2 topic pub --once /bluebot_v2/drive_command std_msgs/msg/String "{data: 'forward'}"
ros2 topic pub --once /bluebot_v2/drive_command std_msgs/msg/String "{data: 'left'}"
ros2 topic pub --once /bluebot_v2/drive_command std_msgs/msg/String "{data: 'stop'}"
```

Timed command (auto-stop after 1.5 sec):

```bash
ros2 topic pub --once /bluebot_v2/drive_command std_msgs/msg/String "{data: 'forward 1.5'}"
```

## Save Map

```bash
ros2 run nav2_map_server map_saver_cli -f /ssd/maps/bluebot_v2_map
```
