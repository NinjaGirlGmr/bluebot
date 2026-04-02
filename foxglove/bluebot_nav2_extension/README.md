# BlueBot Nav2 Foxglove Extension

Local Foxglove extension that adds a custom panel named `BlueBot Nav2 Controls`.

The panel provides:
- Publish single goal to `/goal_pose` (`geometry_msgs/PoseStamped`)
- Publish initial pose to `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`)
- Publish waypoint route to `/foxglove/waypoints` (`geometry_msgs/PoseArray`)
- Cancel active Nav2 goals via:
  - `/navigate_to_pose/_action/cancel_goal`
  - `/navigate_through_poses/_action/cancel_goal`
- Live status readout from:
  - `/navigate_to_pose/_action/status`
  - `/navigate_through_poses/_action/status`
  - `/foxglove/waypoints/status`

## Build And Install

Run from this directory:

```bash
cd /ssd/ros2_ws/foxglove/bluebot_nav2_extension
npm install
npm run local-install
```

Then restart or refresh Foxglove Desktop and add panel:
- `Add panel` -> `BlueBot Nav2 Controls`

## Usage Notes

- This panel expects a ROS2 Foxglove bridge connection (for publish + service calls).
- Start robot navigation first (example):

```bash
/ssd/ros2_ws/scripts/bluebot_nav.sh start my_house
```

- In waypoint text box, use one waypoint per line:
  - `x y`
  - `x y yaw_deg`
  - comma-separated is also accepted, e.g. `1.0, 0.5, 90`
