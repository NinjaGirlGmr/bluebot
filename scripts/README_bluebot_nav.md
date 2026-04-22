# bluebot_nav.sh

Path: `/ssd/ros2_ws/scripts/bluebot_nav.sh`

Navigation wrapper for saved-map Nav2 operation with Foxglove-first workflow.

## Commands

```bash
/ssd/ros2_ws/scripts/bluebot_nav.sh start [map]
/ssd/ros2_ws/scripts/bluebot_nav.sh stop
/ssd/ros2_ws/scripts/bluebot_nav.sh restart [map]
/ssd/ros2_ws/scripts/bluebot_nav.sh status
/ssd/ros2_ws/scripts/bluebot_nav.sh maps
/ssd/ros2_ws/scripts/bluebot_nav.sh foxglove
/ssd/ros2_ws/scripts/bluebot_nav.sh goal <x> <y> [yaw_deg]
/ssd/ros2_ws/scripts/bluebot_nav.sh waypoints [name ...]
```

- `start [map]`
  - If `map` is provided, accepts map name (`office_a`) or full YAML path.
  - If omitted, uses the newest `.yaml` map in `/ssd/maps` (excluding `*_isaac_localizer.yaml` and `waypoints.yaml`).
- `stop` stops the nav stack and RViz process launched by this script (if any).
- `status` shows BlueBot stack status, selected UI mode, and Foxglove connection info.
- `maps` lists available saved maps in `/ssd/maps`.
- `foxglove` prints websocket URL, layout path, and nav topic references.
- `goal <x> <y> [yaw_deg]` publishes a single `PoseStamped` goal on `/goal_pose`.
- `waypoints [name ...]` publishes saved waypoints from `/ssd/maps/waypoints.yaml` to the waypoint follower topic.

## Foxglove Goal Workflow

1. Start navigation (Foxglove mode is the default):

```bash
/ssd/ros2_ws/scripts/bluebot_nav.sh start office_a
```

2. Open Foxglove Desktop and connect to:

```text
ws://<robot-ip>:8765
```

3. Import this layout:

```text
/ssd/ros2_ws/foxglove/bluebot_nav_layout.json
```

Optional BT-focused dashboard (separate import):

```text
/ssd/ros2_ws/foxglove/bluebot_nav_bt_layout.json
```

This dashboard is focused on Nav2 behavior tree and status introspection:
- `/behavior_tree_log`
- `/navigate_to_pose/_action/status`
- `/navigate_through_poses/_action/status`
- `/dock_robot/_action/status`
- `/rosout`

4. In a `3D` panel, click `Publish`, choose `Pose`, and place a goal on the map.
   - Goal topic: `/goal_pose`
   - Pose estimate topic (if needed): `/initialpose`

Nav2 receives the goal on `/goal_pose`.

## Custom Nav2 Panel (Foxglove Extension)

Extension path:

```text
/ssd/ros2_ws/foxglove/bluebot_nav2_extension
```

Install it locally into Foxglove Desktop:

```bash
cd /ssd/ros2_ws/foxglove/bluebot_nav2_extension
npm install
npm run local-install
```

Then in Foxglove:
1. Refresh/restart Foxglove Desktop.
2. `Add panel` -> `BlueBot Nav2 Controls`.
3. Use panel buttons for:
   - goal publish (`/goal_pose`)
   - initial pose publish (`/initialpose`)
   - waypoint route publish (`/foxglove/waypoints`)
   - goal cancel services for Nav2 actions

## Terminal Goal Example

If you already know map coordinates:

```bash
/ssd/ros2_ws/scripts/bluebot_nav.sh goal 1.8 -0.6 90
```

- `x` and `y` are in meters in the map frame.
- `yaw_deg` is optional and defaults to `0`.

## Waypoint Route Example

Send all saved waypoints:

```bash
/ssd/ros2_ws/scripts/bluebot_nav.sh waypoints
```

Send selected waypoints only:

```bash
/ssd/ros2_ws/scripts/bluebot_nav.sh waypoints dock hallway
```

## Environment Overrides

- `NAV_UI` (default `foxglove`, options: `foxglove|rviz|none`)
- `MAP_DIR` (default `/ssd/maps`)
- `FOXGLOVE_LAYOUT` (default `/ssd/ros2_ws/foxglove/bluebot_nav_layout.json`)
- `FOXGLOVE_BT_LAYOUT` (default `/ssd/ros2_ws/foxglove/bluebot_nav_bt_layout.json`)
- `FOXGLOVE_EXTENSION_DIR` (default `/ssd/ros2_ws/foxglove/bluebot_nav2_extension`)
- `FOXGLOVE_BRIDGE_PORT` (default `8765`)
- `FOXGLOVE_WAYPOINT_TOPIC` (default `/foxglove/waypoints`)
- `FOXGLOVE_WAYPOINT_STATUS_TOPIC` (default `/foxglove/waypoints/status`)
- `AUTO_RVIZ` (default `true`, used only when `NAV_UI=rviz`)
- `RVIZ_START_SCRIPT` (default `/ssd/ros2_ws/rviz/start-rviz`)
- `RVIZ_CONFIG` (default `/ssd/ros2_ws/rviz/my_house_nav.rviz`)
