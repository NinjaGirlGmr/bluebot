# bluebot_bt_plugins

Custom Nav2 BehaviorTree.CPP plugins for Bluebot.

## Package Type

- Build type: `ament_cmake`
- Produces: `libbluebot_bt_plugins.so` (loaded by `bt_navigator` at runtime)

## Plugins

### `DockRobotById`

BT Action node that navigates the robot to a named dock and docks it using `opennav_docking`.

Wraps: `opennav_docking_msgs/action/DockRobot` on action server `/dock_robot`

| Port | Direction | Type | Default | Description |
|---|---|---|---|---|
| `dock_id` | Input | `string` | *(required)* | Dock key from `dock_database.yaml` (e.g. `"25"`, `"26"`) |
| `navigate_to_staging_pose` | Input | `bool` | `true` | Have Nav2 drive to the staging pose before docking |
| `max_staging_time` | Input | `float` | `1000.0` | Seconds allowed to reach the staging pose |
| `server_timeout` | Input | `int` (ms) | `2000` | Action server connection timeout |
| `success` | Output | `bool` | — | `true` if docking completed successfully |
| `error_code` | Output | `uint16` | — | Error code from the docking server |
| `num_retries` | Output | `uint16` | — | Number of docking attempts made |

**Success**: action server reported SUCCEED and `result.success == true`  
**Failure**: action server aborted, or `dock_id` port was empty

## Available Docks

Defined in `/ssd/ros2_ws/docks_db/dock_database.yaml`:

| ID | Key | Pose (x, y, yaw) | Type |
|---|---|---|---|
| `"25"` | `dock25` | 0.3, 0.3, 0.0 | NonChargingDock |
| `"26"` | `dock26` | -7.8, 5.1, 0.1 | NonChargingDock |

## External Trigger: `dock_command_node`

A lightweight Python node in `robot_bringup` that lets you dock the robot from anywhere by publishing a dock ID — no BT tree required.

```bash
# Start the trigger node
ros2 run robot_bringup dock_command

# Command the robot to dock at dock 25
ros2 topic pub --once /dock_command std_msgs/String "data: 'dock25'"
```

The node subscribes to `/dock_command` (std_msgs/String) and sends a `DockRobot` action goal directly to `/dock_robot`. A new command is ignored while a previous dock goal is still active.

Parameters:

| Parameter | Default | Description |
|---|---|---|
| `dock_command_topic` | `/dock_command` | Topic to subscribe to |
| `dock_action_server` | `dock_robot` | Action server name |
| `navigate_to_staging_pose` | `true` | Passed through to the docking goal |
| `max_staging_time` | `120.0` | Seconds allowed for staging navigation |

## Build

```bash
cd /ssd/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select bluebot_bt_plugins --symlink-install
source install/setup.bash
```

To rebuild `robot_bringup` for the `dock_command_node` entry point:

```bash
colcon build --packages-select bluebot_bt_plugins robot_bringup --symlink-install
```

## Tests

```bash
colcon build --packages-select bluebot_bt_plugins --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select bluebot_bt_plugins
colcon test-result --verbose
```

Test cases in `test/test_dock_robot_by_id.cpp`:

- Happy path returns `SUCCESS`
- Goal fields (`dock_id`, `use_dock_id`, `navigate_to_staging_pose`) populated correctly from input ports
- `navigate_to_staging_pose=false` propagated to goal
- Output ports (`success`, `error_code`) set on success
- Returns `FAILURE` when action server aborts
- Returns `FAILURE` when action server is unavailable
- Alternate dock ID (`"26"`) sent correctly

## Nav2 Integration

`bluebot_bt_plugins` is already added to `bt_navigator.plugin_lib_names` in
`src/robot_bringup/config/nav2.yaml`. No additional config is needed — the plugin
is available to any BT tree loaded by `bt_navigator`.

### BT XML usage

```xml
<DockRobotById
  action_name="dock_robot"
  server_timeout="5000"
  dock_id="{dock_id}"
  navigate_to_staging_pose="true"
  max_staging_time="120.0"
  success="{dock_success}"
  error_code="{dock_error_code}"
  num_retries="{dock_retries}"/>
```

See `bt_trees/dock_by_id_example.xml` for a full tree including a retry variant.

### Loading a custom BT tree at launch

```bash
ros2 launch robot_bringup navigation.launch.py \
  map:=/ssd/maps/office_a.yaml \
  default_nav_to_pose_bt_xml:=/ssd/ros2_ws/src/bluebot_bt_plugins/bt_trees/dock_by_id_example.xml
```

## Runtime Validation

```bash
# Confirm plugin loaded
ros2 node info /bt_navigator | grep bluebot

# Check docking server is up
ros2 action list | grep dock_robot

# Manual dock command
ros2 topic pub --once /dock_command std_msgs/String "data: '25'"

# Watch docking action status
ros2 action list -t
ros2 topic echo /dock_robot/_action/status
```
