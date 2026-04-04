# bluebot_bringup.sh

`bluebot_bringup.sh` is the process manager and mode launcher for the `robot_bringup` stack.

Script path:

`$ROS_WS/scripts/bluebot_bringup.sh`

## Purpose

- Provide one command surface for starting/stopping ROS 2 launch modes used by `robot_bringup`.
- Keep runtime state in `/tmp` so status/stop/restart are consistent across shells.
- Standardize map save behavior and optional AprilTag-landmark sidecar export.

## Command Surface

```bash
bluebot_bringup.sh start <mode> [mode args] [extra ros2 launch args...]
bluebot_bringup.sh stop
bluebot_bringup.sh restart <mode> [mode args] [extra ros2 launch args...]
bluebot_bringup.sh status
bluebot_bringup.sh list-modes
bluebot_bringup.sh modes
bluebot_bringup.sh save-map [name]
```

## Modes and Mode Args

| Mode | Required mode args | Launch target | Built-in launch args |
| --- | --- | --- | --- |
| `sensors` | none | `robot_bringup sensors.launch.py` | `use_sim_time:=$USE_SIM_TIME` |
| `apriltag` | none | `robot_bringup apriltag_realsense.launch.py` | `use_sim_time:=$USE_SIM_TIME` |
| `mapping` | none | `robot_bringup mapping.launch.py` | `use_sim_time:=$USE_SIM_TIME` `apriltag_realsense_enabled:=$APRILTAG_REALSENSE_ENABLED` `apriltag_map_recorder_enabled:=$APRILTAG_MAP_RECORDER_ENABLED` `apriltag_map_output_yaml:=$APRILTAG_MAP_OUTPUT_YAML` |
| `nav2` | `<map>` | `robot_bringup nav2.launch.py` | `map:=<resolved map yaml>` `use_sim_time:=$USE_SIM_TIME` |
| `navigation` | `<map>` | `robot_bringup navigation.launch.py` | `map:=<resolved map yaml>` `use_sim_time:=$USE_SIM_TIME` `apriltag_realsense_enabled:=$APRILTAG_REALSENSE_ENABLED` `apriltag_behavior_enabled:=$APRILTAG_BEHAVIOR_ENABLED` |
| `localization` | none | `robot_bringup localization.launch.py` | `use_sim_time:=$USE_SIM_TIME` |
| `health` | none | `robot_bringup health.launch.py` | `use_sim_time:=$USE_SIM_TIME` |
| `observability` | none | `robot_bringup observability.launch.py` | `use_sim_time:=$USE_SIM_TIME` |

## Map Argument Rules (`nav2` and `navigation`)

`<map>` is resolved using this logic:

- If value contains `/`, or ends with `.yaml` or `.yml`, it is treated as a direct path.
- Otherwise it resolves to `$MAP_DIR/<map>.yaml`.
- The resolved file must already exist.

Examples:

```bash
"$ROS_WS/scripts/bluebot_bringup.sh" start navigation office_a
"$ROS_WS/scripts/bluebot_bringup.sh" start nav2 /data/maps/warehouse.yaml
"$ROS_WS/scripts/bluebot_bringup.sh" restart navigation office_a use_sim_time:=true
```

## Extra Launch Args Passthrough

For `start` and `restart`, any trailing arguments are appended unchanged to `ros2 launch`.

Examples:

```bash
"$ROS_WS/scripts/bluebot_bringup.sh" start mapping apriltag_realsense_enabled:=true
"$ROS_WS/scripts/bluebot_bringup.sh" start navigation office_a apriltag_behavior_enabled:=false
"$ROS_WS/scripts/bluebot_bringup.sh" restart sensors log_level:=debug
```

This allows per-run launch overrides without editing the script.

## `save-map` Behavior

Syntax:

```bash
bluebot_bringup.sh save-map [name]
```

- If `name` is omitted, the script creates `map_YYYYMMDD_HHMMSS`.
- Output base path is `$MAP_DIR/<name>`.
- The script runs:
  - `ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/<name>"`
- It passes saver parameters from env vars:
  - `save_map_timeout`
  - `occupied_thresh_default`
  - `free_thresh_default`
- If `APRILTAG_LANDMARKS_SAVE_ENABLED=true` and `APRILTAG_LANDMARKS_FILE` exists and is non-empty, it also copies:
  - `$MAP_DIR/<name>.apriltags.yaml`

## Runtime State Files and Logs

The script uses these files:

- `/tmp/bluebot_bringup.pid`
- `/tmp/bluebot_bringup.mode`
- `/tmp/bluebot_bringup.cmd`
- `/tmp/bluebot_bringup.log`

What they are used for:

- `pid`: process tracking for `status`, `stop`, and restart guards.
- `mode`: last launched mode label.
- `cmd`: full launch command string.
- `log`: stdout/stderr of the background launch process.

## Environment Variables

| Variable | Default | Used for |
| --- | --- | --- |
| `ROS_WS` | `/ssd/ros2_ws` | Workspace root for `install/setup.bash` |
| `MAP_DIR` | `/ssd/maps` | Map lookup and save destination |
| `SHUTDOWN_TIMEOUT_SEC` | `10` | Graceful stop wait window before forced kill |
| `USE_SIM_TIME` | `false` | Passed to most launch modes |
| `APRILTAG_REALSENSE_ENABLED` | `false` | Mapping/navigation launch arg |
| `APRILTAG_MAP_RECORDER_ENABLED` | `false` | Mapping launch arg |
| `APRILTAG_MAP_OUTPUT_YAML` | `/tmp/apriltag_map_landmarks.yaml` | Mapping recorder output path |
| `APRILTAG_BEHAVIOR_ENABLED` | `true` | Navigation launch arg |
| `MAP_SAVE_TIMEOUT_SEC` | `15.0` | `map_saver_cli` timeout parameter |
| `MAP_OCCUPIED_THRESH` | `0.65` | `map_saver_cli` occupied threshold |
| `MAP_FREE_THRESH` | `0.25` | `map_saver_cli` free threshold |
| `APRILTAG_LANDMARKS_FILE` | `/tmp/apriltag_map_landmarks.yaml` | Source file copied on `save-map` |
| `APRILTAG_LANDMARKS_SAVE_ENABLED` | `true` | Enable/disable landmarks sidecar copy |

## Function-by-Function Breakdown

- `usage()`
  - Prints command syntax, modes, and examples.
- `list_modes()`
  - Prints all supported modes (used by `list-modes` and `modes`).
- `source_ros()`
  - Sources `/opt/ros/$ROS_DISTRO/setup.bash` and `$ROS_WS/install/setup.bash`.
  - Handles `set -u` safely while sourcing.
- `is_running()`
  - Validates PID file and checks process liveness (`kill -0`).
- `cleanup_state_files()`
  - Removes PID/mode/cmd files from `/tmp`.
- `quoted_command()`
  - Shell-quotes command pieces for the launch wrapper.
- `resolve_map_yaml(map_arg)`
  - Resolves `<map>` into an absolute YAML path and validates existence.
- `stop_stack()`
  - Sends `SIGTERM` to process group when available, waits up to `SHUTDOWN_TIMEOUT_SEC`, then sends `SIGKILL` if still alive.
  - Cleans runtime state files.
- `start_launch(mode, launch_cmd...)`
  - Prevents duplicate start if already running.
  - Writes mode/cmd state files.
  - Launches in background (`setsid` when available), logs to `/tmp/bluebot_bringup.log`, and records PID.
- `start_mode(mode, ...)`
  - Dispatches each mode to its launch file.
  - Applies built-in env-driven launch args.
  - Appends any extra passthrough launch args.
- `save_map([name])`
  - Saves occupancy map and optionally saves/copies AprilTag landmarks sidecar.
- `status()`
  - Reports running state, PID, mode, command, and log path.

## Typical Workflows

Start mapping with recorder on:

```bash
APRILTAG_REALSENSE_ENABLED=true \
APRILTAG_MAP_RECORDER_ENABLED=true \
"$ROS_WS/scripts/bluebot_bringup.sh" start mapping
```

Start navigation with AprilTag behavior disabled for a run:

```bash
"$ROS_WS/scripts/bluebot_bringup.sh" start navigation office_a apriltag_behavior_enabled:=false
```

Save map and landmarks:

```bash
"$ROS_WS/scripts/bluebot_bringup.sh" save-map office_a
```

Inspect and stop:

```bash
"$ROS_WS/scripts/bluebot_bringup.sh" status
"$ROS_WS/scripts/bluebot_bringup.sh" stop
```

