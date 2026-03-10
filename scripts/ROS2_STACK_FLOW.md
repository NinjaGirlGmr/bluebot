# Bluebot ROS2 Stack Flow

This diagram is derived from:
- `/ssd/ros2_ws/scripts/bluebot.sh`
- `/ssd/ros2_ws/src/*` launch/config/node files used by `bluebot.sh`

## 1) Mode Matrix

| Mode | Base Stack | Mapping Job | Nav2 Bringup | Isaac Grid Localizer + Bridge |
|---|---|---|---|---|
| `start` | Yes | No | No | No |
| `start-map` | Yes | Yes | (via `nav2_bringup/slam_launch.py`) | No |
| `start-map-explore` | Yes | Yes | Yes (`bringup_launch.py`, `slam:=True`) | No |
| `start-nav <map>` | Yes | No | Yes (`bringup_launch.py`) | Yes |
| `start` + `NVBLOX_ENABLED=true` | Yes | No | No | No |
| `start-map` + `EXPLORE_LITE_ENABLED=true` | Yes | Yes | (via `nav2_bringup/slam_launch.py`) | No |
| `start-nav <map>` + `EXPLORE_LITE_ENABLED=true` | Yes | No | Yes (`bringup_launch.py`) | Yes |

## 2) Base Stack (all modes)

```mermaid
flowchart LR
%% ---------- STYLE DEFINITIONS ----------
classDef input fill:#cce5ff,stroke:#3399ff,stroke-width:1px,color:#000;
classDef motion fill:#d1ecf1,stroke:#17a2b8,stroke-width:1px,color:#000;
classDef state fill:#fff3cd,stroke:#ffc107,stroke-width:1px,color:#000;
classDef sensors fill:#d4edda,stroke:#28a745,stroke-width:1px,color:#000;
classDef vslam fill:#e2d9f3,stroke:#6f42c1,stroke-width:1px,color:#000;
classDef bridge fill:#f8d7da,stroke:#dc3545,stroke-width:1px,color:#000;

%% ---------- INPUT ----------
subgraph INPUT
Joy["/joy"]:::input
Teleop["teleop_twist_joy"]:::input
end

%% ---------- MOTION / CONTROL ----------
subgraph MOTION
Cmd["/cmd_vel or /cmd_vel_safe"]:::motion
SerialBridge["ros2_serial_diff_drive_bridge<br/>(serial_diff_drive_bridge)"]:::motion
MCU["RobotMotorDriver MCU"]:::motion
end

%% ---------- STATE ----------
subgraph STATE
Odom["/odom"]:::state
Joints["/joint_states"]:::state
TFodom["/tf: odom→base_link"]:::state
RSP["robot_state_publisher"]:::state
TFrobot["/tf robot links"]:::state
end

%% ---------- SENSORS ----------
subgraph SENSORS
Lidar["rplidar_node"]:::sensors
Scan["/scan"]:::sensors
TfLaser["static TF: base_link→laser"]:::sensors
RealSense["realsense2_camera"]:::sensors
Depth["/camera depth"]:::sensors
Infra["/camera infra1 & infra2 + camera_info"]:::sensors
CamImu["camera IMU"]:::sensors
ImuNode["yb_a471_driver imu_node"]:::sensors
end

%% ---------- VSLAM ----------
subgraph VSLAM
Vslam["isaac_ros_visual_slam<br/>(direct or nitros path)"]:::vslam
VslamOdom["/visual_slam/tracking/odometry"]:::vslam
end

%% ---------- BRIDGE ----------
Foxglove["foxglove_bridge :8765"]:::bridge

%% ---------- DATA FLOW WITH LABELS ----------

%% Input → Motion → Hardware
Joy -->|joystick data| Teleop
Teleop -->|cmd_vel| Cmd
Cmd -->|cmd_vel_safe| SerialBridge
SerialBridge -->|motor commands| MCU

%% MCU → State feedback
MCU -->|odom feedback| Odom
MCU -->|joint states| Joints
SerialBridge -->|TF odom→base_link| TFodom
RSP -->|robot TF| TFrobot

%% Sensors → VSLAM
Lidar -->|scan data| Scan
TfLaser -->|laser frame| Scan
RealSense -->|depth frames| Depth
RealSense -->|infra frames + camera_info| Infra
RealSense -->|IMU data| CamImu
Infra -->|infra image data| Vslam
CamImu -->|IMU + camera info| Vslam
Vslam -->|VSLAM odometry| VslamOdom

%% Bridge connections
Cmd -->|teleop commands| Foxglove
Odom -->|odom feedback| Foxglove
Joints -->|joint states| Foxglove
Scan -->|scan data| Foxglove
Depth -->|depth data| Foxglove
VslamOdom -->|VSLAM odometry| Foxglove
```

## 3) Mapping Overlay (`start-map`)

```mermaid
flowchart LR
%% ---------- STYLE DEFINITIONS ----------
classDef sensors fill:#cce5ff,stroke:#3399ff,stroke-width:1px,color:#000;
classDef perception fill:#d4edda,stroke:#28a745,stroke-width:1px,color:#000;
classDef safety fill:#fff3cd,stroke:#ffc107,stroke-width:1px,color:#000;
classDef nav2 fill:#e2d9f3,stroke:#6f42c1,stroke-width:1px,color:#000;
classDef mapping fill:#f8d7da,stroke:#dc3545,stroke-width:1px,color:#000;
classDef hardware fill:#d1ecf1,stroke:#17a2b8,stroke-width:1px,color:#000;
classDef tf fill:#f0f0f0,stroke:#6c757d,stroke-width:1px,color:#000;

%% ---------- SENSORS ----------
subgraph SENSORS
Depth["Depth Camera<br/>(aligned_depth_to_color/image_raw)"]:::sensors
VslamOdom["VSLAM Odometry<br/>(visual_slam/tracking/odometry)"]:::sensors
Scan["Lidar Scan<br/>(/scan)"]:::sensors
IMU["IMU<br/>(/imu/data)"]:::sensors
end

%% ---------- PERCEPTION ----------
subgraph PERCEPTION
DropDetector["Drop Detector Node"]:::perception
DropTopic((" /drop_detected ")):::perception
end

%% ---------- COMMAND INPUT ----------
subgraph COMMAND
CmdIn((" /cmd_vel ")):::safety
end

%% ---------- SAFETY GATE ----------
subgraph SAFETY_CONTROL
Gate["Cmd_vel Safety Gate"]:::safety
CmdSafe((" /cmd_vel_safe ")):::safety
end

%% ---------- NAV2 STACK ----------
subgraph NAV2_STACK
Planner["Nav2 Planner"]:::nav2
Controller["Nav2 Controller"]:::nav2
BehaviorTree["Behavior Tree"]:::nav2
end

%% ---------- MAPPING / SLAM ----------
subgraph MAPPING
SlamTB["SLAM Toolbox"]:::mapping
Map((" /map ")):::mapping
TFmap((" /tf map→odom ")):::mapping
end

%% ---------- ROBOT BASE / HARDWARE ----------
subgraph ROBOT_BASE
SerialBridge["Serial Diff Drive Bridge"]:::hardware
Motors["Motor Controller"]:::hardware
end

%% ---------- TF TREE ----------
subgraph TF_TREE
MapFrame["map"]:::tf
OdomFrame["odom"]:::tf
BaseLink["base_link"]:::tf
DepthFrame["camera_link"]:::tf
ScanFrame["laser_link"]:::tf
end

%% ---------- DATA FLOW WITH LABELS ----------

%% Sensors → Perception
Depth -->|depth frames| DropDetector
VslamOdom -->|VSLAM odometry| DropDetector
DropDetector -->|drop alert| DropTopic

%% Perception & Command → Safety → Hardware
CmdIn -->|raw cmd_vel| Gate
DropTopic -->|drop detected info| Gate
Gate -->|safe cmd_vel| CmdSafe
CmdSafe -->|motor commands| SerialBridge
SerialBridge -->|actuator control| Motors

%% Sensors → SLAM / Mapping
Scan -->|lidar scan data| SlamTB
SlamTB -->|map updates| Map
SlamTB -->|TF updates| TFmap

%% Nav2 stack
Planner -->|plan commands| Controller
Controller -->|velocity commands| CmdSafe
BehaviorTree -->|behavior control| Planner
BehaviorTree -->|behavior feedback| Controller

%% TF hierarchy
MapFrame -->|map frame| OdomFrame
OdomFrame -->|odom frame| BaseLink
BaseLink -->|robot base link| DepthFrame
BaseLink -->|robot base link| ScanFrame
```

## 4) Mapping + Exploration (`start-map-explore`)

```mermaid
flowchart LR
%% ---------- STYLE DEFINITIONS ----------
classDef input fill:#cce5ff,stroke:#3399ff,stroke-width:1px,color:#000;
classDef perception fill:#d4edda,stroke:#28a745,stroke-width:1px,color:#000;
classDef safety fill:#fff3cd,stroke:#ffc107,stroke-width:1px,color:#000;
classDef mapping fill:#f8d7da,stroke:#dc3545,stroke-width:1px,color:#000;
classDef nav fill:#e2d9f3,stroke:#6f42c1,stroke-width:1px,color:#000;
classDef hardware fill:#d1ecf1,stroke:#17a2b8,stroke-width:1px,color:#000;

%% ---------- INPUT / COMMAND ----------
CmdIn["/cmd_vel"]:::input
DropTopic["/drop_detected"]:::perception
CmdSafe["/cmd_vel_safe"]:::safety

%% ---------- PERCEPTION ----------
Depth["/camera/camera/aligned_depth_to_color/image_raw"]:::perception
VslamOdom["/visual_slam/tracking/odometry"]:::perception
Scan["/scan"]:::perception
DropDetector["drop_detector_node"]:::perception
Nvblox["nvblox_node"]:::perception
SLAM["slam_toolbox<br/>(from nav2_bringup/slam_launch.py)"]:::mapping

%% ---------- NAVIGATION ----------
Nav2["nav2_bringup/bringup_launch.py<br/>(slam:=True, nav2 stack)"]:::nav
Explore["Explore Lite"]:::nav
Map["/map"]:::mapping
LocalCM["local_costmap + NvbloxCostmapLayer"]:::mapping

%% ---------- HARDWARE ----------
SerialBridge["serial_diff_drive_bridge<br/>(subscribed to /cmd_vel_safe in map/explore mode)"]:::hardware

%% ---------- DATA FLOW WITH LABELS ----------

%% Command & Safety
CmdIn -->|raw command| Gate["cmd_vel_safety_gate"]:::safety
DropTopic -->|drop alert| Gate
Gate -->|safe command| CmdSafe
CmdSafe -->|cmd_vel_safe| SerialBridge

%% Perception
Depth -->|depth frames| Nvblox
VslamOdom -->|VSLAM odometry| DropDetector
DropDetector -->|drop detected| DropTopic
Scan -->|scan data| SLAM
SLAM -->|map updates| Map

%% Navigation & Mapping
Scan -->|scan data| Nav2
SLAM -->|map info| Nav2
Nvblox -->|3D costmap layer| LocalCM
Map -->|global map| Nav2
Nav2 -->|navigation goals| Explore
```

## 5) Navigation Overlay (`start-nav <map>`)

```mermaid
flowchart LR
%% ---------- STYLE DEFINITIONS ----------
classDef nav2 fill:#e2d9f3,stroke:#6f42c1,stroke-width:1px,color:#000;
classDef initialPose fill:#d4edda,stroke:#28a745,stroke-width:1px,color:#000;
classDef waypoints fill:#cce5ff,stroke:#3399ff,stroke-width:1px,color:#000;
classDef topic fill:#fff3cd,stroke:#ffc107,stroke-width:1px,color:#000;
classDef service fill:#f8d7da,stroke:#dc3545,stroke-width:1px,color:#000;

%% ---------- NAV2 SUBSYSTEM ----------
subgraph NAV2
MapFile["map.yaml"]:::topic
Nav2Node["nav2_bringup/bringup_launch.py<br/>(AMCL + Planner + Controller + BT Navigator + Costmaps)"]:::nav2
Goal["/goal_pose"]:::topic
InitialPose["/initialpose"]:::topic
Scan["/scan"]:::topic
Odom["/odom"]:::topic
TFmap["/tf: map→odom (AMCL)"]:::topic
end

%% ---------- ISAAC INITIAL POSE ----------
subgraph ISAAC_INITIAL_POSE
L2F["LaserScan to FlatScan Node"]:::initialPose
OGL["OccupancyGridLocalizer Node"]:::initialPose
Service["/trigger_grid_search_localization"]:::service
LocResult["/localization_result"]:::topic
Bridge["isaac_nav2_pose_bridge<br/>(isaac_to_nav2_pose)"]:::initialPose
end

%% ---------- WAYPOINTS ----------
subgraph WAYPOINTS
PoseArray["/foxglove/waypoints"]:::waypoints
WPBridge["foxglove_waypoint_bridge"]:::waypoints
NavThroughPoses["/navigate_through_poses action"]:::waypoints
Status["/foxglove/waypoints/status"]:::topic
end

%% ---------- DATA FLOW ----------

%% Nav2 inputs
MapFile -->|map file| Nav2Node
Goal -->|goal pose| Nav2Node
Scan -->|scan data| Nav2Node
Odom -->|odometry| Nav2Node
Nav2Node -->|map->odom TF| TFmap

%% Isaac initial pose flow
Scan -->|scan data| L2F
L2F -->|flat scan| OGL
Service -.->|trigger localization| OGL
OGL -->|localization result| LocResult
LocResult -->|pose msg| Bridge
Bridge -->|initialpose| InitialPose
InitialPose -->|initial pose input| Nav2Node

%% Waypoints flow
PoseArray -->|waypoints| WPBridge
WPBridge -->|waypoint action| NavThroughPoses
NavThroughPoses -->|goal poses| Nav2Node
WPBridge -->|status updates| Status
```

## 6) Utility Flows

```mermaid
flowchart LR
%% ---------- STYLE DEFINITIONS ----------
classDef tf fill:#f0f0f0,stroke:#6c757d,stroke-width:1px,color:#000;
classDef capture fill:#d4edda,stroke:#28a745,stroke-width:1px,color:#000;
classDef fileops fill:#cce5ff,stroke:#3399ff,stroke-width:1px,color:#000;
classDef topics fill:#fff3cd,stroke:#ffc107,stroke-width:1px,color:#000;

%% ---------- TF ----------
TF["/tf map→base_link"]:::tf

%% ---------- WAYPOINT CAPTURE ----------
Capture["bluebot.sh capture-waypoint"]:::capture
YAML["/ssd/maps/waypoints.yaml"]:::fileops
Send["bluebot.sh send-waypoints"]:::capture
PoseArray["/foxglove/waypoints"]:::topics

%% ---------- MAP SAVE ----------
Map["/map"]:::topics
SaveMap["bluebot.sh save-map"]:::capture
MapFiles["/ssd/maps/<name>.yaml + .pgm"]:::fileops

%% ---------- DATA FLOW ----------
TF -->|robot pose TF| Capture
Capture -->|capture waypoint| YAML
YAML -->|waypoints file| Send
Send -->|waypoint messages| PoseArray

Map -->|map data| SaveMap
SaveMap -->|map files| MapFiles
```

## 6) Frame Ownership Summary

- `odom -> base_link`: `ros2_serial_diff_drive_bridge` (from wheel telemetry)
- `map -> odom` in mapping: `slam_toolbox`
- `map -> odom` in navigation: `AMCL` (Nav2)
- `base_link -> laser`: `lidar_with_tf.launch.py` static TF
- `base_link -> camera_link`: `bluebot.sh` static TF
- `isaac_ros_visual_slam` is configured with `publish_odom_to_base_tf=false` and `publish_map_to_odom_tf=false` to avoid TF conflicts.
