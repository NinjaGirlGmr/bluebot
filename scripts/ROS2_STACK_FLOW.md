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
  subgraph Input
    Joy[/joy]
    Teleop[teleop_twist_joy]
  end

  subgraph Motion
    Cmd((/cmd_vel or /cmd_vel_safe))
    SerialBridge[ros2_serial_diff_drive_bridge\nserial_diff_drive_bridge]
    MCU[(RobotMotorDriver MCU)]
  end

  subgraph State
    Odom[/odom]
    Joints[/joint_states]
    TFodom[/tf: odom->base_link]
    RSP[robot_state_publisher]
    TFrobot[/tf robot links]
  end

  subgraph Sensors
    Lidar[rplidar_node]
    Scan[/scan]
    TfLaser[static TF\nbase_link->laser]
    RealSense[realsense2_camera]
    Depth[/camera depth]
    Infra[/camera infra1/infra2 + camera_info]
    CamImu[/camera imu]
    ImuNode[yb_a471_driver imu_node]
  end

  subgraph VSLAM
    Vslam[isaac_ros_visual_slam\n(direct or nitros path)]
    VslamOdom[/visual_slam/tracking/odometry]
  end

  Foxglove[foxglove_bridge :8765]

  Joy --> Teleop --> Cmd --> SerialBridge --> MCU
  MCU --> SerialBridge --> Odom
  MCU --> SerialBridge --> Joints
  SerialBridge --> TFodom
  RSP --> TFrobot
  Lidar --> Scan
  TfLaser --> Scan
  RealSense --> Depth
  RealSense --> Infra
  RealSense --> CamImu
  Infra --> Vslam
  CamImu --> Vslam
  Vslam --> VslamOdom
```

## 3) Mapping Overlay (`start-map`)

```mermaid
flowchart LR
  CmdIn[/cmd_vel]
  DropTopic[/drop_detected]
  CmdSafe[/cmd_vel_safe]
  Depth[/camera/camera/aligned_depth_to_color/image_raw]
  VslamOdom[/visual_slam/tracking/odometry]
  Scan[/scan]

  DropDetector[drop_detector_node]
  Gate[cmd_vel_safety_gate]
  SlamTB[slam_toolbox\n(from nav2_bringup/slam_launch.py)]
  Map[/map]
  SerialBridge[serial_diff_drive_bridge\n(subscribed to /cmd_vel_safe in map mode)]

  Depth --> DropDetector
  VslamOdom --> DropDetector
  DropDetector --> DropTopic

  CmdIn --> Gate
  DropTopic --> Gate
  Gate --> CmdSafe --> SerialBridge

  Scan --> SlamTB
  SlamTB --> Map
  SlamTB --> TFmap[/tf: map->odom]
```

## 4) Mapping + Exploration (`start-map-explore`)

```mermaid
flowchart LR
  CmdIn[/cmd_vel]
  DropTopic[/drop_detected]
  CmdSafe[/cmd_vel_safe]
  Depth[/camera/camera/aligned_depth_to_color/image_raw]
  VslamOdom[/visual_slam/tracking/odometry]
  Scan[/scan]

  DropDetector[drop_detector_node]
  Gate[cmd_vel_safety_gate]
  Nvblox[nvblox_node]
  SLAM[slam_toolbox\n(from nav2_bringup/slam_launch.py)]
  Nav2[nav2_bringup/bringup_launch.py\n(slam:=True, nav2 stack)]
  Explore[Explore Lite]
  Map[/map]
  SerialBridge[serial_diff_drive_bridge\n(subscribed to /cmd_vel_safe in map explore mode)]
  LocalCM[local_costmap + NvbloxCostmapLayer]

  CmdIn --> Gate
  DropTopic --> Gate
  Gate --> CmdSafe --> SerialBridge

  DropDetector --> DropTopic
  Depth --> Nvblox
  VslamOdom --> DropDetector
  Scan --> SLAM --> Map
  Scan --> Nav2
  SLAM --> Nav2
  Nvblox --> LocalCM
  Map --> Nav2
  Nav2 --> Explore
```

## 5) Navigation Overlay (`start-nav <map>`)

```mermaid
flowchart LR
  subgraph Nav2
    MapFile[(map.yaml)]
    Nav2[nav2_bringup/bringup_launch.py\nAMCL + planner + controller + BT navigator + costmaps]
    Goal[/goal_pose]
    InitialPose[/initialpose]
    Scan[/scan]
    Odom[/odom]
    TFmap[/tf: map->odom (AMCL)]
  end

  subgraph IsaacInitialPose
    L2F[LaserScantoFlatScanNode]
    OGL[OccupancyGridLocalizerNode]
    Service[/trigger_grid_search_localization]
    LocResult[/localization_result]
    Bridge[isaac_nav2_pose_bridge\nisaac_to_nav2_pose]
  end

  subgraph Waypoints
    PoseArray[/foxglove/waypoints]
    WPBridge[foxglove_waypoint_bridge]
    NavThroughPoses[[/navigate_through_poses action]]
    Status[/foxglove/waypoints/status]
  end

  MapFile --> Nav2
  Goal --> Nav2
  Scan --> Nav2
  Odom --> Nav2
  Nav2 --> TFmap

  Scan --> L2F --> OGL
  Service -. bluebot.sh auto-call .-> OGL
  OGL --> LocResult --> Bridge --> InitialPose --> Nav2

  PoseArray --> WPBridge --> NavThroughPoses --> Nav2
  WPBridge --> Status
```

## 6) Utility Flows

```mermaid
flowchart LR
  TF[/tf map->base_link]
  Capture[bluebot.sh capture-waypoint]
  YAML[/ssd/maps/waypoints.yaml]
  Send[bluebot.sh send-waypoints]
  PoseArray[/foxglove/waypoints]
  SaveMap[bluebot.sh save-map]
  Map[/map]
  MapFiles[/ssd/maps/<name>.yaml + .pgm]

  TF --> Capture --> YAML
  YAML --> Send --> PoseArray
  Map --> SaveMap --> MapFiles
```

## 6) Frame Ownership Summary

- `odom -> base_link`: `ros2_serial_diff_drive_bridge` (from wheel telemetry)
- `map -> odom` in mapping: `slam_toolbox`
- `map -> odom` in navigation: `AMCL` (Nav2)
- `base_link -> laser`: `lidar_with_tf.launch.py` static TF
- `base_link -> camera_link`: `bluebot.sh` static TF
- `isaac_ros_visual_slam` is configured with `publish_odom_to_base_tf=false` and `publish_map_to_odom_tf=false` to avoid TF conflicts.
