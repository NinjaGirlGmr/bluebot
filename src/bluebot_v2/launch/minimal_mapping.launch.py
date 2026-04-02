#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    arduino_port = LaunchConfiguration("arduino_port")
    arduino_baud = LaunchConfiguration("arduino_baud")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_frame = LaunchConfiguration("lidar_frame")
    base_frame = LaunchConfiguration("base_frame")
    scan_topic = LaunchConfiguration("scan_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    linear_speed = LaunchConfiguration("linear_speed")
    angular_speed = LaunchConfiguration("angular_speed")
    default_command_duration_sec = LaunchConfiguration("default_command_duration_sec")
    command_topic = LaunchConfiguration("command_topic")
    slam_params_file = LaunchConfiguration("slam_params_file")
    lidar_scan_frequency = LaunchConfiguration("lidar_scan_frequency")
    lidar_angle_compensate = LaunchConfiguration("lidar_angle_compensate")
    foxglove_bridge_enabled = LaunchConfiguration("foxglove_bridge_enabled")
    foxglove_bridge_port = LaunchConfiguration("foxglove_bridge_port")
    foxglove_bridge_address = LaunchConfiguration("foxglove_bridge_address")
    foxglove_bridge_capabilities = LaunchConfiguration("foxglove_bridge_capabilities")

    default_slam_params = PathJoinSubstitution(
        [FindPackageShare("bluebot_v2"), "config", "slam_toolbox_mapping.yaml"]
    )

    bridge_node = Node(
        package="ros2_serial_diff_drive_bridge",
        executable="serial_diff_drive_bridge",
        name="serial_diff_drive_bridge",
        output="screen",
        parameters=[
            {
                "port": arduino_port,
                "baud": ParameterValue(arduino_baud, value_type=int),
                "cmd_vel_topic": cmd_vel_topic,
                "odom_topic": odom_topic,
                "publish_tf": ParameterValue(publish_odom_tf, value_type=bool),
                "base_frame_id": base_frame,
                "odom_frame_id": "odom",
            }
        ],
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("lidar_launch"), "launch", "lidar_with_tf.launch.py"])
        ),
        launch_arguments={
            "serial_port": lidar_port,
            "frame_id": lidar_frame,
            "parent_frame": base_frame,
            "scan_frequency": lidar_scan_frequency,
            "angle_compensate": lidar_angle_compensate,
        }.items(),
    )

    foxglove_bridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml"])]
        ),
        condition=IfCondition(foxglove_bridge_enabled),
        launch_arguments={
            "port": foxglove_bridge_port,
            "address": foxglove_bridge_address,
            "capabilities": foxglove_bridge_capabilities,
        }.items(),
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_file,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "scan_topic": scan_topic,
                "base_frame": base_frame,
                "odom_frame": "odom",
            },
        ],
    )

    drive_command_node = Node(
        package="bluebot_v2",
        executable="drive_command_node",
        name="drive_command_node",
        output="screen",
        parameters=[
            {
                "command_topic": command_topic,
                "cmd_vel_topic": cmd_vel_topic,
                "linear_speed": ParameterValue(linear_speed, value_type=float),
                "angular_speed": ParameterValue(angular_speed, value_type=float),
                "default_duration_sec": ParameterValue(default_command_duration_sec, value_type=float),
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("arduino_port", default_value="/dev/arduino"),
            DeclareLaunchArgument("arduino_baud", default_value="115200"),
            DeclareLaunchArgument("lidar_port", default_value="/dev/lidar"),
            DeclareLaunchArgument("lidar_frame", default_value="laser"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            DeclareLaunchArgument("linear_speed", default_value="0.20"),
            DeclareLaunchArgument("angular_speed", default_value="1.00"),
            DeclareLaunchArgument("default_command_duration_sec", default_value="0.0"),
            DeclareLaunchArgument("command_topic", default_value="/bluebot_v2/drive_command"),
            DeclareLaunchArgument("slam_params_file", default_value=default_slam_params),
            DeclareLaunchArgument("lidar_scan_frequency", default_value="6.0"),
            DeclareLaunchArgument("lidar_angle_compensate", default_value="false"),
            DeclareLaunchArgument("foxglove_bridge_enabled", default_value="true"),
            DeclareLaunchArgument("foxglove_bridge_port", default_value="8765"),
            DeclareLaunchArgument("foxglove_bridge_address", default_value="0.0.0.0"),
            DeclareLaunchArgument(
                "foxglove_bridge_capabilities",
                default_value="[clientPublish,services,connectionGraph,assets]",
            ),
            bridge_node,
            lidar_launch,
            foxglove_bridge_launch,
            slam_node,
            drive_command_node,
            rviz_node,
        ]
    )
