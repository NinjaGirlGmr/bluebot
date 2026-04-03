#!/usr/bin/env python3

from pathlib import Path
from typing import Any

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _static_tf_node(name: str, tf_cfg: dict[str, Any], use_sim_time: LaunchConfiguration) -> Node:
    parent_frame = str(tf_cfg.get("parent_frame", "base_link"))
    child_frame = str(tf_cfg.get("child_frame", "sensor_link"))

    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x",
            str(tf_cfg.get("x", 0.0)),
            "--y",
            str(tf_cfg.get("y", 0.0)),
            "--z",
            str(tf_cfg.get("z", 0.0)),
            "--roll",
            str(tf_cfg.get("roll", 0.0)),
            "--pitch",
            str(tf_cfg.get("pitch", 0.0)),
            "--yaw",
            str(tf_cfg.get("yaw", 0.0)),
            "--frame-id",
            parent_frame,
            "--child-frame-id",
            child_frame,
        ],
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
        output="screen",
    )


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _build_tf_nodes(context, *_args, **_kwargs):
    tf_params_file = LaunchConfiguration("sensor_tf_params_file").perform(context)
    tf_file_path = Path(tf_params_file)
    if not tf_file_path.is_file():
        raise RuntimeError(f"sensor_tf_params_file not found: {tf_params_file}")

    with tf_file_path.open("r", encoding="utf-8") as f:
        tf_config = yaml.safe_load(f) or {}

    lidar_tf = tf_config.get("lidar_tf", {})
    imu_tf = tf_config.get("imu_tf", {})
    publish_lidar_tf = _as_bool(LaunchConfiguration("publish_lidar_tf").perform(context))
    publish_imu_tf = _as_bool(LaunchConfiguration("publish_imu_tf").perform(context))
    use_sim_time = LaunchConfiguration("use_sim_time")

    tf_nodes = []
    if publish_lidar_tf:
        tf_nodes.append(_static_tf_node("base_to_lidar_tf", lidar_tf, use_sim_time))
    if publish_imu_tf:
        tf_nodes.append(_static_tf_node("base_to_imu_tf", imu_tf, use_sim_time))
    return tf_nodes


def generate_launch_description() -> LaunchDescription:
    lidar_params_file = LaunchConfiguration("lidar_params_file")
    imu_serial_params_file = LaunchConfiguration("imu_serial_params_file")
    imu_bridge_params_file = LaunchConfiguration("imu_bridge_params_file")
    sensor_tf_params_file = LaunchConfiguration("sensor_tf_params_file")
    health_monitor_enabled = LaunchConfiguration("health_monitor_enabled")
    health_params_file = LaunchConfiguration("health_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    set_use_sim_time = SetParameter(
        name="use_sim_time",
        value=ParameterValue(use_sim_time, value_type=bool),
    )

    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        output="screen",
        parameters=[
            lidar_params_file,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    imu_serial_node = Node(
        package="yb_a471_driver",
        executable="a471_serial_node",
        name="a471_serial_node",
        output="screen",
        parameters=[
            imu_serial_params_file,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    imu_bridge_node = Node(
        package="yb_a471_driver",
        executable="imu_node",
        name="imu_node",
        output="screen",
        parameters=[
            imu_bridge_params_file,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    health_monitor = Node(
        package="robot_bringup",
        executable="health_monitor",
        name="health_monitor",
        output="screen",
        condition=IfCondition(health_monitor_enabled),
        parameters=[
            health_params_file,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_bringup"), "config", "lidar.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "imu_serial_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_bringup"), "config", "imu_serial.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "imu_bridge_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_bringup"), "config", "imu_bridge.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "sensor_tf_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_bringup"), "config", "sensor_transforms.yaml"]
                ),
            ),
            DeclareLaunchArgument("publish_lidar_tf", default_value="true"),
            DeclareLaunchArgument("publish_imu_tf", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("health_monitor_enabled", default_value="true"),
            DeclareLaunchArgument(
                "health_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_bringup"), "config", "sensors_health_monitor.yaml"]
                ),
            ),
            set_use_sim_time,
            lidar_node,
            imu_serial_node,
            imu_bridge_node,
            OpaqueFunction(function=_build_tf_nodes),
            health_monitor,
        ]
    )
