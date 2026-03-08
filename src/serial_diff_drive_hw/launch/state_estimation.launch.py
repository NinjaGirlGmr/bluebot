#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("serial_diff_drive_hw")
    params_file = os.path.join(pkg_share, "config", "robot_localization_ekf.yaml")

    odom_topic = LaunchConfiguration("odom_topic")
    imu_orientation_topic = LaunchConfiguration("imu_orientation_topic")
    imu_raw_topic = LaunchConfiguration("imu_raw_topic")
    fused_odom_topic = LaunchConfiguration("fused_odom_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_odom_topic = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odom_raw",
    )
    declare_imu_orientation_topic = DeclareLaunchArgument(
        "imu_orientation_topic",
        default_value="/imu/orientation",
    )
    declare_imu_raw_topic = DeclareLaunchArgument(
        "imu_raw_topic",
        default_value="/imu/data_raw",
    )
    declare_fused_odom_topic = DeclareLaunchArgument(
        "fused_odom_topic",
        default_value="/odom",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
    )

    imu_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_imu_tf",
        arguments=[
            "0", "0", "0", "0", "0", "0",
            "base_link", "imu_link",
        ],
        output="screen",
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="robot_localization_filter",
        output="screen",
        parameters=[params_file, {
            "use_sim_time": use_sim_time,
            "odom0": odom_topic,
            "imu0": imu_orientation_topic,
            "imu1": imu_raw_topic,
        }],
        remappings=[("/odometry/filtered", fused_odom_topic)],
    )

    return LaunchDescription([
        declare_odom_topic,
        declare_imu_orientation_topic,
        declare_imu_raw_topic,
        declare_fused_odom_topic,
        declare_use_sim_time,
        imu_to_base_tf,
        ekf_node,
    ])
