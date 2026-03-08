#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("serial_diff_drive_hw")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    default_params = os.path.join(pkg_share, "config", "nav2_mapping_params.yaml")
    slam_launch_file = os.path.join(nav2_bringup_share, "launch", "slam_launch.py")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    depth_topic = LaunchConfiguration("depth_topic")
    vslam_odom_topic = LaunchConfiguration("vslam_odom_topic")
    input_cmd_topic = LaunchConfiguration("input_cmd_topic")
    output_cmd_topic = LaunchConfiguration("output_cmd_topic")
    drop_topic = LaunchConfiguration("drop_topic")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    declare_params_file = DeclareLaunchArgument("params_file", default_value=default_params)
    declare_depth_topic = DeclareLaunchArgument(
        "depth_topic", default_value="/camera/camera/aligned_depth_to_color/image_raw"
    )
    declare_vslam_odom_topic = DeclareLaunchArgument(
        "vslam_odom_topic", default_value="/visual_slam/tracking/odometry"
    )
    declare_input_cmd_topic = DeclareLaunchArgument("input_cmd_topic", default_value="/cmd_vel")
    declare_output_cmd_topic = DeclareLaunchArgument(
        "output_cmd_topic", default_value="/cmd_vel_safe"
    )
    declare_drop_topic = DeclareLaunchArgument("drop_topic", default_value="/drop_detected")

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": "True",
        }.items(),
    )

    drop_detector = Node(
        package="serial_diff_drive_hw",
        executable="drop_detector_node",
        name="drop_detector",
        output="screen",
        parameters=[
            {
                "depth_topic": depth_topic,
                "odometry_topic": vslam_odom_topic,
                "drop_topic": drop_topic,
            }
        ],
    )

    cmd_gate = Node(
        package="serial_diff_drive_hw",
        executable="cmd_vel_safety_gate",
        name="cmd_vel_safety_gate",
        output="screen",
        parameters=[
            {
                "input_cmd_topic": input_cmd_topic,
                "output_cmd_topic": output_cmd_topic,
                "drop_topic": drop_topic,
            }
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_params_file,
            declare_depth_topic,
            declare_vslam_odom_topic,
            declare_input_cmd_topic,
            declare_output_cmd_topic,
            declare_drop_topic,
            slam_launch,
            drop_detector,
            cmd_gate,
        ]
    )
