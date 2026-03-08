#!/usr/bin/env python3

"""Launch nvblox using the existing RealSense topics from this robot stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    nvblox_cfg = get_package_share_directory("nvblox_examples_bringup")
    bridge_cfg = get_package_share_directory("serial_diff_drive_hw")

    base_params = os.path.join(
        nvblox_cfg, "config", "nvblox", "nvblox_base.yaml")
    realsense_params = os.path.join(
        nvblox_cfg, "config", "nvblox", "specializations", "nvblox_realsense.yaml")
    bridge_params = os.path.join(
        bridge_cfg, "config", "nvblox_bridge.yaml")

    container_name = LaunchConfiguration("container_name")
    depth_topic = LaunchConfiguration("depth_topic")
    depth_camera_info_topic = LaunchConfiguration("depth_camera_info_topic")
    color_topic = LaunchConfiguration("color_topic")
    color_camera_info_topic = LaunchConfiguration("color_camera_info_topic")

    declare_container_name = DeclareLaunchArgument(
        "container_name", default_value="nvblox_container"
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false")
    declare_depth_topic = DeclareLaunchArgument(
        "depth_topic", default_value="/camera/camera/aligned_depth_to_color/image_raw"
    )
    declare_depth_camera_info_topic = DeclareLaunchArgument(
        "depth_camera_info_topic", default_value="/camera/camera/aligned_depth_to_color/camera_info"
    )
    declare_color_topic = DeclareLaunchArgument(
        "color_topic", default_value="/camera/camera/color/image_raw"
    )
    declare_color_camera_info_topic = DeclareLaunchArgument(
        "color_camera_info_topic", default_value="/camera/camera/color/camera_info"
    )

    nvblox_node = ComposableNode(
        name="nvblox_node",
        package="nvblox_ros",
        plugin="nvblox::NvbloxNode",
        remappings=[
            ("camera_0/depth/image", depth_topic),
            ("camera_0/depth/camera_info", depth_camera_info_topic),
            ("camera_0/color/image", color_topic),
            ("camera_0/color/camera_info", color_camera_info_topic),
        ],
        parameters=[
            base_params,
            realsense_params,
            bridge_params,
            {"use_tf_transforms": True},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    container = ComposableNodeContainer(
        name=container_name,
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[nvblox_node],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        declare_container_name,
        declare_use_sim_time,
        declare_depth_topic,
        declare_depth_camera_info_topic,
        declare_color_topic,
        declare_color_camera_info_topic,
        container,
    ])
