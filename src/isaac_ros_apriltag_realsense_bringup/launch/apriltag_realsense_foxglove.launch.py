#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    size = LaunchConfiguration("size")
    max_tags = LaunchConfiguration("max_tags")
    tag_family = LaunchConfiguration("tag_family")
    backends = LaunchConfiguration("backends")

    camera_namespace = LaunchConfiguration("camera_namespace")
    camera_name = LaunchConfiguration("camera_name")
    color_profile = LaunchConfiguration("color_profile")
    color_format = LaunchConfiguration("color_format")
    output = LaunchConfiguration("output")
    color_image_topic = LaunchConfiguration("color_image_topic")
    color_camera_info_topic = LaunchConfiguration("color_camera_info_topic")

    foxglove_bridge_enabled = LaunchConfiguration("foxglove_bridge_enabled")
    foxglove_bridge_port = LaunchConfiguration("foxglove_bridge_port")
    foxglove_bridge_address = LaunchConfiguration("foxglove_bridge_address")
    foxglove_bridge_capabilities = LaunchConfiguration("foxglove_bridge_capabilities")

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])
        ),
        launch_arguments={
            "camera_namespace": camera_namespace,
            "camera_name": camera_name,
            "enable_color": "true",
            "enable_depth": "false",
            "enable_infra": "false",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "enable_gyro": "false",
            "enable_accel": "false",
            "pointcloud.enable": "false",
            "align_depth.enable": "false",
            "rgb_camera.color_profile": color_profile,
            "rgb_camera.color_format": color_format,
            "output": output,
        }.items(),
    )

    rectify_node = ComposableNode(
        package="isaac_ros_image_proc",
        plugin="nvidia::isaac_ros::image_proc::RectifyNode",
        name="rectify",
        remappings=[
            ("image_raw", color_image_topic),
            ("camera_info", color_camera_info_topic),
            ("image_rect", "image_rect"),
            ("camera_info_rect", "camera_info_rect"),
        ],
    )

    apriltag_node = ComposableNode(
        package="isaac_ros_apriltag",
        plugin="nvidia::isaac_ros::apriltag::AprilTagNode",
        name="apriltag",
        parameters=[
            {
                "size": ParameterValue(size, value_type=float),
                "max_tags": ParameterValue(max_tags, value_type=int),
                "tag_family": tag_family,
                "backends": backends,
            }
        ],
        remappings=[
            ("image", "image_rect"),
            ("camera_info", "camera_info_rect"),
        ],
    )

    apriltag_container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        name="apriltag_container",
        namespace="",
        composable_node_descriptions=[rectify_node, apriltag_node],
        output="screen",
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("size", default_value="0.08"),
            DeclareLaunchArgument("max_tags", default_value="24"),
            DeclareLaunchArgument("tag_family", default_value="tag36h11"),
            DeclareLaunchArgument("backends", default_value="CUDA"),
            DeclareLaunchArgument("camera_namespace", default_value="camera"),
            DeclareLaunchArgument("camera_name", default_value="camera"),
            DeclareLaunchArgument("color_profile", default_value="1280,720,15"),
            DeclareLaunchArgument("color_format", default_value="RGB8"),
            DeclareLaunchArgument("output", default_value="screen"),
            DeclareLaunchArgument("color_image_topic", default_value="/camera/camera/color/image_raw"),
            DeclareLaunchArgument(
                "color_camera_info_topic", default_value="/camera/camera/color/camera_info"
            ),
            DeclareLaunchArgument("foxglove_bridge_enabled", default_value="true"),
            DeclareLaunchArgument("foxglove_bridge_port", default_value="8765"),
            DeclareLaunchArgument("foxglove_bridge_address", default_value="0.0.0.0"),
            DeclareLaunchArgument(
                "foxglove_bridge_capabilities",
                default_value="[clientPublish,services,connectionGraph,assets]",
            ),
            realsense_launch,
            apriltag_container,
            foxglove_bridge_launch,
        ]
    )
