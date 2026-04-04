#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node, SetParameter
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    size = LaunchConfiguration("size")
    max_tags = LaunchConfiguration("max_tags")
    tag_family = LaunchConfiguration("tag_family")
    backends = LaunchConfiguration("backends")
    use_sim_time = LaunchConfiguration("use_sim_time")
    health_monitor_enabled = LaunchConfiguration("health_monitor_enabled")
    health_params_file = LaunchConfiguration("health_params_file")

    config_file = LaunchConfiguration("config_file")
    camera_namespace = LaunchConfiguration("camera_namespace")
    camera_name = LaunchConfiguration("camera_name")
    output = LaunchConfiguration("output")
    color_image_topic = LaunchConfiguration("color_image_topic")
    color_camera_info_topic = LaunchConfiguration("color_camera_info_topic")
    publish_base_to_camera_tf = LaunchConfiguration("publish_base_to_camera_tf")
    camera_parent_frame = LaunchConfiguration("camera_parent_frame")
    camera_base_frame = LaunchConfiguration("camera_base_frame")
    camera_tf_x = LaunchConfiguration("camera_tf_x")
    camera_tf_y = LaunchConfiguration("camera_tf_y")
    camera_tf_z = LaunchConfiguration("camera_tf_z")
    camera_tf_roll = LaunchConfiguration("camera_tf_roll")
    camera_tf_pitch = LaunchConfiguration("camera_tf_pitch")
    camera_tf_yaw = LaunchConfiguration("camera_tf_yaw")

    apriltag_params = PathJoinSubstitution(
        [FindPackageShare("robot_bringup"), "config", "apriltag.yaml"]
    )
    realsense_launch_path = PathJoinSubstitution(
        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
    )

    # Realsense launch script may consume parent launch configurations as ROS params.
    # Isolate it from parent args (map/nav/etc.) to prevent parameter leakage.
    realsense_launch = GroupAction(
        forwarding=False,
        launch_configurations={
            "config_file": config_file,
            "camera_namespace": camera_namespace,
            "camera_name": camera_name,
            "output": output,
        },
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(realsense_launch_path),
            ),
        ],
    )

    set_use_sim_time = SetParameter(
        name="use_sim_time",
        value=ParameterValue(use_sim_time, value_type=bool),
    )

    rectify_node = ComposableNode(
        package="isaac_ros_image_proc",
        plugin="nvidia::isaac_ros::image_proc::RectifyNode",
        name="rectify",
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
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
            apriltag_params,
            {
                "size": ParameterValue(size, value_type=float),
                "max_tags": ParameterValue(max_tags, value_type=int),
                "tag_family": tag_family,
                "backends": backends,
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            },
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

    base_to_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_tf",
        condition=IfCondition(publish_base_to_camera_tf),
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        arguments=[
            "--x",
            camera_tf_x,
            "--y",
            camera_tf_y,
            "--z",
            camera_tf_z,
            "--roll",
            camera_tf_roll,
            "--pitch",
            camera_tf_pitch,
            "--yaw",
            camera_tf_yaw,
            "--frame-id",
            camera_parent_frame,
            "--child-frame-id",
            camera_base_frame,
        ],
        output="screen",
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
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_bringup"), "config", "realsense.yaml"]
                ),
            ),
            DeclareLaunchArgument("camera_namespace", default_value="camera"),
            DeclareLaunchArgument("camera_name", default_value="camera"),
            DeclareLaunchArgument("output", default_value="screen"),
            realsense_launch,
            DeclareLaunchArgument("size", default_value="0.08"),
            DeclareLaunchArgument("max_tags", default_value="24"),
            DeclareLaunchArgument("tag_family", default_value="tag36h11"),
            DeclareLaunchArgument("backends", default_value="CUDA"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("health_monitor_enabled", default_value="true"),
            DeclareLaunchArgument(
                "health_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_bringup"), "config", "apriltag_health_monitor.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "color_image_topic", default_value="/camera/camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "color_camera_info_topic",
                default_value="/camera/camera/color/camera_info",
            ),
            DeclareLaunchArgument("publish_base_to_camera_tf", default_value="true"),
            DeclareLaunchArgument("camera_parent_frame", default_value="base_link"),
            DeclareLaunchArgument("camera_base_frame", default_value="camera_link"),
            DeclareLaunchArgument("camera_tf_x", default_value="0.097"),
            DeclareLaunchArgument("camera_tf_y", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_z", default_value="0.155"),
            DeclareLaunchArgument("camera_tf_roll", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_yaw", default_value="0.0"),
            set_use_sim_time,
            base_to_camera_tf,
            apriltag_container,
            health_monitor,
        ]
    )
