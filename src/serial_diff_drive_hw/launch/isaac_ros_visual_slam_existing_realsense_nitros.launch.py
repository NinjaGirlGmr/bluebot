#!/usr/bin/env python3

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Run Isaac ROS Visual SLAM with NITROS image-format conversion in-container."""
    image_format_converter_left = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_node_left',
        parameters=[{
            'encoding_desired': 'mono8',
            'image_width': 640,
            'image_height': 480,
        }],
        remappings=[
            ('image_raw', '/camera/camera/infra1/image_rect_raw'),
            ('image', '/visual_slam/left/image_rect_mono'),
        ],
    )

    image_format_converter_right = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_node_right',
        parameters=[{
            'encoding_desired': 'mono8',
            'image_width': 640,
            'image_height': 480,
        }],
        remappings=[
            ('image_raw', '/camera/camera/infra2/image_rect_raw'),
            ('image', '/visual_slam/right/image_rect_mono'),
        ],
    )

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'enable_image_denoising': False,
            'rectified_images': True,
            'enable_imu_fusion': True,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            # Allow moderate camera jitter to reduce noisy warnings on Jetson.
            'image_jitter_threshold_ms': 120.0,
            'base_frame': 'base_link',
            'imu_frame': 'camera_gyro_optical_frame',
            # Avoid TF conflicts with wheel odometry and Nav2/SLAM map localization.
            'publish_odom_to_base_tf': False,
            'publish_map_to_odom_tf': False,
            'enable_slam_visualization': False,
            'enable_landmarks_view': False,
            'enable_observations_view': False,
            'camera_optical_frames': [
                'camera_infra1_optical_frame',
                'camera_infra2_optical_frame',
            ],
        }],
        remappings=[
            ('visual_slam/image_0', '/visual_slam/left/image_rect_mono'),
            ('visual_slam/camera_info_0', '/camera/camera/infra1/camera_info'),
            ('visual_slam/image_1', '/visual_slam/right/image_rect_mono'),
            ('visual_slam/camera_info_1', '/camera/camera/infra2/camera_info'),
            ('visual_slam/imu', '/camera/camera/imu'),
        ],
    )

    container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            image_format_converter_left,
            image_format_converter_right,
            visual_slam_node,
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
