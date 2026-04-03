#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')

    sensors_enabled = LaunchConfiguration('sensors_enabled')
    lidar_params_file = LaunchConfiguration('lidar_params_file')
    imu_serial_params_file = LaunchConfiguration('imu_serial_params_file')
    imu_bridge_params_file = LaunchConfiguration('imu_bridge_params_file')
    sensor_tf_params_file = LaunchConfiguration('sensor_tf_params_file')
    sensor_publish_lidar_tf = LaunchConfiguration('sensor_publish_lidar_tf')
    sensor_publish_imu_tf = LaunchConfiguration('sensor_publish_imu_tf')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    drive_stack_enabled = LaunchConfiguration('drive_stack_enabled')
    serial_bridge_params_file = LaunchConfiguration('serial_bridge_params_file')
    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baud')
    serial_cmd_vel_topic = LaunchConfiguration('serial_cmd_vel_topic')
    serial_odom_topic = LaunchConfiguration('serial_odom_topic')
    serial_publish_tf = LaunchConfiguration('serial_publish_tf')
    state_estimation_imu_orientation_topic = LaunchConfiguration(
        'state_estimation_imu_orientation_topic'
    )
    state_estimation_imu_raw_topic = LaunchConfiguration(
        'state_estimation_imu_raw_topic'
    )
    state_estimation_fused_odom_topic = LaunchConfiguration(
        'state_estimation_fused_odom_topic'
    )
    state_estimation_params_file = LaunchConfiguration('state_estimation_params_file')
    state_estimation_base_frame = LaunchConfiguration('state_estimation_base_frame')
    state_estimation_imu_frame = LaunchConfiguration('state_estimation_imu_frame')

    straight_comp_params_file = LaunchConfiguration('straight_comp_params_file')
    straight_comp_enabled = LaunchConfiguration('straight_comp_enabled')
    straight_comp_input_cmd_topic = LaunchConfiguration('straight_comp_input_cmd_topic')
    straight_comp_output_cmd_topic = LaunchConfiguration('straight_comp_output_cmd_topic')
    straight_comp_odom_topic = LaunchConfiguration('straight_comp_odom_topic')

    udp_teleop_enabled = LaunchConfiguration('udp_teleop_enabled')
    udp_params_file = LaunchConfiguration('udp_params_file')
    udp_listen_host = LaunchConfiguration('udp_listen_host')
    udp_listen_port = LaunchConfiguration('udp_listen_port')
    udp_cmd_vel_topic = LaunchConfiguration('udp_cmd_vel_topic')
    udp_timeout_s = LaunchConfiguration('udp_timeout_s')

    health_monitor_enabled = LaunchConfiguration('health_monitor_enabled')
    health_params_file = LaunchConfiguration('health_params_file')

    apriltag_realsense_enabled = LaunchConfiguration('apriltag_realsense_enabled')
    apriltag_behavior_enabled = LaunchConfiguration('apriltag_behavior_enabled')
    apriltag_behavior_params_file = LaunchConfiguration('apriltag_behavior_params_file')
    apriltag_behavior_rules_file = LaunchConfiguration('apriltag_behavior_rules_file')
    apriltag_landmarks_file = LaunchConfiguration('apriltag_landmarks_file')
    docking_params_file = LaunchConfiguration('docking_params_file')

    set_use_sim_time = SetParameter(
        name='use_sim_time',
        value=ParameterValue(use_sim_time, value_type=bool),
    )

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('robot_bringup'), 'launch', 'sensors.launch.py'])
        ),
        condition=IfCondition(sensors_enabled),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'lidar_params_file': lidar_params_file,
            'imu_serial_params_file': imu_serial_params_file,
            'imu_bridge_params_file': imu_bridge_params_file,
            'sensor_tf_params_file': sensor_tf_params_file,
            'publish_lidar_tf': sensor_publish_lidar_tf,
            'publish_imu_tf': sensor_publish_imu_tf,
            'health_monitor_enabled': 'false',
        }.items(),
    )

    apriltag_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('robot_bringup'), 'launch', 'apriltag_realsense.launch.py']
            )
        ),
        condition=IfCondition(apriltag_realsense_enabled),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'health_monitor_enabled': 'false',
        }.items(),
    )

    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('robot_bringup'), 'launch', 'nav2.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'log_level': log_level,
            'drive_stack_enabled': drive_stack_enabled,
            'serial_bridge_params_file': serial_bridge_params_file,
            'serial_port': serial_port,
            'serial_baud': serial_baud,
            'serial_cmd_vel_topic': serial_cmd_vel_topic,
            'serial_odom_topic': serial_odom_topic,
            'serial_publish_tf': serial_publish_tf,
            'state_estimation_imu_orientation_topic': state_estimation_imu_orientation_topic,
            'state_estimation_imu_raw_topic': state_estimation_imu_raw_topic,
            'state_estimation_fused_odom_topic': state_estimation_fused_odom_topic,
            'state_estimation_params_file': state_estimation_params_file,
            'state_estimation_base_frame': state_estimation_base_frame,
            'state_estimation_imu_frame': state_estimation_imu_frame,
            'straight_comp_params_file': straight_comp_params_file,
            'straight_comp_enabled': straight_comp_enabled,
            'straight_comp_input_cmd_topic': straight_comp_input_cmd_topic,
            'straight_comp_output_cmd_topic': straight_comp_output_cmd_topic,
            'straight_comp_odom_topic': straight_comp_odom_topic,
            'udp_teleop_enabled': udp_teleop_enabled,
            'udp_params_file': udp_params_file,
            'udp_listen_host': udp_listen_host,
            'udp_listen_port': udp_listen_port,
            'udp_cmd_vel_topic': udp_cmd_vel_topic,
            'udp_timeout_s': udp_timeout_s,
            'health_monitor_enabled': health_monitor_enabled,
            'health_params_file': health_params_file,
        }.items(),
    )

    apriltag_behavior_tree = Node(
        package='robot_bringup',
        executable='apriltag_nav_behavior_tree',
        name='apriltag_nav_behavior_tree',
        output='screen',
        condition=IfCondition(apriltag_behavior_enabled),
        parameters=[
            apriltag_behavior_params_file,
            docking_params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'rules_file': apriltag_behavior_rules_file,
                'map_yaml': map_file,
                'landmarks_file': apriltag_landmarks_file,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('sensors_enabled', default_value='true'),
        DeclareLaunchArgument(
            'lidar_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_bringup'), 'config', 'lidar.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'imu_serial_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_bringup'), 'config', 'imu_serial.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'imu_bridge_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_bringup'), 'config', 'imu_bridge.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'sensor_tf_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_bringup'), 'config', 'sensor_transforms.yaml']
            ),
        ),
        DeclareLaunchArgument('sensor_publish_lidar_tf', default_value='true'),
        DeclareLaunchArgument('sensor_publish_imu_tf', default_value='false'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_bringup'), 'config', 'nav2.yaml']
            ),
        ),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='true'),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('drive_stack_enabled', default_value='true'),
        DeclareLaunchArgument(
            'serial_bridge_params_file',
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare('robot_bringup'),
                    'config',
                    'nav2_serial_diff_drive_bridge.yaml',
                ]
            ),
        ),
        DeclareLaunchArgument('serial_port', default_value='/dev/arduino'),
        DeclareLaunchArgument('serial_baud', default_value='115200'),
        DeclareLaunchArgument('serial_cmd_vel_topic', default_value='/cmd_vel_compensated'),
        DeclareLaunchArgument('serial_odom_topic', default_value='/odom_raw'),
        DeclareLaunchArgument('serial_publish_tf', default_value='false'),
        DeclareLaunchArgument(
            'state_estimation_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_bringup'), 'config', 'state_estimation_drift.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'state_estimation_imu_orientation_topic',
            default_value='/imu/orientation',
        ),
        DeclareLaunchArgument('state_estimation_imu_raw_topic', default_value='/imu/data_raw'),
        DeclareLaunchArgument('state_estimation_fused_odom_topic', default_value='/odom'),
        DeclareLaunchArgument('state_estimation_base_frame', default_value='base_link'),
        DeclareLaunchArgument('state_estimation_imu_frame', default_value='imu_link'),
        DeclareLaunchArgument(
            'straight_comp_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_bringup'), 'config', 'straight_line_compensator.yaml']
            ),
        ),
        DeclareLaunchArgument('straight_comp_enabled', default_value='true'),
        DeclareLaunchArgument('straight_comp_input_cmd_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('straight_comp_output_cmd_topic', default_value='/cmd_vel_compensated'),
        DeclareLaunchArgument('straight_comp_odom_topic', default_value='/odom_raw'),
        DeclareLaunchArgument('udp_teleop_enabled', default_value='true'),
        DeclareLaunchArgument(
            'udp_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_bringup'), 'config', 'nav2_udp_cmd_vel_bridge.yaml']
            ),
        ),
        DeclareLaunchArgument('udp_listen_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('udp_listen_port', default_value='8766'),
        DeclareLaunchArgument('udp_cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('udp_timeout_s', default_value='0.5'),
        DeclareLaunchArgument('health_monitor_enabled', default_value='true'),
        DeclareLaunchArgument(
            'health_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_bringup'), 'config', 'navigation_health_monitor.yaml']
            ),
        ),
        DeclareLaunchArgument('apriltag_realsense_enabled', default_value='false'),
        DeclareLaunchArgument('apriltag_behavior_enabled', default_value='true'),
        DeclareLaunchArgument(
            'apriltag_behavior_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_bringup'), 'config', 'apriltag_nav_behavior.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'apriltag_behavior_rules_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_bringup'), 'config', 'apriltag_nav_rules.yaml']
            ),
        ),
        DeclareLaunchArgument('apriltag_landmarks_file', default_value=''),
        DeclareLaunchArgument(
            'docking_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_bringup'), 'config', 'docking_server.yaml']
            ),
        ),
        set_use_sim_time,
        sensors,
        apriltag_realsense,
        nav2_stack,
        apriltag_behavior_tree,
    ])
