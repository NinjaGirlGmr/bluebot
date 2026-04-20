#!/usr/bin/env python3

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    try:
        docking_server_params_dir = os.path.join(
            get_package_share_directory('docking_server'),
            'params',
        )
    except PackageNotFoundError:
        docking_server_params_dir = os.path.join(
            get_package_share_directory('robot_bringup'),
            'config',
        )

    sensors_enabled = LaunchConfiguration('sensors_enabled')
    lidar_params_file = LaunchConfiguration('lidar_params_file')
    imu_serial_params_file = LaunchConfiguration('imu_serial_params_file')
    imu_bridge_params_file = LaunchConfiguration('imu_bridge_params_file')
    sensor_tf_params_file = LaunchConfiguration('sensor_tf_params_file')
    sensor_publish_lidar_tf = LaunchConfiguration('sensor_publish_lidar_tf')
    sensor_publish_imu_tf = LaunchConfiguration('sensor_publish_imu_tf')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    smoother_params_file = LaunchConfiguration('smoother_params_file')
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
    state_estimation_global_enabled = LaunchConfiguration('state_estimation_global_enabled')
    state_estimation_global_params_file = LaunchConfiguration(
        'state_estimation_global_params_file'
    )
    state_estimation_global_pose_topic = LaunchConfiguration(
        'state_estimation_global_pose_topic'
    )
    state_estimation_global_initialpose_topic = LaunchConfiguration(
        'state_estimation_global_initialpose_topic'
    )
    state_estimation_global_odom_topic = LaunchConfiguration(
        'state_estimation_global_odom_topic'
    )
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
    apriltag_realsense_startup_delay_sec = LaunchConfiguration('apriltag_realsense_startup_delay_sec')
    apriltag_behavior_enabled = LaunchConfiguration('apriltag_behavior_enabled')
    apriltag_behavior_params_file = LaunchConfiguration('apriltag_behavior_params_file')
    apriltag_behavior_rules_file = LaunchConfiguration('apriltag_behavior_rules_file')
    apriltag_landmarks_file = LaunchConfiguration('apriltag_landmarks_file')
    apriltag_landmark_tf_enabled = LaunchConfiguration('apriltag_landmark_tf_enabled')
    apriltag_landmark_tf_params_file = LaunchConfiguration('apriltag_landmark_tf_params_file')
    apriltag_map_localization_enabled = LaunchConfiguration('apriltag_map_localization_enabled')
    apriltag_map_localization_params_file = LaunchConfiguration(
        'apriltag_map_localization_params_file'
    )
    docking_params_file = LaunchConfiguration('docking_params_file')
    grid_localization_enabled = LaunchConfiguration('grid_localization_enabled')
    grid_localization_scan_topic = LaunchConfiguration('grid_localization_scan_topic')
    grid_localization_result_topic = LaunchConfiguration('grid_localization_result_topic')
    grid_localization_output_topic = LaunchConfiguration('grid_localization_output_topic')
    grid_localization_output_frame = LaunchConfiguration('grid_localization_output_frame')
    grid_localization_use_current_output_stamp = LaunchConfiguration(
        'grid_localization_use_current_output_stamp'
    )
    grid_localization_fallback_enabled = LaunchConfiguration('grid_localization_fallback_enabled')
    grid_localization_fallback_wait_sec = LaunchConfiguration('grid_localization_fallback_wait_sec')
    grid_localization_fallback_publish_count = LaunchConfiguration(
        'grid_localization_fallback_publish_count'
    )
    grid_localization_fallback_publish_period_sec = LaunchConfiguration(
        'grid_localization_fallback_publish_period_sec'
    )
    grid_localization_fallback_x = LaunchConfiguration('grid_localization_fallback_x')
    grid_localization_fallback_y = LaunchConfiguration('grid_localization_fallback_y')
    grid_localization_fallback_yaw = LaunchConfiguration('grid_localization_fallback_yaw')
    occupancy_grid_localizer_enabled = LaunchConfiguration(
        'occupancy_grid_localizer_enabled'
    )
    grid_localization_trigger_enabled = LaunchConfiguration(
        'grid_localization_trigger_enabled'
    )
    grid_localization_trigger_service = LaunchConfiguration(
        'grid_localization_trigger_service'
    )
    grid_localization_trigger_delay_sec = LaunchConfiguration(
        'grid_localization_trigger_delay_sec'
    )
    grid_localization_trigger_retries = LaunchConfiguration(
        'grid_localization_trigger_retries'
    )
    grid_localization_trigger_retry_interval_sec = LaunchConfiguration(
        'grid_localization_trigger_retry_interval_sec'
    )
    grid_localization_trigger_service_ready_timeout_sec = LaunchConfiguration(
        'grid_localization_trigger_service_ready_timeout_sec'
    )
    grid_localization_trigger_service_call_timeout_sec = LaunchConfiguration(
        'grid_localization_trigger_service_call_timeout_sec'
    )

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

    apriltag_realsense = TimerAction(
        period=apriltag_realsense_startup_delay_sec,
        actions=[
            IncludeLaunchDescription(
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
        ],
    )

    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('robot_bringup'), 'launch', 'nav2.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': params_file,
            'smoother_params_file': smoother_params_file,
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
            'state_estimation_global_enabled': state_estimation_global_enabled,
            'state_estimation_global_params_file': state_estimation_global_params_file,
            'state_estimation_global_pose_topic': state_estimation_global_pose_topic,
            'state_estimation_global_initialpose_topic': (
                state_estimation_global_initialpose_topic
            ),
            'state_estimation_global_odom_topic': state_estimation_global_odom_topic,
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

    apriltag_landmark_tf = Node(
        package='robot_bringup',
        executable='apriltag_landmark_tf_publisher',
        name='apriltag_landmark_tf_publisher',
        output='screen',
        condition=IfCondition(apriltag_landmark_tf_enabled),
        parameters=[
            apriltag_landmark_tf_params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'map_yaml': ParameterValue(map_file, value_type=str),
                'landmarks_file': ParameterValue(apriltag_landmarks_file, value_type=str),
            },
        ],
    )

    apriltag_map_localization = Node(
        package='robot_bringup',
        executable='apriltag_map_localization',
        name='apriltag_map_localization',
        output='screen',
        condition=IfCondition(apriltag_map_localization_enabled),
        parameters=[
            apriltag_map_localization_params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'map_yaml': ParameterValue(map_file, value_type=str),
                'landmarks_file': ParameterValue(apriltag_landmarks_file, value_type=str),
            },
        ],
    )

    grid_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('isaac_nav2_pose_bridge'),
                    'launch',
                    'isaac_grid_localization_to_nav2.launch.py',
                ]
            )
        ),
        condition=IfCondition(grid_localization_enabled),
        launch_arguments={
            'map_yaml_path': map_file,
            'use_sim_time': use_sim_time,
            'scan_topic': grid_localization_scan_topic,
            'localization_result_topic': grid_localization_result_topic,
            'output_topic': grid_localization_output_topic,
            'output_frame_id': grid_localization_output_frame,
            'use_current_output_stamp': grid_localization_use_current_output_stamp,
            'fallback_initial_pose_enabled': grid_localization_fallback_enabled,
            'fallback_initial_pose_wait_sec': grid_localization_fallback_wait_sec,
            'fallback_initial_pose_publish_count': grid_localization_fallback_publish_count,
            'fallback_initial_pose_publish_period_sec': (
                grid_localization_fallback_publish_period_sec
            ),
            'fallback_initial_pose_x': grid_localization_fallback_x,
            'fallback_initial_pose_y': grid_localization_fallback_y,
            'fallback_initial_pose_yaw': grid_localization_fallback_yaw,
            'occupancy_grid_localizer_enabled': occupancy_grid_localizer_enabled,
        }.items(),
    )

    grid_localization_trigger = Node(
        package='robot_bringup',
        executable='grid_localization_trigger',
        name='grid_localization_trigger',
        output='screen',
        condition=IfCondition(grid_localization_trigger_enabled),
        parameters=[
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'service_name': ParameterValue(grid_localization_trigger_service, value_type=str),
                'startup_delay_sec': ParameterValue(
                    grid_localization_trigger_delay_sec,
                    value_type=float,
                ),
                'retries': ParameterValue(grid_localization_trigger_retries, value_type=int),
                'retry_interval_sec': ParameterValue(
                    grid_localization_trigger_retry_interval_sec,
                    value_type=float,
                ),
                'service_ready_timeout_sec': ParameterValue(
                    grid_localization_trigger_service_ready_timeout_sec,
                    value_type=float,
                ),
                'service_call_timeout_sec': ParameterValue(
                    grid_localization_trigger_service_call_timeout_sec,
                    value_type=float,
                ),
                'result_topic': ParameterValue(
                    grid_localization_result_topic,
                    value_type=str,
                ),
            }
        ],
    )

    grid_localization_stack = GroupAction(
        condition=IfCondition(grid_localization_enabled),
        actions=[
            grid_localization,
            grid_localization_trigger,
        ],
    )

    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[
            docking_params_file,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    docking_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'autostart': True,
                'node_names': ['docking_server'],
            }
        ],
    )

    dock_command = Node(
        package='robot_bringup',
        executable='dock_command',
        name='dock_command_node',
        output='screen',
        parameters=[
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    dock_detector = Node(
        package='robot_bringup',
        executable='dock_detector',
        name='dock_detector_node',
        output='screen',
        condition=IfCondition(apriltag_realsense_enabled),
        parameters=[
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    apriltag_behavior_tree = Node(
        package='robot_bringup',
        executable='apriltag_nav_behavior_tree',
        name='apriltag_nav_behavior_tree',
        output='screen',
        condition=IfCondition(apriltag_behavior_enabled),
        parameters=[
            apriltag_behavior_params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'rules_file': ParameterValue(apriltag_behavior_rules_file, value_type=str),
                'map_yaml': ParameterValue(map_file, value_type=str),
                'landmarks_file': ParameterValue(apriltag_landmarks_file, value_type=str),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
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
        DeclareLaunchArgument(
            'smoother_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_bringup'), 'config', 'nav2_smoother.yaml']
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
        DeclareLaunchArgument('state_estimation_global_enabled', default_value='true'),
        DeclareLaunchArgument(
            'state_estimation_global_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_bringup'), 'config', 'state_estimation_map_fusion.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'state_estimation_global_pose_topic',
            default_value='/apriltag/map_pose',
        ),
        DeclareLaunchArgument(
            'state_estimation_global_initialpose_topic',
            default_value='/initialpose',
        ),
        DeclareLaunchArgument(
            'state_estimation_global_odom_topic',
            default_value='/odometry/global',
        ),
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
        DeclareLaunchArgument('apriltag_realsense_enabled', default_value='true'),
        DeclareLaunchArgument('apriltag_realsense_startup_delay_sec', default_value='15.0'),
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
        DeclareLaunchArgument('apriltag_landmark_tf_enabled', default_value='true'),
        DeclareLaunchArgument(
            'apriltag_landmark_tf_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_bringup'), 'config', 'apriltag_landmark_tf.yaml']
            ),
        ),
        DeclareLaunchArgument('apriltag_map_localization_enabled', default_value='true'),
        DeclareLaunchArgument(
            'apriltag_map_localization_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_bringup'), 'config', 'apriltag_map_localization.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'docking_params_file',
            default_value=os.path.join(docking_server_params_dir, 'docking_server.yaml'),
        ),
        DeclareLaunchArgument('grid_localization_enabled', default_value='true'),
        DeclareLaunchArgument('grid_localization_scan_topic', default_value='/scan'),
        DeclareLaunchArgument('grid_localization_result_topic', default_value='/localization_result'),
        DeclareLaunchArgument('grid_localization_output_topic', default_value='/initialpose'),
        DeclareLaunchArgument('grid_localization_output_frame', default_value='map'),
        DeclareLaunchArgument(
            'grid_localization_use_current_output_stamp',
            default_value='false',
        ),
        DeclareLaunchArgument('grid_localization_fallback_enabled', default_value='false'),
        DeclareLaunchArgument('grid_localization_fallback_wait_sec', default_value='6.0'),
        DeclareLaunchArgument('grid_localization_fallback_publish_count', default_value='5'),
        DeclareLaunchArgument('grid_localization_fallback_publish_period_sec', default_value='0.5'),
        DeclareLaunchArgument('grid_localization_fallback_x', default_value='0.0'),
        DeclareLaunchArgument('grid_localization_fallback_y', default_value='0.0'),
        DeclareLaunchArgument('grid_localization_fallback_yaw', default_value='0.0'),
        DeclareLaunchArgument('occupancy_grid_localizer_enabled', default_value='true'),
        DeclareLaunchArgument('grid_localization_trigger_enabled', default_value='true'),
        DeclareLaunchArgument(
            'grid_localization_trigger_service',
            default_value='/trigger_grid_search_localization',
        ),
        DeclareLaunchArgument('grid_localization_trigger_delay_sec', default_value='2.5'),
        DeclareLaunchArgument('grid_localization_trigger_retries', default_value='120'),
        DeclareLaunchArgument(
            'grid_localization_trigger_retry_interval_sec',
            default_value='1.0',
        ),
        DeclareLaunchArgument(
            'grid_localization_trigger_service_ready_timeout_sec',
            default_value='90.0',
        ),
        DeclareLaunchArgument(
            'grid_localization_trigger_service_call_timeout_sec',
            default_value='3.0',
        ),
        set_use_sim_time,
        sensors,
        apriltag_realsense,
        apriltag_landmark_tf,
        apriltag_map_localization,
        nav2_stack,
        docking_server,
        docking_lifecycle_manager,
        dock_command,
        dock_detector,
        grid_localization_stack,
        apriltag_behavior_tree,
    ])
