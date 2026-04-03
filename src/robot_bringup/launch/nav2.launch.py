from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    health_monitor_enabled = LaunchConfiguration('health_monitor_enabled')
    health_params_file = LaunchConfiguration('health_params_file')

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

    set_use_sim_time = SetParameter(
        name='use_sim_time',
        value=ParameterValue(use_sim_time, value_type=bool),
    )

    serial_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('ros2_serial_diff_drive_bridge'),
                    'launch',
                    'ros2_serial_diff_drive_bridge.launch.py',
                ]
            )
        ),
        condition=IfCondition(drive_stack_enabled),
        launch_arguments={
            'params_file': serial_bridge_params_file,
            'port': serial_port,
            'baud': serial_baud,
            'cmd_vel_topic': serial_cmd_vel_topic,
            'odom_topic': serial_odom_topic,
            'publish_tf': serial_publish_tf,
        }.items(),
    )

    serial_diff_drive_hw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('serial_diff_drive_hw'), 'launch', 'bringup.launch.py']
            )
        ),
        condition=IfCondition(drive_stack_enabled),
    )

    imu_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        output='screen',
        condition=IfCondition(drive_stack_enabled),
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        arguments=[
            '0', '0', '0', '0', '0', '0',
            state_estimation_base_frame,
            state_estimation_imu_frame,
        ],
    )

    state_estimation = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization_filter',
        output='screen',
        condition=IfCondition(drive_stack_enabled),
        parameters=[
            state_estimation_params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'odom0': serial_odom_topic,
                'imu0': state_estimation_imu_orientation_topic,
                'imu1': state_estimation_imu_raw_topic,
            },
        ],
        remappings=[('/odometry/filtered', state_estimation_fused_odom_topic)],
    )

    straight_line_compensator = Node(
        package='bluebot_v3',
        executable='straight_line_compensator_node',
        name='straight_line_compensator',
        output='screen',
        condition=IfCondition(drive_stack_enabled),
        parameters=[
            straight_comp_params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'enabled': ParameterValue(straight_comp_enabled, value_type=bool),
                'input_cmd_topic': straight_comp_input_cmd_topic,
                'output_cmd_topic': straight_comp_output_cmd_topic,
                'odom_topic': straight_comp_odom_topic,
            },
        ],
    )

    udp_cmd_vel_bridge = Node(
        package='udp_cmd_vel_bridge',
        executable='udp_cmd_vel_bridge',
        name='udp_cmd_vel_bridge',
        output='screen',
        condition=IfCondition(udp_teleop_enabled),
        parameters=[
            udp_params_file,
            {
                'listen_host': udp_listen_host,
                'listen_port': ParameterValue(udp_listen_port, value_type=int),
                'cmd_vel_topic': udp_cmd_vel_topic,
                'timeout_s': ParameterValue(udp_timeout_s, value_type=float),
            },
        ],
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
            )
        ),
        launch_arguments={
            'slam': 'False',
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'log_level': log_level,
        }.items(),
    )

    health_monitor = Node(
        package='robot_bringup',
        executable='health_monitor',
        name='health_monitor',
        output='screen',
        condition=IfCondition(health_monitor_enabled),
        parameters=[health_params_file, {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
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
                [FindPackageShare('robot_bringup'), 'config', 'nav2_health_monitor.yaml']
            ),
        ),
        set_use_sim_time,
        serial_bridge,
        serial_diff_drive_hw,
        imu_to_base_tf,
        state_estimation,
        straight_line_compensator,
        udp_cmd_vel_bridge,
        nav2_bringup,
        health_monitor,
    ])
