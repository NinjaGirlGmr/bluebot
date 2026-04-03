from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    health_monitor_enabled = LaunchConfiguration('health_monitor_enabled')
    health_params_file = LaunchConfiguration('health_params_file')
    foxglove_params = PathJoinSubstitution([
        FindPackageShare('robot_bringup'), 'config', 'foxglove_bridge.yaml'
    ])

    set_use_sim_time = SetParameter(
        name='use_sim_time',
        value=ParameterValue(use_sim_time, value_type=bool),
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[foxglove_params, {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
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
        DeclareLaunchArgument('health_monitor_enabled', default_value='true'),
        DeclareLaunchArgument(
            'health_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('robot_bringup'), 'config', 'observability_health_monitor.yaml'
            ]),
        ),
        set_use_sim_time,
        foxglove_bridge,
        health_monitor,
    ])
