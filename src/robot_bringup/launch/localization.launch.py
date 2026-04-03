from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_topic = LaunchConfiguration('odom_topic')
    slam_params_file = LaunchConfiguration('slam_params_file')
    health_monitor_enabled = LaunchConfiguration('health_monitor_enabled')
    health_params_file = LaunchConfiguration('health_params_file')

    set_use_sim_time = SetParameter(
        name='use_sim_time',
        value=ParameterValue(use_sim_time, value_type=bool),
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        remappings=[('odom', odom_topic)],
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
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('robot_bringup'), 'config', 'slam_toolbox.yaml'
            ]),
        ),
        DeclareLaunchArgument('health_monitor_enabled', default_value='true'),
        DeclareLaunchArgument(
            'health_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('robot_bringup'), 'config', 'localization_health_monitor.yaml'
            ]),
        ),
        set_use_sim_time,
        slam_toolbox,
        health_monitor,
    ])
