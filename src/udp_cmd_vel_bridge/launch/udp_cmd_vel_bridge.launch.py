from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    listen_host = LaunchConfiguration("listen_host")
    listen_port = LaunchConfiguration("listen_port")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    timeout_s = LaunchConfiguration("timeout_s")

    return LaunchDescription(
        [
            DeclareLaunchArgument("listen_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("listen_port", default_value="8765"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("timeout_s", default_value="0.5"),
            Node(
                package="udp_cmd_vel_bridge",
                executable="udp_cmd_vel_bridge",
                name="udp_cmd_vel_bridge",
                parameters=[
                    {
                        "listen_host": listen_host,
                        "listen_port": listen_port,
                        "cmd_vel_topic": cmd_vel_topic,
                        "timeout_s": timeout_s,
                    }
                ],
                output="screen",
            ),
        ]
    )
