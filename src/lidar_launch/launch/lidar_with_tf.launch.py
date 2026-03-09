from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    frame_id = LaunchConfiguration("frame_id")
    parent_frame = LaunchConfiguration("parent_frame")
    scan_frequency = LaunchConfiguration("scan_frequency")
    angle_compensate = LaunchConfiguration("angle_compensate")
    scan_mode = LaunchConfiguration("scan_mode")
    tf_x = LaunchConfiguration("tf_x")
    tf_y = LaunchConfiguration("tf_y")
    tf_z = LaunchConfiguration("tf_z")
    tf_roll = LaunchConfiguration("tf_roll")
    tf_pitch = LaunchConfiguration("tf_pitch")
    tf_yaw = LaunchConfiguration("tf_yaw")

    declare_serial_port = DeclareLaunchArgument("serial_port", default_value="/dev/lidar")
    declare_frame_id = DeclareLaunchArgument("frame_id", default_value="laser")
    declare_parent_frame = DeclareLaunchArgument("parent_frame", default_value="base_link")
    declare_scan_frequency = DeclareLaunchArgument("scan_frequency", default_value="6.0")
    declare_angle_compensate = DeclareLaunchArgument("angle_compensate", default_value="false")
    declare_scan_mode = DeclareLaunchArgument("scan_mode", default_value="")
    declare_tf_x = DeclareLaunchArgument("tf_x", default_value="0.0")
    declare_tf_y = DeclareLaunchArgument("tf_y", default_value="0.0")
    declare_tf_z = DeclareLaunchArgument("tf_z", default_value="0.199")
    # REP-103 convention: x-forward, y-left, z-up.
    declare_tf_roll = DeclareLaunchArgument("tf_roll", default_value="0.0")
    declare_tf_pitch = DeclareLaunchArgument("tf_pitch", default_value="0.0")
    declare_tf_yaw = DeclareLaunchArgument("tf_yaw", default_value="0.0")

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': serial_port,
            'serial_baudrate': 115200,
            'frame_id': frame_id,
            'scan_frequency': scan_frequency,
            'angle_compensate': angle_compensate,
            'scan_mode': scan_mode,
            'inverted': False,
        }],
        output='screen'
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', tf_x,
            '--y', tf_y,
            '--z', tf_z,
            '--roll', tf_roll,
            '--pitch', tf_pitch,
            '--yaw', tf_yaw,
            '--frame-id', parent_frame,
            '--child-frame-id', frame_id
        ]
    )

    return LaunchDescription([
        declare_serial_port,
        declare_frame_id,
        declare_parent_frame,
        declare_scan_frequency,
        declare_angle_compensate,
        declare_scan_mode,
        declare_tf_x,
        declare_tf_y,
        declare_tf_z,
        declare_tf_roll,
        declare_tf_pitch,
        declare_tf_yaw,
        rplidar_node,
        static_tf
    ])
