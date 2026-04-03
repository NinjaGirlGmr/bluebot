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
    arduino_port = LaunchConfiguration("arduino_port")
    arduino_baud = LaunchConfiguration("arduino_baud")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    cmd_vel_compensated_topic = LaunchConfiguration("cmd_vel_compensated_topic")
    odom_raw_topic = LaunchConfiguration("odom_raw_topic")
    fused_odom_topic = LaunchConfiguration("fused_odom_topic")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    wheel_radius_m = LaunchConfiguration("wheel_radius_m")
    wheel_separation_m = LaunchConfiguration("wheel_separation_m")
    publish_bridge_tf = LaunchConfiguration("publish_bridge_tf")

    lidar_port = LaunchConfiguration("lidar_port")
    lidar_frame = LaunchConfiguration("lidar_frame")
    lidar_scan_frequency = LaunchConfiguration("lidar_scan_frequency")
    lidar_angle_compensate = LaunchConfiguration("lidar_angle_compensate")
    lidar_scan_mode = LaunchConfiguration("lidar_scan_mode")
    lidar_tf_x = LaunchConfiguration("lidar_tf_x")
    lidar_tf_y = LaunchConfiguration("lidar_tf_y")
    lidar_tf_z = LaunchConfiguration("lidar_tf_z")
    lidar_tf_roll = LaunchConfiguration("lidar_tf_roll")
    lidar_tf_pitch = LaunchConfiguration("lidar_tf_pitch")
    lidar_tf_yaw = LaunchConfiguration("lidar_tf_yaw")

    imu_frame = LaunchConfiguration("imu_frame")
    imu_orientation_topic = LaunchConfiguration("imu_orientation_topic")
    imu_raw_topic = LaunchConfiguration("imu_raw_topic")
    imu_serial_port = LaunchConfiguration("imu_serial_port")
    imu_serial_baud = LaunchConfiguration("imu_serial_baud")
    imu_tf_x = LaunchConfiguration("imu_tf_x")
    imu_tf_y = LaunchConfiguration("imu_tf_y")
    imu_tf_z = LaunchConfiguration("imu_tf_z")
    imu_tf_roll = LaunchConfiguration("imu_tf_roll")
    imu_tf_pitch = LaunchConfiguration("imu_tf_pitch")
    imu_tf_yaw = LaunchConfiguration("imu_tf_yaw")

    scan_topic = LaunchConfiguration("scan_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_ekf = LaunchConfiguration("use_ekf")
    straight_comp_enabled = LaunchConfiguration("straight_comp_enabled")
    straight_comp_odom_topic = LaunchConfiguration("straight_comp_odom_topic")
    straight_comp_linear_speed_min = LaunchConfiguration("straight_comp_linear_speed_min")
    straight_comp_angular_deadband = LaunchConfiguration("straight_comp_angular_deadband")
    straight_comp_turn_memory_angular_threshold = LaunchConfiguration("straight_comp_turn_memory_angular_threshold")
    straight_comp_kp = LaunchConfiguration("straight_comp_kp")
    straight_comp_ki = LaunchConfiguration("straight_comp_ki")
    straight_comp_kd = LaunchConfiguration("straight_comp_kd")
    straight_comp_max_angular = LaunchConfiguration("straight_comp_max_angular")
    straight_comp_integral_limit = LaunchConfiguration("straight_comp_integral_limit")
    straight_comp_caster_enabled = LaunchConfiguration("straight_comp_caster_enabled")
    straight_comp_caster_gain = LaunchConfiguration("straight_comp_caster_gain")
    straight_comp_caster_decay_sec = LaunchConfiguration("straight_comp_caster_decay_sec")
    straight_comp_caster_max = LaunchConfiguration("straight_comp_caster_max")
    straight_comp_caster_max_age_sec = LaunchConfiguration("straight_comp_caster_max_age_sec")
    straight_comp_caster_forward_only = LaunchConfiguration("straight_comp_caster_forward_only")
    straight_comp_odom_timeout_sec = LaunchConfiguration("straight_comp_odom_timeout_sec")
    straight_comp_reset_on_reverse = LaunchConfiguration("straight_comp_reset_on_reverse")
    slam_params_file = LaunchConfiguration("slam_params_file")
    ekf_params_file = LaunchConfiguration("ekf_params_file")

    default_slam_params = PathJoinSubstitution(
        [FindPackageShare("bluebot_v3"), "config", "slam_toolbox_mapping_nav2.yaml"]
    )
    default_ekf_params = PathJoinSubstitution(
        [FindPackageShare("bluebot_v3"), "config", "ekf_mapping.yaml"]
    )
    set_use_sim_time = SetParameter(
        name="use_sim_time",
        value=ParameterValue(use_sim_time, value_type=bool),
    )

    bridge_node = Node(
        package="ros2_serial_diff_drive_bridge",
        executable="serial_diff_drive_bridge",
        name="serial_diff_drive_bridge",
        output="screen",
        parameters=[
            {
                "port": arduino_port,
                "baud": ParameterValue(arduino_baud, value_type=int),
                "cmd_vel_topic": cmd_vel_compensated_topic,
                "odom_topic": odom_raw_topic,
                "publish_tf": ParameterValue(publish_bridge_tf, value_type=bool),
                "base_frame_id": base_frame,
                "odom_frame_id": odom_frame,
                "use_wheel_kinematics_odom": True,
                "wheel_radius_m": ParameterValue(wheel_radius_m, value_type=float),
                "wheel_separation_m": ParameterValue(wheel_separation_m, value_type=float),
            }
        ],
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("lidar_launch"), "launch", "lidar_with_tf.launch.py"])
        ),
        launch_arguments={
            "serial_port": lidar_port,
            "frame_id": lidar_frame,
            "parent_frame": base_frame,
            "scan_frequency": lidar_scan_frequency,
            "angle_compensate": lidar_angle_compensate,
            "scan_mode": lidar_scan_mode,
            "use_sim_time": use_sim_time,
            "tf_x": lidar_tf_x,
            "tf_y": lidar_tf_y,
            "tf_z": lidar_tf_z,
            "tf_roll": lidar_tf_roll,
            "tf_pitch": lidar_tf_pitch,
            "tf_yaw": lidar_tf_yaw,
        }.items(),
    )

    a471_serial_node = Node(
        package="yb_a471_driver",
        executable="a471_serial_node",
        name="a471_serial_node",
        output="screen",
        parameters=[
            {
                "port": imu_serial_port,
                "baud": ParameterValue(imu_serial_baud, value_type=int),
                "frame_id": imu_frame,
                "imu_topic": "/imu/data",
                "publish_legacy_imu_topic": True,
                "legacy_imu_topic": "/imu",
            }
        ],
    )

    imu_node = Node(
        package="yb_a471_driver",
        executable="imu_node",
        name="imu_node",
        output="screen",
        parameters=[
            {
                "frame_id": imu_frame,
                "raw_topic": imu_raw_topic,
                "orientation_topic": imu_orientation_topic,
                "source_topics": ["/imu/data", "/imu"],
                # Do not publish zeroed placeholder IMU when source is absent.
                # Fake IMU can destabilize EKF and degrade SLAM map quality.
                "fallback_publish": False,
            }
        ],
    )

    imu_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_imu_tf",
        arguments=[
            "--x", imu_tf_x,
            "--y", imu_tf_y,
            "--z", imu_tf_z,
            "--roll", imu_tf_roll,
            "--pitch", imu_tf_pitch,
            "--yaw", imu_tf_yaw,
            "--frame-id", base_frame,
            "--child-frame-id", imu_frame,
        ],
        output="screen",
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="robot_localization_filter",
        output="screen",
        condition=IfCondition(use_ekf),
        parameters=[
            ekf_params_file,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "base_link_frame": base_frame,
                "world_frame": odom_frame,
                "odom0": odom_raw_topic,
                "imu0": imu_raw_topic,
            },
        ],
        remappings=[("/odometry/filtered", fused_odom_topic)],
    )

    straight_line_compensator_node = Node(
        package="bluebot_v3",
        executable="straight_line_compensator_node",
        name="straight_line_compensator",
        output="screen",
        parameters=[
            {
                "input_cmd_topic": cmd_vel_topic,
                "output_cmd_topic": cmd_vel_compensated_topic,
                "odom_topic": straight_comp_odom_topic,
                "enabled": ParameterValue(straight_comp_enabled, value_type=bool),
                "linear_speed_min": ParameterValue(straight_comp_linear_speed_min, value_type=float),
                "angular_deadband": ParameterValue(straight_comp_angular_deadband, value_type=float),
                "turn_memory_angular_threshold": ParameterValue(
                    straight_comp_turn_memory_angular_threshold, value_type=float
                ),
                "kp": ParameterValue(straight_comp_kp, value_type=float),
                "ki": ParameterValue(straight_comp_ki, value_type=float),
                "kd": ParameterValue(straight_comp_kd, value_type=float),
                "max_angular_correction": ParameterValue(straight_comp_max_angular, value_type=float),
                "integral_limit": ParameterValue(straight_comp_integral_limit, value_type=float),
                "caster_comp_enabled": ParameterValue(straight_comp_caster_enabled, value_type=bool),
                "caster_comp_gain": ParameterValue(straight_comp_caster_gain, value_type=float),
                "caster_comp_decay_sec": ParameterValue(straight_comp_caster_decay_sec, value_type=float),
                "caster_comp_max": ParameterValue(straight_comp_caster_max, value_type=float),
                "caster_comp_max_age_sec": ParameterValue(straight_comp_caster_max_age_sec, value_type=float),
                "caster_comp_forward_only": ParameterValue(straight_comp_caster_forward_only, value_type=bool),
                "odom_timeout_sec": ParameterValue(straight_comp_odom_timeout_sec, value_type=float),
                "reset_heading_on_reverse": ParameterValue(straight_comp_reset_on_reverse, value_type=bool),
            }
        ],
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_file,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "scan_topic": scan_topic,
                "base_frame": base_frame,
                "odom_frame": odom_frame,
                "map_frame": map_frame,
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("arduino_port", default_value="/dev/arduino"),
            DeclareLaunchArgument("arduino_baud", default_value="115200"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("cmd_vel_compensated_topic", default_value="/cmd_vel_compensated"),
            DeclareLaunchArgument("odom_raw_topic", default_value="/odom_raw"),
            DeclareLaunchArgument("fused_odom_topic", default_value="/odom"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("wheel_radius_m", default_value="0.03354"),
            DeclareLaunchArgument("wheel_separation_m", default_value="0.195"),
            # Keep this false when EKF publishes odom->base_link TF to avoid duplicate TF.
            DeclareLaunchArgument("publish_bridge_tf", default_value="false"),
            DeclareLaunchArgument("lidar_port", default_value="/dev/lidar"),
            DeclareLaunchArgument("lidar_frame", default_value="laser"),
            DeclareLaunchArgument("lidar_scan_frequency", default_value="8.0"),
            DeclareLaunchArgument("lidar_angle_compensate", default_value="true"),
            DeclareLaunchArgument("lidar_scan_mode", default_value=""),
            DeclareLaunchArgument("lidar_tf_x", default_value="0.0"),
            DeclareLaunchArgument("lidar_tf_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_tf_z", default_value="0.199"),
            DeclareLaunchArgument("lidar_tf_roll", default_value="0.0"),
            DeclareLaunchArgument("lidar_tf_pitch", default_value="0.0"),
            DeclareLaunchArgument("lidar_tf_yaw", default_value="0.0"),
            DeclareLaunchArgument("imu_frame", default_value="imu_link"),
            DeclareLaunchArgument("imu_orientation_topic", default_value="/imu/orientation"),
            DeclareLaunchArgument("imu_raw_topic", default_value="/imu/data_raw"),
            DeclareLaunchArgument("imu_serial_port", default_value="/dev/myimu"),
            DeclareLaunchArgument("imu_serial_baud", default_value="115200"),
            DeclareLaunchArgument("imu_tf_x", default_value="0.0"),
            DeclareLaunchArgument("imu_tf_y", default_value="0.0"),
            DeclareLaunchArgument("imu_tf_z", default_value="0.0"),
            DeclareLaunchArgument("imu_tf_roll", default_value="0.0"),
            DeclareLaunchArgument("imu_tf_pitch", default_value="0.0"),
            DeclareLaunchArgument("imu_tf_yaw", default_value="0.0"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("use_ekf", default_value="true"),
            DeclareLaunchArgument("straight_comp_enabled", default_value="true"),
            DeclareLaunchArgument("straight_comp_odom_topic", default_value="/odom_raw"),
            DeclareLaunchArgument("straight_comp_linear_speed_min", default_value="0.03"),
            DeclareLaunchArgument("straight_comp_angular_deadband", default_value="0.05"),
            DeclareLaunchArgument("straight_comp_turn_memory_angular_threshold", default_value="0.20"),
            DeclareLaunchArgument("straight_comp_kp", default_value="1.8"),
            DeclareLaunchArgument("straight_comp_ki", default_value="0.0"),
            DeclareLaunchArgument("straight_comp_kd", default_value="0.20"),
            DeclareLaunchArgument("straight_comp_max_angular", default_value="0.50"),
            DeclareLaunchArgument("straight_comp_integral_limit", default_value="0.40"),
            DeclareLaunchArgument("straight_comp_caster_enabled", default_value="true"),
            DeclareLaunchArgument("straight_comp_caster_gain", default_value="0.12"),
            DeclareLaunchArgument("straight_comp_caster_decay_sec", default_value="0.60"),
            DeclareLaunchArgument("straight_comp_caster_max", default_value="0.25"),
            DeclareLaunchArgument("straight_comp_caster_max_age_sec", default_value="2.0"),
            DeclareLaunchArgument("straight_comp_caster_forward_only", default_value="true"),
            DeclareLaunchArgument("straight_comp_odom_timeout_sec", default_value="0.40"),
            DeclareLaunchArgument("straight_comp_reset_on_reverse", default_value="true"),
            DeclareLaunchArgument("slam_params_file", default_value=default_slam_params),
            DeclareLaunchArgument("ekf_params_file", default_value=default_ekf_params),
            set_use_sim_time,
            bridge_node,
            lidar_launch,
            a471_serial_node,
            imu_node,
            imu_to_base_tf,
            ekf_node,
            straight_line_compensator_node,
            slam_node,
            rviz_node,
        ]
    )
