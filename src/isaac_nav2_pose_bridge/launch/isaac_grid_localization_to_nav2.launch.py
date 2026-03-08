"""Launch Isaac occupancy-grid localization and bridge its result to Nav2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create launch description for occupancy-grid localization + bridge."""
    map_yaml_path = LaunchConfiguration('map_yaml_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    flatscan_topic = LaunchConfiguration('flatscan_topic')
    localization_result_topic = LaunchConfiguration('localization_result_topic')
    output_topic = LaunchConfiguration('output_topic')
    output_frame_id = LaunchConfiguration('output_frame_id')
    pose_stamped_topic = LaunchConfiguration('pose_stamped_topic')
    pose_with_covariance_topic = LaunchConfiguration(
        'pose_with_covariance_topic')
    enable_pose_stamped_input = LaunchConfiguration(
        'enable_pose_stamped_input')
    enable_pose_with_covariance_input = LaunchConfiguration(
        'enable_pose_with_covariance_input')
    fallback_initial_pose_enabled = LaunchConfiguration(
        'fallback_initial_pose_enabled')
    fallback_initial_pose_wait_sec = LaunchConfiguration(
        'fallback_initial_pose_wait_sec')
    fallback_initial_pose_publish_count = LaunchConfiguration(
        'fallback_initial_pose_publish_count')
    fallback_initial_pose_publish_period_sec = LaunchConfiguration(
        'fallback_initial_pose_publish_period_sec')
    fallback_initial_pose_x = LaunchConfiguration('fallback_initial_pose_x')
    fallback_initial_pose_y = LaunchConfiguration('fallback_initial_pose_y')
    fallback_initial_pose_yaw = LaunchConfiguration('fallback_initial_pose_yaw')

    occupancy_grid_localizer_map_yaml = ParameterValue(map_yaml_path, value_type=str)

    occupancy_grid_localizer_node = ComposableNode(
        package='isaac_ros_occupancy_grid_localizer',
        plugin='nvidia::isaac_ros::occupancy_grid_localizer::'
               'OccupancyGridLocalizerNode',
        name='occupancy_grid_localizer',
        parameters=[map_yaml_path, {
            'loc_result_frame': output_frame_id,
            'map_yaml_path': occupancy_grid_localizer_map_yaml,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('flatscan', flatscan_topic),
            # Publish directly to Nav2 /initialpose for maximum compatibility.
            ('localization_result', output_topic),
        ],
    )

    laserscan_to_flatscan_node = ComposableNode(
        package='isaac_ros_pointcloud_utils',
        plugin='nvidia::isaac_ros::pointcloud_utils::LaserScantoFlatScanNode',
        name='laserscan_to_flatscan',
        remappings=[
            ('scan', scan_topic),
            ('flatscan', flatscan_topic),
        ],
    )

    occupancy_grid_localizer_container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container_mt',
        name='isaac_grid_localization_container',
        namespace='',
        composable_node_descriptions=[
            occupancy_grid_localizer_node,
            laserscan_to_flatscan_node,
        ],
        output='screen',
    )

    isaac_to_nav2_pose_node = Node(
        package='isaac_nav2_pose_bridge',
        executable='isaac_to_nav2_pose',
        name='isaac_to_nav2_pose',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'output_topic': output_topic,
            'pose_stamped_topic': pose_stamped_topic,
            'pose_with_covariance_topic': pose_with_covariance_topic,
            'enable_pose_stamped_input': enable_pose_stamped_input,
            'enable_pose_with_covariance_input': (
                enable_pose_with_covariance_input
            ),
            'output_frame_id': output_frame_id,
            'fallback_initial_pose_enabled': fallback_initial_pose_enabled,
            'fallback_initial_pose_wait_sec': fallback_initial_pose_wait_sec,
            'fallback_initial_pose_publish_count':
                fallback_initial_pose_publish_count,
            'fallback_initial_pose_publish_period_sec':
                fallback_initial_pose_publish_period_sec,
            'fallback_initial_pose_x': fallback_initial_pose_x,
            'fallback_initial_pose_y': fallback_initial_pose_y,
            'fallback_initial_pose_yaw': fallback_initial_pose_yaw,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml_path',
            default_value='/ssd/maps/map.yaml',
            description='Full path to the map YAML used for localization.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='LaserScan input topic.',
        ),
        DeclareLaunchArgument(
            'flatscan_topic',
            default_value='/flatscan',
            description='FlatScan topic between converter and localizer.',
        ),
        DeclareLaunchArgument(
            'localization_result_topic',
            default_value='/localization_result',
            description='Output topic from occupancy-grid localizer.',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/initialpose',
            description='Nav2 initial pose topic.',
        ),
        DeclareLaunchArgument(
            'output_frame_id',
            default_value='map',
            description='Frame ID stamped on outgoing initial pose.',
        ),
        DeclareLaunchArgument(
            'pose_stamped_topic',
            default_value='/grid_search_pose',
            description='Optional PoseStamped input topic for the bridge.',
        ),
        DeclareLaunchArgument(
            'pose_with_covariance_topic',
            default_value='/localization_result',
            description='PoseWithCovarianceStamped input topic for the bridge.',
        ),
        DeclareLaunchArgument(
            'enable_pose_stamped_input',
            default_value='false',
            description='Enable PoseStamped bridge input.',
        ),
        DeclareLaunchArgument(
            'enable_pose_with_covariance_input',
            default_value='false',
            description='Enable PoseWithCovarianceStamped bridge input.',
        ),
        DeclareLaunchArgument(
            'fallback_initial_pose_enabled',
            default_value='true',
            description='Publish fallback initial pose if no localization input arrives.',
        ),
        DeclareLaunchArgument(
            'fallback_initial_pose_wait_sec',
            default_value='6.0',
            description='Seconds to wait before fallback initial pose publish.',
        ),
        DeclareLaunchArgument(
            'fallback_initial_pose_publish_count',
            default_value='5',
            description='Number of fallback initial pose messages to publish.',
        ),
        DeclareLaunchArgument(
            'fallback_initial_pose_publish_period_sec',
            default_value='0.5',
            description='Seconds between fallback initial pose publishes.',
        ),
        DeclareLaunchArgument(
            'fallback_initial_pose_x',
            default_value='0.0',
            description='Fallback initial pose X in map frame.',
        ),
        DeclareLaunchArgument(
            'fallback_initial_pose_y',
            default_value='0.0',
            description='Fallback initial pose Y in map frame.',
        ),
        DeclareLaunchArgument(
            'fallback_initial_pose_yaw',
            default_value='0.0',
            description='Fallback initial pose yaw (radians).',
        ),
        occupancy_grid_localizer_container,
        isaac_to_nav2_pose_node,
    ])
