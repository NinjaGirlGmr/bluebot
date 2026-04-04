"""Launch Isaac occupancy-grid localization and bridge its result to Nav2."""

from pathlib import Path

import yaml
from PIL import Image

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


SUPPORTED_IMAGE_EXTENSIONS = {'.png', '.jpg', '.jpeg'}


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in ('true', '1', 'yes', 'on')


def _prepare_localizer_map_yaml(map_yaml_path: str):
    """Return a map YAML path supported by occupancy_grid_localizer.

    The localizer only supports png/jpg/jpeg map images. If the map references
    a .pgm image, create a converted png + yaml copy in /tmp and use that.
    """
    map_yaml = Path(map_yaml_path).expanduser()
    if not map_yaml.exists():
        return None, f"[isaac_grid_localization] map yaml does not exist: {map_yaml}"

    try:
        data = yaml.safe_load(map_yaml.read_text()) or {}
    except Exception as exc:  # pragma: no cover - runtime path handling
        return None, (
            f"[isaac_grid_localization] failed to parse map yaml '{map_yaml}': {exc}"
        )

    image_field = str(data.get('image', '')).strip()
    if not image_field:
        return None, (
            f"[isaac_grid_localization] map yaml '{map_yaml}' has empty 'image' field"
        )

    image_path = Path(image_field)
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    image_path = image_path.resolve()

    ext = image_path.suffix.lower()
    if ext not in SUPPORTED_IMAGE_EXTENSIONS and ext != '.pgm':
        return None, (
            f"[isaac_grid_localization] unsupported map image '{image_path}'. "
            "Supported: .png/.jpg/.jpeg"
        )

    out_dir = Path('/tmp/isaac_grid_localizer_maps')
    out_dir.mkdir(parents=True, exist_ok=True)
    out_image = out_dir / f"{map_yaml.stem}{'.png' if ext == '.pgm' else ext}"
    out_yaml = out_dir / f"{map_yaml.stem}.localizer.yaml"

    if ext == '.pgm':
        try:
            with Image.open(str(image_path)) as pgm_image:
                pgm_image.save(str(out_image))
        except Exception as exc:  # pragma: no cover - runtime path handling
            return None, (
                f"[isaac_grid_localization] failed to convert '{image_path}' to png: {exc}"
            )
        image_message = (
            f"[isaac_grid_localization] converted '{image_path.name}' -> "
            f"'{out_image}' for occupancy_grid_localizer"
        )
    else:
        out_image.write_bytes(image_path.read_bytes())
        image_message = (
            f"[isaac_grid_localization] copied '{image_path.name}' -> "
            f"'{out_image}' for occupancy_grid_localizer"
        )

    # OccupancyGridLocalizer prepends the map-yaml directory to the `image` value,
    # so this must remain a relative filename.
    data['image'] = out_image.name
    out_yaml.write_text(yaml.safe_dump(data, sort_keys=False))
    return str(out_yaml), image_message


def _launch_setup(context, *args, **kwargs):
    map_yaml_path = LaunchConfiguration('map_yaml_path').perform(context)
    use_sim_time = _as_bool(LaunchConfiguration('use_sim_time').perform(context))
    scan_topic = LaunchConfiguration('scan_topic').perform(context)
    flatscan_topic = LaunchConfiguration('flatscan_topic').perform(context)
    localization_result_topic = LaunchConfiguration('localization_result_topic').perform(
        context
    )
    output_topic = LaunchConfiguration('output_topic').perform(context)
    output_frame_id = LaunchConfiguration('output_frame_id').perform(context)
    pose_stamped_topic = LaunchConfiguration('pose_stamped_topic').perform(context)
    pose_with_covariance_topic = LaunchConfiguration('pose_with_covariance_topic').perform(
        context
    )
    enable_pose_stamped_input = _as_bool(
        LaunchConfiguration('enable_pose_stamped_input').perform(context)
    )
    enable_pose_with_covariance_input = _as_bool(
        LaunchConfiguration('enable_pose_with_covariance_input').perform(context)
    )
    fallback_initial_pose_enabled = _as_bool(
        LaunchConfiguration('fallback_initial_pose_enabled').perform(context)
    )
    fallback_initial_pose_wait_sec = float(
        LaunchConfiguration('fallback_initial_pose_wait_sec').perform(context)
    )
    fallback_initial_pose_publish_count = int(
        LaunchConfiguration('fallback_initial_pose_publish_count').perform(context)
    )
    fallback_initial_pose_publish_period_sec = float(
        LaunchConfiguration('fallback_initial_pose_publish_period_sec').perform(context)
    )
    fallback_initial_pose_x = float(
        LaunchConfiguration('fallback_initial_pose_x').perform(context)
    )
    fallback_initial_pose_y = float(
        LaunchConfiguration('fallback_initial_pose_y').perform(context)
    )
    fallback_initial_pose_yaw = float(
        LaunchConfiguration('fallback_initial_pose_yaw').perform(context)
    )
    occupancy_grid_localizer_enabled = _as_bool(
        LaunchConfiguration('occupancy_grid_localizer_enabled').perform(context)
    )

    actions = []

    if occupancy_grid_localizer_enabled:
        resolved_localizer_map_yaml, info_message = _prepare_localizer_map_yaml(
            map_yaml_path
        )
        if info_message:
            actions.append(LogInfo(msg=info_message))

        if resolved_localizer_map_yaml:
            occupancy_grid_localizer_node = ComposableNode(
                package='isaac_ros_occupancy_grid_localizer',
                plugin='nvidia::isaac_ros::occupancy_grid_localizer::'
                'OccupancyGridLocalizerNode',
                name='occupancy_grid_localizer',
                parameters=[
                    resolved_localizer_map_yaml,
                    {
                        'loc_result_frame': output_frame_id,
                        'map_yaml_path': resolved_localizer_map_yaml,
                        'use_sim_time': use_sim_time,
                    }
                ],
                remappings=[
                    ('flatscan', flatscan_topic),
                    ('localization_result', localization_result_topic),
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
            actions.append(occupancy_grid_localizer_container)
        else:
            actions.append(
                LogInfo(
                    msg=(
                        '[isaac_grid_localization] localizer disabled for this run; '
                        'bridge fallback initial pose remains enabled'
                    )
                )
            )

    isaac_to_nav2_pose_node = Node(
        package='isaac_nav2_pose_bridge',
        executable='isaac_to_nav2_pose',
        name='isaac_to_nav2_pose',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'output_topic': output_topic,
                'pose_stamped_topic': pose_stamped_topic,
                'pose_with_covariance_topic': pose_with_covariance_topic,
                'enable_pose_stamped_input': enable_pose_stamped_input,
                'enable_pose_with_covariance_input': enable_pose_with_covariance_input,
                'output_frame_id': output_frame_id,
                'fallback_initial_pose_enabled': fallback_initial_pose_enabled,
                'fallback_initial_pose_wait_sec': fallback_initial_pose_wait_sec,
                'fallback_initial_pose_publish_count': fallback_initial_pose_publish_count,
                'fallback_initial_pose_publish_period_sec': (
                    fallback_initial_pose_publish_period_sec
                ),
                'fallback_initial_pose_x': fallback_initial_pose_x,
                'fallback_initial_pose_y': fallback_initial_pose_y,
                'fallback_initial_pose_yaw': fallback_initial_pose_yaw,
            }
        ],
    )
    actions.append(isaac_to_nav2_pose_node)
    return actions


def generate_launch_description():
    """Create launch description for occupancy-grid localization + bridge."""
    return LaunchDescription(
        [
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
                default_value='true',
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
            DeclareLaunchArgument(
                'occupancy_grid_localizer_enabled',
                default_value='true',
                description='Start occupancy grid localizer pipeline when map format is supported.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
