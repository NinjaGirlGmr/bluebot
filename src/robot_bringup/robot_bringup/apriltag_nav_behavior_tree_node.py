#!/usr/bin/env python3

from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional
import math

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from std_srvs.srv import Empty
import yaml


BT_SUCCESS = 'SUCCESS'
BT_FAILURE = 'FAILURE'
BT_RUNNING = 'RUNNING'
DOCK_STAGE_APPROACH = 'approach'
DOCK_STAGE_PAUSE = 'pause'
DOCK_STAGE_SLEEP = 'sleep'
DOCK_STAGE_RESUME = 'resume'
RELOCALIZE_STAGE_PAUSE = 'pause'
RELOCALIZE_STAGE_APPROACH = 'approach'
RELOCALIZE_STAGE_TRIGGER = 'trigger_localization'
RELOCALIZE_STAGE_WAIT = 'wait_after_localization'
RELOCALIZE_STAGE_BACKAWAY = 'backaway'
RELOCALIZE_STAGE_SPIN = 'spin'
RELOCALIZE_STAGE_RESUME = 'resume'


class BTNode:
    def tick(self) -> str:
        raise NotImplementedError


class ConditionNode(BTNode):
    def __init__(self, condition_fn: Callable[[], bool]) -> None:
        self._condition_fn = condition_fn

    def tick(self) -> str:
        return BT_SUCCESS if self._condition_fn() else BT_FAILURE


class ActionNode(BTNode):
    def __init__(self, action_fn: Callable[[], str]) -> None:
        self._action_fn = action_fn

    def tick(self) -> str:
        return self._action_fn()


class SequenceNode(BTNode):
    def __init__(self, children: List[BTNode]) -> None:
        self._children = children

    def tick(self) -> str:
        for child in self._children:
            status = child.tick()
            if status != BT_SUCCESS:
                return status
        return BT_SUCCESS


class SelectorNode(BTNode):
    def __init__(self, children: List[BTNode]) -> None:
        self._children = children

    def tick(self) -> str:
        for child in self._children:
            status = child.tick()
            if status in (BT_SUCCESS, BT_RUNNING):
                return status
        return BT_FAILURE


@dataclass
class DetectionSnapshot:
    tag_id: int
    tag_family: str
    distance_m: float
    stamp_ns: int


@dataclass
class TagRule:
    key: str
    tag_id: int
    tag_family: str
    action: str
    priority: int
    cooldown_sec: float
    once: bool
    min_range_m: Optional[float]
    max_range_m: Optional[float]
    duration_sec: float
    linear_x: float
    angular_z: float
    frame_id: str
    x: float
    y: float
    yaw_deg: float
    cancel_active_goal: bool
    covariance_xy: float
    covariance_yaw: float
    dock_timeout_sec: float
    dock_target_range_m: float
    sleep_duration_sec: float
    pause_nav2_during_sleep: bool
    relocalize_wait_sec: float
    backaway_duration_sec: float
    backaway_linear_x: float
    spin_angle_deg: float
    spin_angular_z: float
    message: str


@dataclass
class ActiveTwistAction:
    rule_key: str
    linear_x: float
    angular_z: float
    end_time_ns: int


@dataclass
class ActiveDockAction:
    rule_key: str
    linear_x: float
    angular_z: float
    deadline_ns: int
    dock_target_range_m: float
    sleep_duration_ns: int
    stage: str
    sleep_end_ns: int = 0
    pause_future: Optional[Future] = None
    resume_future: Optional[Future] = None
    lidar_stop_future: Optional[Future] = None
    lidar_start_future: Optional[Future] = None


@dataclass
class ActiveRelocalizeManeuver:
    rule_key: str
    linear_x: float
    angular_z: float
    deadline_ns: int
    dock_target_range_m: float
    relocalize_wait_ns: int
    backaway_linear_x: float
    backaway_duration_ns: int
    spin_angular_z: float
    spin_duration_ns: int
    stage: str
    resume_required: bool = False
    final_status_success: bool = True
    wait_end_ns: int = 0
    backaway_end_ns: int = 0
    spin_end_ns: int = 0
    pause_future: Optional[Future] = None
    resume_future: Optional[Future] = None
    reinitialize_future: Optional[Future] = None


@dataclass
class TagLandmark:
    family: str
    tag_id: int
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    observations: int


class AprilTagNavBehaviorTreeNode(Node):
    def __init__(self) -> None:
        super().__init__('apriltag_nav_behavior_tree')

        self.declare_parameter('detections_topic', '/tag_detections')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('navigate_to_pose_action', '/navigate_to_pose')
        self.declare_parameter('rules_file', '')
        self.declare_parameter('tick_rate_hz', 5.0)
        self.declare_parameter('detection_timeout_sec', 0.75)
        self.declare_parameter('default_cooldown_sec', 8.0)
        self.declare_parameter('stop_action_default_duration_sec', 1.0)
        self.declare_parameter('log_detection_events', False)
        self.declare_parameter(
            'lifecycle_manager_service',
            '/lifecycle_manager_navigation/manage_nodes',
        )
        self.declare_parameter('docking_default_timeout_sec', 12.0)
        self.declare_parameter('docking_default_target_range_m', 0.30)
        self.declare_parameter('docking_default_linear_x', -0.08)
        self.declare_parameter('docking_default_angular_z', 0.0)
        self.declare_parameter('docking_sleep_duration_sec', 15.0)
        self.declare_parameter('docking_pause_nav2', True)
        self.declare_parameter('docking_lidar_power_control_enabled', True)
        self.declare_parameter('lidar_stop_motor_service', '/stop_motor')
        self.declare_parameter('lidar_start_motor_service', '/start_motor')
        self.declare_parameter(
            'reinitialize_localization_service',
            '/reinitialize_global_localization',
        )
        self.declare_parameter('landmarks_file', '')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('auto_landmarks_from_map', True)
        self.declare_parameter('use_landmark_yaw', True)
        self.declare_parameter('auto_relocalize_on_landmarks', False)
        self.declare_parameter('auto_relocalize_max_range_m', 2.0)
        self.declare_parameter('auto_relocalize_covariance_xy', 0.25)
        self.declare_parameter('auto_relocalize_covariance_yaw', 0.0685)
        self.declare_parameter('auto_relocalize_cooldown_sec', 20.0)
        self.declare_parameter('auto_relocalize_priority', 50)

        self._detections_topic = str(self.get_parameter('detections_topic').value)
        self._cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self._initial_pose_topic = str(self.get_parameter('initial_pose_topic').value)
        self._navigate_to_pose_action = str(self.get_parameter('navigate_to_pose_action').value)
        self._rules_file = str(self.get_parameter('rules_file').value)
        self._tick_rate_hz = float(self.get_parameter('tick_rate_hz').value)
        self._detection_timeout_sec = float(self.get_parameter('detection_timeout_sec').value)
        self._default_cooldown_sec = float(self.get_parameter('default_cooldown_sec').value)
        self._stop_action_default_duration_sec = float(
            self.get_parameter('stop_action_default_duration_sec').value
        )
        self._log_detection_events = bool(self.get_parameter('log_detection_events').value)
        self._lifecycle_manager_service = str(
            self.get_parameter('lifecycle_manager_service').value
        )
        self._docking_default_timeout_sec = float(
            self.get_parameter('docking_default_timeout_sec').value
        )
        self._docking_default_target_range_m = float(
            self.get_parameter('docking_default_target_range_m').value
        )
        self._docking_default_linear_x = float(
            self.get_parameter('docking_default_linear_x').value
        )
        self._docking_default_angular_z = float(
            self.get_parameter('docking_default_angular_z').value
        )
        self._docking_sleep_duration_sec = float(
            self.get_parameter('docking_sleep_duration_sec').value
        )
        self._docking_pause_nav2 = bool(self.get_parameter('docking_pause_nav2').value)
        self._docking_lidar_power_control_enabled = bool(
            self.get_parameter('docking_lidar_power_control_enabled').value
        )
        self._lidar_stop_motor_service = str(
            self.get_parameter('lidar_stop_motor_service').value
        )
        self._lidar_start_motor_service = str(
            self.get_parameter('lidar_start_motor_service').value
        )
        self._reinitialize_localization_service = str(
            self.get_parameter('reinitialize_localization_service').value
        )
        self._landmarks_file_param = str(self.get_parameter('landmarks_file').value)
        self._map_yaml = str(self.get_parameter('map_yaml').value)
        self._auto_landmarks_from_map = bool(
            self.get_parameter('auto_landmarks_from_map').value
        )
        self._use_landmark_yaw = bool(self.get_parameter('use_landmark_yaw').value)
        self._auto_relocalize_on_landmarks = bool(
            self.get_parameter('auto_relocalize_on_landmarks').value
        )
        self._auto_relocalize_max_range_m = float(
            self.get_parameter('auto_relocalize_max_range_m').value
        )
        self._auto_relocalize_covariance_xy = float(
            self.get_parameter('auto_relocalize_covariance_xy').value
        )
        self._auto_relocalize_covariance_yaw = float(
            self.get_parameter('auto_relocalize_covariance_yaw').value
        )
        self._auto_relocalize_cooldown_sec = float(
            self.get_parameter('auto_relocalize_cooldown_sec').value
        )
        self._auto_relocalize_priority = int(
            self.get_parameter('auto_relocalize_priority').value
        )

        if self._tick_rate_hz <= 0.0:
            self._tick_rate_hz = 5.0
        if self._detection_timeout_sec <= 0.0:
            self._detection_timeout_sec = 0.75
        if self._default_cooldown_sec < 0.0:
            self._default_cooldown_sec = 0.0
        if self._stop_action_default_duration_sec <= 0.0:
            self._stop_action_default_duration_sec = 1.0
        if self._docking_default_timeout_sec <= 0.0:
            self._docking_default_timeout_sec = 12.0
        if self._docking_default_target_range_m <= 0.0:
            self._docking_default_target_range_m = 0.30
        if self._docking_sleep_duration_sec < 0.0:
            self._docking_sleep_duration_sec = 15.0

        self._detections_by_id: Dict[int, DetectionSnapshot] = {}
        self._detections_by_family_id: Dict[tuple[str, int], DetectionSnapshot] = {}
        self._last_trigger_time_ns: Dict[str, int] = {}
        self._fired_once: set[str] = set()
        self._rules_by_key: Dict[str, TagRule] = {}
        self._landmarks_by_family_id: Dict[tuple[str, int], TagLandmark] = {}
        self._landmarks_by_id: Dict[int, TagLandmark] = {}
        self._resolved_landmarks_file = self._resolve_landmarks_file_path()
        self._load_landmarks(self._resolved_landmarks_file)
        try:
            self._rules: List[TagRule] = self._load_rules(self._rules_file)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f'Failed to load rules file "{self._rules_file}": {exc}')
            self._rules = []

        if self._auto_relocalize_on_landmarks:
            auto_rules = self._generate_landmark_relocalize_rules()
            if auto_rules:
                self._rules.extend(auto_rules)
                self._rules.sort(key=lambda r: r.priority, reverse=True)
                self.get_logger().info(
                    f'Auto-generated {len(auto_rules)} relocalize rule(s) from landmarks.'
                )

        self._selected_rule: Optional[TagRule] = None
        self._selected_detection: Optional[DetectionSnapshot] = None
        self._active_twist: Optional[ActiveTwistAction] = None
        self._active_dock: Optional[ActiveDockAction] = None
        self._active_relocalize_maneuver: Optional[ActiveRelocalizeManeuver] = None

        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self._initial_pose_topic, 10
        )
        self.create_subscription(
            AprilTagDetectionArray,
            self._detections_topic,
            self._on_detections,
            qos_profile_sensor_data,
        )

        self._navigate_client = ActionClient(
            self,
            NavigateToPose,
            self._navigate_to_pose_action,
        )

        action_name = self._navigate_to_pose_action.rstrip('/')
        if not action_name:
            action_name = '/navigate_to_pose'
        self._cancel_goal_service_name = f'{action_name}/_action/cancel_goal'
        self._cancel_goal_client = self.create_client(
            CancelGoal,
            self._cancel_goal_service_name,
        )
        self._lifecycle_manager_client = self.create_client(
            ManageLifecycleNodes,
            self._lifecycle_manager_service,
        )
        self._lidar_stop_client = self.create_client(
            Empty,
            self._lidar_stop_motor_service,
        )
        self._lidar_start_client = self.create_client(
            Empty,
            self._lidar_start_motor_service,
        )
        self._reinitialize_localization_client = self.create_client(
            Empty,
            self._reinitialize_localization_service,
        )

        self._tree = SelectorNode(
            [
                SequenceNode(
                    [
                        ConditionNode(self._select_rule_for_tick),
                        ActionNode(self._execute_selected_rule),
                    ]
                ),
                ActionNode(self._idle_action),
            ]
        )

        self.create_timer(1.0 / self._tick_rate_hz, self._tick_tree)

        self.get_logger().info(
            'AprilTag navigation behavior tree started. '
            f'detections_topic={self._detections_topic}, rules={len(self._rules)}, '
            f'rules_file="{self._rules_file}", landmarks_file="{self._resolved_landmarks_file}", '
            f'lifecycle_manager_service="{self._lifecycle_manager_service}", '
            f'lidar_power_control={self._docking_lidar_power_control_enabled}, '
            f'lidar_stop_service="{self._lidar_stop_motor_service}", '
            f'lidar_start_service="{self._lidar_start_motor_service}"'
        )

    def _generate_landmark_relocalize_rules(self) -> List[TagRule]:
        explicit_tag_ids = {r.tag_id for r in self._rules}
        auto_rules: List[TagRule] = []
        for (family, tag_id), _landmark in self._landmarks_by_family_id.items():
            if tag_id in explicit_tag_ids:
                continue
            raw = {
                'name': f'auto_relocalize_{family}_{tag_id}',
                'tag_id': tag_id,
                'tag_family': family,
                'action': 'set_initial_pose',
                'use_landmark_pose': True,
                'max_range_m': self._auto_relocalize_max_range_m,
                'covariance_xy': self._auto_relocalize_covariance_xy,
                'covariance_yaw': self._auto_relocalize_covariance_yaw,
                'cooldown_sec': self._auto_relocalize_cooldown_sec,
                'priority': self._auto_relocalize_priority,
            }
            try:
                rule = self._parse_rule(len(self._rules) + len(auto_rules), raw)
                auto_rules.append(rule)
                self._rules_by_key[rule.key] = rule
            except (TypeError, ValueError) as exc:
                self.get_logger().warning(
                    f'Failed to auto-generate relocalize rule for {family}:{tag_id}: {exc}'
                )
        return auto_rules

    def _resolve_landmarks_file_path(self) -> str:
        explicit = self._landmarks_file_param.strip()
        if explicit:
            return explicit

        if not self._auto_landmarks_from_map:
            return ''

        map_yaml = self._map_yaml.strip()
        if not map_yaml.endswith('.yaml'):
            return ''

        return f'{map_yaml[:-5]}.apriltags.yaml'

    def _load_landmarks(self, landmarks_file: str) -> None:
        self._landmarks_by_family_id.clear()
        self._landmarks_by_id.clear()

        if not landmarks_file:
            return

        file_path = Path(landmarks_file).expanduser()
        if not file_path.is_file():
            self.get_logger().warning(f'Landmarks file not found: {file_path}')
            return

        with file_path.open('r', encoding='utf-8') as f:
            loaded = yaml.safe_load(f) or {}

        raw_tags = loaded.get('tags', [])
        if not isinstance(raw_tags, list):
            self.get_logger().warning(
                f'Landmarks file "{file_path}" has invalid "tags" format; expected a list.'
            )
            return

        loaded_count = 0
        for idx, raw_tag in enumerate(raw_tags):
            if not isinstance(raw_tag, dict):
                self.get_logger().warning(f'Skipping non-dict landmark at index {idx}')
                continue

            try:
                family = str(raw_tag.get('family', 'unknown')).strip() or 'unknown'
                tag_id = int(raw_tag['id'])
                position = raw_tag.get('position', {}) or {}
                orientation = raw_tag.get('orientation', {}) or {}
                observations = int(raw_tag.get('observations', 0))

                landmark = TagLandmark(
                    family=family,
                    tag_id=tag_id,
                    x=float(position.get('x', 0.0)),
                    y=float(position.get('y', 0.0)),
                    z=float(position.get('z', 0.0)),
                    qx=float(orientation.get('x', 0.0)),
                    qy=float(orientation.get('y', 0.0)),
                    qz=float(orientation.get('z', 0.0)),
                    qw=float(orientation.get('w', 1.0)),
                    observations=observations,
                )
            except (TypeError, ValueError, KeyError) as exc:
                self.get_logger().warning(f'Skipping invalid landmark at index {idx}: {exc}')
                continue

            self._landmarks_by_family_id[(landmark.family, landmark.tag_id)] = landmark
            current = self._landmarks_by_id.get(landmark.tag_id)
            if current is None or landmark.observations >= current.observations:
                self._landmarks_by_id[landmark.tag_id] = landmark
            loaded_count += 1

        self.get_logger().info(
            f'Loaded {loaded_count} AprilTag landmarks from {file_path}'
        )

    def _load_rules(self, rules_file: str) -> List[TagRule]:
        if not rules_file:
            self.get_logger().warning('rules_file is empty. No tag behaviors loaded.')
            return []

        rules_path = Path(rules_file).expanduser()
        if not rules_path.is_file():
            self.get_logger().warning(f'rules_file not found: {rules_path}')
            return []

        with rules_path.open('r', encoding='utf-8') as f:
            loaded = yaml.safe_load(f) or {}

        raw_rules = loaded.get('rules', [])
        if not isinstance(raw_rules, list):
            raise ValueError('AprilTag behavior rules file must contain a list under "rules".')

        rules: List[TagRule] = []
        for idx, raw_rule in enumerate(raw_rules):
            if not isinstance(raw_rule, dict):
                self.get_logger().warning(f'Skipping non-dict rule at index {idx}')
                continue

            try:
                rule = self._parse_rule(idx, raw_rule)
            except (TypeError, ValueError) as exc:
                self.get_logger().warning(f'Skipping invalid rule at index {idx}: {exc}')
                continue

            rules.append(rule)
            self._rules_by_key[rule.key] = rule

        rules.sort(key=lambda item: item.priority, reverse=True)
        return rules

    def _parse_rule(self, idx: int, raw_rule: dict) -> TagRule:
        action = str(raw_rule.get('action', 'log')).strip().lower()
        supported_actions = {
            'log',
            'stop',
            'cmd_vel',
            'cancel_navigation',
            'navigate_to_pose',
            'set_initial_pose',
            'dock_then_sleep',
            'dock_relocalize_spin_resume',
        }
        if action not in supported_actions:
            raise ValueError(f'unsupported action "{action}"')

        tag_id = int(raw_rule['tag_id'])
        tag_family = str(raw_rule.get('tag_family', '')).strip()
        key = str(raw_rule.get('name', f'tag_{tag_id}_{action}_{idx}'))

        cooldown_sec = float(raw_rule.get('cooldown_sec', self._default_cooldown_sec))
        if cooldown_sec < 0.0:
            cooldown_sec = 0.0

        min_range_m = raw_rule.get('min_range_m')
        max_range_m = raw_rule.get('max_range_m')
        min_range_value = float(min_range_m) if min_range_m is not None else None
        max_range_value = float(max_range_m) if max_range_m is not None else None

        default_linear_x = 0.0
        default_angular_z = 0.0
        if action == 'dock_then_sleep':
            default_linear_x = self._docking_default_linear_x
            default_angular_z = self._docking_default_angular_z

        dock_timeout_sec = float(raw_rule.get('dock_timeout_sec', self._docking_default_timeout_sec))
        if dock_timeout_sec <= 0.0:
            dock_timeout_sec = self._docking_default_timeout_sec

        dock_target_range_m = float(
            raw_rule.get('dock_target_range_m', self._docking_default_target_range_m)
        )
        if dock_target_range_m <= 0.0:
            dock_target_range_m = self._docking_default_target_range_m

        sleep_duration_sec = float(
            raw_rule.get('sleep_duration_sec', self._docking_sleep_duration_sec)
        )
        if sleep_duration_sec < 0.0:
            sleep_duration_sec = self._docking_sleep_duration_sec

        relocalize_wait_sec = float(raw_rule.get('relocalize_wait_sec', 5.0))
        if relocalize_wait_sec < 0.0:
            relocalize_wait_sec = 5.0

        backaway_duration_sec = float(raw_rule.get('backaway_duration_sec', 1.5))
        if backaway_duration_sec < 0.0:
            backaway_duration_sec = 1.5

        spin_angle_deg = float(raw_rule.get('spin_angle_deg', 360.0))
        if spin_angle_deg < 0.0:
            spin_angle_deg = 360.0

        spin_angular_z = float(raw_rule.get('spin_angular_z', 0.8))

        parsed_rule = TagRule(
            key=key,
            tag_id=tag_id,
            tag_family=tag_family,
            action=action,
            priority=int(raw_rule.get('priority', 0)),
            cooldown_sec=cooldown_sec,
            once=bool(raw_rule.get('once', False)),
            min_range_m=min_range_value,
            max_range_m=max_range_value,
            duration_sec=float(raw_rule.get('duration_sec', 0.0)),
            linear_x=float(raw_rule.get('linear_x', default_linear_x)),
            angular_z=float(raw_rule.get('angular_z', default_angular_z)),
            frame_id=str(raw_rule.get('frame_id', 'map')),
            x=float(raw_rule.get('x', 0.0)),
            y=float(raw_rule.get('y', 0.0)),
            yaw_deg=float(raw_rule.get('yaw_deg', 0.0)),
            cancel_active_goal=bool(raw_rule.get('cancel_active_goal', True)),
            covariance_xy=float(raw_rule.get('covariance_xy', 0.25)),
            covariance_yaw=float(raw_rule.get('covariance_yaw', 0.0685)),
            dock_timeout_sec=dock_timeout_sec,
            dock_target_range_m=dock_target_range_m,
            sleep_duration_sec=sleep_duration_sec,
            pause_nav2_during_sleep=bool(
                raw_rule.get('pause_nav2_during_sleep', self._docking_pause_nav2)
            ),
            relocalize_wait_sec=relocalize_wait_sec,
            backaway_duration_sec=backaway_duration_sec,
            backaway_linear_x=float(raw_rule.get('backaway_linear_x', 0.0)),
            spin_angle_deg=spin_angle_deg,
            spin_angular_z=spin_angular_z,
            message=str(raw_rule.get('message', '')),
        )

        if 'backaway_linear_x' not in raw_rule:
            if abs(parsed_rule.linear_x) > 1.0e-6:
                parsed_rule.backaway_linear_x = -parsed_rule.linear_x
            else:
                parsed_rule.backaway_linear_x = -self._docking_default_linear_x

        use_landmark_pose = bool(raw_rule.get('use_landmark_pose', False))
        x_in_rule = 'x' in raw_rule
        y_in_rule = 'y' in raw_rule
        yaw_in_rule = 'yaw_deg' in raw_rule

        if action in ('navigate_to_pose', 'set_initial_pose'):
            landmark = self._lookup_landmark(tag_id=tag_id, tag_family=tag_family)
            if landmark is not None:
                if use_landmark_pose or not x_in_rule:
                    parsed_rule.x = landmark.x
                if use_landmark_pose or not y_in_rule:
                    parsed_rule.y = landmark.y
                if (use_landmark_pose or not yaw_in_rule) and self._use_landmark_yaw:
                    parsed_rule.yaw_deg = math.degrees(
                        self._quat_to_yaw(
                            (landmark.qx, landmark.qy, landmark.qz, landmark.qw)
                        )
                    )
            elif use_landmark_pose or (not x_in_rule or not y_in_rule):
                family_text = f'{tag_family}:' if tag_family else ''
                self.get_logger().warning(
                    f'Rule "{key}" requested landmark pose for tag '
                    f'{family_text}{tag_id}, but no landmark was found.'
                )

        return parsed_rule

    def _lookup_landmark(self, tag_id: int, tag_family: str) -> Optional[TagLandmark]:
        if tag_family:
            landmark = self._landmarks_by_family_id.get((tag_family, tag_id))
            if landmark is not None:
                return landmark
        return self._landmarks_by_id.get(tag_id)

    @staticmethod
    def _quat_to_yaw(quat: tuple[float, float, float, float]) -> float:
        qx, qy, qz, qw = quat
        siny_cosp = 2.0 * ((qw * qz) + (qx * qy))
        cosy_cosp = 1.0 - (2.0 * ((qy * qy) + (qz * qz)))
        return math.atan2(siny_cosp, cosy_cosp)

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        now_ns = self.get_clock().now().nanoseconds
        for detection in msg.detections:
            pos = detection.pose.pose.pose.position
            distance_m = math.sqrt((pos.x * pos.x) + (pos.y * pos.y) + (pos.z * pos.z))
            tag_id = int(detection.id)
            tag_family = str(detection.family).strip()

            current = self._detections_by_id.get(tag_id)
            if current is None or distance_m < current.distance_m:
                self._detections_by_id[tag_id] = DetectionSnapshot(
                    tag_id=tag_id,
                    tag_family=tag_family,
                    distance_m=distance_m,
                    stamp_ns=now_ns,
                )

            family_key = (tag_family, tag_id)
            current_family = self._detections_by_family_id.get(family_key)
            if current_family is None or distance_m < current_family.distance_m:
                self._detections_by_family_id[family_key] = DetectionSnapshot(
                    tag_id=tag_id,
                    tag_family=tag_family,
                    distance_m=distance_m,
                    stamp_ns=now_ns,
                )

            if self._log_detection_events:
                family_text = f' family={tag_family}' if tag_family else ''
                self.get_logger().info(
                    f'Detected tag {tag_id}{family_text} at {distance_m:.2f} m'
                )

    def _tick_tree(self) -> None:
        status = self._tree.tick()
        if status == BT_FAILURE:
            self.get_logger().debug('Behavior tree tick returned FAILURE')

    def _select_rule_for_tick(self) -> bool:
        now_ns = self.get_clock().now().nanoseconds
        stale_before_ns = now_ns - int(self._detection_timeout_sec * 1.0e9)

        stale_ids = [
            tag_id for tag_id, snap in self._detections_by_id.items()
            if snap.stamp_ns < stale_before_ns
        ]
        for tag_id in stale_ids:
            self._detections_by_id.pop(tag_id, None)

        stale_family_keys = [
            family_key
            for family_key, snap in self._detections_by_family_id.items()
            if snap.stamp_ns < stale_before_ns
        ]
        for family_key in stale_family_keys:
            self._detections_by_family_id.pop(family_key, None)

        if self._active_relocalize_maneuver is not None:
            active_rule = self._rules_by_key.get(self._active_relocalize_maneuver.rule_key)
            if active_rule is not None:
                self._selected_rule = active_rule
                self._selected_detection = self._get_detection_for_rule(active_rule)
                return True
            self._active_relocalize_maneuver = None

        if self._active_dock is not None:
            active_rule = self._rules_by_key.get(self._active_dock.rule_key)
            if active_rule is not None:
                self._selected_rule = active_rule
                self._selected_detection = self._get_detection_for_rule(active_rule)
                return True
            self._active_dock = None

        if self._active_twist is not None:
            active_rule = self._rules_by_key.get(self._active_twist.rule_key)
            if active_rule is not None:
                self._selected_rule = active_rule
                self._selected_detection = None
                return True
            self._active_twist = None

        best_rule: Optional[TagRule] = None
        best_detection: Optional[DetectionSnapshot] = None
        best_rank: Optional[tuple] = None

        for rule in self._rules:
            detection = self._get_detection_for_rule(rule)
            if detection is None:
                continue
            if not self._rule_is_eligible(rule, detection, now_ns):
                continue

            rank = (rule.priority, -detection.distance_m)
            if best_rank is None or rank > best_rank:
                best_rule = rule
                best_detection = detection
                best_rank = rank

        self._selected_rule = best_rule
        self._selected_detection = best_detection
        return self._selected_rule is not None

    def _get_detection_for_rule(self, rule: TagRule) -> Optional[DetectionSnapshot]:
        if rule.tag_family:
            return self._detections_by_family_id.get((rule.tag_family, rule.tag_id))
        return self._detections_by_id.get(rule.tag_id)

    def _rule_is_eligible(
        self,
        rule: TagRule,
        detection: DetectionSnapshot,
        now_ns: int,
    ) -> bool:
        if rule.once and rule.key in self._fired_once:
            return False

        last_trigger_ns = self._last_trigger_time_ns.get(rule.key)
        if last_trigger_ns is not None:
            cooldown_ns = int(rule.cooldown_sec * 1.0e9)
            if now_ns < (last_trigger_ns + cooldown_ns):
                return False

        if rule.min_range_m is not None and detection.distance_m < rule.min_range_m:
            return False
        if rule.max_range_m is not None and detection.distance_m > rule.max_range_m:
            return False
        return True

    def _execute_selected_rule(self) -> str:
        if self._selected_rule is None:
            return BT_FAILURE

        rule = self._selected_rule
        if rule.action == 'log':
            self._action_log(rule)
            return BT_SUCCESS
        if rule.action == 'stop':
            return self._action_timed_cmd_vel(rule, 0.0, 0.0)
        if rule.action == 'cmd_vel':
            return self._action_timed_cmd_vel(rule, rule.linear_x, rule.angular_z)
        if rule.action == 'cancel_navigation':
            self._action_cancel_navigation(rule)
            return BT_SUCCESS
        if rule.action == 'navigate_to_pose':
            self._action_navigate_to_pose(rule)
            return BT_SUCCESS
        if rule.action == 'set_initial_pose':
            self._action_set_initial_pose(rule)
            return BT_SUCCESS
        if rule.action == 'dock_then_sleep':
            return self._action_dock_then_sleep(rule)
        if rule.action == 'dock_relocalize_spin_resume':
            return self._action_dock_relocalize_spin_resume(rule)

        self.get_logger().warning(f'Unsupported action "{rule.action}" in rule "{rule.key}"')
        return BT_FAILURE

    def _idle_action(self) -> str:
        return BT_SUCCESS

    def _action_log(self, rule: TagRule) -> None:
        detection_text = ''
        if self._selected_detection is not None:
            family_text = (
                f', family={self._selected_detection.tag_family}'
                if self._selected_detection.tag_family
                else ''
            )
            detection_text = (
                f' (distance={self._selected_detection.distance_m:.2f}m, '
                f'tag={rule.tag_id}{family_text})'
            )
        message = rule.message or f'AprilTag rule triggered: {rule.key}'
        self.get_logger().info(message + detection_text)
        self._mark_rule_triggered(rule)

    def _action_timed_cmd_vel(self, rule: TagRule, linear_x: float, angular_z: float) -> str:
        now_ns = self.get_clock().now().nanoseconds
        active = self._active_twist

        if active is None or active.rule_key != rule.key:
            duration_sec = rule.duration_sec
            if duration_sec <= 0.0:
                duration_sec = self._stop_action_default_duration_sec
            end_time_ns = now_ns + int(duration_sec * 1.0e9)
            self._active_twist = ActiveTwistAction(
                rule_key=rule.key,
                linear_x=linear_x,
                angular_z=angular_z,
                end_time_ns=end_time_ns,
            )
            self._mark_rule_triggered(rule)
            self.get_logger().info(
                f'Executing timed cmd_vel rule "{rule.key}" '
                f'(vx={linear_x:.3f}, wz={angular_z:.3f}, duration={duration_sec:.2f}s)'
            )
            active = self._active_twist

        if active is None:
            return BT_FAILURE

        self._publish_cmd_vel(active.linear_x, active.angular_z)

        if now_ns < active.end_time_ns:
            return BT_RUNNING

        self._active_twist = None
        return BT_SUCCESS

    def _action_dock_then_sleep(self, rule: TagRule) -> str:
        now_ns = self.get_clock().now().nanoseconds
        active = self._active_dock

        if active is None or active.rule_key != rule.key:
            if rule.cancel_active_goal:
                self._request_cancel_navigation(rule.key)

            sleep_duration_ns = int(rule.sleep_duration_sec * 1.0e9)
            deadline_ns = now_ns + int(rule.dock_timeout_sec * 1.0e9)

            active = ActiveDockAction(
                rule_key=rule.key,
                linear_x=rule.linear_x,
                angular_z=rule.angular_z,
                deadline_ns=deadline_ns,
                dock_target_range_m=rule.dock_target_range_m,
                sleep_duration_ns=sleep_duration_ns,
                stage=DOCK_STAGE_APPROACH,
            )
            self._active_dock = active
            self._mark_rule_triggered(rule)

            direction = 'backward' if active.linear_x < 0.0 else 'forward'
            self.get_logger().info(
                f'Rule "{rule.key}" starting dock_then_sleep '
                f'(direction={direction}, target_range={active.dock_target_range_m:.2f}m, '
                f'timeout={rule.dock_timeout_sec:.1f}s, sleep={rule.sleep_duration_sec:.1f}s)'
            )

        if active.stage == DOCK_STAGE_APPROACH:
            detection = self._get_detection_for_rule(rule)
            if detection is not None and detection.distance_m <= active.dock_target_range_m:
                self._publish_cmd_vel(0.0, 0.0)
                self.get_logger().info(
                    f'Rule "{rule.key}" docked at range {detection.distance_m:.2f}m. '
                    'Entering sleep mode.'
                )

                active.stage = DOCK_STAGE_PAUSE
                return BT_RUNNING

            if now_ns >= active.deadline_ns:
                self._publish_cmd_vel(0.0, 0.0)
                self._active_dock = None
                self.get_logger().warning(
                    f'Rule "{rule.key}" dock_then_sleep timed out before dock completion.'
                )
                return BT_FAILURE

            if detection is None:
                # Keep stationary when tag sight is lost; retry until timeout.
                self._publish_cmd_vel(0.0, 0.0)
                return BT_RUNNING

            self._publish_cmd_vel(active.linear_x, active.angular_z)
            return BT_RUNNING

        if active.stage == DOCK_STAGE_PAUSE:
            if rule.pause_nav2_during_sleep:
                if active.pause_future is None:
                    active.pause_future = self._request_lifecycle_command(
                        ManageLifecycleNodes.Request.PAUSE,
                        f'rule "{rule.key}" entering sleep mode',
                    )
                    if active.pause_future is None:
                        active.pause_future = Future()
                        active.pause_future.set_result(None)

                if not active.pause_future.done():
                    return BT_RUNNING

                try:
                    response = active.pause_future.result()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warning(
                        f'Rule "{rule.key}" pause request failed: {exc}. '
                        'Continuing sleep mode.'
                    )
                else:
                    if response is not None and not response.success:
                        self.get_logger().warning(
                            f'Rule "{rule.key}" lifecycle pause returned success=false.'
                        )
                    elif response is not None:
                        self.get_logger().info(
                            f'Rule "{rule.key}" paused lifecycle-managed navigation nodes.'
                        )

            if self._docking_lidar_power_control_enabled:
                if active.lidar_stop_future is None:
                    active.lidar_stop_future = self._request_lidar_motor(
                        start=False,
                        context=f'rule "{rule.key}" entering sleep mode',
                    )
                    if active.lidar_stop_future is None:
                        active.lidar_stop_future = Future()
                        active.lidar_stop_future.set_result(None)

                if not active.lidar_stop_future.done():
                    return BT_RUNNING

                try:
                    active.lidar_stop_future.result()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warning(
                        f'Rule "{rule.key}" failed to stop lidar motor: {exc}.'
                    )
                else:
                    self.get_logger().info(
                        f'Rule "{rule.key}" requested lidar motor stop for sleep mode.'
                    )

            active.stage = DOCK_STAGE_SLEEP
            active.sleep_end_ns = now_ns + active.sleep_duration_ns
            return BT_RUNNING

        if active.stage == DOCK_STAGE_SLEEP:
            self._publish_cmd_vel(0.0, 0.0)
            if now_ns < active.sleep_end_ns:
                return BT_RUNNING

            active.stage = DOCK_STAGE_RESUME
            return BT_RUNNING

        if active.stage == DOCK_STAGE_RESUME:
            if self._docking_lidar_power_control_enabled:
                if active.lidar_start_future is None:
                    active.lidar_start_future = self._request_lidar_motor(
                        start=True,
                        context=f'rule "{rule.key}" leaving sleep mode',
                    )
                    if active.lidar_start_future is None:
                        active.lidar_start_future = Future()
                        active.lidar_start_future.set_result(None)

                if not active.lidar_start_future.done():
                    return BT_RUNNING

                try:
                    active.lidar_start_future.result()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warning(
                        f'Rule "{rule.key}" failed to start lidar motor: {exc}.'
                    )
                else:
                    self.get_logger().info(
                        f'Rule "{rule.key}" requested lidar motor start after sleep mode.'
                    )

            if rule.pause_nav2_during_sleep:
                if active.resume_future is None:
                    active.resume_future = self._request_lifecycle_command(
                        ManageLifecycleNodes.Request.RESUME,
                        f'rule "{rule.key}" leaving sleep mode',
                    )
                    if active.resume_future is None:
                        self._active_dock = None
                        self.get_logger().warning(
                            f'Rule "{rule.key}" could not resume lifecycle manager service.'
                        )
                        return BT_FAILURE

                if not active.resume_future.done():
                    return BT_RUNNING

                try:
                    response = active.resume_future.result()
                except Exception as exc:  # noqa: BLE001
                    self._active_dock = None
                    self.get_logger().warning(
                        f'Rule "{rule.key}" resume request failed: {exc}.'
                    )
                    return BT_FAILURE

                if not response.success:
                    self._active_dock = None
                    self.get_logger().warning(
                        f'Rule "{rule.key}" lifecycle resume returned success=false.'
                    )
                    return BT_FAILURE

                self.get_logger().info(
                    f'Rule "{rule.key}" resumed lifecycle-managed navigation nodes.'
                )

            self._active_dock = None
            self.get_logger().info(
                f'Rule "{rule.key}" completed dock_then_sleep (sleep mode complete).'
            )
            return BT_SUCCESS

        self._active_dock = None
        self.get_logger().warning(
            f'Rule "{rule.key}" entered unknown docking stage "{active.stage}".'
        )
        return BT_FAILURE

    def _finalize_relocalize_maneuver(
        self,
        active: ActiveRelocalizeManeuver,
        success: bool,
    ) -> str:
        active.final_status_success = success
        self._publish_cmd_vel(0.0, 0.0)
        if active.resume_required:
            active.stage = RELOCALIZE_STAGE_RESUME
            return BT_RUNNING

        self._active_relocalize_maneuver = None
        return BT_SUCCESS if success else BT_FAILURE

    def _action_dock_relocalize_spin_resume(self, rule: TagRule) -> str:
        now_ns = self.get_clock().now().nanoseconds
        active = self._active_relocalize_maneuver

        if active is None or active.rule_key != rule.key:
            if rule.cancel_active_goal:
                self._request_cancel_navigation(rule.key)

            relocalize_wait_ns = int(rule.relocalize_wait_sec * 1.0e9)
            backaway_duration_ns = int(rule.backaway_duration_sec * 1.0e9)
            spin_duration_ns = 0
            if abs(rule.spin_angular_z) > 1.0e-6:
                spin_duration_ns = int(
                    abs(math.radians(rule.spin_angle_deg) / rule.spin_angular_z) * 1.0e9
                )

            stage = (
                RELOCALIZE_STAGE_PAUSE
                if rule.pause_nav2_during_sleep
                else RELOCALIZE_STAGE_APPROACH
            )
            active = ActiveRelocalizeManeuver(
                rule_key=rule.key,
                linear_x=rule.linear_x,
                angular_z=rule.angular_z,
                deadline_ns=now_ns + int(rule.dock_timeout_sec * 1.0e9),
                dock_target_range_m=rule.dock_target_range_m,
                relocalize_wait_ns=relocalize_wait_ns,
                backaway_linear_x=rule.backaway_linear_x,
                backaway_duration_ns=backaway_duration_ns,
                spin_angular_z=rule.spin_angular_z,
                spin_duration_ns=spin_duration_ns,
                stage=stage,
            )
            self._active_relocalize_maneuver = active
            self._mark_rule_triggered(rule)
            self.get_logger().info(
                f'Rule "{rule.key}" starting dock_relocalize_spin_resume '
                f'(target_range={rule.dock_target_range_m:.2f}m, '
                f'relocalize_wait={rule.relocalize_wait_sec:.1f}s, '
                f'backaway={rule.backaway_duration_sec:.1f}s @ {rule.backaway_linear_x:.3f} m/s, '
                f'spin={rule.spin_angle_deg:.1f}deg @ {rule.spin_angular_z:.3f} rad/s)'
            )

        if active.stage == RELOCALIZE_STAGE_PAUSE:
            if active.pause_future is None:
                active.pause_future = self._request_lifecycle_command(
                    ManageLifecycleNodes.Request.PAUSE,
                    f'rule "{rule.key}" starting dock_relocalize_spin_resume',
                )
                if active.pause_future is None:
                    self.get_logger().warning(
                        f'Rule "{rule.key}" could not pause lifecycle manager; '
                        'continuing maneuver without pause.'
                    )
                    active.stage = RELOCALIZE_STAGE_APPROACH
                    return BT_RUNNING

            if not active.pause_future.done():
                return BT_RUNNING

            try:
                response = active.pause_future.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(
                    f'Rule "{rule.key}" pause request failed: {exc}. '
                    'Continuing maneuver without pause.'
                )
            else:
                if response is not None and not response.success:
                    self.get_logger().warning(
                        f'Rule "{rule.key}" lifecycle pause returned success=false. '
                        'Continuing maneuver without pause.'
                    )
                else:
                    active.resume_required = True
                    self.get_logger().info(
                        f'Rule "{rule.key}" paused lifecycle-managed navigation nodes.'
                    )

            active.stage = RELOCALIZE_STAGE_APPROACH
            return BT_RUNNING

        if active.stage == RELOCALIZE_STAGE_APPROACH:
            detection = self._get_detection_for_rule(rule)
            if detection is not None and detection.distance_m <= active.dock_target_range_m:
                self._publish_cmd_vel(0.0, 0.0)
                self.get_logger().info(
                    f'Rule "{rule.key}" reached target range at {detection.distance_m:.2f}m.'
                )
                active.stage = RELOCALIZE_STAGE_TRIGGER
                return BT_RUNNING

            if now_ns >= active.deadline_ns:
                self.get_logger().warning(
                    f'Rule "{rule.key}" timed out while approaching AprilTag {rule.tag_id}.'
                )
                return self._finalize_relocalize_maneuver(active, success=False)

            if detection is None:
                # Keep stationary while tag is temporarily lost; retry until timeout.
                self._publish_cmd_vel(0.0, 0.0)
                return BT_RUNNING

            self._publish_cmd_vel(active.linear_x, active.angular_z)
            return BT_RUNNING

        if active.stage == RELOCALIZE_STAGE_TRIGGER:
            if active.reinitialize_future is None:
                active.reinitialize_future = self._request_reinitialize_localization(
                    context=f'rule "{rule.key}" retrigger localization',
                )
                if active.reinitialize_future is None:
                    self.get_logger().warning(
                        f'Rule "{rule.key}" could not call localization reinitialize service; '
                        'continuing.'
                    )
                    active.wait_end_ns = now_ns + active.relocalize_wait_ns
                    active.stage = RELOCALIZE_STAGE_WAIT
                    return BT_RUNNING

            if not active.reinitialize_future.done():
                return BT_RUNNING

            try:
                active.reinitialize_future.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(
                    f'Rule "{rule.key}" reinitialize localization call failed: {exc}'
                )
            else:
                self.get_logger().info(
                    f'Rule "{rule.key}" retriggered localization service.'
                )

            active.wait_end_ns = now_ns + active.relocalize_wait_ns
            active.stage = RELOCALIZE_STAGE_WAIT
            return BT_RUNNING

        if active.stage == RELOCALIZE_STAGE_WAIT:
            self._publish_cmd_vel(0.0, 0.0)
            if now_ns < active.wait_end_ns:
                return BT_RUNNING

            if active.backaway_duration_ns <= 0:
                active.stage = RELOCALIZE_STAGE_SPIN
                active.spin_end_ns = now_ns + active.spin_duration_ns
                return BT_RUNNING

            active.backaway_end_ns = now_ns + active.backaway_duration_ns
            active.stage = RELOCALIZE_STAGE_BACKAWAY
            return BT_RUNNING

        if active.stage == RELOCALIZE_STAGE_BACKAWAY:
            if now_ns < active.backaway_end_ns:
                self._publish_cmd_vel(active.backaway_linear_x, 0.0)
                return BT_RUNNING

            self._publish_cmd_vel(0.0, 0.0)
            active.stage = RELOCALIZE_STAGE_SPIN
            active.spin_end_ns = now_ns + active.spin_duration_ns
            return BT_RUNNING

        if active.stage == RELOCALIZE_STAGE_SPIN:
            if active.spin_duration_ns <= 0:
                return self._finalize_relocalize_maneuver(active, success=True)

            if now_ns < active.spin_end_ns:
                self._publish_cmd_vel(0.0, active.spin_angular_z)
                return BT_RUNNING

            return self._finalize_relocalize_maneuver(active, success=True)

        if active.stage == RELOCALIZE_STAGE_RESUME:
            self._publish_cmd_vel(0.0, 0.0)
            if active.resume_future is None:
                active.resume_future = self._request_lifecycle_command(
                    ManageLifecycleNodes.Request.RESUME,
                    f'rule "{rule.key}" finishing dock_relocalize_spin_resume',
                )
                if active.resume_future is None:
                    self._active_relocalize_maneuver = None
                    self.get_logger().warning(
                        f'Rule "{rule.key}" could not resume lifecycle manager service.'
                    )
                    return BT_FAILURE

            if not active.resume_future.done():
                return BT_RUNNING

            try:
                response = active.resume_future.result()
            except Exception as exc:  # noqa: BLE001
                self._active_relocalize_maneuver = None
                self.get_logger().warning(
                    f'Rule "{rule.key}" resume request failed: {exc}.'
                )
                return BT_FAILURE

            if response is not None and not response.success:
                self._active_relocalize_maneuver = None
                self.get_logger().warning(
                    f'Rule "{rule.key}" lifecycle resume returned success=false.'
                )
                return BT_FAILURE

            self.get_logger().info(
                f'Rule "{rule.key}" resumed lifecycle-managed navigation nodes.'
            )
            self._active_relocalize_maneuver = None
            return BT_SUCCESS if active.final_status_success else BT_FAILURE

        self._active_relocalize_maneuver = None
        self.get_logger().warning(
            f'Rule "{rule.key}" entered unknown relocalize stage "{active.stage}".'
        )
        return BT_FAILURE

    def _request_lifecycle_command(
        self,
        command: int,
        context: str,
    ) -> Optional[Future]:
        if not self._lifecycle_manager_client.service_is_ready():
            self.get_logger().warning(
                f'Lifecycle manager service not ready: {self._lifecycle_manager_service} '
                f'({context})'
            )
            return None

        request = ManageLifecycleNodes.Request()
        request.command = command
        return self._lifecycle_manager_client.call_async(request)

    def _request_reinitialize_localization(self, context: str) -> Optional[Future]:
        if not self._reinitialize_localization_client.service_is_ready():
            self.get_logger().warning(
                f'Localization reinitialize service not ready: '
                f'{self._reinitialize_localization_service} ({context})'
            )
            return None

        request = Empty.Request()
        return self._reinitialize_localization_client.call_async(request)

    def _request_lidar_motor(self, start: bool, context: str) -> Optional[Future]:
        client = self._lidar_start_client if start else self._lidar_stop_client
        service_name = self._lidar_start_motor_service if start else self._lidar_stop_motor_service

        if not client.service_is_ready():
            self.get_logger().warning(
                f'Lidar motor service not ready: {service_name} ({context})'
            )
            return None

        request = Empty.Request()
        return client.call_async(request)

    def _action_cancel_navigation(self, rule: TagRule) -> None:
        self._request_cancel_navigation(rule.key)
        self._mark_rule_triggered(rule)

    def _request_cancel_navigation(self, rule_key: str) -> None:
        request = CancelGoal.Request()
        if self._cancel_goal_client.service_is_ready():
            self._cancel_goal_client.call_async(request)
            self.get_logger().info(f'Rule "{rule_key}" requested navigation cancel.')
        else:
            self.get_logger().warning(
                f'Cancel service not ready: {self._cancel_goal_service_name}'
            )

    def _action_navigate_to_pose(self, rule: TagRule) -> None:
        if rule.cancel_active_goal:
            self._request_cancel_navigation(rule.key)

        if not self._navigate_client.server_is_ready():
            if not self._navigate_client.wait_for_server(timeout_sec=0.0):
                self.get_logger().warning(
                    f'NavigateToPose action server not ready: {self._navigate_to_pose_action}'
                )
                return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = rule.frame_id
        goal_msg.pose.pose.position.x = rule.x
        goal_msg.pose.pose.position.y = rule.y
        goal_msg.pose.pose.position.z = 0.0

        yaw_rad = math.radians(rule.yaw_deg)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw_rad * 0.5)
        goal_msg.pose.pose.orientation.w = math.cos(yaw_rad * 0.5)

        send_goal_future = self._navigate_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda future, rule_key=rule.key: self._on_goal_response(future, rule_key)
        )

        self.get_logger().info(
            f'Rule "{rule.key}" sent NavigateToPose goal '
            f'({rule.x:.2f}, {rule.y:.2f}, yaw={rule.yaw_deg:.1f}deg)'
        )
        self._mark_rule_triggered(rule)

    def _on_goal_response(self, future, rule_key: str) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Rule "{rule_key}" goal send failed: {exc}')
            return

        if not goal_handle.accepted:
            self.get_logger().warning(f'Rule "{rule_key}" NavigateToPose goal was rejected.')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future, key=rule_key: self._on_goal_result(done_future, key)
        )

    def _on_goal_result(self, future, rule_key: str) -> None:
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Rule "{rule_key}" goal result failed: {exc}')
            return
        self.get_logger().info(
            f'Rule "{rule_key}" NavigateToPose result received (status={result.status}).'
        )

    def _action_set_initial_pose(self, rule: TagRule) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = rule.frame_id
        msg.pose.pose.position.x = rule.x
        msg.pose.pose.position.y = rule.y
        msg.pose.pose.position.z = 0.0

        yaw_rad = math.radians(rule.yaw_deg)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw_rad * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw_rad * 0.5)

        msg.pose.covariance[0] = rule.covariance_xy
        msg.pose.covariance[7] = rule.covariance_xy
        msg.pose.covariance[35] = rule.covariance_yaw

        self._initial_pose_pub.publish(msg)
        self.get_logger().info(
            f'Rule "{rule.key}" published initial pose '
            f'({rule.x:.2f}, {rule.y:.2f}, yaw={rule.yaw_deg:.1f}deg)'
        )
        self._mark_rule_triggered(rule)

    def _publish_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_vel_pub.publish(msg)

    def _mark_rule_triggered(self, rule: TagRule) -> None:
        now_ns = self.get_clock().now().nanoseconds
        self._last_trigger_time_ns[rule.key] = now_ns
        if rule.once:
            self._fired_once.add(rule.key)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AprilTagNavBehaviorTreeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
