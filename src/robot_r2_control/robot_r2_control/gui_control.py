from collections import deque
from functools import partial
import math
import os
from pathlib import Path
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from robot_r2_control.joint_control_gui import (
    JointControlGuiMixin,
    JointControlNodeMixin,
)
from robot_r2_interfaces.msg import LiftCommand
from robot_r2_interfaces.srv import (
    Align,
    KfsAction,
    MoveRelative,
    SetBasePose,
    StageOne,
    StageTwoPointOne,
    StageTwoPointTwo,
    StageTwoPointTwoExit,
    StageThree,
    TraverseStep,
)
from std_msgs.msg import Float64


MOTION_KEYS = {'w', 'a', 's', 'd', 'q', 'e'}
KFS_LOADER_PARAMETER_NAMES = {
    'service_timeout_sec',
    'mode_1_sequence',
    'mode_2_sequence',
    'mode_3_sequence',
    'mode_4_sequence',
    'mode_5_sequence',
    'release_sequence',
    'pop_1_sequence',
    'pop_2_sequence',
}
KFS_LOADER_SOURCE_RELATIVE_PATH = Path(
    'src/robot_r2_control/config/kfs_loader.yaml')
STAGE_ONE_PARAMETER_NAMES = {
    'action_1_lift_height_m',
    'action_2_left_m',
    'action_2_backward_m',
    'action_3_lift_height_m',
    'action_4_pixel_tolerance_px',
    'action_5_weapon_rotate_rad',
    'action_5_weapon_grip_m',
    'action_6_backward_m',
    'action_7_weapon_grip_m',
    'action_8_pre_lift_height_m',
    'action_8_weapon_rotate_rad',
    'action_9_lift_height_m',
    'action_10_forward_m',
    'action_11_yaw_delta_rad',
    'lift_tolerance_m',
    'position_tolerance_m',
    'yaw_tolerance_rad',
    'weapon_rotate_tolerance_rad',
    'weapon_grip_tolerance_m',
    'dependency_timeout_sec',
    'move_timeout_sec',
    'lift_timeout_sec',
    'alignment_timeout_sec',
    'weapon_timeout_sec',
}
STAGE_ONE_SOURCE_RELATIVE_PATH = Path(
    'src/robot_r2_control/config/stage_one.yaml')
STEP_TRAVERSE_PARAMETER_NAMES = {
    'dependency_timeout_sec',
    'move_timeout_sec',
    'lift_timeout_sec',
    'a1',
    'a1_backoff',
    'a2',
    'a2_backoff',
    'a3',
    'up_pre_lift_clearance',
    'b1',
    'b2',
    'b3',
    'lift_all_front',
    'lift_all_rear',
    'lift_front_only_front',
    'lift_front_only_rear',
    'lift_rear_only_front',
    'lift_rear_only_rear',
    'lift_down_front',
    'lift_down_rear',
}
STAGE_TWO_POINT_ONE_PARAMETER_NAMES = {
    'dependency_timeout_sec',
    'move_timeout_sec',
    'lift_timeout_sec',
    'detection_timeout_sec',
    'align_timeout_sec',
    'load_timeout_sec',
    'release_timeout_sec',
    'cell_5_3_high_kfs_edge_pose',
    'cell_5_2_high_kfs_edge_pose',
    'cell_5_1_high_kfs_edge_pose',
    'high_kfs_edge_offset',
    'release_edge_offset',
    'detection_sample_count',
    'lift_up_front',
    'lift_up_rear',
    'lift_initial_front',
    'lift_initial_rear',
    'lift_down_front',
    'lift_down_rear',
}
STAGE_TWO_POINT_TWO_PARAMETER_NAMES = {
    'dependency_timeout_sec',
    'pose_timeout_sec',
    'move_timeout_sec',
    'traverse_timeout_sec',
    'detection_timeout_sec',
    'align_timeout_sec',
    'load_timeout_sec',
    'release_timeout_sec',
    'forward_x',
    'lateral_y',
    'cell_heights',
    'initial_forward_index',
    'initial_lateral_index',
    'terminal_forward_index',
    'chassis_front_offset',
    'higher_kfs_edge_offset',
    'lower_kfs_edge_offset',
    'release_edge_offset',
    'detection_sample_count',
    'exit_cell_0_0_pose',
    'exit_x_offset',
    'exit_lift_height',
    'lift_timeout_sec',
}
STEP_TRAVERSE_SOURCE_RELATIVE_PATH = Path(
    'src/robot_r2_control/config/step_traverse.yaml')
STAGE_TWO_POINT_ONE_SOURCE_RELATIVE_PATH = Path(
    'src/robot_r2_control/config/stage_two_point_one.yaml')
STAGE_TWO_POINT_TWO_SOURCE_RELATIVE_PATH = Path(
    'src/robot_r2_control/config/stage_two_point_two.yaml')
RELOCALIZATION_FIELDS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


def normalize_motion_key(keysym):
    key = str(keysym).lower()
    return key if key in MOTION_KEYS else None


def resolve_source_config(package_share_directory, source_relative_path):
    package_share_path = Path(package_share_directory).resolve()
    search_roots = (package_share_path, *package_share_path.parents)

    for root in search_roots:
        candidate = root / source_relative_path
        if candidate.is_file():
            return os.fspath(candidate)

    for root in search_roots:
        if root.name == 'install':
            return os.fspath(root.parent / source_relative_path)

    return os.fspath(Path.cwd() / source_relative_path)


def resolve_kfs_loader_source_config(package_share_directory):
    return resolve_source_config(
        package_share_directory,
        KFS_LOADER_SOURCE_RELATIVE_PATH,
    )


def resolve_stage_one_source_config(package_share_directory):
    return resolve_source_config(
        package_share_directory,
        STAGE_ONE_SOURCE_RELATIVE_PATH,
    )


def resolve_step_traverse_source_config(package_share_directory):
    return resolve_source_config(
        package_share_directory,
        STEP_TRAVERSE_SOURCE_RELATIVE_PATH,
    )


def resolve_stage_two_point_one_source_config(package_share_directory):
    return resolve_source_config(
        package_share_directory,
        STAGE_TWO_POINT_ONE_SOURCE_RELATIVE_PATH,
    )


def resolve_stage_two_point_two_source_config(package_share_directory):
    return resolve_source_config(
        package_share_directory,
        STAGE_TWO_POINT_TWO_SOURCE_RELATIVE_PATH,
    )


def manual_twist_components(active_keys, linear_speed, angular_speed):
    active = set(active_keys)
    return (
        linear_speed * (int('w' in active) - int('s' in active)),
        linear_speed * (int('a' in active) - int('d' in active)),
        angular_speed * (int('q' in active) - int('e' in active)),
    )


def velocity_test_twist_components(kind, linear_speed, angular_speed):
    if kind == 'forward':
        return linear_speed, 0.0, 0.0
    if kind == 'left':
        return 0.0, linear_speed, 0.0
    if kind == 'rotate_left':
        return 0.0, 0.0, angular_speed
    raise ValueError(f'Unknown velocity test: {kind}')


def format_parameter_value(value):
    return format(float(value), '.6g')


def motion_control_text(config):
    manual_linear = format_parameter_value(config['manual_linear_speed'])
    manual_angular = format_parameter_value(config['manual_angular_speed'])
    test_linear = format_parameter_value(
        config['velocity_test_linear_speed'])
    test_angular = format_parameter_value(
        config['velocity_test_angular_speed'])
    test_duration = format_parameter_value(
        config['velocity_test_duration_sec'])
    pose_distance = format_parameter_value(
        config['pose_test_linear_distance'])
    pose_yaw = format_parameter_value(config['pose_test_yaw'])
    return {
        'keyboard_hint': (
            '键盘控制（GUI 窗口聚焦时）\n'
            f'W/S：前后 {manual_linear} m/s  '
            f'A/D：左右平移 {manual_linear} m/s  '
            f'Q/E：左右旋转 {manual_angular} rad/s'
        ),
        'velocity': {
            'forward': f'{test_linear} m/s 前进 {test_duration} s',
            'left': f'{test_linear} m/s 左平移 {test_duration} s',
            'rotate_left': (
                f'{test_angular} rad/s 逆时针旋转 {test_duration} s'),
        },
        'pose': {
            'serial': {
                'forward': f'位置伺服前进 {pose_distance} m（下位机）',
                'left': f'位置伺服左平移 {pose_distance} m（下位机）',
                'rotate_left': (
                    f'位置伺服逆时针旋转 {pose_yaw} rad（下位机）'),
            },
            'odin': {
                'forward': f'位置伺服前进 {pose_distance} m（Odin）',
                'left': f'位置伺服左平移 {pose_distance} m（Odin）',
                'rotate_left': f'位置伺服逆时针旋转 {pose_yaw} rad（Odin）',
            },
        },
        'kfs_alignment': 'KFS 对齐',
        'tip_alignment': '端头对齐',
    }


def parse_relocalization_values(raw_values):
    try:
        value_count = len(raw_values)
    except TypeError as exc:
        raise ValueError('重定位参数必须是数组') from exc
    if value_count != len(RELOCALIZATION_FIELDS):
        raise ValueError('重定位需要 6 个参数')

    values = []
    for name, raw_value in zip(RELOCALIZATION_FIELDS, raw_values):
        try:
            value = float(raw_value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f'{name} 必须是数值') from exc
        if not math.isfinite(value):
            raise ValueError(f'{name} 必须是有限数值')
        values.append(value)
    return tuple(values)


def parse_stage_two_point_one_route(raw_value):
    parts = [part.strip() for part in str(raw_value).split(',')]
    if not parts or any(not part for part in parts):
        raise ValueError('2.1 路线不能为空，格式示例：3,1,2')
    try:
        route = tuple(int(part) for part in parts)
    except ValueError as exc:
        raise ValueError(
            '2.1 路线只能填写用逗号分隔的整数') from exc
    if len(route) > 3:
        raise ValueError('2.1 路线最多包含 3 个格子')
    if any(cell not in (1, 2, 3) for cell in route):
        raise ValueError('2.1 路线格子只能是 1、2、3')
    if len(set(route)) != len(route):
        raise ValueError('2.1 路线不能包含重复格子')
    return route


def summarize_named_parameter_load_result(
        display_name, expected_names, returncode, stdout, stderr):
    output_lines = [
        line.strip()
        for line in f'{stdout}\n{stderr}'.splitlines()
        if line.strip()
    ]
    failures = [line for line in output_lines if ' failed:' in line]
    successful_names = {
        line.removeprefix('Set parameter ').removesuffix(' successful')
        for line in output_lines
        if line.startswith('Set parameter ') and line.endswith(' successful')
    }
    missing = expected_names - successful_names
    if returncode == 0 and not failures and not missing:
        return (
            True,
            f'{display_name}参数写入成功：共 {len(successful_names)} 项',
        )

    details = list(failures)
    if returncode != 0 and not details:
        details.append(f'ros2 param load 退出码 {returncode}')
    if missing and not failures:
        details.append('未确认写入：' + ', '.join(sorted(missing)))
    if not details:
        details.append('没有收到参数写入结果')
    return False, f'{display_name}参数写入失败：' + '；'.join(details)


def summarize_parameter_load_result(returncode, stdout, stderr):
    return summarize_named_parameter_load_result(
        'KFS Load ',
        KFS_LOADER_PARAMETER_NAMES,
        returncode,
        stdout,
        stderr,
    )


def summarize_stage_one_parameter_load_result(returncode, stdout, stderr):
    return summarize_named_parameter_load_result(
        'Step1 ',
        STAGE_ONE_PARAMETER_NAMES,
        returncode,
        stdout,
        stderr,
    )


def make_parameter_load_command(config_path, node_name='/kfs_loader_control'):
    return [
        'ros2',
        'param',
        'load',
        '--no-daemon',
        '--spin-time',
        '2.0',
        node_name,
        config_path,
    ]


def make_stage_one_parameter_load_command(config_path):
    return make_parameter_load_command(config_path, '/stage_one')


class GuiControlNode(JointControlNodeMixin, Node):
    CMD_VEL_TOPIC = '/r2/cmd_vel'
    MOVE_RELATIVE_SERVICE = '/r2/move_relative'
    KFS_ALIGNMENT_SERVICE = '/r2/align_to_kfs'
    TIP_ALIGNMENT_SERVICE = '/r2/align_to_tip'
    KFS_ACTION_SERVICE = '/r2/kfs/action'
    SET_BASE_POSE_SERVICE = '/r2/set_base_pose'
    SET_BASE_POSE_ODIN_SERVICE = '/r2/set_base_pose_odin'
    STEP_TRAVERSE_SERVICE = '/r2/step_traverse'
    STAGE_ONE_SERVICE = '/r2/stage_one'
    STAGE_TWO_POINT_ONE_SERVICE = '/r2/stage_two_point_one'
    STAGE_TWO_POINT_TWO_SERVICE = '/r2/stage_two_point_two'
    STAGE_TWO_POINT_TWO_EXIT_SERVICE = '/r2/stage_two_point_two_exit'
    STAGE_THREE_SERVICE = '/r2/stage_three'
    LIFT_COMMAND_TOPIC = JointControlNodeMixin.LIFT_COMMAND_TOPIC

    # Step2 测试重定位位姿：base_link 在 map 中的目标位姿
    # （x, y, z, roll, pitch, yaw）；yaw=pi 使车头朝 -x，面向梅林区。
    # 2.1：红方 Y 为 -2.2，蓝方镜像为 +2.2；即蓝方在
    # (4,3) 格心 Y=+1.8 的基础上再增加 0.4 m。原有 X 保持不变。
    STEP_TWO_POINT_ONE_RELOCALIZATION_DEFAULT = (
        5.568, -2.2, 0.0, 0.0, 0.0, math.pi)
    # 2.2：(5,2) 格心 (3.4, -3.0)，该格为最低高度层（cell_heights=0.0），
    # 同时也是 2.2 的 initial_index 起点。
    STEP_TWO_POINT_TWO_RELOCALIZATION_DEFAULT = (
        3.4, -3.0, 0.0, 0.0, 0.0, math.pi)
    # 2.2 后续动作从 (0,3) 格心开始，再由服务移动到 (0,0) 并离场。
    STEP_TWO_POINT_TWO_EXIT_RELOCALIZATION_DEFAULT = (
        -2.6, -1.8, 0.0, 0.0, 0.0, math.pi)

    KFS_LOAD_MOTOR_FEEDBACK_TOPICS = (
        JointControlNodeMixin.KFS_LOAD_MOTOR_FEEDBACK_TOPICS)

    # GUI 中可从工作区 src 源码直接加载参数的 Step2 相关节点。
    STEP_TWO_PARAMETER_LOAD_TARGETS = {
        'step_traverse': {
            'display': '台阶跨越',
            'parameter_names': STEP_TRAVERSE_PARAMETER_NAMES,
            'source_relative_path': STEP_TRAVERSE_SOURCE_RELATIVE_PATH,
            'node_name': '/step_traverse',
        },
        'stage_two_point_one': {
            'display': '2.1',
            'parameter_names': STAGE_TWO_POINT_ONE_PARAMETER_NAMES,
            'source_relative_path': STAGE_TWO_POINT_ONE_SOURCE_RELATIVE_PATH,
            'node_name': '/stage_two_point_one',
        },
        'stage_two_point_two': {
            'display': '2.2',
            'parameter_names': STAGE_TWO_POINT_TWO_PARAMETER_NAMES,
            'source_relative_path': STAGE_TWO_POINT_TWO_SOURCE_RELATIVE_PATH,
            'node_name': '/stage_two_point_two',
        },
    }

    FLOAT_CONTROL_PARAMETERS = JointControlNodeMixin.FLOAT_CONTROL_PARAMETERS

    MOTION_PARAMETER_DEFAULTS = {
        'motion_publish_rate': 20.0,
        'manual_linear_speed': 0.2,
        'manual_angular_speed': 1.0,
        'velocity_test_linear_speed': 0.5,
        'velocity_test_angular_speed': 1.57,
        'velocity_test_duration_sec': 1.0,
        'pose_test_linear_distance': 0.5,
        'pose_test_yaw': 1.57,
        'move_timeout_sec': 20.0,
    }
    STAGE_ONE_RELOCALIZATION_DEFAULT = (
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def __init__(self):
        super().__init__('gui_control')

        self.state_lock = threading.RLock()
        self.status_events = deque()
        self.config_generation = 0
        self.kfs_load_feedback_generation = 0
        self.kfs_load_motor_feedback = {
            name: None for name in self.KFS_LOAD_MOTOR_FEEDBACK_TOPICS
        }
        self.active_manual_keys = set()
        self.velocity_test_kind = None
        self.velocity_test_deadline = None
        self.pose_request_in_flight = False
        self.kfs_alignment_request_in_flight = False
        self.tip_alignment_request_in_flight = False
        self.relocalization_request_in_flight = False
        self.step_test_request_in_flight = False
        self.step_test_direction = None
        self.stage_one_request_in_flight = False
        self.stage_one_team = None
        self.stage_two_point_one_request_in_flight = False
        self.stage_two_point_one_mode = None
        self.stage_two_point_one_route = ()
        self.stage_two_point_one_team = None
        self.stage_two_point_two_request_in_flight = False
        self.stage_two_point_two_mode = None
        self.stage_two_point_two_team = None
        self.stage_two_point_two_exit_request_in_flight = False
        self.stage_two_point_two_exit_team = None
        self.stage_three_request_in_flight = False
        self.stage_three_team = None
        self.stage_three_loaded_count = None
        self.kfs_action_request_in_flight = False
        self.kfs_parameter_load_in_flight = False
        self.stage_one_parameter_load_in_flight = False
        self.kfs_loader_config_path = resolve_kfs_loader_source_config(
            get_package_share_directory('robot_r2_control'),
        )
        self.stage_one_config_path = resolve_stage_one_source_config(
            get_package_share_directory('robot_r2_control'),
        )
        self.step_two_parameter_load_in_flight = {
            key: False for key in self.STEP_TWO_PARAMETER_LOAD_TARGETS
        }
        self.step_two_parameter_load_config_paths = {
            key: resolve_source_config(
                get_package_share_directory('robot_r2_control'),
                target['source_relative_path'],
            )
            for key, target in self.STEP_TWO_PARAMETER_LOAD_TARGETS.items()
        }

        self.declare_parameter('lift_min', 0.0)
        self.declare_parameter('lift_max', 0.376)
        for parameters in self.FLOAT_CONTROL_PARAMETERS.values():
            minimum_parameter, minimum_default = parameters['minimum']
            maximum_parameter, maximum_default = parameters['maximum']
            self.declare_parameter(minimum_parameter, minimum_default)
            self.declare_parameter(maximum_parameter, maximum_default)
        for name, default in self.MOTION_PARAMETER_DEFAULTS.items():
            self.declare_parameter(name, default)
        self.declare_parameter(
            'stage_one_relocalization_pose',
            list(self.STAGE_ONE_RELOCALIZATION_DEFAULT),
        )
        self.declare_parameter(
            'stage_two_point_one_relocalization_pose',
            list(self.STEP_TWO_POINT_ONE_RELOCALIZATION_DEFAULT),
        )
        self.declare_parameter(
            'stage_two_point_two_relocalization_pose',
            list(self.STEP_TWO_POINT_TWO_RELOCALIZATION_DEFAULT),
        )
        self.declare_parameter(
            'stage_two_point_two_exit_relocalization_pose',
            list(self.STEP_TWO_POINT_TWO_EXIT_RELOCALIZATION_DEFAULT),
        )

        self.lift_min = float(self.get_parameter('lift_min').value)
        self.lift_max = float(self.get_parameter('lift_max').value)
        self._validate_range('lift', self.lift_min, self.lift_max)

        self.float_control_ranges = {}
        for control_name, parameters in self.FLOAT_CONTROL_PARAMETERS.items():
            minimum_parameter, _ = parameters['minimum']
            maximum_parameter, _ = parameters['maximum']
            minimum = float(self.get_parameter(minimum_parameter).value)
            maximum = float(self.get_parameter(maximum_parameter).value)
            self._validate_range(control_name, minimum, maximum)
            self.float_control_ranges[control_name] = (minimum, maximum)

        self.motion_config = {
            name: float(self.get_parameter(name).value)
            for name in self.MOTION_PARAMETER_DEFAULTS
        }
        error = self._validate_motion_config(self.motion_config)
        if error:
            raise ValueError(error)
        self.stage_one_relocalization_pose = parse_relocalization_values(
            self.get_parameter('stage_one_relocalization_pose').value)
        self.stage_two_point_one_relocalization_pose = (
            parse_relocalization_values(self.get_parameter(
                'stage_two_point_one_relocalization_pose').value))
        self.stage_two_point_two_relocalization_pose = (
            parse_relocalization_values(self.get_parameter(
                'stage_two_point_two_relocalization_pose').value))
        self.stage_two_point_two_exit_relocalization_pose = (
            parse_relocalization_values(self.get_parameter(
                'stage_two_point_two_exit_relocalization_pose').value))

        self.lift_command_publisher = self.create_publisher(
            LiftCommand, self.LIFT_COMMAND_TOPIC, 10)
        self.float_command_publishers = {
            control_name: self.create_publisher(
                Float64, parameters['topic'], 10)
            for control_name, parameters in self.FLOAT_CONTROL_PARAMETERS.items()
        }
        self.cmd_vel_publisher = self.create_publisher(
            Twist, self.CMD_VEL_TOPIC, 10)
        self.kfs_load_feedback_subscribers = {
            name: self.create_subscription(
                Float64,
                topic,
                partial(self._on_kfs_load_motor_feedback, name),
                10,
            )
            for name, topic in self.KFS_LOAD_MOTOR_FEEDBACK_TOPICS.items()
        }
        self.move_relative_client = self.create_client(
            MoveRelative, self.MOVE_RELATIVE_SERVICE)
        self.kfs_alignment_client = self.create_client(
            Align, self.KFS_ALIGNMENT_SERVICE)
        self.tip_alignment_client = self.create_client(
            Align, self.TIP_ALIGNMENT_SERVICE)
        self.kfs_action_client = self.create_client(
            KfsAction, self.KFS_ACTION_SERVICE)
        self.set_base_pose_client = self.create_client(
            SetBasePose, self.SET_BASE_POSE_SERVICE)
        self.set_base_pose_odin_client = self.create_client(
            SetBasePose, self.SET_BASE_POSE_ODIN_SERVICE)
        self.step_traverse_client = self.create_client(
            TraverseStep, self.STEP_TRAVERSE_SERVICE)
        self.stage_one_client = self.create_client(
            StageOne, self.STAGE_ONE_SERVICE)
        self.stage_two_point_one_client = self.create_client(
            StageTwoPointOne, self.STAGE_TWO_POINT_ONE_SERVICE)
        self.stage_two_point_two_client = self.create_client(
            StageTwoPointTwo, self.STAGE_TWO_POINT_TWO_SERVICE)
        self.stage_two_point_two_exit_client = self.create_client(
            StageTwoPointTwoExit, self.STAGE_TWO_POINT_TWO_EXIT_SERVICE)
        self.stage_three_client = self.create_client(
            StageThree, self.STAGE_THREE_SERVICE)
        self.motion_timer = self.create_timer(
            1.0 / self.motion_config['motion_publish_rate'],
            self._publish_motion_tick,
        )
        self.add_on_set_parameters_callback(self._on_parameters_changed)

    @staticmethod
    def _validate_range(name, minimum, maximum):
        if not math.isfinite(minimum):
            raise ValueError(f'{name} minimum must be finite')
        if not math.isfinite(maximum):
            raise ValueError(f'{name} maximum must be finite')
        if minimum >= maximum:
            raise ValueError(f'{name} minimum must be less than maximum')

    @staticmethod
    def _validate_motion_config(config):
        for name, value in config.items():
            if not math.isfinite(value):
                return f'{name} must be finite'
            if value <= 0.0:
                return f'{name} must be greater than zero'
        return ''

    def _on_parameters_changed(self, parameters):
        updates = {parameter.name: parameter.value for parameter in parameters}
        numeric_names = {
            'lift_min',
            'lift_max',
            *self.MOTION_PARAMETER_DEFAULTS.keys(),
        }
        for control in self.FLOAT_CONTROL_PARAMETERS.values():
            numeric_names.add(control['minimum'][0])
            numeric_names.add(control['maximum'][0])

        for name, value in updates.items():
            if name not in numeric_names:
                continue
            if (
                isinstance(value, bool) or
                not isinstance(value, (int, float))
            ):
                return SetParametersResult(
                    successful=False,
                    reason=f'{name} must be numeric',
                )
            if not math.isfinite(float(value)):
                return SetParametersResult(
                    successful=False,
                    reason=f'{name} must be finite',
                )

        with self.state_lock:
            stage_one_relocalization_pose = (
                self.stage_one_relocalization_pose)
            stage_two_point_one_relocalization_pose = (
                self.stage_two_point_one_relocalization_pose)
            stage_two_point_two_relocalization_pose = (
                self.stage_two_point_two_relocalization_pose)
            stage_two_point_two_exit_relocalization_pose = (
                self.stage_two_point_two_exit_relocalization_pose)
            relocalization_names = (
                'stage_one_relocalization_pose',
                'stage_two_point_one_relocalization_pose',
                'stage_two_point_two_relocalization_pose',
                'stage_two_point_two_exit_relocalization_pose',
            )
            parsed_relocalization = {
                'stage_one_relocalization_pose': (
                    stage_one_relocalization_pose),
                'stage_two_point_one_relocalization_pose': (
                    stage_two_point_one_relocalization_pose),
                'stage_two_point_two_relocalization_pose': (
                    stage_two_point_two_relocalization_pose),
                'stage_two_point_two_exit_relocalization_pose': (
                    stage_two_point_two_exit_relocalization_pose),
            }
            for parameter_name in relocalization_names:
                if parameter_name not in updates:
                    continue
                try:
                    parsed_relocalization[parameter_name] = (
                        parse_relocalization_values(updates[parameter_name]))
                except ValueError as exc:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{parameter_name} invalid: {exc}',
                    )

            lift_min = float(updates.get('lift_min', self.lift_min))
            lift_max = float(updates.get('lift_max', self.lift_max))
            try:
                self._validate_range('lift', lift_min, lift_max)
            except ValueError as exc:
                return SetParametersResult(successful=False, reason=str(exc))

            new_ranges = {}
            for control_name, definition in (
                    self.FLOAT_CONTROL_PARAMETERS.items()):
                minimum_name, _ = definition['minimum']
                maximum_name, _ = definition['maximum']
                old_minimum, old_maximum = self.float_control_ranges[
                    control_name]
                minimum = float(updates.get(minimum_name, old_minimum))
                maximum = float(updates.get(maximum_name, old_maximum))
                try:
                    self._validate_range(control_name, minimum, maximum)
                except ValueError as exc:
                    return SetParametersResult(
                        successful=False, reason=str(exc))
                new_ranges[control_name] = (minimum, maximum)

            new_motion_config = dict(self.motion_config)
            for name in self.MOTION_PARAMETER_DEFAULTS:
                if name in updates:
                    new_motion_config[name] = float(updates[name])
            error = self._validate_motion_config(new_motion_config)
            if error:
                return SetParametersResult(successful=False, reason=error)

            old_publish_rate = self.motion_config['motion_publish_rate']
            self.lift_min = lift_min
            self.lift_max = lift_max
            self.float_control_ranges = new_ranges
            self.motion_config = new_motion_config
            self.stage_one_relocalization_pose = (
                parsed_relocalization['stage_one_relocalization_pose'])
            self.stage_two_point_one_relocalization_pose = (
                parsed_relocalization[
                    'stage_two_point_one_relocalization_pose'])
            self.stage_two_point_two_relocalization_pose = (
                parsed_relocalization[
                    'stage_two_point_two_relocalization_pose'])
            self.stage_two_point_two_exit_relocalization_pose = (
                parsed_relocalization[
                    'stage_two_point_two_exit_relocalization_pose'])
            self.config_generation += 1

        new_publish_rate = new_motion_config['motion_publish_rate']
        if abs(new_publish_rate - old_publish_rate) > 1e-9:
            self.motion_timer.cancel()
            self.motion_timer = self.create_timer(
                1.0 / new_publish_rate,
                self._publish_motion_tick,
            )
        return SetParametersResult(successful=True)

    def get_range_snapshot(self):
        with self.state_lock:
            return (
                self.config_generation,
                self.lift_min,
                self.lift_max,
                dict(self.float_control_ranges),
                dict(self.motion_config),
            )

    def get_joint_range_snapshot(self):
        generation, lift_min, lift_max, ranges, _ = self.get_range_snapshot()
        return generation, lift_min, lift_max, ranges

    def publish_lift_command(self, front_lift, rear_lift):
        return super().publish_lift_command(front_lift, rear_lift)

    def publish_float_command(self, control_name, value):
        return super().publish_float_command(control_name, value)

    def _on_kfs_load_motor_feedback(self, motor_name, message):
        return super()._on_kfs_load_motor_feedback(motor_name, message)

    def get_kfs_load_feedback_snapshot(self):
        return super().get_kfs_load_feedback_snapshot()

    @staticmethod
    def _make_twist(x=0.0, y=0.0, yaw=0.0):
        command = Twist()
        command.linear.x = float(x)
        command.linear.y = float(y)
        command.angular.z = float(yaw)
        return command

    def _manual_twist_locked(self):
        components = manual_twist_components(
            self.active_manual_keys,
            self.motion_config['manual_linear_speed'],
            self.motion_config['manual_angular_speed'],
        )
        return self._make_twist(*components)

    def _test_twist_locked(self):
        components = velocity_test_twist_components(
            self.velocity_test_kind,
            self.motion_config['velocity_test_linear_speed'],
            self.motion_config['velocity_test_angular_speed'],
        )
        return self._make_twist(*components)

    def _publish_motion_tick(self):
        command = None
        completed_test = False
        with self.state_lock:
            if self._chassis_service_in_flight_locked():
                return
            if self.active_manual_keys:
                command = self._manual_twist_locked()
            elif self.velocity_test_kind is not None:
                if time.monotonic() < self.velocity_test_deadline:
                    command = self._test_twist_locked()
                else:
                    self.velocity_test_kind = None
                    self.velocity_test_deadline = None
                    command = Twist()
                    completed_test = True
        if command is not None:
            self.cmd_vel_publisher.publish(command)
        if completed_test:
            self._queue_status('速度测试完成，已发送零速度')

    def press_manual_key(self, key):
        if key not in MOTION_KEYS:
            return False
        with self.state_lock:
            if self._chassis_service_in_flight_locked():
                return False
            if key in self.active_manual_keys:
                return False
            self.velocity_test_kind = None
            self.velocity_test_deadline = None
            self.active_manual_keys.add(key)
            command = self._manual_twist_locked()
        self.cmd_vel_publisher.publish(command)
        return True

    def release_manual_key(self, key):
        if key not in MOTION_KEYS:
            return False
        with self.state_lock:
            if key not in self.active_manual_keys:
                return False
            self.active_manual_keys.discard(key)
            command = (
                self._manual_twist_locked()
                if self.active_manual_keys
                else Twist()
            )
        self.cmd_vel_publisher.publish(command)
        return True

    def release_all_manual_keys(self):
        with self.state_lock:
            if not self.active_manual_keys:
                return False
            self.active_manual_keys.clear()
        self.cmd_vel_publisher.publish(Twist())
        return True

    def start_velocity_test(self, kind):
        if kind not in {'forward', 'left', 'rotate_left'}:
            raise ValueError(f'Unknown velocity test: {kind}')
        with self.state_lock:
            if self._chassis_service_in_flight_locked():
                return False
            self.active_manual_keys.clear()
            self.velocity_test_kind = kind
            self.velocity_test_deadline = (
                time.monotonic() +
                self.motion_config['velocity_test_duration_sec']
            )
            command = self._test_twist_locked()
        self.cmd_vel_publisher.publish(command)
        return True

    def request_relative_pose(self, kind, pose_source):
        if pose_source not in (
            MoveRelative.Request.SERIAL,
            MoveRelative.Request.ODIN,
        ):
            return False, f'未知位姿来源：{pose_source}'
        source_label = (
            '下位机'
            if pose_source == MoveRelative.Request.SERIAL
            else 'Odin'
        )
        with self.state_lock:
            if self.pose_request_in_flight:
                return False, '位置伺服正在执行'
            if self.kfs_alignment_request_in_flight:
                return False, 'KFS 对齐正在执行'
            if self.tip_alignment_request_in_flight:
                return False, '端头对齐正在执行'
            if (
                self.relocalization_request_in_flight or
                self.step_test_request_in_flight
            ):
                return False, '底盘操作正在执行'
            if not self.move_relative_client.service_is_ready():
                return False, '/r2/move_relative 服务不可用'

            distance = self.motion_config['pose_test_linear_distance']
            yaw_delta = self.motion_config['pose_test_yaw']
            if kind == 'forward':
                offsets = (distance, 0.0, 0.0)
            elif kind == 'left':
                offsets = (0.0, distance, 0.0)
            elif kind == 'rotate_left':
                offsets = (0.0, 0.0, yaw_delta)
            else:
                raise ValueError(f'Unknown pose test: {kind}')

            timeout_sec = self.motion_config['move_timeout_sec']
            self.active_manual_keys.clear()
            self.velocity_test_kind = None
            self.velocity_test_deadline = None
            self.pose_request_in_flight = True

        self.cmd_vel_publisher.publish(Twist())
        request = MoveRelative.Request()
        request.pose_source = pose_source
        request.forward = offsets[0]
        request.left = offsets[1]
        request.yaw_delta = offsets[2]
        request.position_tolerance = 0.0
        request.yaw_tolerance = 0.0
        request.timeout_sec = timeout_sec
        try:
            future = self.move_relative_client.call_async(request)
        except Exception as exc:
            with self.state_lock:
                self.pose_request_in_flight = False
            return False, f'位置伺服请求发送失败：{exc}'
        future.add_done_callback(self._on_move_complete)
        return True, (
            f'已发送 {source_label} 相对移动：'
            f'前 {offsets[0]:.3f} m，左 {offsets[1]:.3f} m，'
            f'旋转 {offsets[2]:.3f} rad'
        )

    def _on_move_complete(self, future):
        with self.state_lock:
            self.pose_request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self._queue_status(f'位置伺服调用异常：{exc}')
            return
        if response is None:
            self._queue_status('位置伺服调用失败：无响应')
        elif response.success:
            self._queue_status(
                f'位置伺服完成：x={response.final_x:.3f} m，'
                f'y={response.final_y:.3f} m，'
                f'yaw={response.final_yaw:.3f} rad')
        else:
            self._queue_status(f'位置伺服失败：{response.message}')

    def request_kfs_alignment(self):
        with self.state_lock:
            if self.kfs_alignment_request_in_flight:
                return False, 'KFS 对齐正在执行'
            if self.tip_alignment_request_in_flight:
                return False, '端头对齐正在执行'
            if self.pose_request_in_flight:
                return False, '位置伺服正在执行'
            if (
                self.relocalization_request_in_flight or
                self.step_test_request_in_flight
            ):
                return False, '底盘操作正在执行'
            if not self.kfs_alignment_client.service_is_ready():
                return False, '/r2/align_to_kfs 服务不可用'

            self.active_manual_keys.clear()
            self.velocity_test_kind = None
            self.velocity_test_deadline = None
            self.kfs_alignment_request_in_flight = True

        self.cmd_vel_publisher.publish(Twist())
        request = Align.Request()
        # 0.0 -> the alignment node uses its own pixel_tolerance / timeout
        # parameters (dynamic ros2 params on /kfs_alignment).
        request.pixel_tolerance = 0.0
        request.timeout_sec = 0.0
        try:
            future = self.kfs_alignment_client.call_async(request)
        except Exception as exc:
            with self.state_lock:
                self.kfs_alignment_request_in_flight = False
            return False, f'KFS 对齐请求发送失败：{exc}'
        future.add_done_callback(self._on_kfs_alignment_complete)
        return True, '已发送 KFS 对齐请求'

    def _on_kfs_alignment_complete(self, future):
        with self.state_lock:
            self.kfs_alignment_request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self._queue_status(f'KFS 对齐调用异常：{exc}')
            return
        if response is None:
            self._queue_status('KFS 对齐调用失败：无响应')
        elif response.success:
            self._queue_status(
                f'KFS 对齐完成：最终偏差 '
                f'{response.final_offset_x} px')
        else:
            self._queue_status(f'KFS 对齐失败：{response.message}')

    def request_tip_alignment(self):
        with self.state_lock:
            if self.tip_alignment_request_in_flight:
                return False, '端头对齐正在执行'
            if self.kfs_alignment_request_in_flight:
                return False, 'KFS 对齐正在执行'
            if self.pose_request_in_flight:
                return False, '位置伺服正在执行'
            if (
                self.relocalization_request_in_flight or
                self.step_test_request_in_flight
            ):
                return False, '底盘操作正在执行'
            if not self.tip_alignment_client.service_is_ready():
                return False, '/r2/align_to_tip 服务不可用'

            self.active_manual_keys.clear()
            self.velocity_test_kind = None
            self.velocity_test_deadline = None
            self.tip_alignment_request_in_flight = True

        self.cmd_vel_publisher.publish(Twist())
        request = Align.Request()
        # 0.0 -> the alignment node uses its own pixel_tolerance / timeout
        # parameters (dynamic ros2 params on /tip_alignment).
        request.pixel_tolerance = 0.0
        request.timeout_sec = 0.0
        try:
            future = self.tip_alignment_client.call_async(request)
        except Exception as exc:
            with self.state_lock:
                self.tip_alignment_request_in_flight = False
            return False, f'端头对齐请求发送失败：{exc}'
        future.add_done_callback(self._on_tip_alignment_complete)
        return True, '已发送端头对齐请求'

    def _on_tip_alignment_complete(self, future):
        with self.state_lock:
            self.tip_alignment_request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self._queue_status(f'端头对齐调用异常：{exc}')
            return
        if response is None:
            self._queue_status('端头对齐调用失败：无响应')
        elif response.success:
            self._queue_status(
                f'端头对齐完成：最终偏差 '
                f'{response.final_offset_x} px')
        else:
            self._queue_status(f'端头对齐失败：{response.message}')

    @staticmethod
    def _make_set_base_pose_request(values):
        request = SetBasePose.Request()
        for name, value in zip(RELOCALIZATION_FIELDS, values):
            setattr(request, name, value)
        return request

    def request_relocalization(self, raw_values):
        try:
            values = parse_relocalization_values(raw_values)
        except ValueError as exc:
            return False, f'重定位参数无效：{exc}'

        with self.state_lock:
            if self._chassis_service_in_flight_locked():
                return False, '底盘操作正在执行'
            if not self.set_base_pose_client.service_is_ready():
                return False, '/r2/set_base_pose 服务不可用'
            self.active_manual_keys.clear()
            self.velocity_test_kind = None
            self.velocity_test_deadline = None
            self.relocalization_request_in_flight = True

        self.cmd_vel_publisher.publish(Twist())
        request = self._make_set_base_pose_request(values)
        try:
            future = self.set_base_pose_client.call_async(request)
        except Exception as exc:
            with self.state_lock:
                self.relocalization_request_in_flight = False
            return False, f'重定位请求发送失败：{exc}'
        future.add_done_callback(self._on_relocalization_complete)
        return True, (
            f'已发送重定位请求：x={values[0]:.3f} m，'
            f'y={values[1]:.3f} m，z={values[2]:.3f} m，'
            f'roll={values[3]:.3f} rad，'
            f'pitch={values[4]:.3f} rad，yaw={values[5]:.3f} rad'
        )

    def _on_relocalization_complete(self, future):
        with self.state_lock:
            self.relocalization_request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self._queue_status(f'重定位调用异常：{exc}')
            return
        if response is None:
            self._queue_status('重定位调用失败：无响应')
        elif response.success:
            self._queue_status(f'重定位完成：{response.message}')
        else:
            self._queue_status(f'重定位失败：{response.message}')

    def request_up_step_test(self):
        return self._request_step_test(
            TraverseStep.Request.UP)

    def request_down_step_test(self):
        return self._request_step_test(
            TraverseStep.Request.DOWN)

    def _request_step_test(self, direction):
        with self.state_lock:
            if self._chassis_service_in_flight_locked():
                return False, '底盘操作正在执行'
            if not self.step_traverse_client.service_is_ready():
                return False, '/r2/step_traverse 服务不可用'
            self.active_manual_keys.clear()
            self.velocity_test_kind = None
            self.velocity_test_deadline = None
            self.step_test_request_in_flight = True
            self.step_test_direction = direction

        direction_name = (
            '上' if direction == TraverseStep.Request.UP else '下')
        self.cmd_vel_publisher.publish(Twist())
        request = TraverseStep.Request()
        request.direction = direction
        request.distance_to_step = 0.0
        try:
            future = self.step_traverse_client.call_async(request)
        except Exception as exc:
            self._finish_step_test(f'{direction_name}台阶请求发送失败：{exc}')
            return False, f'{direction_name}台阶请求发送失败：{exc}'
        future.add_done_callback(self._on_step_test_complete)
        return True, f'{direction_name}台阶测试：正在跨越（距离 0.0 m）'

    def _on_step_test_complete(self, future):
        with self.state_lock:
            direction = self.step_test_direction
        direction_name = (
            '上' if direction == TraverseStep.Request.UP else '下')
        try:
            response = future.result()
        except Exception as exc:
            self._finish_step_test(f'{direction_name}台阶调用异常：{exc}')
            return
        if response is None:
            self._finish_step_test(f'{direction_name}台阶失败：无响应')
        elif response.success:
            self._finish_step_test(
                f'{direction_name}台阶测试完成：{response.message}')
        else:
            self._finish_step_test(
                f'{direction_name}台阶失败：{response.message}')

    def _finish_step_test(self, message):
        with self.state_lock:
            self.step_test_request_in_flight = False
            self.step_test_direction = None
        self._queue_status(message)


    @staticmethod
    def _stage_one_team_label(team):
        if team == StageOne.Request.RED:
            return '红方'
        if team == StageOne.Request.BLUE:
            return '蓝方'
        raise ValueError(f'Unknown Stage 1 team: {team}')

    @staticmethod
    def _stage_two_team_label(team):
        if team == StageTwoPointOne.Request.RED:
            return '红方'
        if team == StageTwoPointOne.Request.BLUE:
            return '蓝方'
        raise ValueError(f'Unknown Stage 2 team: {team}')

    @staticmethod
    def _stage_three_team_label(team):
        if team == StageThree.Request.RED:
            return '红方'
        if team == StageThree.Request.BLUE:
            return '蓝方'
        raise ValueError(f'Unknown Stage 3 team: {team}')

    @classmethod
    def _stage_two_relocalization_pose(cls, base_pose, team):
        cls._stage_two_team_label(team)
        if team == StageTwoPointOne.Request.RED:
            return base_pose
        mirrored = list(base_pose)
        mirrored[1] = -mirrored[1]
        return tuple(mirrored)

    def request_stage_one(self, team):
        try:
            team_label = self._stage_one_team_label(team)
        except ValueError as exc:
            return False, str(exc)

        with self.state_lock:
            if self._chassis_service_in_flight_locked():
                return False, '底盘操作正在执行'
            if not self.set_base_pose_client.service_is_ready():
                return False, '/r2/set_base_pose 服务不可用'
            if not self.stage_one_client.service_is_ready():
                return False, '/r2/stage_one 服务不可用'
            relocalization_pose = self.stage_one_relocalization_pose
            self.active_manual_keys.clear()
            self.velocity_test_kind = None
            self.velocity_test_deadline = None
            self.stage_one_request_in_flight = True
            self.stage_one_team = team

        self.cmd_vel_publisher.publish(Twist())
        request = self._make_set_base_pose_request(relocalization_pose)
        try:
            future = self.set_base_pose_client.call_async(request)
        except Exception as exc:
            with self.state_lock:
                self.stage_one_request_in_flight = False
                self.stage_one_team = None
            return False, f'Step1 {team_label}重定位请求发送失败：{exc}'
        future.add_done_callback(
            self._on_stage_one_relocalization_complete)
        return True, f'Step1 {team_label}：正在重定位'

    def _on_stage_one_relocalization_complete(self, future):
        with self.state_lock:
            team = self.stage_one_team
        team_label = self._stage_one_team_label(team)
        try:
            response = future.result()
        except Exception as exc:
            self._finish_stage_one(
                f'Step1 {team_label}重定位调用异常：{exc}')
            return
        if response is None:
            self._finish_stage_one(
                f'Step1 {team_label}重定位失败：无响应')
            return
        if not response.success:
            self._finish_stage_one(
                f'Step1 {team_label}重定位失败：{response.message}')
            return

        self._queue_status(
            f'Step1 {team_label}：重定位完成，正在调用 Step1')
        request = StageOne.Request()
        request.team = team
        try:
            stage_future = self.stage_one_client.call_async(request)
        except Exception as exc:
            self._finish_stage_one(
                f'Step1 {team_label}请求发送失败：{exc}')
            return
        stage_future.add_done_callback(self._on_stage_one_complete)

    def _on_stage_one_complete(self, future):
        with self.state_lock:
            team = self.stage_one_team
        team_label = self._stage_one_team_label(team)
        try:
            response = future.result()
        except Exception as exc:
            self._finish_stage_one(
                f'Step1 {team_label}调用异常：{exc}')
            return
        if response is None:
            self._finish_stage_one(f'Step1 {team_label}失败：无响应')
        elif response.success:
            self._finish_stage_one(
                f'Step1 {team_label}完成：{response.message}')
        else:
            self._finish_stage_one(
                f'Step1 {team_label}失败：{response.message}')

    def _finish_stage_one(self, message):
        with self.state_lock:
            self.stage_one_request_in_flight = False
            self.stage_one_team = None
        self._queue_status(message)

    @staticmethod
    def _stage_two_point_one_mode_label(mode, route_cells=()):
        if mode == StageTwoPointOne.Request.STANDARD:
            return '正常'
        if mode == StageTwoPointOne.Request.SKIP:
            return 'skip'
        if mode == StageTwoPointOne.Request.ROUTE:
            route_text = ','.join(str(cell) for cell in route_cells)
            return f'路线[{route_text}]'
        raise ValueError(f'Unknown Stage 2.1 mode: {mode}')

    def request_stage_two_point_one(self, team, mode, route_cells=()):
        try:
            team_label = self._stage_two_team_label(team)
            route = tuple(int(cell) for cell in route_cells)
            mode_label = self._stage_two_point_one_mode_label(mode, route)
            if mode == StageTwoPointOne.Request.ROUTE:
                route = parse_stage_two_point_one_route(
                    ','.join(str(cell) for cell in route))
            elif route:
                raise ValueError('非路线模式不能携带 2.1 路线')
        except ValueError as exc:
            return False, str(exc)
        with self.state_lock:
            if self._chassis_service_in_flight_locked():
                return False, '底盘操作正在执行'
            if not self.set_base_pose_odin_client.service_is_ready():
                return False, '/r2/set_base_pose_odin 服务不可用'
            if not self.stage_two_point_one_client.service_is_ready():
                return False, '/r2/stage_two_point_one 服务不可用'
            self.active_manual_keys.clear()
            self.velocity_test_kind = None
            self.velocity_test_deadline = None
            self.stage_two_point_one_request_in_flight = True
            self.stage_two_point_one_mode = int(mode)
            self.stage_two_point_one_route = route
            self.stage_two_point_one_team = team
            relocalization_pose = self._stage_two_relocalization_pose(
                self.stage_two_point_one_relocalization_pose, team)

        self.cmd_vel_publisher.publish(Twist())
        request = self._make_set_base_pose_request(
            relocalization_pose)
        try:
            future = self.set_base_pose_odin_client.call_async(request)
        except Exception as exc:
            with self.state_lock:
                self.stage_two_point_one_request_in_flight = False
                self.stage_two_point_one_mode = None
                self.stage_two_point_one_route = ()
                self.stage_two_point_one_team = None
            return False, (
                f'2.1 {team_label}测试重定位请求发送失败：{exc}')
        future.add_done_callback(
            self._on_stage_two_point_one_relocalization_complete)
        return True, (
            f'2.1 {team_label}{mode_label}测试：'
            '正在重定位到测试起点')

    def _on_stage_two_point_one_relocalization_complete(self, future):
        with self.state_lock:
            team = self.stage_two_point_one_team
        team_label = self._stage_two_team_label(team)
        try:
            response = future.result()
        except Exception as exc:
            self._finish_stage_two_point_one(
                f'2.1 {team_label}测试重定位调用异常：{exc}')
            return
        if response is None:
            self._finish_stage_two_point_one(
                f'2.1 {team_label}测试重定位失败：无响应')
            return
        if not response.success:
            self._finish_stage_two_point_one(
                f'2.1 {team_label}测试重定位失败：{response.message}')
            return

        with self.state_lock:
            mode = self.stage_two_point_one_mode
            route = self.stage_two_point_one_route
            team = self.stage_two_point_one_team
        team_label = self._stage_two_team_label(team)
        mode_label = self._stage_two_point_one_mode_label(mode, route)
        self._queue_status(
            f'2.1 {team_label}{mode_label}测试：'
            '重定位完成，正在调用 2.1')

        request = StageTwoPointOne.Request()
        request.team = team
        request.loaded_count = 0
        request.mode = mode
        request.route_cells = list(route)
        try:
            step_future = self.stage_two_point_one_client.call_async(request)
        except Exception as exc:
            self._finish_stage_two_point_one(
                f'2.1 {team_label}{mode_label}测试请求发送失败：'
                f'{exc}')
            return
        step_future.add_done_callback(self._on_stage_two_point_one_complete)

    def _on_stage_two_point_one_complete(self, future):
        with self.state_lock:
            mode = self.stage_two_point_one_mode
            route = self.stage_two_point_one_route
            team = self.stage_two_point_one_team
        team_label = self._stage_two_team_label(team)
        mode_label = self._stage_two_point_one_mode_label(mode, route)
        try:
            response = future.result()
        except Exception as exc:
            self._finish_stage_two_point_one(
                f'2.1 {team_label}{mode_label}测试调用异常：{exc}')
            return
        if response is None:
            self._finish_stage_two_point_one(
                f'2.1 {team_label}{mode_label}测试失败：无响应')
        elif response.success:
            self._finish_stage_two_point_one(
                f'2.1 {team_label}{mode_label}测试完成：'
                f'{response.message}')
        else:
            self._finish_stage_two_point_one(
                f'2.1 {team_label}{mode_label}测试失败：'
                f'{response.message}')

    def _finish_stage_two_point_one(self, message):
        with self.state_lock:
            self.stage_two_point_one_request_in_flight = False
            self.stage_two_point_one_mode = None
            self.stage_two_point_one_route = ()
            self.stage_two_point_one_team = None
        self._queue_status(message)

    def request_stage_two_point_two(self, team, mode):
        try:
            team_label = self._stage_two_team_label(team)
            if mode == StageTwoPointTwo.Request.STANDARD:
                mode_label = '正常'
            elif mode == StageTwoPointTwo.Request.SKIP:
                mode_label = 'skip'
            else:
                raise ValueError(f'Unknown Stage 2.2 GUI mode: {mode}')
        except ValueError as exc:
            return False, str(exc)
        with self.state_lock:
            if self._chassis_service_in_flight_locked():
                return False, '底盘操作正在执行'
            if not self.set_base_pose_odin_client.service_is_ready():
                return False, '/r2/set_base_pose_odin 服务不可用'
            if not self.stage_two_point_two_client.service_is_ready():
                return False, '/r2/stage_two_point_two 服务不可用'
            self.active_manual_keys.clear()
            self.velocity_test_kind = None
            self.velocity_test_deadline = None
            self.stage_two_point_two_request_in_flight = True
            self.stage_two_point_two_mode = int(mode)
            self.stage_two_point_two_team = team
            relocalization_pose = self._stage_two_relocalization_pose(
                self.stage_two_point_two_relocalization_pose, team)

        self.cmd_vel_publisher.publish(Twist())
        request = self._make_set_base_pose_request(
            relocalization_pose)
        try:
            future = self.set_base_pose_odin_client.call_async(request)
        except Exception as exc:
            with self.state_lock:
                self.stage_two_point_two_request_in_flight = False
                self.stage_two_point_two_mode = None
                self.stage_two_point_two_team = None
            return False, f'2.2 {team_label}测试重定位请求发送失败：{exc}'
        future.add_done_callback(
            self._on_stage_two_point_two_relocalization_complete)
        return True, (
            f'2.2 {team_label}{mode_label}测试：正在重定位到测试起点')

    def _on_stage_two_point_two_relocalization_complete(self, future):
        with self.state_lock:
            team = self.stage_two_point_two_team
        team_label = self._stage_two_team_label(team)
        try:
            response = future.result()
        except Exception as exc:
            self._finish_stage_two_point_two(
                f'2.2 {team_label}测试重定位调用异常：{exc}')
            return
        if response is None:
            self._finish_stage_two_point_two(
                f'2.2 {team_label}测试重定位失败：无响应')
            return
        if not response.success:
            self._finish_stage_two_point_two(
                f'2.2 {team_label}测试重定位失败：{response.message}')
            return

        with self.state_lock:
            mode = self.stage_two_point_two_mode
            team = self.stage_two_point_two_team
        team_label = self._stage_two_team_label(team)
        mode_label = (
            'skip'
            if mode == StageTwoPointTwo.Request.SKIP
            else '正常'
        )
        self._queue_status(
            f'2.2 {team_label}{mode_label}测试：重定位完成，正在调用 2.2')

        request = StageTwoPointTwo.Request()
        request.team = team
        request.fake_kfs_decision = StageTwoPointTwo.Request.LEFT
        request.loaded_count = 0
        request.mode = mode
        request.move_cells = []
        request.load_cells = []
        try:
            step_future = self.stage_two_point_two_client.call_async(request)
        except Exception as exc:
            self._finish_stage_two_point_two(
                f'2.2 {team_label}{mode_label}测试请求发送失败：{exc}')
            return
        step_future.add_done_callback(self._on_stage_two_point_two_complete)

    def _on_stage_two_point_two_complete(self, future):
        with self.state_lock:
            mode = self.stage_two_point_two_mode
            team = self.stage_two_point_two_team
        team_label = self._stage_two_team_label(team)
        mode_label = (
            'skip'
            if mode == StageTwoPointTwo.Request.SKIP
            else '正常'
        )
        try:
            response = future.result()
        except Exception as exc:
            self._finish_stage_two_point_two(
                f'2.2 {team_label}{mode_label}测试调用异常：{exc}')
            return
        if response is None:
            self._finish_stage_two_point_two(
                f'2.2 {team_label}{mode_label}测试失败：无响应')
        elif response.success:
            self._finish_stage_two_point_two(
                f'2.2 {team_label}{mode_label}测试完成：{response.message}')
        else:
            self._finish_stage_two_point_two(
                f'2.2 {team_label}{mode_label}测试失败：{response.message}')

    def _finish_stage_two_point_two(self, message):
        with self.state_lock:
            self.stage_two_point_two_request_in_flight = False
            self.stage_two_point_two_mode = None
            self.stage_two_point_two_team = None
        self._queue_status(message)

    def request_stage_two_point_two_exit(self, team):
        try:
            team_label = self._stage_two_team_label(team)
        except ValueError as exc:
            return False, str(exc)
        with self.state_lock:
            if self._chassis_service_in_flight_locked():
                return False, '底盘操作正在执行'
            if not self.set_base_pose_odin_client.service_is_ready():
                return False, '/r2/set_base_pose_odin 服务不可用'
            if not self.stage_two_point_two_exit_client.service_is_ready():
                return False, '/r2/stage_two_point_two_exit 服务不可用'
            self.active_manual_keys.clear()
            self.velocity_test_kind = None
            self.velocity_test_deadline = None
            self.stage_two_point_two_exit_request_in_flight = True
            self.stage_two_point_two_exit_team = team
            relocalization_pose = self._stage_two_relocalization_pose(
                self.stage_two_point_two_exit_relocalization_pose, team)

        self.cmd_vel_publisher.publish(Twist())
        request = self._make_set_base_pose_request(relocalization_pose)
        try:
            future = self.set_base_pose_odin_client.call_async(request)
        except Exception as exc:
            with self.state_lock:
                self.stage_two_point_two_exit_request_in_flight = False
                self.stage_two_point_two_exit_team = None
            return False, (
                f'2.2 后续动作 {team_label}重定位请求发送失败：{exc}')
        future.add_done_callback(
            self._on_stage_two_point_two_exit_relocalization_complete)
        return True, f'2.2 后续动作 {team_label}：正在重定位到 (0,3)'

    def _on_stage_two_point_two_exit_relocalization_complete(self, future):
        with self.state_lock:
            team = self.stage_two_point_two_exit_team
        team_label = self._stage_two_team_label(team)
        try:
            response = future.result()
        except Exception as exc:
            self._finish_stage_two_point_two_exit(
                f'2.2 后续动作 {team_label}重定位调用异常：{exc}')
            return
        if response is None:
            self._finish_stage_two_point_two_exit(
                f'2.2 后续动作 {team_label}重定位失败：无响应')
            return
        if not response.success:
            self._finish_stage_two_point_two_exit(
                f'2.2 后续动作 {team_label}重定位失败：{response.message}')
            return

        self._queue_status(
            f'2.2 后续动作 {team_label}：重定位完成，正在调用离场服务')
        request = StageTwoPointTwoExit.Request()
        request.team = team
        try:
            exit_future = self.stage_two_point_two_exit_client.call_async(
                request)
        except Exception as exc:
            self._finish_stage_two_point_two_exit(
                f'2.2 后续动作 {team_label}请求发送失败：{exc}')
            return
        exit_future.add_done_callback(
            self._on_stage_two_point_two_exit_complete)

    def _on_stage_two_point_two_exit_complete(self, future):
        with self.state_lock:
            team = self.stage_two_point_two_exit_team
        team_label = self._stage_two_team_label(team)
        try:
            response = future.result()
        except Exception as exc:
            self._finish_stage_two_point_two_exit(
                f'2.2 后续动作 {team_label}调用异常：{exc}')
            return
        if response is None:
            self._finish_stage_two_point_two_exit(
                f'2.2 后续动作 {team_label}失败：无响应')
        elif response.success:
            self._finish_stage_two_point_two_exit(
                f'2.2 后续动作 {team_label}完成：{response.message}')
        else:
            self._finish_stage_two_point_two_exit(
                f'2.2 后续动作 {team_label}失败：{response.message}')

    def _finish_stage_two_point_two_exit(self, message):
        with self.state_lock:
            self.stage_two_point_two_exit_request_in_flight = False
            self.stage_two_point_two_exit_team = None
        self._queue_status(message)

    def request_stage_three(self, team, loaded_count):
        try:
            team_label = self._stage_three_team_label(team)
        except ValueError as exc:
            return False, str(exc)
        if loaded_count not in (1, 2, 3):
            return False, (
                f'Stage 3 loaded_count must be 1, 2 or 3, '
                f'got {loaded_count}')

        with self.state_lock:
            if self._chassis_service_in_flight_locked():
                return False, '底盘操作正在执行'
            if self.kfs_action_request_in_flight:
                return False, 'KFS 动作正在执行'
            if not self.stage_three_client.service_is_ready():
                return False, '/r2/stage_three 服务不可用'
            self.active_manual_keys.clear()
            self.velocity_test_kind = None
            self.velocity_test_deadline = None
            self.stage_three_request_in_flight = True
            self.stage_three_team = team
            self.stage_three_loaded_count = loaded_count

        self.cmd_vel_publisher.publish(Twist())
        request = StageThree.Request()
        request.team = team
        request.loaded_count = loaded_count
        try:
            future = self.stage_three_client.call_async(request)
        except Exception as exc:
            with self.state_lock:
                self.stage_three_request_in_flight = False
                self.stage_three_team = None
                self.stage_three_loaded_count = None
            return False, f'Step3 {team_label}请求发送失败：{exc}'
        future.add_done_callback(self._on_stage_three_complete)
        return True, (
            f'Step3 {team_label}（已有 {loaded_count} 个 KFS）：正在执行')

    def _on_stage_three_complete(self, future):
        with self.state_lock:
            team = self.stage_three_team
            loaded_count = self.stage_three_loaded_count
        team_label = self._stage_three_team_label(team)
        prefix = f'Step3 {team_label}（已有 {loaded_count} 个 KFS）'
        try:
            response = future.result()
        except Exception as exc:
            self._finish_stage_three(f'{prefix}调用异常：{exc}')
            return
        if response is None:
            self._finish_stage_three(f'{prefix}失败：无响应')
        elif response.success:
            self._finish_stage_three(f'{prefix}完成：{response.message}')
        else:
            self._finish_stage_three(f'{prefix}失败：{response.message}')

    def _finish_stage_three(self, message):
        with self.state_lock:
            self.stage_three_request_in_flight = False
            self.stage_three_team = None
            self.stage_three_loaded_count = None
        self._queue_status(message)

    def request_kfs_action(self, action, mode=0):
        if action == KfsAction.Request.LOAD:
            mode_labels = {
                KfsAction.Request.MODE_1: (
                    '模式 1：前方装载（当前数量 0，装载到车上）'),
                KfsAction.Request.MODE_2: (
                    '模式 2：前方装载（当前数量 2，留在夹爪）'),
                KfsAction.Request.MODE_3: (
                    '模式 3：上方装载（当前数量 0，装载到车上）'),
                KfsAction.Request.MODE_4: (
                    '模式 4：上方装载（当前数量 2，留在夹爪）'),
                KfsAction.Request.MODE_5: (
                    '模式 5：上方装载（当前数量 1）'),
            }
            if mode not in mode_labels:
                raise ValueError(f'Unknown KFS load mode: {mode}')
            action_label = mode_labels[mode]
        elif action == KfsAction.Request.RELEASE:
            action_label = '释放'
        elif action == KfsAction.Request.POP:
            mode_labels = {
                KfsAction.Request.MODE_1: '弹出模式 1：从夹爪直接放置',
                KfsAction.Request.MODE_2: '弹出模式 2：从车上拿取并放置',
            }
            if mode not in mode_labels:
                raise ValueError(f'Unknown KFS pop mode: {mode}')
            action_label = mode_labels[mode]
        else:
            raise ValueError(f'Unknown KFS action: {action}')

        with self.state_lock:
            if self.kfs_action_request_in_flight:
                return False, 'KFS 动作正在执行'
            if self.stage_three_request_in_flight:
                return False, 'Step3 正在执行'
            if not self.kfs_action_client.service_is_ready():
                return False, '/r2/kfs/action 服务不可用'
            self.kfs_action_request_in_flight = True

        request = KfsAction.Request()
        request.action = action
        if action in (KfsAction.Request.LOAD, KfsAction.Request.POP):
            request.mode = mode
        try:
            future = self.kfs_action_client.call_async(request)
        except Exception as exc:
            with self.state_lock:
                self.kfs_action_request_in_flight = False
            return False, f'KFS {action_label}请求发送失败：{exc}'
        future.add_done_callback(partial(
            self._on_kfs_action_complete,
            action_label=action_label,
        ))
        return True, f'已发送 KFS {action_label}请求'

    def _on_kfs_action_complete(self, future, action_label):
        with self.state_lock:
            self.kfs_action_request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self._queue_status(f'KFS {action_label}调用异常：{exc}')
            return
        if response is None:
            self._queue_status(f'KFS {action_label}调用失败：无响应')
        elif response.success:
            self._queue_status(
                f'KFS {action_label}完成：{response.message}')
        else:
            self._queue_status(
                f'KFS {action_label}失败：{response.message}')

    def _chassis_service_in_flight_locked(self):
        return (
            self.pose_request_in_flight or
            self.kfs_alignment_request_in_flight or
            self.tip_alignment_request_in_flight or
            self.relocalization_request_in_flight or
            self.step_test_request_in_flight or
            self.stage_one_request_in_flight or
            self.stage_two_point_one_request_in_flight or
            self.stage_two_point_two_request_in_flight or
            self.stage_two_point_two_exit_request_in_flight or
            self.stage_three_request_in_flight
        )

    def is_pose_request_in_flight(self):
        with self.state_lock:
            return self.pose_request_in_flight

    def is_chassis_request_in_flight(self):
        with self.state_lock:
            return self._chassis_service_in_flight_locked()

    def is_kfs_action_request_in_flight(self):
        with self.state_lock:
            return (
                self.kfs_action_request_in_flight or
                self.stage_three_request_in_flight
            )

    def request_kfs_parameter_load(self):
        with self.state_lock:
            if self.kfs_parameter_load_in_flight:
                return False, 'KFS Load 参数正在写入'
            if not os.path.isfile(self.kfs_loader_config_path):
                return (
                    False,
                    f'KFS Load 参数文件不存在：'
                    f'{self.kfs_loader_config_path}',
                )
            self.kfs_parameter_load_in_flight = True

        worker = threading.Thread(
            target=self._load_kfs_parameters,
            daemon=True,
        )
        worker.start()
        return (
            True,
            f'正在从 YAML 写入 KFS Load 参数：'
            f'{self.kfs_loader_config_path}',
        )

    def _load_kfs_parameters(self):
        try:
            result = subprocess.run(
                make_parameter_load_command(
                    self.kfs_loader_config_path),
                capture_output=True,
                check=False,
                text=True,
                timeout=15.0,
            )
            _, message = summarize_parameter_load_result(
                result.returncode,
                result.stdout,
                result.stderr,
            )
        except FileNotFoundError:
            message = 'KFS Load 参数写入失败：找不到 ros2 命令'
        except subprocess.TimeoutExpired:
            message = 'KFS Load 参数写入失败：15 秒内未完成'
        except Exception as exc:
            message = f'KFS Load 参数写入异常：{exc}'
        finally:
            with self.state_lock:
                self.kfs_parameter_load_in_flight = False
        self._queue_status(message)

    def is_kfs_parameter_load_in_flight(self):
        with self.state_lock:
            return self.kfs_parameter_load_in_flight

    def request_stage_one_parameter_load(self):
        with self.state_lock:
            if self.stage_one_parameter_load_in_flight:
                return False, 'Step1 参数正在写入'
            if not os.path.isfile(self.stage_one_config_path):
                return (
                    False,
                    f'Step1 参数文件不存在：{self.stage_one_config_path}',
                )
            self.stage_one_parameter_load_in_flight = True

        worker = threading.Thread(
            target=self._load_stage_one_parameters,
            daemon=True,
        )
        worker.start()
        return (
            True,
            f'正在从 YAML 写入 Step1 参数：{self.stage_one_config_path}',
        )

    def _load_stage_one_parameters(self):
        try:
            result = subprocess.run(
                make_stage_one_parameter_load_command(
                    self.stage_one_config_path),
                capture_output=True,
                check=False,
                text=True,
                timeout=15.0,
            )
            _, message = summarize_stage_one_parameter_load_result(
                result.returncode,
                result.stdout,
                result.stderr,
            )
        except FileNotFoundError:
            message = 'Step1 参数写入失败：找不到 ros2 命令'
        except subprocess.TimeoutExpired:
            message = 'Step1 参数写入失败：15 秒内未完成'
        except Exception as exc:
            message = f'Step1 参数写入异常：{exc}'
        finally:
            with self.state_lock:
                self.stage_one_parameter_load_in_flight = False
        self._queue_status(message)

    def is_stage_one_parameter_load_in_flight(self):
        with self.state_lock:
            return self.stage_one_parameter_load_in_flight

    def request_step_two_parameter_load(self, key):
        target = self.STEP_TWO_PARAMETER_LOAD_TARGETS[key]
        display_name = target['display']
        config_path = self.step_two_parameter_load_config_paths[key]
        with self.state_lock:
            if self.step_two_parameter_load_in_flight[key]:
                return False, f'{display_name}参数正在写入'
            if not os.path.isfile(config_path):
                return (
                    False,
                    f'{display_name}参数文件不存在：{config_path}',
                )
            self.step_two_parameter_load_in_flight[key] = True

        worker = threading.Thread(
            target=self._load_step_two_parameters,
            args=(key,),
            daemon=True,
        )
        worker.start()
        return (
            True,
            f'正在从 YAML 写入 {display_name}参数：{config_path}',
        )

    def _load_step_two_parameters(self, key):
        target = self.STEP_TWO_PARAMETER_LOAD_TARGETS[key]
        display_name = target['display']
        config_path = self.step_two_parameter_load_config_paths[key]
        try:
            result = subprocess.run(
                make_parameter_load_command(
                    config_path,
                    target['node_name'],
                ),
                capture_output=True,
                check=False,
                text=True,
                timeout=15.0,
            )
            _, message = summarize_named_parameter_load_result(
                display_name + ' ',
                target['parameter_names'],
                result.returncode,
                result.stdout,
                result.stderr,
            )
        except FileNotFoundError:
            message = f'{display_name}参数写入失败：找不到 ros2 命令'
        except subprocess.TimeoutExpired:
            message = f'{display_name}参数写入失败：15 秒内未完成'
        except Exception as exc:
            message = f'{display_name}参数写入异常：{exc}'
        finally:
            with self.state_lock:
                self.step_two_parameter_load_in_flight[key] = False
        self._queue_status(message)

    def is_step_two_parameter_load_in_flight(self, key):
        with self.state_lock:
            return self.step_two_parameter_load_in_flight[key]

    def stop_chassis(self):
        with self.state_lock:
            self.active_manual_keys.clear()
            self.velocity_test_kind = None
            self.velocity_test_deadline = None
        self.cmd_vel_publisher.publish(Twist())

    def _queue_status(self, message):
        with self.state_lock:
            self.status_events.append(message)

    def pop_status_events(self):
        with self.state_lock:
            events = list(self.status_events)
            self.status_events.clear()
        return events


class GuiControlApp(JointControlGuiMixin):
    KFS_CONTROLS = JointControlGuiMixin.KFS_CONTROLS
    WEAPON_CONTROLS = JointControlGuiMixin.WEAPON_CONTROLS
    FLOAT_CONTROLS = JointControlGuiMixin.FLOAT_CONTROLS

    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Robot R2 GUI 控制')
        self.root.resizable(False, False)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.initialize_joint_ui()
        self.relocalization_values = {
            name: tk.StringVar(value='0.0')
            for name in RELOCALIZATION_FIELDS
        }
        self.team_value = tk.StringVar(value=StageOne.Request.RED)
        self.stage_two_point_one_route_value = tk.StringVar(value='3,1,2')
        self.status_text = tk.StringVar(value='已就绪')

        self.last_lift_command = None
        self.last_float_commands = {}
        self.last_config_generation = -1
        self.last_kfs_load_feedback_generation = -1
        self.joint_control_widgets = []
        self.last_chassis_busy = None
        self.last_kfs_action_busy = None
        self.last_kfs_parameter_load_busy = None
        self.last_stage_one_parameter_load_busy = None
        self.last_step_two_parameter_load_busy = {
            key: None
            for key in GuiControlNode.STEP_TWO_PARAMETER_LOAD_TARGETS
        }
        self._closed = False
        self.chassis_buttons = []
        self.velocity_test_buttons = {}
        self.pose_test_buttons = {}
        self.relocalization_button = None
        self.up_step_test_button = None
        self.down_step_test_button = None
        self.stage_one_button = None
        self.stage_one_parameter_load_button = None
        self.step_traverse_parameter_load_button = None
        self.stage_two_point_one_parameter_load_button = None
        self.stage_two_point_two_parameter_load_button = None
        self.stage_two_point_one_skip_button = None
        self.stage_two_point_one_normal_button = None
        self.stage_two_point_one_route_button = None
        self.stage_two_point_two_skip_button = None
        self.stage_two_point_two_normal_button = None
        self.stage_two_point_two_exit_button = None
        self.stage_three_buttons = []
        self.kfs_test_button = None
        self.tip_test_button = None
        self.kfs_action_buttons = []
        self.kfs_parameter_load_button = None

        self._build_ui()
        self._sync_dynamic_ranges()
        self.root.bind_all('<KeyPress>', self._on_key_press, add='+')
        self.root.bind_all('<KeyRelease>', self._on_key_release, add='+')
        self.root.bind_all('<FocusOut>', self._on_focus_out, add='+')
        self.root.bind('<Unmap>', self._on_window_unmap, add='+')
        self.root.focus_set()
        self.root.after(10, self._poll_ros)

    def _build_ui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.grid(row=0, column=0, sticky='nsew')

        chassis_column = ttk.Frame(main_frame)
        chassis_column.grid(row=0, column=0, sticky='new', padx=(0, 8))
        task_column = ttk.Frame(main_frame)
        task_column.grid(row=0, column=1, sticky='new', padx=(0, 8))
        mechanism_column = ttk.Frame(main_frame)
        mechanism_column.grid(row=0, column=2, sticky='new')

        self.keyboard_hint = ttk.Label(
            chassis_column,
            anchor='center',
            justify='center',
        )
        self.keyboard_hint.grid(
            row=0, column=0, sticky='ew', pady=(0, 10))

        test_frame = ttk.LabelFrame(
            chassis_column, text='底盘测试', padding=12)
        test_frame.grid(row=1, column=0, sticky='ew')
        test_kinds = ('forward', 'left', 'rotate_left')
        pose_sources = ('serial', 'odin')
        for row, kind in enumerate(test_kinds):
            button = ttk.Button(
                test_frame,
                command=partial(self._start_velocity_test, kind),
            )
            button.grid(row=row, column=0, sticky='ew', pady=3)
            self.velocity_test_buttons[kind] = button
            self.chassis_buttons.append(button)
        for column, pose_source in enumerate(pose_sources, start=1):
            for row, kind in enumerate(test_kinds):
                button = ttk.Button(
                    test_frame,
                    command=partial(
                        self._start_pose_test,
                        pose_source,
                        kind,
                    ),
                )
                button.grid(
                    row=row,
                    column=column,
                    sticky='ew',
                    padx=(8, 0),
                    pady=3,
                )
                self.pose_test_buttons[(pose_source, kind)] = button
                self.chassis_buttons.append(button)

        relocalization_frame = ttk.LabelFrame(
            chassis_column, text='重定位', padding=12)
        relocalization_frame.grid(
            row=2, column=0, sticky='ew', pady=(10, 0))
        field_units = {
            'x': 'm',
            'y': 'm',
            'z': 'm',
            'roll': 'rad',
            'pitch': 'rad',
            'yaw': 'rad',
        }
        for index, name in enumerate(RELOCALIZATION_FIELDS):
            row = index // 3
            column = (index % 3) * 2
            label = ttk.Label(
                relocalization_frame,
                text=f'{name} ({field_units[name]})',
            )
            label.grid(
                row=row, column=column, sticky='e',
                padx=(0 if column == 0 else 8, 4), pady=3,
            )
            entry = ttk.Entry(
                relocalization_frame,
                textvariable=self.relocalization_values[name],
                width=9,
            )
            entry.grid(row=row, column=column + 1, sticky='ew', pady=3)
        self.relocalization_button = ttk.Button(
            relocalization_frame,
            text='重定位',
            command=self._start_relocalization,
        )
        self.relocalization_button.grid(
            row=2, column=0, columnspan=6, sticky='ew', pady=(8, 0))
        self.chassis_buttons.append(self.relocalization_button)

        traverse_test_frame = ttk.LabelFrame(
            chassis_column, text='跨越测试', padding=12)
        traverse_test_frame.grid(
            row=3, column=0, sticky='ew', pady=(10, 0))
        self.up_step_test_button = ttk.Button(
            traverse_test_frame,
            text='上一个台阶（距离 0.0 m）',
            command=self._start_up_step_test,
        )
        self.up_step_test_button.grid(row=0, column=0, sticky='ew')
        self.chassis_buttons.append(self.up_step_test_button)
        self.down_step_test_button = ttk.Button(
            traverse_test_frame,
            text='下一个台阶（距离 0.0 m）',
            command=self._start_down_step_test,
        )
        self.down_step_test_button.grid(
            row=0, column=1, sticky='ew', padx=(8, 0))
        self.chassis_buttons.append(self.down_step_test_button)
        self.step_traverse_parameter_load_button = ttk.Button(
            traverse_test_frame,
            text='从 YAML 写入台阶跨越参数',
            command=self._write_step_traverse_parameters,
        )
        self.step_traverse_parameter_load_button.grid(
            row=1, column=0, columnspan=2, sticky='ew', pady=(8, 0))

        team_frame = ttk.LabelFrame(
            task_column, text='比赛队伍（Step1 / Step2 / Step3）', padding=12)
        team_frame.grid(row=0, column=0, sticky='ew')
        ttk.Radiobutton(
            team_frame,
            text='红方（负 Y）',
            variable=self.team_value,
            value=StageOne.Request.RED,
        ).grid(row=0, column=0, sticky='w')
        ttk.Radiobutton(
            team_frame,
            text='蓝方（正 Y）',
            variable=self.team_value,
            value=StageOne.Request.BLUE,
        ).grid(row=0, column=1, sticky='w', padx=(8, 0))

        stage_one_frame = ttk.LabelFrame(
            task_column, text='Step1', padding=12)
        stage_one_frame.grid(
            row=1, column=0, sticky='ew', pady=(8, 0))
        self.stage_one_button = ttk.Button(
            stage_one_frame,
            text='执行 Step1（重定位后执行）',
            command=self._start_stage_one,
        )
        self.stage_one_button.grid(
            row=0, column=0, columnspan=2, sticky='ew')
        self.chassis_buttons.append(self.stage_one_button)
        self.stage_one_parameter_load_button = ttk.Button(
            stage_one_frame,
            text='从 YAML 写入 Step1 参数',
            command=self._write_stage_one_parameters,
        )
        self.stage_one_parameter_load_button.grid(
            row=1,
            column=0,
            columnspan=2,
            sticky='ew',
            pady=(8, 0),
        )

        stage_two_point_one_frame = ttk.LabelFrame(
            task_column, text='Step2.1 测试', padding=12)
        stage_two_point_one_frame.grid(
            row=2, column=0, sticky='ew', pady=(8, 0))
        self.stage_two_point_one_skip_button = ttk.Button(
            stage_two_point_one_frame,
            text='2.1 测试（skip 识别）',
            command=self._start_stage_two_point_one_skip,
        )
        self.stage_two_point_one_skip_button.grid(
            row=0, column=0, sticky='ew')
        self.chassis_buttons.append(self.stage_two_point_one_skip_button)
        self.stage_two_point_one_normal_button = ttk.Button(
            stage_two_point_one_frame,
            text='2.1 测试（正常识别）',
            command=self._start_stage_two_point_one_normal,
        )
        self.stage_two_point_one_normal_button.grid(
            row=0, column=1, sticky='ew', padx=(8, 0))
        self.chassis_buttons.append(self.stage_two_point_one_normal_button)
        ttk.Label(
            stage_two_point_one_frame,
            text='路线格子（逗号分隔）',
        ).grid(row=1, column=0, sticky='w', pady=(8, 0))
        ttk.Entry(
            stage_two_point_one_frame,
            textvariable=self.stage_two_point_one_route_value,
            width=16,
        ).grid(row=1, column=1, sticky='ew', padx=(8, 0), pady=(8, 0))
        self.stage_two_point_one_route_button = ttk.Button(
            stage_two_point_one_frame,
            text='2.1 测试（路线直接装载）',
            command=self._start_stage_two_point_one_route,
        )
        self.stage_two_point_one_route_button.grid(
            row=2, column=0, columnspan=2, sticky='ew', pady=(8, 0))
        self.chassis_buttons.append(self.stage_two_point_one_route_button)
        self.stage_two_point_one_parameter_load_button = ttk.Button(
            stage_two_point_one_frame,
            text='从 YAML 写入 2.1 参数',
            command=self._write_stage_two_point_one_parameters,
        )
        self.stage_two_point_one_parameter_load_button.grid(
            row=3, column=0, columnspan=2, sticky='ew', pady=(8, 0))

        stage_two_point_two_frame = ttk.LabelFrame(
            task_column, text='Step2.2 测试', padding=12)
        stage_two_point_two_frame.grid(
            row=3, column=0, sticky='ew', pady=(8, 0))
        self.stage_two_point_two_skip_button = ttk.Button(
            stage_two_point_two_frame,
            text='2.2 测试（skip 识别）',
            command=self._start_stage_two_point_two_skip,
        )
        self.stage_two_point_two_skip_button.grid(
            row=0, column=0, sticky='ew')
        self.chassis_buttons.append(self.stage_two_point_two_skip_button)
        self.stage_two_point_two_normal_button = ttk.Button(
            stage_two_point_two_frame,
            text='2.2 测试（正常识别）',
            command=self._start_stage_two_point_two_normal,
        )
        self.stage_two_point_two_normal_button.grid(
            row=0, column=1, sticky='ew', padx=(8, 0))
        self.chassis_buttons.append(self.stage_two_point_two_normal_button)
        self.stage_two_point_two_parameter_load_button = ttk.Button(
            stage_two_point_two_frame,
            text='从 YAML 写入 2.2 参数',
            command=self._write_stage_two_point_two_parameters,
        )
        self.stage_two_point_two_parameter_load_button.grid(
            row=1, column=0, columnspan=2, sticky='ew', pady=(8, 0))
        self.stage_two_point_two_exit_button = ttk.Button(
            stage_two_point_two_frame,
            text='2.2 后续动作（重定位后执行）',
            command=self._start_stage_two_point_two_exit,
        )
        self.stage_two_point_two_exit_button.grid(
            row=2, column=0, columnspan=2, sticky='ew', pady=(8, 0))
        self.chassis_buttons.append(self.stage_two_point_two_exit_button)

        stage_three_frame = ttk.LabelFrame(
            task_column, text='Step3', padding=12)
        stage_three_frame.grid(
            row=4, column=0, sticky='ew', pady=(8, 0))
        for column, loaded_count in enumerate((1, 2, 3)):
            button = ttk.Button(
                stage_three_frame,
                text=f'已有 {loaded_count} 个 KFS',
                command=partial(self._start_stage_three, loaded_count),
            )
            button.grid(
                row=0,
                column=column,
                sticky='ew',
                padx=(0 if column == 0 else 8, 0),
            )
            self.stage_three_buttons.append(button)
            self.chassis_buttons.append(button)

        kfs_test_frame = ttk.LabelFrame(
            task_column, text='KFS 测试', padding=12)
        kfs_test_frame.grid(row=5, column=0, sticky='ew', pady=(8, 0))
        self.kfs_test_button = ttk.Button(
            kfs_test_frame,
            command=self._start_kfs_alignment_test,
        )
        self.kfs_test_button.grid(row=0, column=0, sticky='ew')
        self.chassis_buttons.append(self.kfs_test_button)
        tip_alignment_frame = ttk.LabelFrame(
            chassis_column, text='端头对齐', padding=12)
        tip_alignment_frame.grid(
            row=5, column=0, sticky='ew', pady=(8, 0))
        self.tip_test_button = ttk.Button(
            tip_alignment_frame,
            command=self._start_tip_alignment_test,
        )
        self.tip_test_button.grid(row=0, column=0, sticky='ew')
        self.chassis_buttons.append(self.tip_test_button)
        kfs_actions = (
            (
                '模式 1：前方，数量 0，装载到车上',
                KfsAction.Request.LOAD, KfsAction.Request.MODE_1,
            ),
            (
                '模式 2：前方，数量 2，留在夹爪',
                KfsAction.Request.LOAD, KfsAction.Request.MODE_2,
            ),
            (
                '模式 3：上方，数量 0，装载到车上',
                KfsAction.Request.LOAD, KfsAction.Request.MODE_3,
            ),
            (
                '模式 4：上方，数量 2，留在夹爪',
                KfsAction.Request.LOAD, KfsAction.Request.MODE_4,
            ),
            (
                '模式 5：上方，数量 1',
                KfsAction.Request.LOAD, KfsAction.Request.MODE_5,
            ),
            ('释放', KfsAction.Request.RELEASE, 0),
            (
                '弹出模式 1：从夹爪直接放置',
                KfsAction.Request.POP, KfsAction.Request.MODE_1,
            ),
            (
                '弹出模式 2：从车上拿取并放置',
                KfsAction.Request.POP, KfsAction.Request.MODE_2,
            ),
        )
        for index, (label, action, mode) in enumerate(
                kfs_actions):
            button = ttk.Button(
                kfs_test_frame,
                text=label,
                command=partial(
                    self._start_kfs_action_test,
                    action,
                    mode,
                ),
            )
            button.grid(
                row=1 + index // 2,
                column=index % 2,
                sticky='ew',
                padx=(0 if index % 2 == 0 else 8, 0),
                pady=(8, 0),
            )
            self.kfs_action_buttons.append(button)
        self.kfs_parameter_load_button = ttk.Button(
            kfs_test_frame,
            text='从 YAML 写入 KFS Load 参数',
            command=self._write_kfs_load_parameters,
        )
        self.kfs_parameter_load_button.grid(
            row=6,
            column=0,
            columnspan=2,
            sticky='ew',
            pady=(8, 0),
        )

        self.build_joint_controls(mechanism_column)

        status_label = ttk.Label(
            main_frame, textvariable=self.status_text, anchor='w')
        status_label.grid(
            row=1, column=0, columnspan=3, sticky='ew', pady=(8, 0))

    def _create_control_frame(self, parent, row, title, controls):
        return super()._create_control_frame(parent, row, title, controls)

    @staticmethod
    def _create_slider(
        parent, row, label, variable, minimum, maximum, resolution,
        release_callback,
    ):
        return JointControlGuiMixin._create_slider(
            parent,
            row,
            label,
            variable,
            minimum,
            maximum,
            resolution,
            release_callback,
        )

    def _on_key_press(self, event):
        if isinstance(event.widget, (tk.Entry, ttk.Entry)):
            return
        key = normalize_motion_key(event.keysym)
        if key is not None and self.node.press_manual_key(key):
            self.status_text.set(f'底盘键盘控制：{key.upper()} 按下')

    def _on_key_release(self, event):
        key = normalize_motion_key(event.keysym)
        if key is not None and self.node.release_manual_key(key):
            self.status_text.set(f'底盘键盘控制：{key.upper()} 松开')

    def _on_focus_out(self, _event=None):
        if not self._closed:
            self.root.after_idle(self._stop_if_window_unfocused)

    def _on_window_unmap(self, _event=None):
        if self.node.release_all_manual_keys():
            self.status_text.set('GUI 已失焦，底盘键盘控制已停止')

    def _stop_if_window_unfocused(self):
        if self._closed:
            return
        if (
            self.root.focus_displayof() is None and
            self.node.release_all_manual_keys()
        ):
            self.status_text.set('GUI 已失焦，底盘键盘控制已停止')

    def _start_velocity_test(self, kind):
        if self.node.start_velocity_test(kind):
            labels = {
                'forward': '前进速度测试',
                'left': '左平移速度测试',
                'rotate_left': '逆时针旋转速度测试',
            }
            self.status_text.set(f'{labels[kind]}已开始')

    def _start_pose_test(self, pose_source, kind):
        _, message = self.node.request_relative_pose(kind, pose_source)
        self.status_text.set(message)

    def _start_kfs_alignment_test(self):
        _, message = self.node.request_kfs_alignment()
        self.status_text.set(message)

    def _start_tip_alignment_test(self):
        _, message = self.node.request_tip_alignment()
        self.status_text.set(message)

    def _start_relocalization(self):
        raw_values = tuple(
            self.relocalization_values[name].get()
            for name in RELOCALIZATION_FIELDS
        )
        _, message = self.node.request_relocalization(raw_values)
        self.status_text.set(message)

    def _start_up_step_test(self):
        _, message = self.node.request_up_step_test()
        self.status_text.set(message)

    def _start_down_step_test(self):
        _, message = self.node.request_down_step_test()
        self.status_text.set(message)

    def _start_stage_one(self):
        _, message = self.node.request_stage_one(self.team_value.get())
        self.status_text.set(message)

    def _start_stage_two_point_one_skip(self):
        _, message = self.node.request_stage_two_point_one(
            self.team_value.get(), StageTwoPointOne.Request.SKIP)
        self.status_text.set(message)

    def _start_stage_two_point_one_normal(self):
        _, message = self.node.request_stage_two_point_one(
            self.team_value.get(), StageTwoPointOne.Request.STANDARD)
        self.status_text.set(message)

    def _start_stage_two_point_one_route(self):
        try:
            route = parse_stage_two_point_one_route(
                self.stage_two_point_one_route_value.get())
        except ValueError as exc:
            self.status_text.set(str(exc))
            return
        _, message = self.node.request_stage_two_point_one(
            self.team_value.get(),
            StageTwoPointOne.Request.ROUTE,
            route,
        )
        self.status_text.set(message)

    def _start_stage_two_point_two_skip(self):
        _, message = self.node.request_stage_two_point_two(
            self.team_value.get(), StageTwoPointTwo.Request.SKIP)
        self.status_text.set(message)

    def _start_stage_two_point_two_normal(self):
        _, message = self.node.request_stage_two_point_two(
            self.team_value.get(), StageTwoPointTwo.Request.STANDARD)
        self.status_text.set(message)

    def _start_stage_two_point_two_exit(self):
        _, message = self.node.request_stage_two_point_two_exit(
            self.team_value.get())
        self.status_text.set(message)

    def _start_stage_three(self, loaded_count):
        _, message = self.node.request_stage_three(
            self.team_value.get(), loaded_count)
        self.status_text.set(message)

    def _start_kfs_action_test(self, action, mode):
        _, message = self.node.request_kfs_action(action, mode)
        self.status_text.set(message)

    def _write_kfs_load_parameters(self):
        _, message = self.node.request_kfs_parameter_load()
        self.status_text.set(message)

    def _write_stage_one_parameters(self):
        _, message = self.node.request_stage_one_parameter_load()
        self.status_text.set(message)

    def _write_step_traverse_parameters(self):
        _, message = self.node.request_step_two_parameter_load(
            'step_traverse')
        self.status_text.set(message)

    def _write_stage_two_point_one_parameters(self):
        _, message = self.node.request_step_two_parameter_load(
            'stage_two_point_one')
        self.status_text.set(message)

    def _write_stage_two_point_two_parameters(self):
        _, message = self.node.request_step_two_parameter_load(
            'stage_two_point_two')
        self.status_text.set(message)

    def _publish_lift_command(self, _event=None):
        return super()._publish_lift_command(_event)

    def _publish_combined_lift_command(self, _event=None):
        return super()._publish_combined_lift_command(_event)

    def _publish_float_command(self, control_name, _event=None):
        return super()._publish_float_command(control_name, _event)

    def _sync_dynamic_ranges(self):
        if not self.sync_joint_ranges():
            return
        motion_config = self.node.get_range_snapshot()[4]
        control_text = motion_control_text(motion_config)
        self.keyboard_hint.configure(text=control_text['keyboard_hint'])
        for kind, button in self.velocity_test_buttons.items():
            button.configure(text=control_text['velocity'][kind])
        for (pose_source, kind), button in self.pose_test_buttons.items():
            button.configure(
                text=control_text['pose'][pose_source][kind])
        self.kfs_test_button.configure(
            text=control_text['kfs_alignment'])
        self.tip_test_button.configure(
            text=control_text['tip_alignment'])

    def _sync_kfs_load_feedback(self):
        return self.sync_joint_feedback()

    def _poll_ros(self):
        if self._closed:
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
            self._sync_dynamic_ranges()
            self._sync_kfs_load_feedback()
            for message in self.node.pop_status_events():
                self.status_text.set(message)
            chassis_busy = self.node.is_chassis_request_in_flight()
            if chassis_busy != self.last_chassis_busy:
                self.last_chassis_busy = chassis_busy
                state = tk.DISABLED if chassis_busy else tk.NORMAL
                for button in self.chassis_buttons:
                    button.configure(state=state)
            kfs_action_busy = self.node.is_kfs_action_request_in_flight()
            if kfs_action_busy != self.last_kfs_action_busy:
                self.last_kfs_action_busy = kfs_action_busy
                state = tk.DISABLED if kfs_action_busy else tk.NORMAL
                for button in self.kfs_action_buttons:
                    button.configure(state=state)
            parameter_load_busy = (
                self.node.is_kfs_parameter_load_in_flight())
            if parameter_load_busy != self.last_kfs_parameter_load_busy:
                self.last_kfs_parameter_load_busy = parameter_load_busy
                state = tk.DISABLED if parameter_load_busy else tk.NORMAL
                self.kfs_parameter_load_button.configure(state=state)
            stage_one_parameter_load_busy = (
                self.node.is_stage_one_parameter_load_in_flight())
            if (
                stage_one_parameter_load_busy !=
                self.last_stage_one_parameter_load_busy
            ):
                self.last_stage_one_parameter_load_busy = (
                    stage_one_parameter_load_busy)
                state = (
                    tk.DISABLED
                    if stage_one_parameter_load_busy
                    else tk.NORMAL
                )
                self.stage_one_parameter_load_button.configure(state=state)
            step_two_buttons = {
                'step_traverse': self.step_traverse_parameter_load_button,
                'stage_two_point_one': (
                    self.stage_two_point_one_parameter_load_button),
                'stage_two_point_two': (
                    self.stage_two_point_two_parameter_load_button),
            }
            for key, button in step_two_buttons.items():
                busy = self.node.is_step_two_parameter_load_in_flight(key)
                if busy != self.last_step_two_parameter_load_busy[key]:
                    self.last_step_two_parameter_load_busy[key] = busy
                    button.configure(
                        state=tk.DISABLED if busy else tk.NORMAL)
        except Exception as exc:
            self.node.get_logger().error(f'GUI ROS 回调异常：{exc}')
            self.status_text.set(f'ROS 回调异常：{exc}')
        if not self._closed:
            self.root.after(10, self._poll_ros)

    def run(self):
        self.root.mainloop()

    def close(self):
        if self._closed:
            return
        self._closed = True
        self.node.stop_chassis()
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GuiControlNode()
        app = GuiControlApp(node)
        app.run()
    except KeyboardInterrupt:
        pass
    except tk.TclError as exc:
        if node is not None:
            node.get_logger().error(f'无法启动图形界面：{exc}')
        else:
            raise
    finally:
        if node is not None:
            node.stop_chassis()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
