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
from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from robot_r2_interfaces.msg import LiftCommand
from robot_r2_interfaces.srv import (
    Align,
    KfsAction,
    MoveToPose,
    SetBasePose,
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
    'pop_sequence',
}
KFS_LOADER_SOURCE_RELATIVE_PATH = Path(
    'src/robot_r2_control/config/kfs_loader.yaml')
RELOCALIZATION_FIELDS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


def normalize_motion_key(keysym):
    key = str(keysym).lower()
    return key if key in MOTION_KEYS else None


def resolve_kfs_loader_source_config(package_share_directory):
    package_share_path = Path(package_share_directory).resolve()
    search_roots = (package_share_path, *package_share_path.parents)

    for root in search_roots:
        candidate = root / KFS_LOADER_SOURCE_RELATIVE_PATH
        if candidate.is_file():
            return os.fspath(candidate)

    for root in search_roots:
        if root.name == 'install':
            return os.fspath(
                root.parent / KFS_LOADER_SOURCE_RELATIVE_PATH)

    return os.fspath(Path.cwd() / KFS_LOADER_SOURCE_RELATIVE_PATH)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z +
        quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y +
        quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


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
    kfs_tolerance = format_parameter_value(
        config['kfs_alignment_pixel_tolerance'])
    kfs_timeout = format_parameter_value(
        config['kfs_alignment_timeout_sec'])
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
            'forward': f'位置伺服前进 {pose_distance} m',
            'left': f'位置伺服左平移 {pose_distance} m',
            'rotate_left': f'位置伺服逆时针旋转 {pose_yaw} rad',
        },
        'kfs_alignment': (
            f'KFS 对齐（容忍 {kfs_tolerance} px，'
            f'超时 {kfs_timeout} s）'
        ),
    }


def relative_pose_goal(current_pose, forward, left, yaw_delta):
    current_x, current_y, current_yaw = current_pose
    cos_yaw = math.cos(current_yaw)
    sin_yaw = math.sin(current_yaw)
    return (
        current_x + cos_yaw * forward - sin_yaw * left,
        current_y + sin_yaw * forward + cos_yaw * left,
        normalize_angle(current_yaw + yaw_delta),
    )


def parse_relocalization_values(raw_values):
    if len(raw_values) != len(RELOCALIZATION_FIELDS):
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


def summarize_parameter_load_result(returncode, stdout, stderr):
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
    missing = KFS_LOADER_PARAMETER_NAMES - successful_names
    if returncode == 0 and not failures and not missing:
        return (
            True,
            f'KFS Load 参数写入成功：共 {len(successful_names)} 项',
        )

    details = list(failures)
    if returncode != 0 and not details:
        details.append(f'ros2 param load 退出码 {returncode}')
    if missing and not failures:
        details.append('未确认写入：' + ', '.join(sorted(missing)))
    if not details:
        details.append('没有收到参数写入结果')
    return False, 'KFS Load 参数写入失败：' + '；'.join(details)


def make_parameter_load_command(config_path):
    return [
        'ros2',
        'param',
        'load',
        '--no-daemon',
        '--spin-time',
        '2.0',
        '/kfs_loader_control',
        config_path,
    ]


class GuiControlNode(Node):
    CMD_VEL_TOPIC = '/r2/cmd_vel'
    POSE_FEEDBACK_TOPIC = '/r2/pose_feedback'
    MOVE_TO_POSE_SERVICE = '/r2/move_to_pose'
    KFS_ALIGNMENT_SERVICE = '/r2/align_to_kfs'
    KFS_ACTION_SERVICE = '/r2/kfs/action'
    SET_BASE_POSE_SERVICE = '/r2/set_base_pose'
    STEP_TRAVERSE_SERVICE = '/r2/step_traverse'
    LIFT_COMMAND_TOPIC = '/r2/lift/cmd_lift'

    KFS_LOAD_MOTOR_FEEDBACK_TOPICS = {
        'root_rotate': '/r2/gripper/rotate_feedback',
        'tip_rotate': '/r2/gripper/tip_rotate_feedback',
        'grip': '/r2/gripper/grip_feedback',
    }

    FLOAT_CONTROL_PARAMETERS = {
        'kfs_lift': {
            'topic': '/r2/kfs_lift/cmd',
            'minimum': ('kfs_lift_min', 0.0),
            'maximum': ('kfs_lift_max', 0.42),
        },
        'root_rotate': {
            'topic': '/r2/gripper/rotate_cmd',
            'minimum': ('root_rotate_min', -math.radians(15.0)),
            'maximum': ('root_rotate_max', 2.356194490192345),
        },
        'tip_rotate': {
            'topic': '/r2/gripper/tip_rotate_cmd',
            'minimum': ('tip_rotate_min', -math.pi),
            'maximum': ('tip_rotate_max', 0.0),
        },
        'grip': {
            'topic': '/r2/gripper/grip_cmd',
            'minimum': ('grip_min', 0.0),
            'maximum': ('grip_max', 0.209),
        },
        'weapon_rotate': {
            'topic': '/r2/weapon/rotate_cmd',
            'minimum': ('weapon_rotate_min', 0.0),
            'maximum': ('weapon_rotate_max', math.radians(200.0)),
        },
        'weapon_grip': {
            'topic': '/r2/weapon/grip_cmd',
            'minimum': ('weapon_grip_min', 0.0),
            'maximum': ('weapon_grip_max', 0.03),
        },
    }

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
        'kfs_alignment_pixel_tolerance': 10.0,
        'kfs_alignment_timeout_sec': 3.0,
    }

    def __init__(self):
        super().__init__('gui_control')

        self.state_lock = threading.RLock()
        self.status_events = deque()
        self.config_generation = 0
        self.kfs_load_feedback_generation = 0
        self.kfs_load_motor_feedback = {
            name: None for name in self.KFS_LOAD_MOTOR_FEEDBACK_TOPICS
        }
        self.current_pose = None
        self.active_manual_keys = set()
        self.velocity_test_kind = None
        self.velocity_test_deadline = None
        self.pose_request_in_flight = False
        self.kfs_alignment_request_in_flight = False
        self.relocalization_request_in_flight = False
        self.step_test_request_in_flight = False
        self.step_test_direction = None
        self.kfs_action_request_in_flight = False
        self.kfs_parameter_load_in_flight = False
        self.kfs_loader_config_path = resolve_kfs_loader_source_config(
            get_package_share_directory('robot_r2_control'),
        )

        self.declare_parameter('lift_min', 0.0)
        self.declare_parameter('lift_max', 0.376)
        for parameters in self.FLOAT_CONTROL_PARAMETERS.values():
            minimum_parameter, minimum_default = parameters['minimum']
            maximum_parameter, maximum_default = parameters['maximum']
            self.declare_parameter(minimum_parameter, minimum_default)
            self.declare_parameter(maximum_parameter, maximum_default)
        for name, default in self.MOTION_PARAMETER_DEFAULTS.items():
            self.declare_parameter(name, default)

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

        self.lift_command_publisher = self.create_publisher(
            LiftCommand, self.LIFT_COMMAND_TOPIC, 10)
        self.float_command_publishers = {
            control_name: self.create_publisher(
                Float64, parameters['topic'], 10)
            for control_name, parameters in self.FLOAT_CONTROL_PARAMETERS.items()
        }
        self.cmd_vel_publisher = self.create_publisher(
            Twist, self.CMD_VEL_TOPIC, 10)
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            self.POSE_FEEDBACK_TOPIC,
            self._on_pose_feedback,
            10,
        )
        self.kfs_load_feedback_subscribers = {
            name: self.create_subscription(
                Float64,
                topic,
                partial(self._on_kfs_load_motor_feedback, name),
                10,
            )
            for name, topic in self.KFS_LOAD_MOTOR_FEEDBACK_TOPICS.items()
        }
        self.move_client = self.create_client(
            MoveToPose, self.MOVE_TO_POSE_SERVICE)
        self.kfs_alignment_client = self.create_client(
            Align, self.KFS_ALIGNMENT_SERVICE)
        self.kfs_action_client = self.create_client(
            KfsAction, self.KFS_ACTION_SERVICE)
        self.set_base_pose_client = self.create_client(
            SetBasePose, self.SET_BASE_POSE_SERVICE)
        self.step_traverse_client = self.create_client(
            TraverseStep, self.STEP_TRAVERSE_SERVICE)
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

    def publish_lift_command(self, front_lift, rear_lift):
        command = LiftCommand()
        command.front_lift = float(front_lift)
        command.rear_lift = float(rear_lift)
        self.lift_command_publisher.publish(command)

    def publish_float_command(self, control_name, value):
        command = Float64()
        command.data = float(value)
        self.float_command_publishers[control_name].publish(command)

    def _on_pose_feedback(self, message):
        pose = message.pose
        current = (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )
        if not all(math.isfinite(value) for value in current):
            return
        with self.state_lock:
            self.current_pose = current

    def _on_kfs_load_motor_feedback(self, motor_name, message):
        value = float(message.data)
        if not math.isfinite(value):
            return
        with self.state_lock:
            self.kfs_load_motor_feedback[motor_name] = value
            self.kfs_load_feedback_generation += 1

    def get_kfs_load_feedback_snapshot(self):
        with self.state_lock:
            return (
                self.kfs_load_feedback_generation,
                dict(self.kfs_load_motor_feedback),
            )

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

    def request_relative_pose(self, kind):
        with self.state_lock:
            if self.pose_request_in_flight:
                return False, '位置伺服正在执行'
            if self.kfs_alignment_request_in_flight:
                return False, 'KFS 对齐正在执行'
            if (
                self.relocalization_request_in_flight or
                self.step_test_request_in_flight
            ):
                return False, '底盘操作正在执行'
            if self.current_pose is None:
                return False, '尚未收到 /r2/pose_feedback'
            if not self.move_client.service_is_ready():
                return False, '/r2/move_to_pose 服务不可用'

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

            target = relative_pose_goal(self.current_pose, *offsets)
            timeout_sec = self.motion_config['move_timeout_sec']
            self.active_manual_keys.clear()
            self.velocity_test_kind = None
            self.velocity_test_deadline = None
            self.pose_request_in_flight = True

        self.cmd_vel_publisher.publish(Twist())
        request = MoveToPose.Request()
        request.x = target[0]
        request.y = target[1]
        request.yaw = target[2]
        request.position_tolerance = 0.0
        request.yaw_tolerance = 0.0
        request.timeout_sec = timeout_sec
        try:
            future = self.move_client.call_async(request)
        except Exception as exc:
            with self.state_lock:
                self.pose_request_in_flight = False
            return False, f'位置伺服请求发送失败：{exc}'
        future.add_done_callback(self._on_move_complete)
        return True, (
            f'已发送位置目标：x={target[0]:.3f} m，'
            f'y={target[1]:.3f} m，yaw={target[2]:.3f} rad'
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
            if self.pose_request_in_flight:
                return False, '位置伺服正在执行'
            if (
                self.relocalization_request_in_flight or
                self.step_test_request_in_flight
            ):
                return False, '底盘操作正在执行'
            if not self.kfs_alignment_client.service_is_ready():
                return False, '/r2/align_to_kfs 服务不可用'

            pixel_tolerance = self.motion_config[
                'kfs_alignment_pixel_tolerance']
            timeout_sec = self.motion_config[
                'kfs_alignment_timeout_sec']
            self.active_manual_keys.clear()
            self.velocity_test_kind = None
            self.velocity_test_deadline = None
            self.kfs_alignment_request_in_flight = True

        self.cmd_vel_publisher.publish(Twist())
        request = Align.Request()
        request.pixel_tolerance = pixel_tolerance
        request.timeout_sec = timeout_sec
        try:
            future = self.kfs_alignment_client.call_async(request)
        except Exception as exc:
            with self.state_lock:
                self.kfs_alignment_request_in_flight = False
            return False, f'KFS 对齐请求发送失败：{exc}'
        future.add_done_callback(self._on_kfs_alignment_complete)
        return True, (
            f'已发送 KFS 对齐请求：容忍 {pixel_tolerance:g} px，'
            f'超时 {timeout_sec:g} s'
        )

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
            if not self.set_base_pose_client.service_is_ready():
                return False, '/r2/set_base_pose 服务不可用'
            if not self.step_traverse_client.service_is_ready():
                return False, '/r2/step_traverse 服务不可用'
            self.active_manual_keys.clear()
            self.velocity_test_kind = None
            self.velocity_test_deadline = None
            self.step_test_request_in_flight = True
            self.step_test_direction = direction

        self.cmd_vel_publisher.publish(Twist())
        request = self._make_set_base_pose_request((0.0,) * 6)
        try:
            future = self.set_base_pose_client.call_async(request)
        except Exception as exc:
            with self.state_lock:
                self.step_test_request_in_flight = False
                self.step_test_direction = None
            return False, f'跨越测试重定位请求发送失败：{exc}'
        future.add_done_callback(self._on_step_test_relocalization_complete)
        return True, '跨越测试：正在重定位到原点'

    def _on_step_test_relocalization_complete(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self._finish_step_test(
                f'跨越测试重定位调用异常：{exc}')
            return
        if response is None:
            self._finish_step_test('跨越测试重定位失败：无响应')
            return
        if not response.success:
            self._finish_step_test(
                f'跨越测试重定位失败：{response.message}')
            return

        with self.state_lock:
            direction = self.step_test_direction
        direction_name = (
            '上' if direction == TraverseStep.Request.UP else '下')
        self._queue_status(f'跨越测试：重定位完成，正在{direction_name}台阶')
        request = TraverseStep.Request()
        request.direction = direction
        request.distance_to_step = 0.0
        try:
            step_future = self.step_traverse_client.call_async(request)
        except Exception as exc:
            self._finish_step_test(f'{direction_name}台阶请求发送失败：{exc}')
            return
        step_future.add_done_callback(self._on_step_test_complete)

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
            action_label = '弹出'
        else:
            raise ValueError(f'Unknown KFS action: {action}')

        with self.state_lock:
            if self.kfs_action_request_in_flight:
                return False, 'KFS 动作正在执行'
            if not self.kfs_action_client.service_is_ready():
                return False, '/r2/kfs/action 服务不可用'
            self.kfs_action_request_in_flight = True

        request = KfsAction.Request()
        request.action = action
        if action == KfsAction.Request.LOAD:
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
            self.relocalization_request_in_flight or
            self.step_test_request_in_flight
        )

    def is_pose_request_in_flight(self):
        with self.state_lock:
            return self.pose_request_in_flight

    def is_chassis_request_in_flight(self):
        with self.state_lock:
            return self._chassis_service_in_flight_locked()

    def is_kfs_action_request_in_flight(self):
        with self.state_lock:
            return self.kfs_action_request_in_flight

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


class GuiControlApp:
    KFS_CONTROLS = (
        {
            'name': 'kfs_lift',
            'label': 'KFS 升降',
            'unit': 'm',
            'resolution': 0.001,
            'decimals': 3,
        },
        {
            'name': 'root_rotate',
            'label': '夹爪根部旋转',
            'unit': 'rad',
            'resolution': 0.001,
            'decimals': 3,
        },
        {
            'name': 'tip_rotate',
            'label': '夹爪尖端旋转（0 初始，工作方向为负）',
            'unit': 'rad',
            'resolution': 0.001,
            'decimals': 3,
            'reverse_slider': True,
        },
        {
            'name': 'grip',
            'label': '夹爪开度（0 为闭合）',
            'unit': 'm',
            'resolution': 0.001,
            'decimals': 3,
        },
    )
    WEAPON_CONTROLS = (
        {
            'name': 'weapon_rotate',
            'label': '武器夹爪旋转',
            'unit': '°',
            'resolution': 1.0,
            'decimals': 0,
            'to_display': math.degrees,
            'to_command': math.radians,
        },
        {
            'name': 'weapon_grip',
            'label': '武器夹爪开合',
            'unit': 'cm',
            'resolution': 0.1,
            'decimals': 1,
            'to_display': lambda value: value * 100.0,
            'to_command': lambda value: value / 100.0,
        },
    )
    FLOAT_CONTROLS = KFS_CONTROLS + WEAPON_CONTROLS

    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Robot R2 GUI 控制')
        self.root.resizable(False, False)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.front_lift_value = tk.DoubleVar(value=node.lift_min)
        self.rear_lift_value = tk.DoubleVar(value=node.lift_min)
        self.combined_lift_value = tk.DoubleVar(value=node.lift_min)
        self.float_control_values = {}
        for control in self.FLOAT_CONTROLS:
            control_name = control['name']
            self.float_control_values[control_name] = tk.DoubleVar(
                value=0.0)
        self.kfs_load_feedback_text = {
            name: tk.StringVar(value='实际反馈：尚未收到')
            for name in self.node.KFS_LOAD_MOTOR_FEEDBACK_TOPICS
        }
        self.relocalization_values = {
            name: tk.StringVar(value='0.0')
            for name in RELOCALIZATION_FIELDS
        }
        self.status_text = tk.StringVar(value='已就绪')

        self.last_lift_command = None
        self.last_float_commands = {}
        self.last_config_generation = -1
        self.last_kfs_load_feedback_generation = -1
        self.last_chassis_busy = None
        self.last_kfs_action_busy = None
        self.last_kfs_parameter_load_busy = None
        self._closed = False
        self.chassis_buttons = []
        self.velocity_test_buttons = {}
        self.pose_test_buttons = {}
        self.relocalization_button = None
        self.up_step_test_button = None
        self.down_step_test_button = None
        self.kfs_test_button = None
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
        main_frame = ttk.Frame(self.root, padding=16)
        main_frame.grid(row=0, column=0, sticky='nsew')

        chassis_column = ttk.Frame(main_frame)
        chassis_column.grid(row=0, column=0, sticky='new', padx=(0, 10))
        mechanism_column = ttk.Frame(main_frame)
        mechanism_column.grid(row=0, column=1, sticky='new')

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
        for row, kind in enumerate(test_kinds):
            button = ttk.Button(
                test_frame,
                command=partial(self._start_velocity_test, kind),
            )
            button.grid(row=row, column=0, sticky='ew', pady=3)
            self.velocity_test_buttons[kind] = button
            self.chassis_buttons.append(button)
        for row, kind in enumerate(test_kinds):
            button = ttk.Button(
                test_frame,
                command=partial(self._start_pose_test, kind),
            )
            button.grid(row=row, column=1, sticky='ew', padx=(8, 0), pady=3)
            self.pose_test_buttons[kind] = button
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
            text='上一个台阶（先重定位，距离 0.0 m）',
            command=self._start_up_step_test,
        )
        self.up_step_test_button.grid(row=0, column=0, sticky='ew')
        self.chassis_buttons.append(self.up_step_test_button)
        self.down_step_test_button = ttk.Button(
            traverse_test_frame,
            text='下一个台阶（先重定位，距离 0.0 m）',
            command=self._start_down_step_test,
        )
        self.down_step_test_button.grid(
            row=0, column=1, sticky='ew', padx=(8, 0))
        self.chassis_buttons.append(self.down_step_test_button)

        kfs_test_frame = ttk.LabelFrame(
            chassis_column, text='KFS 测试', padding=12)
        kfs_test_frame.grid(row=4, column=0, sticky='ew', pady=(10, 0))
        self.kfs_test_button = ttk.Button(
            kfs_test_frame,
            command=self._start_kfs_alignment_test,
        )
        self.kfs_test_button.grid(row=0, column=0, sticky='ew')
        self.chassis_buttons.append(self.kfs_test_button)
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
            ('弹出', KfsAction.Request.POP, 0),
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
            row=5,
            column=0,
            columnspan=2,
            sticky='ew',
            pady=(8, 0),
        )

        lift_frame = ttk.LabelFrame(
            mechanism_column, text='底盘抬升', padding=12)
        lift_frame.grid(row=0, column=0, sticky='ew')
        self.front_lift_slider = self._create_slider(
            lift_frame, 0, '前轮抬升 (m)', self.front_lift_value,
            self.node.lift_min, self.node.lift_max, 0.001,
            self._publish_lift_command)
        self.rear_lift_slider = self._create_slider(
            lift_frame, 1, '后轮抬升 (m)', self.rear_lift_value,
            self.node.lift_min, self.node.lift_max, 0.001,
            self._publish_lift_command)
        self.combined_lift_slider = self._create_slider(
            lift_frame, 2, '一起抬升 (m)', self.combined_lift_value,
            self.node.lift_min, self.node.lift_max, 0.001,
            self._publish_combined_lift_command)

        self.float_control_sliders = {}
        self._create_control_frame(
            mechanism_column, 1, 'KFS 机构', self.KFS_CONTROLS)
        self._create_control_frame(
            mechanism_column, 2, '武器机构', self.WEAPON_CONTROLS)

        status_label = ttk.Label(
            main_frame, textvariable=self.status_text, anchor='w')
        status_label.grid(
            row=1, column=0, columnspan=2, sticky='ew', pady=(10, 0))

    def _create_control_frame(self, parent, row, title, controls):
        frame = ttk.LabelFrame(parent, text=title, padding=12)
        frame.grid(row=row, column=0, sticky='ew', pady=(10, 0))
        control_row = 0
        for control in controls:
            control_name = control['name']
            minimum, maximum = self.node.float_control_ranges[control_name]
            to_display = control.get('to_display', float)
            slider_start = to_display(minimum)
            slider_end = to_display(maximum)
            if control.get('reverse_slider', False):
                slider_start, slider_end = slider_end, slider_start
            self.float_control_sliders[control_name] = self._create_slider(
                frame,
                control_row,
                f"{control['label']} ({control['unit']})",
                self.float_control_values[control_name],
                slider_start,
                slider_end,
                control['resolution'],
                partial(self._publish_float_command, control_name),
            )
            control_row += 1
            if control_name in self.kfs_load_feedback_text:
                feedback_label = ttk.Label(
                    frame,
                    textvariable=self.kfs_load_feedback_text[control_name],
                    anchor='e',
                )
                feedback_label.grid(
                    row=control_row,
                    column=0,
                    sticky='ew',
                    pady=(0, 8),
                )
                control_row += 1

    @staticmethod
    def _create_slider(
        parent, row, label, variable, minimum, maximum, resolution,
        release_callback,
    ):
        slider = tk.Scale(
            parent,
            label=label,
            variable=variable,
            from_=minimum,
            to=maximum,
            resolution=resolution,
            orient=tk.HORIZONTAL,
            length=380,
            showvalue=True,
            digits=4,
        )
        slider.grid(row=row, column=0, sticky='ew', pady=(0, 8))
        slider.bind('<ButtonRelease-1>', release_callback)
        for key_name in ('Left', 'Right', 'Up', 'Down', 'Home', 'End'):
            slider.bind(f'<KeyRelease-{key_name}>', release_callback)
        return slider

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

    def _start_pose_test(self, kind):
        _, message = self.node.request_relative_pose(kind)
        self.status_text.set(message)

    def _start_kfs_alignment_test(self):
        _, message = self.node.request_kfs_alignment()
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

    def _start_kfs_action_test(self, action, mode):
        _, message = self.node.request_kfs_action(action, mode)
        self.status_text.set(message)

    def _write_kfs_load_parameters(self):
        _, message = self.node.request_kfs_parameter_load()
        self.status_text.set(message)

    def _publish_lift_command(self, _event=None):
        targets = (
            round(self.front_lift_value.get(), 3),
            round(self.rear_lift_value.get(), 3),
        )
        if targets == self.last_lift_command:
            self.status_text.set('底盘抬升目标值未变化')
            return
        self.node.publish_lift_command(*targets)
        self.last_lift_command = targets
        self.status_text.set(
            f'已发送：前 {targets[0]:.3f} m，后 {targets[1]:.3f} m')

    def _publish_combined_lift_command(self, _event=None):
        target = round(self.combined_lift_value.get(), 3)
        self.front_lift_value.set(target)
        self.rear_lift_value.set(target)
        self.node.publish_lift_command(target, target)
        self.last_lift_command = (target, target)
        self.status_text.set(f'已发送：前后均为 {target:.3f} m')

    def _publish_float_command(self, control_name, _event=None):
        control = next(
            item for item in self.FLOAT_CONTROLS
            if item['name'] == control_name)
        decimals = control['decimals']
        display_target = round(
            self.float_control_values[control_name].get(), decimals)
        if self.last_float_commands.get(control_name) == display_target:
            self.status_text.set(f"{control['label']}目标值未变化")
            return
        to_command = control.get('to_command', float)
        self.node.publish_float_command(
            control_name, to_command(display_target))
        self.last_float_commands[control_name] = display_target
        self.status_text.set(
            f"已发送：{control['label']} "
            f"{display_target:.{decimals}f} {control['unit']}")

    def _sync_dynamic_ranges(self):
        generation, lift_min, lift_max, ranges, motion_config = (
            self.node.get_range_snapshot())
        if generation == self.last_config_generation:
            return
        self.last_config_generation = generation
        for slider in (
            self.front_lift_slider,
            self.rear_lift_slider,
            self.combined_lift_slider,
        ):
            slider.configure(from_=lift_min, to=lift_max)
        for control in self.FLOAT_CONTROLS:
            minimum, maximum = ranges[control['name']]
            to_display = control.get('to_display', float)
            slider_start = to_display(minimum)
            slider_end = to_display(maximum)
            if control.get('reverse_slider', False):
                slider_start, slider_end = slider_end, slider_start
            self.float_control_sliders[control['name']].configure(
                from_=slider_start, to=slider_end)
        control_text = motion_control_text(motion_config)
        self.keyboard_hint.configure(text=control_text['keyboard_hint'])
        for kind, button in self.velocity_test_buttons.items():
            button.configure(text=control_text['velocity'][kind])
        for kind, button in self.pose_test_buttons.items():
            button.configure(text=control_text['pose'][kind])
        self.kfs_test_button.configure(
            text=control_text['kfs_alignment'])

    def _sync_kfs_load_feedback(self):
        generation, feedback = (
            self.node.get_kfs_load_feedback_snapshot())
        if generation == self.last_kfs_load_feedback_generation:
            return
        self.last_kfs_load_feedback_generation = generation
        controls = {item['name']: item for item in self.KFS_CONTROLS}
        for name, text_variable in self.kfs_load_feedback_text.items():
            value = feedback[name]
            if value is None:
                text_variable.set('实际反馈：尚未收到')
                continue
            control = controls[name]
            display_value = control.get('to_display', float)(value)
            decimals = control['decimals']
            text_variable.set(
                f"实际反馈：{display_value:.{decimals}f} "
                f"{control['unit']}")

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
