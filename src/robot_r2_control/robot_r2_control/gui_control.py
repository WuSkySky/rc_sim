from collections import deque
from functools import partial
import math
import threading
import time
import tkinter as tk
from tkinter import ttk

from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from robot_r2_interfaces.msg import LiftCommand
from robot_r2_interfaces.srv import MoveToPose
from std_msgs.msg import Float64


MOTION_KEYS = {'w', 'a', 's', 'd', 'q', 'e'}


def normalize_motion_key(keysym):
    key = str(keysym).lower()
    return key if key in MOTION_KEYS else None


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


class GuiControlNode(Node):
    CMD_VEL_TOPIC = '/r2/cmd_vel'
    POSE_FEEDBACK_TOPIC = '/r2/pose_feedback'
    MOVE_TO_POSE_SERVICE = '/r2/move_to_pose'
    LIFT_COMMAND_TOPIC = '/r2/lift/cmd_lift'

    FLOAT_CONTROL_PARAMETERS = {
        'kfs_lift': {
            'topic': '/r2/kfs_lift/cmd',
            'minimum': ('kfs_lift_min', 0.0),
            'maximum': ('kfs_lift_max', 0.42),
        },
        'root_rotate': {
            'topic': '/r2/gripper/rotate_cmd',
            'minimum': ('root_rotate_min', 0.0),
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
    }

    def __init__(self):
        super().__init__('gui_control')

        self.state_lock = threading.RLock()
        self.status_events = deque()
        self.config_generation = 0
        self.current_pose = None
        self.active_manual_keys = set()
        self.velocity_test_kind = None
        self.velocity_test_deadline = None
        self.pose_request_in_flight = False

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
        self.move_client = self.create_client(
            MoveToPose, self.MOVE_TO_POSE_SERVICE)
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
            if self.pose_request_in_flight:
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
            if self.pose_request_in_flight:
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
            if self.pose_request_in_flight:
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

    def is_pose_request_in_flight(self):
        with self.state_lock:
            return self.pose_request_in_flight

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
        self.status_text = tk.StringVar(value='已就绪')

        self.last_lift_command = None
        self.last_float_commands = {}
        self.last_config_generation = -1
        self.last_pose_busy = None
        self._closed = False
        self.chassis_buttons = []
        self.velocity_test_buttons = {}
        self.pose_test_buttons = {}

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
        for control_row, control in enumerate(controls):
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

    def _poll_ros(self):
        if self._closed:
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
            self._sync_dynamic_ranges()
            for message in self.node.pop_status_events():
                self.status_text.set(message)
            pose_busy = self.node.is_pose_request_in_flight()
            if pose_busy != self.last_pose_busy:
                self.last_pose_busy = pose_busy
                state = tk.DISABLED if pose_busy else tk.NORMAL
                for button in self.chassis_buttons:
                    button.configure(state=state)
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
