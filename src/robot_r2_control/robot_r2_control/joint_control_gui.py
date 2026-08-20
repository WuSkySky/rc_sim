import math
from functools import partial
import tkinter as tk
from tkinter import ttk

from rcl_interfaces.msg import SetParametersResult
from robot_r2_interfaces.msg import LiftCommand
from std_msgs.msg import Float64


class JointControlNodeMixin:
    LIFT_COMMAND_TOPIC = '/r2/lift/cmd_lift'
    KFS_LOAD_MOTOR_FEEDBACK_TOPICS = {
        'root_rotate': '/r2/gripper/rotate_feedback',
        'tip_rotate': '/r2/gripper/tip_rotate_feedback',
        'grip': '/r2/gripper/grip_feedback',
        'weapon_rotate': '/r2/weapon/rotate_feedback',
        'weapon_grip': '/r2/weapon/grip_feedback',
    }
    FLOAT_CONTROL_PARAMETERS = {
        'kfs_lift': {
            'topic': '/r2/kfs_lift/cmd',
            'minimum': ('kfs_lift_min', 0.0),
            'maximum': ('kfs_lift_max', 0.42),
        },
        'root_rotate': {
            'topic': '/r2/gripper/rotate_cmd',
            'minimum': ('root_rotate_min', -0.262),
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

    @staticmethod
    def _validate_range(name, minimum, maximum):
        if not math.isfinite(minimum):
            raise ValueError(f'{name} minimum must be finite')
        if not math.isfinite(maximum):
            raise ValueError(f'{name} maximum must be finite')
        if minimum >= maximum:
            raise ValueError(f'{name} minimum must be less than maximum')

    def initialize_joint_control(self):
        self.config_generation = 0
        self.kfs_load_feedback_generation = 0
        self.kfs_load_motor_feedback = {
            name: None for name in self.KFS_LOAD_MOTOR_FEEDBACK_TOPICS
        }

        self.declare_parameter('lift_min', 0.0)
        self.declare_parameter('lift_max', 0.376)
        for definition in self.FLOAT_CONTROL_PARAMETERS.values():
            minimum_name, minimum_default = definition['minimum']
            maximum_name, maximum_default = definition['maximum']
            self.declare_parameter(minimum_name, minimum_default)
            self.declare_parameter(maximum_name, maximum_default)

        self.lift_min = float(self.get_parameter('lift_min').value)
        self.lift_max = float(self.get_parameter('lift_max').value)
        self._validate_range('lift', self.lift_min, self.lift_max)
        self.float_control_ranges = {}
        for control_name, definition in self.FLOAT_CONTROL_PARAMETERS.items():
            minimum_name, _ = definition['minimum']
            maximum_name, _ = definition['maximum']
            minimum = float(self.get_parameter(minimum_name).value)
            maximum = float(self.get_parameter(maximum_name).value)
            self._validate_range(control_name, minimum, maximum)
            self.float_control_ranges[control_name] = (minimum, maximum)

        self.lift_command_publisher = self.create_publisher(
            LiftCommand, self.LIFT_COMMAND_TOPIC, 10)
        self.float_command_publishers = {
            control_name: self.create_publisher(
                Float64, definition['topic'], 10)
            for control_name, definition in self.FLOAT_CONTROL_PARAMETERS.items()
        }
        self.kfs_load_feedback_subscribers = {
            name: self.create_subscription(
                Float64,
                topic,
                partial(self._on_kfs_load_motor_feedback, name),
                10,
            )
            for name, topic in self.KFS_LOAD_MOTOR_FEEDBACK_TOPICS.items()
        }

    def on_joint_parameters_changed(self, parameters):
        updates = {parameter.name: parameter.value for parameter in parameters}
        joint_names = {'lift_min', 'lift_max'}
        for definition in self.FLOAT_CONTROL_PARAMETERS.values():
            joint_names.add(definition['minimum'][0])
            joint_names.add(definition['maximum'][0])

        try:
            for name, value in updates.items():
                if name not in joint_names:
                    continue
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise ValueError(f'{name} must be numeric')
                if not math.isfinite(float(value)):
                    raise ValueError(f'{name} must be finite')

            with self.state_lock:
                lift_min = float(updates.get('lift_min', self.lift_min))
                lift_max = float(updates.get('lift_max', self.lift_max))
                self._validate_range('lift', lift_min, lift_max)
                ranges = {}
                for control_name, definition in (
                        self.FLOAT_CONTROL_PARAMETERS.items()):
                    minimum_name, _ = definition['minimum']
                    maximum_name, _ = definition['maximum']
                    old_minimum, old_maximum = self.float_control_ranges[
                        control_name]
                    minimum = float(updates.get(minimum_name, old_minimum))
                    maximum = float(updates.get(maximum_name, old_maximum))
                    self._validate_range(control_name, minimum, maximum)
                    ranges[control_name] = (minimum, maximum)

                self.lift_min = lift_min
                self.lift_max = lift_max
                self.float_control_ranges = ranges
                self.config_generation += 1
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def get_joint_range_snapshot(self):
        with self.state_lock:
            return (
                self.config_generation,
                self.lift_min,
                self.lift_max,
                dict(self.float_control_ranges),
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


class JointControlGuiMixin:
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

    def initialize_joint_ui(self):
        self.front_lift_value = tk.DoubleVar(value=self.node.lift_min)
        self.rear_lift_value = tk.DoubleVar(value=self.node.lift_min)
        self.combined_lift_value = tk.DoubleVar(value=self.node.lift_min)
        self.float_control_values = {
            control['name']: tk.DoubleVar(value=0.0)
            for control in self.FLOAT_CONTROLS
        }
        self.kfs_load_feedback_text = {
            name: tk.StringVar(value='实际反馈：尚未收到')
            for name in self.node.KFS_LOAD_MOTOR_FEEDBACK_TOPICS
        }
        self.last_lift_command = None
        self.last_float_commands = {}
        self.last_config_generation = -1
        self.last_kfs_load_feedback_generation = -1
        self.joint_control_widgets = []

    def build_joint_controls(self, parent):
        lift_frame = ttk.LabelFrame(parent, text='底盘抬升', padding=12)
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
        self.joint_control_widgets.extend((
            self.front_lift_slider,
            self.rear_lift_slider,
            self.combined_lift_slider,
        ))

        self.float_control_sliders = {}
        self._create_control_frame(parent, 1, 'KFS 机构', self.KFS_CONTROLS)
        self._create_control_frame(parent, 2, '武器机构', self.WEAPON_CONTROLS)

    def _create_control_frame(self, parent, row, title, controls):
        frame = ttk.LabelFrame(parent, text=title, padding=8)
        frame.grid(row=row, column=0, sticky='ew', pady=(6, 0))
        control_row = 0
        for control in controls:
            name = control['name']
            minimum, maximum = self.node.float_control_ranges[name]
            to_display = control.get('to_display', float)
            slider_start = to_display(minimum)
            slider_end = to_display(maximum)
            if control.get('reverse_slider', False):
                slider_start, slider_end = slider_end, slider_start
            slider = self._create_slider(
                frame,
                control_row,
                f"{control['label']} ({control['unit']})",
                self.float_control_values[name],
                slider_start,
                slider_end,
                control['resolution'],
                partial(self._publish_float_command, name),
            )
            self.float_control_sliders[name] = slider
            self.joint_control_widgets.append(slider)
            control_row += 1
            if name in self.kfs_load_feedback_text:
                ttk.Label(
                    frame,
                    textvariable=self.kfs_load_feedback_text[name],
                    anchor='e',
                ).grid(
                    row=control_row,
                    column=0,
                    sticky='ew',
                    pady=(0, 4),
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
            length=300,
            showvalue=True,
            digits=4,
        )
        slider.grid(row=row, column=0, sticky='ew', pady=(0, 2))
        slider.bind('<ButtonRelease-1>', release_callback)
        for key_name in ('Left', 'Right', 'Up', 'Down', 'Home', 'End'):
            slider.bind(f'<KeyRelease-{key_name}>', release_callback)
        return slider

    def joint_commands_allowed(self):
        return True

    def _publish_lift_command(self, _event=None):
        if not self.joint_commands_allowed():
            return
        targets = (
            round(self.front_lift_value.get(), 3),
            round(self.rear_lift_value.get(), 3),
        )
        if self.last_lift_command == targets:
            self.status_text.set('底盘抬升目标值未变化')
            return
        self.node.publish_lift_command(*targets)
        self.last_lift_command = targets
        self.status_text.set(
            f'已发送：前 {targets[0]:.3f} m，后 {targets[1]:.3f} m')

    def _publish_combined_lift_command(self, _event=None):
        if not self.joint_commands_allowed():
            return
        target = round(self.combined_lift_value.get(), 3)
        self.front_lift_value.set(target)
        self.rear_lift_value.set(target)
        self.node.publish_lift_command(target, target)
        self.last_lift_command = (target, target)
        self.status_text.set(f'已发送：前后均为 {target:.3f} m')

    def _publish_float_command(self, control_name, _event=None):
        if not self.joint_commands_allowed():
            return
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

    def sync_joint_ranges(self):
        generation, lift_min, lift_max, ranges = (
            self.node.get_joint_range_snapshot())
        if generation == self.last_config_generation:
            return False
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
        return True

    def sync_joint_feedback(self):
        generation, feedback = self.node.get_kfs_load_feedback_snapshot()
        if generation == self.last_kfs_load_feedback_generation:
            return
        self.last_kfs_load_feedback_generation = generation
        controls = {item['name']: item for item in self.FLOAT_CONTROLS}
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

    def set_joint_controls_state(self, state):
        for widget in self.joint_control_widgets:
            widget.configure(state=state)
