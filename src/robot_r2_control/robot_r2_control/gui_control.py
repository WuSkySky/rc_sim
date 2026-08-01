from functools import partial
import math
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from robot_r2_interfaces.msg import LiftCommand
from std_msgs.msg import Float64


class GuiControlNode(Node):
    FLOAT_CONTROL_PARAMETERS = {
        'kfs_lift': {
            'topic': (
                'kfs_lift_command_topic',
                '/r2/kfs_lift/cmd',
            ),
            'minimum': ('kfs_lift_min', 0.0),
            'maximum': ('kfs_lift_max', 0.42),
        },
        'root_rotate': {
            'topic': (
                'root_rotate_command_topic',
                '/r2/gripper/rotate_cmd',
            ),
            'minimum': ('root_rotate_min', 0.0),
            'maximum': ('root_rotate_max', 2.356194490192345),
        },
        'tip_rotate': {
            'topic': (
                'tip_rotate_command_topic',
                '/r2/gripper/tip_rotate_cmd',
            ),
            'minimum': ('tip_rotate_min', 0.0),
            'maximum': ('tip_rotate_max', math.pi),
        },
        'grip': {
            'topic': (
                'grip_command_topic',
                '/r2/gripper/grip_cmd',
            ),
            'minimum': ('grip_min', 0.0),
            'maximum': ('grip_max', 0.209),
        },
        'weapon_rotate': {
            'topic': (
                'weapon_rotate_command_topic',
                '/r2/weapon/rotate_cmd',
            ),
            'minimum': ('weapon_rotate_min', 0.0),
            'maximum': (
                'weapon_rotate_max',
                math.radians(200.0),
            ),
        },
        'weapon_grip': {
            'topic': (
                'weapon_grip_command_topic',
                '/r2/weapon/grip_cmd',
            ),
            'minimum': ('weapon_grip_min', 0.0),
            'maximum': ('weapon_grip_max', 0.03),
        },
    }

    def __init__(self):
        super().__init__('gui_control')

        self.declare_parameter(
            'lift_command_topic',
            '/r2/lift/cmd_lift',
        )
        self.declare_parameter('lift_min', 0.0)
        self.declare_parameter('lift_max', 0.376)

        for parameters in self.FLOAT_CONTROL_PARAMETERS.values():
            topic_parameter, topic_default = parameters['topic']
            minimum_parameter, minimum_default = parameters['minimum']
            maximum_parameter, maximum_default = parameters['maximum']
            self.declare_parameter(topic_parameter, topic_default)
            self.declare_parameter(minimum_parameter, minimum_default)
            self.declare_parameter(maximum_parameter, maximum_default)

        lift_command_topic = self.get_parameter(
            'lift_command_topic').value
        self.lift_min = float(self.get_parameter('lift_min').value)
        self.lift_max = float(self.get_parameter('lift_max').value)

        if not lift_command_topic:
            raise ValueError('lift_command_topic must not be empty')
        self._validate_range('lift', self.lift_min, self.lift_max)

        self.lift_command_publisher = self.create_publisher(
            LiftCommand,
            lift_command_topic,
            10,
        )

        self.float_control_ranges = {}
        self.float_command_publishers = {}
        for control_name, parameters in (
                self.FLOAT_CONTROL_PARAMETERS.items()):
            topic_parameter, _ = parameters['topic']
            minimum_parameter, _ = parameters['minimum']
            maximum_parameter, _ = parameters['maximum']

            command_topic = self.get_parameter(topic_parameter).value
            minimum = float(self.get_parameter(minimum_parameter).value)
            maximum = float(self.get_parameter(maximum_parameter).value)

            if not command_topic:
                raise ValueError(f'{topic_parameter} must not be empty')
            self._validate_range(control_name, minimum, maximum)

            self.float_control_ranges[control_name] = (minimum, maximum)
            self.float_command_publishers[control_name] = (
                self.create_publisher(Float64, command_topic, 10)
            )

    @staticmethod
    def _validate_range(name, minimum, maximum):
        if not math.isfinite(minimum):
            raise ValueError(f'{name} minimum must be finite')
        if not math.isfinite(maximum):
            raise ValueError(f'{name} maximum must be finite')
        if minimum >= maximum:
            raise ValueError(
                f'{name} minimum must be less than maximum')

    def publish_lift_command(self, front_lift, rear_lift):
        command = LiftCommand()
        command.front_lift = float(front_lift)
        command.rear_lift = float(rear_lift)
        self.lift_command_publisher.publish(command)

    def publish_float_command(self, control_name, value):
        command = Float64()
        command.data = float(value)
        self.float_command_publishers[control_name].publish(command)


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
            'label': '夹爪尖端旋转（0 初始，正向）',
            'unit': 'rad',
            'resolution': 0.001,
            'decimals': 3,
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
            _, maximum = node.float_control_ranges[control_name]
            to_display = control.get('to_display', float)
            initial_value = (
                to_display(maximum)
                if control_name == 'grip'
                else 0.0
            )
            self.float_control_values[control_name] = tk.DoubleVar(
                value=initial_value)
        self.status_text = tk.StringVar(value='已就绪')

        self.last_lift_command = None
        self.last_float_commands = {}
        self._closed = False

        self._build_ui()

    def _build_ui(self):
        main_frame = ttk.Frame(self.root, padding=16)
        main_frame.grid(row=0, column=0, sticky='nsew')

        lift_frame = ttk.LabelFrame(
            main_frame,
            text='底盘抬升',
            padding=12,
        )
        lift_frame.grid(row=0, column=0, sticky='ew')

        self.front_lift_slider = self._create_slider(
            lift_frame,
            row=0,
            label='前轮抬升 (m)',
            variable=self.front_lift_value,
            minimum=self.node.lift_min,
            maximum=self.node.lift_max,
            resolution=0.001,
            release_callback=self._publish_lift_command,
        )
        self.rear_lift_slider = self._create_slider(
            lift_frame,
            row=1,
            label='后轮抬升 (m)',
            variable=self.rear_lift_value,
            minimum=self.node.lift_min,
            maximum=self.node.lift_max,
            resolution=0.001,
            release_callback=self._publish_lift_command,
        )
        self.combined_lift_slider = self._create_slider(
            lift_frame,
            row=2,
            label='一起抬升 (m)',
            variable=self.combined_lift_value,
            minimum=self.node.lift_min,
            maximum=self.node.lift_max,
            resolution=0.001,
            release_callback=self._publish_combined_lift_command,
        )

        self.float_control_sliders = {}
        self._create_control_frame(
            main_frame,
            row=1,
            title='KFS 机构',
            controls=self.KFS_CONTROLS,
        )
        self._create_control_frame(
            main_frame,
            row=2,
            title='武器机构',
            controls=self.WEAPON_CONTROLS,
        )

        status_label = ttk.Label(
            main_frame,
            textvariable=self.status_text,
            anchor='w',
        )
        status_label.grid(
            row=3,
            column=0,
            sticky='ew',
            pady=(10, 0),
        )

    def _create_control_frame(self, parent, row, title, controls):
        frame = ttk.LabelFrame(
            parent,
            text=title,
            padding=12,
        )
        frame.grid(
            row=row,
            column=0,
            sticky='ew',
            pady=(10, 0),
        )

        for control_row, control in enumerate(controls):
            control_name = control['name']
            minimum, maximum = self.node.float_control_ranges[
                control_name]
            to_display = control.get('to_display', float)
            self.float_control_sliders[control_name] = (
                self._create_slider(
                    frame,
                    row=control_row,
                    label=(
                        f"{control['label']} "
                        f"({control['unit']})"
                    ),
                    variable=self.float_control_values[control_name],
                    minimum=to_display(minimum),
                    maximum=to_display(maximum),
                    resolution=control['resolution'],
                    release_callback=partial(
                        self._publish_float_command,
                        control_name,
                    ),
                )
            )

    def _create_slider(
        self,
        parent,
        row,
        label,
        variable,
        minimum,
        maximum,
        resolution,
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
        slider.bind('<KeyRelease>', release_callback)
        return slider

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
            f'已发送：前 {targets[0]:.3f} m，'
            f'后 {targets[1]:.3f} m')

    def _publish_combined_lift_command(self, _event=None):
        target = round(self.combined_lift_value.get(), 3)
        self.front_lift_value.set(target)
        self.rear_lift_value.set(target)
        self.node.publish_lift_command(target, target)
        self.last_lift_command = (target, target)
        self.status_text.set(f'已发送：前后均为 {target:.3f} m')

    def _publish_float_command(self, control_name, _event=None):
        control = next(
            item
            for item in self.FLOAT_CONTROLS
            if item['name'] == control_name
        )
        decimals = control['decimals']
        display_target = round(
            self.float_control_values[control_name].get(),
            decimals,
        )
        if self.last_float_commands.get(
                control_name) == display_target:
            self.status_text.set(
                f"{control['label']}目标值未变化")
            return

        to_command = control.get('to_command', float)
        self.node.publish_float_command(
            control_name,
            to_command(display_target),
        )
        self.last_float_commands[control_name] = display_target
        self.status_text.set(
            f"已发送：{control['label']} "
            f"{display_target:.{decimals}f} "
            f"{control['unit']}")

    def run(self):
        self.root.mainloop()

    def close(self):
        if self._closed:
            return
        self._closed = True
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
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
