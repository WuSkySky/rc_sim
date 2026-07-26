import math
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from robot_r2_interfaces.msg import LiftCommand


class GuiControlNode(Node):
    def __init__(self):
        super().__init__('gui_control')

        self.declare_parameter(
            'lift_command_topic',
            '/r2/lift/cmd_lift',
        )
        self.declare_parameter('lift_min', 0.0)
        self.declare_parameter('lift_max', 0.376)

        lift_command_topic = self.get_parameter(
            'lift_command_topic').value
        self.lift_min = float(self.get_parameter('lift_min').value)
        self.lift_max = float(self.get_parameter('lift_max').value)

        if not lift_command_topic:
            raise ValueError('lift_command_topic must not be empty')
        if not math.isfinite(self.lift_min):
            raise ValueError('lift_min must be finite')
        if not math.isfinite(self.lift_max):
            raise ValueError('lift_max must be finite')
        if self.lift_min >= self.lift_max:
            raise ValueError('lift_min must be less than lift_max')

        self.lift_command_publisher = self.create_publisher(
            LiftCommand,
            lift_command_topic,
            10,
        )

    def publish_lift_command(self, front_lift, rear_lift):
        command = LiftCommand()
        command.front_lift = float(front_lift)
        command.rear_lift = float(rear_lift)
        self.lift_command_publisher.publish(command)


class GuiControlApp:
    LIFT_VALUE_RESOLUTION = 0.001

    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Robot R2 GUI 控制')
        self.root.resizable(False, False)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.front_lift_value = tk.DoubleVar(value=node.lift_min)
        self.rear_lift_value = tk.DoubleVar(value=node.lift_min)
        self.combined_lift_value = tk.DoubleVar(value=node.lift_min)
        self.status_text = tk.StringVar(value='已就绪')

        self.last_lift_command = None
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

        self.front_lift_slider = self._create_lift_slider(
            lift_frame,
            row=0,
            label='前轮抬升 (m)',
            variable=self.front_lift_value,
        )
        self.rear_lift_slider = self._create_lift_slider(
            lift_frame,
            row=1,
            label='后轮抬升 (m)',
            variable=self.rear_lift_value,
        )
        self.combined_lift_slider = self._create_lift_slider(
            lift_frame,
            row=2,
            label='一起抬升 (m)',
            variable=self.combined_lift_value,
            release_callback=self._publish_combined_lift_command,
        )

        status_label = ttk.Label(
            main_frame,
            textvariable=self.status_text,
            anchor='w',
        )
        status_label.grid(
            row=1,
            column=0,
            sticky='ew',
            pady=(10, 0),
        )

    def _create_lift_slider(
        self,
        parent,
        row,
        label,
        variable,
        release_callback=None,
    ):
        if release_callback is None:
            release_callback = self._publish_lift_command

        slider = tk.Scale(
            parent,
            label=label,
            variable=variable,
            from_=self.node.lift_min,
            to=self.node.lift_max,
            resolution=self.LIFT_VALUE_RESOLUTION,
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
