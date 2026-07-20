"""ROS 2 bridge between Robot R2 command topics and the serial port."""

import math
import threading
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from robot_r2_interfaces.msg import LiftCommand, LiftFeedback
import serial
from std_msgs.msg import Float64, String

from serial_pkg.protocol import (
    FLOAT_FIELD_COUNT,
    FrameParser,
    decode_frame,
    encode_frame,
)


class SerialBridge(Node):
    VX = 0
    VY = 1
    VW = 2
    FRONT_LIFT = 3
    REAR_LIFT = 4
    KFS_LIFT = 5
    KFS_ROOT_ROTATE = 6
    KFS_TIP_ROTATE = 7
    KFS_GRIP = 8
    WEAPON_ROTATE = 9
    WEAPON_GRIP = 10

    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('send_rate', 50.0)
        self.declare_parameter('read_rate', 200.0)
        self.declare_parameter('receive_feedback_enabled', True)
        self.declare_parameter('reconnect_interval_sec', 1.0)
        self.declare_parameter('write_timeout_sec', 0.1)
        self.declare_parameter('frame_header', 0xAA)
        self.declare_parameter('frame_tail', 0x55)
        self.declare_parameter('float_endianness', 'little')
        self.declare_parameter('raw_rx_topic', '/r2/serial/raw_rx')
        self.declare_parameter('raw_tx_topic', '/r2/serial/raw_tx')

        self.declare_parameter('cmd_vel_topic', '/r2/cmd_vel')
        self.declare_parameter(
            'velocity_feedback_topic', '/r2/velocity_feedback')
        self.declare_parameter('lift_command_topic', '/r2/lift/cmd_lift')
        self.declare_parameter(
            'lift_feedback_topic', '/r2/lift/position_feedback')
        self.declare_parameter('kfs_lift_topic', '/r2/kfs_lift/cmd')
        self.declare_parameter(
            'kfs_lift_feedback_topic', '/r2/kfs_lift/feedback')
        self.declare_parameter(
            'kfs_root_rotate_topic', '/r2/gripper/rotate_cmd')
        self.declare_parameter(
            'kfs_root_rotate_feedback_topic',
            '/r2/gripper/rotate_feedback')
        self.declare_parameter(
            'kfs_tip_rotate_topic', '/r2/gripper/tip_rotate_cmd')
        self.declare_parameter(
            'kfs_tip_rotate_feedback_topic',
            '/r2/gripper/tip_rotate_feedback')
        self.declare_parameter('kfs_grip_topic', '/r2/gripper/grip_cmd')
        self.declare_parameter(
            'kfs_grip_feedback_topic', '/r2/gripper/grip_feedback')
        self.declare_parameter(
            'weapon_rotate_topic', '/r2/weapon/rotate_cmd')
        self.declare_parameter(
            'weapon_rotate_feedback_topic', '/r2/weapon/rotate_feedback')
        self.declare_parameter('weapon_grip_topic', '/r2/weapon/grip_cmd')
        self.declare_parameter(
            'weapon_grip_feedback_topic', '/r2/weapon/grip_feedback')
        self.declare_parameter(
            'initial_command_values',
            [
                0.0, 0.0, 0.0,
                0.0, 0.0,
                0.0, -math.pi / 2.0, math.pi / 2.0, 0.0,
                0.0, 0.0,
            ],
        )

        self.serial_port_name = str(
            self.get_parameter('serial_port').value)
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.send_rate = self._positive_float_parameter('send_rate')
        self.read_rate = self._positive_float_parameter('read_rate')
        self.receive_feedback_enabled = bool(
            self.get_parameter('receive_feedback_enabled').value)
        self.reconnect_interval_sec = self._positive_float_parameter(
            'reconnect_interval_sec')
        self.write_timeout_sec = self._positive_float_parameter(
            'write_timeout_sec')
        self.frame_header = self._byte_parameter('frame_header')
        self.frame_tail = self._byte_parameter('frame_tail')
        self.float_endianness = str(
            self.get_parameter('float_endianness').value)
        if self.float_endianness not in ('little', 'big'):
            raise ValueError(
                "float_endianness must be 'little' or 'big'")
        if self.baud_rate <= 0:
            raise ValueError('baud_rate must be positive')

        initial_values = tuple(
            float(value)
            for value in self.get_parameter('initial_command_values').value
        )
        if len(initial_values) != FLOAT_FIELD_COUNT:
            raise ValueError(
                f'initial_command_values must contain '
                f'{FLOAT_FIELD_COUNT} values')
        if not all(math.isfinite(value) for value in initial_values):
            raise ValueError('initial_command_values must all be finite')

        self.command_lock = threading.Lock()
        self.command_values = list(initial_values)
        self.serial_port = None
        self.next_reconnect_time = 0.0
        self.parser = FrameParser(self.frame_header, self.frame_tail)

        raw_rx_topic = str(self.get_parameter('raw_rx_topic').value)
        self.raw_rx_publisher = self.create_publisher(
            String, raw_rx_topic, 10)
        raw_tx_topic = str(self.get_parameter('raw_tx_topic').value)
        self.raw_tx_publisher = self.create_publisher(
            String, raw_tx_topic, 10)

        self.velocity_feedback_publisher = self.create_publisher(
            Twist,
            str(self.get_parameter('velocity_feedback_topic').value),
            10,
        )
        self.lift_feedback_publisher = self.create_publisher(
            LiftFeedback,
            str(self.get_parameter('lift_feedback_topic').value),
            10,
        )
        float_feedback_topics = (
            ('kfs_lift_feedback_topic', self.KFS_LIFT),
            ('kfs_root_rotate_feedback_topic', self.KFS_ROOT_ROTATE),
            ('kfs_tip_rotate_feedback_topic', self.KFS_TIP_ROTATE),
            ('kfs_grip_feedback_topic', self.KFS_GRIP),
            ('weapon_rotate_feedback_topic', self.WEAPON_ROTATE),
            ('weapon_grip_feedback_topic', self.WEAPON_GRIP),
        )
        self.float_feedback_publishers = {
            field_index: self.create_publisher(
                Float64,
                str(self.get_parameter(parameter_name).value),
                10,
            )
            for parameter_name, field_index in float_feedback_topics
        }

        self.command_subscriptions = [
            self.create_subscription(
                Twist,
                str(self.get_parameter('cmd_vel_topic').value),
                self.on_cmd_vel,
                10,
            ),
            self.create_subscription(
                LiftCommand,
                str(self.get_parameter('lift_command_topic').value),
                self.on_lift_command,
                10,
            ),
        ]
        float_topics = (
            ('kfs_lift_topic', self.KFS_LIFT),
            ('kfs_root_rotate_topic', self.KFS_ROOT_ROTATE),
            ('kfs_tip_rotate_topic', self.KFS_TIP_ROTATE),
            ('kfs_grip_topic', self.KFS_GRIP),
            ('weapon_rotate_topic', self.WEAPON_ROTATE),
            ('weapon_grip_topic', self.WEAPON_GRIP),
        )
        for parameter_name, field_index in float_topics:
            self.command_subscriptions.append(self.create_subscription(
                Float64,
                str(self.get_parameter(parameter_name).value),
                self.make_float_callback(field_index),
                10,
            ))

        self.send_timer = self.create_timer(
            1.0 / self.send_rate, self.send_latest_commands)
        self.read_timer = None
        if self.receive_feedback_enabled:
            self.read_timer = self.create_timer(
                1.0 / self.read_rate, self.read_serial_data)
        else:
            self.get_logger().info(
                'Serial feedback reception is disabled; command frames '
                'will still be transmitted')
        self.ensure_serial_connection()

    def _positive_float_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def _byte_parameter(self, name):
        value = int(self.get_parameter(name).value)
        if not 0 <= value <= 0xFF:
            raise ValueError(f'{name} must be between 0 and 255')
        return value

    def on_cmd_vel(self, message):
        values = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warn('Ignored non-finite cmd_vel')
            return
        with self.command_lock:
            self.command_values[self.VX:self.VW + 1] = values

    def on_lift_command(self, message):
        values = (float(message.front_lift), float(message.rear_lift))
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warn('Ignored non-finite lift command')
            return
        with self.command_lock:
            self.command_values[self.FRONT_LIFT:self.REAR_LIFT + 1] = values

    def make_float_callback(self, field_index):
        def callback(message):
            value = float(message.data)
            if not math.isfinite(value):
                self.get_logger().warn('Ignored non-finite float command')
                return
            with self.command_lock:
                self.command_values[field_index] = value

        return callback

    def ensure_serial_connection(self):
        if self.serial_port is not None and self.serial_port.is_open:
            return True

        now = time.monotonic()
        if now < self.next_reconnect_time:
            return False
        self.next_reconnect_time = now + self.reconnect_interval_sec

        try:
            self.serial_port = serial.Serial(
                port=self.serial_port_name,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.0,
                write_timeout=self.write_timeout_sec,
            )
        except (OSError, serial.SerialException) as exc:
            self.serial_port = None
            self.get_logger().warn(
                f'Unable to open {self.serial_port_name}: {exc}')
            return False

        self.parser.clear()
        self.get_logger().info(
            f'Opened {self.serial_port_name} at {self.baud_rate} baud')
        return True

    def mark_serial_disconnected(self, description, exception):
        self.get_logger().error(f'{description}: {exception}')
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except (OSError, serial.SerialException):
                pass
        self.serial_port = None
        self.parser.clear()
        self.next_reconnect_time = (
            time.monotonic() + self.reconnect_interval_sec)

    def send_latest_commands(self):
        if not self.ensure_serial_connection():
            return
        with self.command_lock:
            values = tuple(self.command_values)

        try:
            frame = encode_frame(
                values,
                self.frame_header,
                self.frame_tail,
                self.float_endianness,
            )
            written = self.serial_port.write(frame)
            if written != len(frame):
                raise serial.SerialTimeoutException(
                    f'wrote {written} of {len(frame)} bytes')
            message = String()
            message.data = frame.hex(' ').upper()
            self.raw_tx_publisher.publish(message)
        except (OSError, serial.SerialException) as exc:
            self.mark_serial_disconnected('Serial write failed', exc)

    def read_serial_data(self):
        if not self.ensure_serial_connection():
            return

        try:
            waiting = self.serial_port.in_waiting
            if waiting <= 0:
                return
            data = self.serial_port.read(waiting)
        except (OSError, serial.SerialException) as exc:
            self.mark_serial_disconnected('Serial read failed', exc)
            return

        for frame in self.parser.feed(data):
            message = String()
            message.data = frame.hex(' ').upper()
            self.raw_rx_publisher.publish(message)
            try:
                values = decode_frame(
                    frame,
                    self.frame_header,
                    self.frame_tail,
                    self.float_endianness,
                )
            except ValueError as exc:
                self.get_logger().warn(
                    f'Ignored invalid serial feedback frame: {exc}')
                continue
            self.publish_feedback(values)

    def publish_feedback(self, values):
        velocity = Twist()
        velocity.linear.x = values[self.VX]
        velocity.linear.y = values[self.VY]
        velocity.angular.z = values[self.VW]
        self.velocity_feedback_publisher.publish(velocity)

        lift = LiftFeedback()
        lift.front_left_lift = values[self.FRONT_LIFT]
        lift.front_right_lift = values[self.FRONT_LIFT]
        lift.rear_left_lift = values[self.REAR_LIFT]
        lift.rear_right_lift = values[self.REAR_LIFT]
        self.lift_feedback_publisher.publish(lift)

        for field_index, publisher in self.float_feedback_publishers.items():
            publisher.publish(Float64(data=values[field_index]))

    def close(self):
        if self.serial_port is None:
            return
        try:
            self.serial_port.close()
        except (OSError, serial.SerialException):
            pass
        self.serial_port = None


def main():
    rclpy.init()
    node = SerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
