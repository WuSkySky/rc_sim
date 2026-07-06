import threading
import time

from geometry_msgs.msg import Twist
from pynput import keyboard
import rclpy
from rclpy.node import Node
from robot_r2_interfaces.msg import LiftCommand


HELP_TEXT = """
Robot R2 WASD teleop
--------------------
W/S : forward / backward
A/D : left / right strafe
Q/E : rotate left / right
R/F : front lift fine up / down
T/G : rear lift fine up / down
1/2 : front lift to +0.20 / -0.20 m, rear lift to 0
3/4 : rear lift to +0.20 / -0.20 m, front lift to 0
5   : front and rear lift to 0
X or Space : stop

CTRL-C to quit
"""


class WasdTeleop(Node):
    def __init__(self):
        super().__init__('robot_r2_wasd_teleop')

        self.declare_parameter('cmd_vel_topic', '/r2/cmd_vel')
        self.declare_parameter('cmd_lift_topic', '/r2/lift/cmd_lift')
        self.declare_parameter('linear_speed', 1.5)
        self.declare_parameter('angular_speed', 1.57)
        self.declare_parameter('lift_speed', 0.25)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('min_lift', -0.3)
        self.declare_parameter('max_lift', 0.3)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        cmd_lift_topic = self.get_parameter('cmd_lift_topic').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.lift_speed = self.get_parameter('lift_speed').value
        publish_rate = self.get_parameter('publish_rate').value
        self.min_lift = self.get_parameter('min_lift').value
        self.max_lift = self.get_parameter('max_lift').value

        if self.min_lift > self.max_lift:
            self.min_lift, self.max_lift = self.max_lift, self.min_lift

        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.cmd_lift_publisher = self.create_publisher(
            LiftCommand, cmd_lift_topic, 10)

        self.pressed_keys = set()
        self.preset_keys_down = set()
        self.key_lock = threading.Lock()
        self.front_lift_target = 0.0
        self.rear_lift_target = 0.0
        self.last_tick = time.monotonic()
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_commands)
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release,
        )
        self.listener.start()

        self.get_logger().info(
            f'Publishing Twist commands on {cmd_vel_topic} '
            f'and lift commands on {cmd_lift_topic}')

    def publish_commands(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now

        with self.key_lock:
            active_keys = set(self.pressed_keys)

        linear_x = self.linear_speed * (
            int('w' in active_keys) - int('s' in active_keys))
        linear_y = self.linear_speed * (
            int('a' in active_keys) - int('d' in active_keys))
        angular_z = self.angular_speed * (
            int('q' in active_keys) - int('e' in active_keys))

        self.front_lift_target = self.clamp(
            self.front_lift_target + self.lift_speed * dt * (
                int('r' in active_keys) - int('f' in active_keys)),
            self.min_lift,
            self.max_lift,
        )
        self.rear_lift_target = self.clamp(
            self.rear_lift_target + self.lift_speed * dt * (
                int('t' in active_keys) - int('g' in active_keys)),
            self.min_lift,
            self.max_lift,
        )

        cmd_vel = Twist()
        cmd_vel.linear.x = linear_x
        cmd_vel.linear.y = linear_y
        cmd_vel.angular.z = angular_z

        cmd_lift = LiftCommand()
        cmd_lift.front_lift = self.front_lift_target
        cmd_lift.rear_lift = self.rear_lift_target

        self.cmd_vel_publisher.publish(cmd_vel)
        self.cmd_lift_publisher.publish(cmd_lift)

    def on_press(self, key):
        key_name = self.normalize_key(key)
        if key_name is None:
            return

        with self.key_lock:
            if key_name in ('space', 'x'):
                self.pressed_keys.clear()
                self.preset_keys_down.clear()
                return
            if key_name in ('1', '2', '3', '4', '5'):
                if key_name in self.preset_keys_down:
                    return
                self.preset_keys_down.add(key_name)
                self.apply_lift_preset(key_name)
                return
            self.pressed_keys.add(key_name)

    def on_release(self, key):
        key_name = self.normalize_key(key)
        if key_name is None:
            return

        with self.key_lock:
            if key_name in ('1', '2', '3', '4', '5'):
                self.preset_keys_down.discard(key_name)
            self.pressed_keys.discard(key_name)

    def stop(self):
        with self.key_lock:
            self.pressed_keys.clear()
            self.preset_keys_down.clear()

        cmd_vel = Twist()
        cmd_lift = LiftCommand()
        cmd_lift.front_lift = self.front_lift_target
        cmd_lift.rear_lift = self.rear_lift_target

        self.cmd_vel_publisher.publish(cmd_vel)
        self.cmd_lift_publisher.publish(cmd_lift)

        if self.listener.running:
            self.listener.stop()

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(value, maximum))

    def apply_lift_preset(self, key_name):
        if key_name == '1':
            self.front_lift_target = self.clamp(0.20, self.min_lift, self.max_lift)
            self.rear_lift_target = 0.0
        elif key_name == '2':
            self.front_lift_target = self.clamp(-0.20, self.min_lift, self.max_lift)
            self.rear_lift_target = 0.0
        elif key_name == '3':
            self.front_lift_target = 0.0
            self.rear_lift_target = self.clamp(0.20, self.min_lift, self.max_lift)
        elif key_name == '4':
            self.front_lift_target = 0.0
            self.rear_lift_target = self.clamp(-0.20, self.min_lift, self.max_lift)
        elif key_name == '5':
            self.front_lift_target = 0.0
            self.rear_lift_target = 0.0

    @staticmethod
    def normalize_key(key):
        if isinstance(key, keyboard.KeyCode) and key.char is not None:
            return key.char.lower()
        if key == keyboard.Key.space:
            return 'space'
        return None


def main():
    rclpy.init()
    node = WasdTeleop()

    print(HELP_TEXT)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
