import threading

from geometry_msgs.msg import Twist
from pynput import keyboard
import rclpy
from rclpy.node import Node
from robot_r2_interfaces.srv import SetLift
from std_msgs.msg import Float64


HELP_TEXT = """
Robot R2 WASD teleop
--------------------
W/S : forward / backward
A/D : left / right strafe
Q/E : rotate left / right
U/J : bar lift up / down (hold)
I/K : bar rotate forward / back (hold)
O/L : gripper open / close (hold)
1   : front lift to +0.20 m
2   : rear lift to +0.20 m
3   : both front and rear lift to +0.20 m
4   : both front and rear lift to 0
X or Space : stop

CTRL-C to quit
"""

MOTION_KEYS = {'w', 'a', 's', 'd', 'q', 'e'}
GRIPPER_KEYS = {'u', 'j', 'i', 'k', 'o', 'l'}

LIFT_PRESETS = {
    '1': (0.20, 0.0),
    '2': (0.0, 0.20),
    '3': (0.20, 0.20),
    '4': (0.0, 0.0),
}


class WasdTeleop(Node):
    def __init__(self):
        super().__init__('teleop_control')

        self.declare_parameter('cmd_vel_topic', '/r2/cmd_vel')
        self.declare_parameter('set_lift_service', '/r2/lift/set')
        self.declare_parameter('linear_speed', 1.5)
        self.declare_parameter('angular_speed', 1.57)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('lift_timeout_sec', 10.0)
        self.declare_parameter('bar_lift_speed', 0.05)
        self.declare_parameter('bar_rotate_speed', 0.5)
        self.declare_parameter('gripper_speed', 0.05)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        set_lift_service = self.get_parameter('set_lift_service').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        publish_rate = self.get_parameter('publish_rate').value
        self.lift_timeout_sec = self.get_parameter('lift_timeout_sec').value
        self.bar_lift_speed = self.get_parameter('bar_lift_speed').value
        self.bar_rotate_speed = self.get_parameter('bar_rotate_speed').value
        self.gripper_speed = self.get_parameter('gripper_speed').value

        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.bar_lift_pub = self.create_publisher(Float64, '/r2/gripper/lift_cmd', 10)
        self.bar_rotate_pub = self.create_publisher(Float64, '/r2/gripper/rotate_cmd', 10)
        self.gripper_pub = self.create_publisher(Float64, '/r2/gripper/grip_cmd', 10)

        self.set_lift_client = self.create_client(SetLift, set_lift_service)

        self.pressed_keys = set()
        self.preset_keys_down = set()
        self.key_lock = threading.Lock()

        self.lift_target = -0.180
        self.rotate_target = 0.0
        self.grip_target = 0.0

        self.timer = self.create_timer(1.0 / publish_rate, self.publish_commands)
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release,
        )
        self.listener.start()

    def publish_commands(self):
        dt = 1.0 / 20.0

        with self.key_lock:
            active = set(self.pressed_keys)

        # Chassis motion
        if any(k in active for k in MOTION_KEYS):
            cmd_vel = Twist()
            cmd_vel.linear.x = self.linear_speed * (
                int('w' in active) - int('s' in active))
            cmd_vel.linear.y = self.linear_speed * (
                int('a' in active) - int('d' in active))
            cmd_vel.angular.z = self.angular_speed * (
                int('q' in active) - int('e' in active))
            self.cmd_vel_publisher.publish(cmd_vel)

        # Gripper controls (hold to move, release = stop at current position)
        if any(k in active for k in GRIPPER_KEYS):
            if 'u' in active:
                self.lift_target += self.bar_lift_speed * dt
            if 'j' in active:
                self.lift_target -= self.bar_lift_speed * dt
            self.lift_target = max(-0.180, min(0.240, self.lift_target))

            if 'i' in active:
                self.rotate_target -= self.bar_rotate_speed * dt
            if 'k' in active:
                self.rotate_target += self.bar_rotate_speed * dt
            self.rotate_target = max(-3.14159, min(0.0, self.rotate_target))

            if 'o' in active:
                self.grip_target -= self.gripper_speed * dt
            if 'l' in active:
                self.grip_target += self.gripper_speed * dt
            self.grip_target = max(0.0, min(0.209, self.grip_target))

            self.bar_lift_pub.publish(Float64(data=self.lift_target))
            self.bar_rotate_pub.publish(Float64(data=self.rotate_target))
            self.gripper_pub.publish(Float64(data=self.grip_target))

    def on_press(self, key):
        key_name = self.normalize_key(key)
        if key_name is None:
            return

        publish_zero = False
        lift_preset = None

        with self.key_lock:
            if key_name in ('space', 'x'):
                self.pressed_keys.clear()
                self.preset_keys_down.clear()
                publish_zero = True
            elif key_name in LIFT_PRESETS:
                if key_name in self.preset_keys_down:
                    return
                self.preset_keys_down.add(key_name)
                lift_preset = LIFT_PRESETS[key_name]
            else:
                self.pressed_keys.add(key_name)

        if publish_zero:
            self.publish_zero_all()
        if lift_preset is not None:
            self.send_lift_request(*lift_preset)

    def on_release(self, key):
        key_name = self.normalize_key(key)
        if key_name is None:
            return

        publish_zero = False
        with self.key_lock:
            if key_name in LIFT_PRESETS:
                self.preset_keys_down.discard(key_name)
            self.pressed_keys.discard(key_name)
            publish_zero = not any(
                k in self.pressed_keys for k in MOTION_KEYS
            )

        if publish_zero:
            self.publish_zero_twist()

    def publish_zero_all(self):
        self.publish_zero_twist()
        self.lift_target = -0.180
        self.rotate_target = 0.0
        self.grip_target = 0.0
        self.bar_lift_pub.publish(Float64(data=0.0))
        self.bar_rotate_pub.publish(Float64(data=0.0))
        self.gripper_pub.publish(Float64(data=0.0))

    def publish_zero_twist(self):
        self.cmd_vel_publisher.publish(Twist())

    def send_lift_request(self, front_lift, rear_lift):
        if not self.set_lift_client.service_is_ready():
            return

        request = SetLift.Request()
        request.front_lift = float(front_lift)
        request.rear_lift = float(rear_lift)
        request.timeout_sec = float(self.lift_timeout_sec)

        future = self.set_lift_client.call_async(request)
        future.add_done_callback(self.on_lift_response)

    def on_lift_response(self, future):
        response = future.result()
        if response is None:
            return

    def stop(self):
        with self.key_lock:
            self.pressed_keys.clear()
            self.preset_keys_down.clear()

        self.publish_zero_all()

        if self.listener.running:
            self.listener.stop()

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
