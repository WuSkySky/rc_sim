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

U/J : root rotate forward / back (hold)
I/K : tip rotate forward / back (hold)
O/L : gripper open / close (hold)
1   : front lift to +0.20 m
2   : rear lift to +0.20 m
3   : both front and rear lift to +0.20 m
4   : both front and rear lift to 0
5   : both front and rear lift to maximum (+0.376 m)
X   : stop
|   : toggle keyboard control enable / disable
Space is reserved for Gazebo pause/resume

CTRL-C to quit
"""

MOTION_KEYS = {'w', 'a', 's', 'd', 'q', 'e'}
GRIPPER_KEYS = {'u', 'j', 'i', 'k', 'o', 'l'}

LIFT_PRESETS = {
    '1': (0.20, 0.0),
    '2': (0.0, 0.20),
    '3': (0.20, 0.20),
    '4': (0.0, 0.0),
    '5': (0.376, 0.376),
}


class WasdTeleop(Node):
    def __init__(self):
        super().__init__('teleop_control')

        self.declare_parameter('cmd_vel_topic', '/r2/cmd_vel')
        self.declare_parameter('set_lift_service', '/r2/lift/set')
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.57)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('lift_timeout_sec', 10.0)
        self.declare_parameter('bar_rotate_speed', 0.5)
        self.declare_parameter('gripper_speed', 0.05)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        set_lift_service = self.get_parameter('set_lift_service').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        publish_rate = self.get_parameter('publish_rate').value
        self.lift_timeout_sec = self.get_parameter('lift_timeout_sec').value
        self.bar_rotate_speed = self.get_parameter('bar_rotate_speed').value
        self.gripper_speed = self.get_parameter('gripper_speed').value

        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.root_rotate_pub = self.create_publisher(Float64, '/r2/gripper/rotate_cmd', 10)
        self.tip_rotate_pub = self.create_publisher(Float64, '/r2/gripper/tip_rotate_cmd', 10)
        self.gripper_pub = self.create_publisher(Float64, '/r2/gripper/grip_cmd', 10)

        self.set_lift_client = self.create_client(SetLift, set_lift_service)

        self.pressed_keys = set()
        self.preset_keys_down = set()
        self.key_lock = threading.Lock()
        self.keyboard_enabled = True
        self.toggle_key_down = False

        self.rotate_target = -1.5708
        self.tip_rotate_target = 1.5708
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
                self.rotate_target -= self.bar_rotate_speed * dt
            if 'j' in active:
                self.rotate_target += self.bar_rotate_speed * dt
            self.rotate_target = max(
                -1.5707963267948966,
                min(0.7853981633974483, self.rotate_target),
            )

            if 'i' in active:
                self.tip_rotate_target -= self.bar_rotate_speed * dt
            if 'k' in active:
                self.tip_rotate_target += self.bar_rotate_speed * dt
            self.tip_rotate_target = max(-1.5708, min(1.5708, self.tip_rotate_target))

            if 'o' in active:
                self.grip_target -= self.gripper_speed * dt
            if 'l' in active:
                self.grip_target += self.gripper_speed * dt
            self.grip_target = max(0.0, min(0.209, self.grip_target))

            self.root_rotate_pub.publish(Float64(data=self.rotate_target))
            self.tip_rotate_pub.publish(Float64(data=self.tip_rotate_target))
            self.gripper_pub.publish(Float64(data=self.grip_target))

    def on_press(self, key):
        key_name = self.normalize_key(key)
        if key_name is None:
            return

        stop_requested = False
        lift_preset = None
        enabled_changed = None

        with self.key_lock:
            if key_name == '|':
                if self.toggle_key_down:
                    return
                self.toggle_key_down = True
                self.keyboard_enabled = not self.keyboard_enabled
                enabled_changed = self.keyboard_enabled
                self.pressed_keys.clear()
                self.preset_keys_down.clear()
            elif not self.keyboard_enabled:
                return
            elif key_name == 'x':
                self.pressed_keys.clear()
                self.preset_keys_down.clear()
                stop_requested = True
            elif key_name in LIFT_PRESETS:
                if key_name in self.preset_keys_down:
                    return
                self.preset_keys_down.add(key_name)
                lift_preset = LIFT_PRESETS[key_name]
            else:
                self.pressed_keys.add(key_name)

        if enabled_changed is not None:
            self.stop_motion()
            state = 'enabled' if enabled_changed else 'disabled'
            self.get_logger().info(f'Keyboard control {state}')
        if stop_requested:
            self.stop_motion()
        if lift_preset is not None:
            self.send_lift_request(*lift_preset)

    def on_release(self, key):
        key_name = self.normalize_key(key)
        if key_name is None:
            return

        publish_zero = False
        with self.key_lock:
            if key_name == '|':
                self.toggle_key_down = False
                return
            if not self.keyboard_enabled:
                return
            if key_name in LIFT_PRESETS:
                self.preset_keys_down.discard(key_name)
            self.pressed_keys.discard(key_name)
            publish_zero = not any(
                k in self.pressed_keys for k in MOTION_KEYS
            )

        if publish_zero:
            self.publish_zero_twist()

    def stop_motion(self):
        # Position-controlled joints stop moving when their targets stop
        # changing. Publishing 0.0 here would command a new position instead.
        self.publish_zero_twist()

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
            self.toggle_key_down = False

        self.stop_motion()

        if self.listener.running:
            self.listener.stop()

    @staticmethod
    def normalize_key(key):
        if isinstance(key, keyboard.KeyCode) and key.char is not None:
            return key.char.lower()
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
