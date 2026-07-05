import select
import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


HELP_TEXT = """
Robot R2 WASD teleop
--------------------
W/S : forward / backward
A/D : left / right strafe
Q/E : rotate left / right
X or Space : stop

CTRL-C to quit
"""


class WasdTeleop(Node):
    def __init__(self):
        super().__init__('robot_r2_wasd_teleop')

        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('linear_speed', 1.5)
        self.declare_parameter('angular_speed', 1.57)
        self.declare_parameter('publish_rate', 20.0)

        topic = self.get_parameter('cmd_vel_topic').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        publish_rate = self.get_parameter('publish_rate').value

        self.publisher = self.create_publisher(Twist, topic, 10)
        self.command = Twist()
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_command)

        self.get_logger().info(f'Publishing Twist commands on /{topic}')

    def publish_command(self):
        self.publisher.publish(self.command)

    def set_command(self, key):
        self.command = Twist()

        if key == 'w':
            self.command.linear.x = self.linear_speed
        elif key == 's':
            self.command.linear.x = -self.linear_speed
        elif key == 'a':
            self.command.linear.y = self.linear_speed
        elif key == 'd':
            self.command.linear.y = -self.linear_speed
        elif key == 'q':
            self.command.angular.z = self.angular_speed
        elif key == 'e':
            self.command.angular.z = -self.angular_speed
        elif key in ('x', ' '):
            pass
        else:
            return False

        return True

    def stop(self):
        self.command = Twist()
        self.publisher.publish(self.command)


def read_key(timeout_sec):
    readable, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    if readable:
        return sys.stdin.read(1).lower()
    return None


def main():
    rclpy.init()
    node = WasdTeleop()
    old_settings = termios.tcgetattr(sys.stdin)

    print(HELP_TEXT)

    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key(0.05)
            if key is None:
                continue
            if key == '\x03':
                break
            node.set_command(key)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
