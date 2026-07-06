import math
import time

from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult


class PidAxis:
    def __init__(self, kp, ki, kd, integral_limit, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self.output_limit = abs(output_limit)
        self.integral = 0.0
        self.last_error = 0.0
        self.has_last_error = False

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.has_last_error = False

    def update(self, error, dt):
        if dt <= 0.0:
            return 0.0

        self.integral += error * dt
        if self.integral_limit > 0.0:
            self.integral = max(
                -self.integral_limit,
                min(self.integral, self.integral_limit),
            )

        derivative = 0.0
        if self.has_last_error:
            derivative = (error - self.last_error) / dt

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.last_error = error
        self.has_last_error = True

        if self.output_limit > 0.0:
            output = max(-self.output_limit, min(output, self.output_limit))

        return output

    def configure(self, kp, ki, kd, integral_limit, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self.output_limit = abs(output_limit)


class PoseServo(Node):
    def __init__(self):
        super().__init__('robot_r2_pose_servo')

        self.declare_parameter('current_pose_topic', '/r2/pose_feedback')
        self.declare_parameter('target_pose_topic', '/r2/target_pose')
        self.declare_parameter('cmd_vel_topic', '/r2/cmd_vel')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('position_tolerance', 0.02)
        self.declare_parameter('yaw_tolerance', 0.03)

        self.declare_parameter('x_kp', 2.5)
        self.declare_parameter('x_ki', 0.0)
        self.declare_parameter('x_kd', 0.2)
        self.declare_parameter('x_integral_limit', 0.5)
        self.declare_parameter('x_output_limit', 2.0)

        self.declare_parameter('y_kp', 2.5)
        self.declare_parameter('y_ki', 0.0)
        self.declare_parameter('y_kd', 0.2)
        self.declare_parameter('y_integral_limit', 0.5)
        self.declare_parameter('y_output_limit', 2.0)

        self.declare_parameter('yaw_kp', 3.0)
        self.declare_parameter('yaw_ki', 0.0)
        self.declare_parameter('yaw_kd', 0.2)
        self.declare_parameter('yaw_integral_limit', 0.5)
        self.declare_parameter('yaw_output_limit', 2.0)

        current_pose_topic = self.get_parameter('current_pose_topic').value
        target_pose_topic = self.get_parameter('target_pose_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        publish_rate = self.get_parameter('publish_rate').value
        self.position_tolerance = self.get_parameter(
            'position_tolerance').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value

        self.x_pid = PidAxis(
            self.get_parameter('x_kp').value,
            self.get_parameter('x_ki').value,
            self.get_parameter('x_kd').value,
            self.get_parameter('x_integral_limit').value,
            self.get_parameter('x_output_limit').value,
        )
        self.y_pid = PidAxis(
            self.get_parameter('y_kp').value,
            self.get_parameter('y_ki').value,
            self.get_parameter('y_kd').value,
            self.get_parameter('y_integral_limit').value,
            self.get_parameter('y_output_limit').value,
        )
        self.yaw_pid = PidAxis(
            self.get_parameter('yaw_kp').value,
            self.get_parameter('yaw_ki').value,
            self.get_parameter('yaw_kd').value,
            self.get_parameter('yaw_integral_limit').value,
            self.get_parameter('yaw_output_limit').value,
        )

        self.current_pose = None
        self.target_pose = None
        self.last_tick = time.monotonic()

        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.current_pose_subscriber = self.create_subscription(
            PoseStamped,
            current_pose_topic,
            self.on_current_pose,
            10,
        )
        self.target_pose_subscriber = self.create_subscription(
            PoseStamped,
            target_pose_topic,
            self.on_target_pose,
            10,
        )

        self.timer = self.create_timer(1.0 / publish_rate, self.control_loop)
        self.add_on_set_parameters_callback(self.on_parameters_changed)

        self.get_logger().info(
            f'Pose servo active: current={current_pose_topic}, '
            f'target={target_pose_topic}, cmd={cmd_vel_topic}')

    def on_current_pose(self, msg):
        self.current_pose = msg

    def on_target_pose(self, msg):
        self.target_pose = msg
        self.reset_controllers()

    def control_loop(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now

        if self.current_pose is None or self.target_pose is None:
            return

        current = self.current_pose.pose
        target = self.target_pose.pose

        current_yaw = self.yaw_from_quaternion(current.orientation)
        target_yaw = self.yaw_from_quaternion(target.orientation)

        dx_world = target.position.x - current.position.x
        dy_world = target.position.y - current.position.y
        yaw_error = self.normalize_angle(target_yaw - current_yaw)

        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        error_x = cos_yaw * dx_world + sin_yaw * dy_world
        error_y = -sin_yaw * dx_world + cos_yaw * dy_world

        if (
            abs(error_x) <= self.position_tolerance and
            abs(error_y) <= self.position_tolerance and
            abs(yaw_error) <= self.yaw_tolerance
        ):
            self.reset_controllers()
            self.publish_zero_twist()
            return

        cmd_vel = Twist()
        cmd_vel.linear.x = self.x_pid.update(error_x, dt)
        cmd_vel.linear.y = self.y_pid.update(error_y, dt)
        cmd_vel.angular.z = self.yaw_pid.update(yaw_error, dt)
        self.cmd_vel_publisher.publish(cmd_vel)

    def reset_controllers(self):
        self.x_pid.reset()
        self.y_pid.reset()
        self.yaw_pid.reset()

    def publish_zero_twist(self):
        self.cmd_vel_publisher.publish(Twist())

    def on_parameters_changed(self, params):
        values = {}
        for param in params:
            values[param.name] = param.value

        position_tolerance = values.get(
            'position_tolerance', self.position_tolerance)

        yaw_tolerance = values.get(
            'yaw_tolerance', self.yaw_tolerance)

        current_period = self.timer.timer_period_ns / 1e9
        current_rate = 1.0 / current_period

        publish_rate = values.get(
            'publish_rate', current_rate)

        if position_tolerance < 0.0:
            return SetParametersResult(
                successful=False,
                reason='position_tolerance must be non-negative',
            )
        if yaw_tolerance < 0.0:
            return SetParametersResult(
                successful=False,
                reason='yaw_tolerance must be non-negative',
            )
        if publish_rate <= 0.0:
            return SetParametersResult(
                successful=False,
                reason='publish_rate must be greater than zero',
            )

        x_kp = values.get('x_kp', self.x_pid.kp)
        x_ki = values.get('x_ki', self.x_pid.ki)
        x_kd = values.get('x_kd', self.x_pid.kd)
        x_integral_limit = values.get(
            'x_integral_limit', self.x_pid.integral_limit)
        x_output_limit = values.get('x_output_limit', self.x_pid.output_limit)

        y_kp = values.get('y_kp', self.y_pid.kp)
        y_ki = values.get('y_ki', self.y_pid.ki)
        y_kd = values.get('y_kd', self.y_pid.kd)
        y_integral_limit = values.get(
            'y_integral_limit', self.y_pid.integral_limit)
        y_output_limit = values.get('y_output_limit', self.y_pid.output_limit)

        yaw_kp = values.get('yaw_kp', self.yaw_pid.kp)
        yaw_ki = values.get('yaw_ki', self.yaw_pid.ki)
        yaw_kd = values.get('yaw_kd', self.yaw_pid.kd)
        yaw_integral_limit = values.get(
            'yaw_integral_limit', self.yaw_pid.integral_limit)
        yaw_output_limit = values.get(
            'yaw_output_limit', self.yaw_pid.output_limit)

        self.position_tolerance = position_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.x_pid.configure(
            x_kp, x_ki, x_kd, x_integral_limit, x_output_limit)
        self.y_pid.configure(
            y_kp, y_ki, y_kd, y_integral_limit, y_output_limit)
        self.yaw_pid.configure(
            yaw_kp, yaw_ki, yaw_kd, yaw_integral_limit, yaw_output_limit)

        new_period = 1.0 / publish_rate
        current_period = self.timer.timer_period_ns / 1e9
        if abs(new_period - current_period) > 1e-9:
            self.timer.cancel()
            self.timer = self.create_timer(new_period, self.control_loop)

        return SetParametersResult(successful=True)

    @staticmethod
    def yaw_from_quaternion(quaternion):
        siny_cosp = 2.0 * (
            quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main():
    rclpy.init()
    node = PoseServo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero_twist()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
