import math
import threading
import time

from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.srv import MoveToPose


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
        super().__init__('chassis_pose_servo')

        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter('current_pose_topic', '/r2/pose_feedback')
        self.declare_parameter('cmd_vel_topic', '/r2/cmd_vel')
        self.declare_parameter('move_to_pose_service', '/r2/move_to_pose')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('position_tolerance', 0.02)
        self.declare_parameter('yaw_tolerance', 0.03)
        self.declare_parameter('yaw_stable_cycles', 10)
        self.declare_parameter('default_timeout_sec', 20.0)

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
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        move_to_pose_service = self.get_parameter(
            'move_to_pose_service').value
        publish_rate = self.get_parameter('publish_rate').value
        self.position_tolerance = self.get_parameter(
            'position_tolerance').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value
        self.yaw_stable_cycles_required = int(
            self.get_parameter('yaw_stable_cycles').value)
        if self.yaw_stable_cycles_required <= 0:
            raise ValueError('yaw_stable_cycles must be greater than zero')
        self.default_timeout_sec = self.get_parameter(
            'default_timeout_sec').value

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

        self.state_condition = threading.Condition()
        self.service_lock = threading.Lock()
        self.current_pose = None
        self.current_yaw = 0.0
        self.active_goal = None
        self.goal_completed = False
        self.yaw_stable_cycle_count = 0
        self.last_tick = time.monotonic()

        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.current_pose_subscriber = self.create_subscription(
            PoseStamped,
            current_pose_topic,
            self.on_current_pose,
            10,
            callback_group=self.callback_group,
        )
        self.move_to_pose_service = self.create_service(
            MoveToPose,
            move_to_pose_service,
            self.handle_move_to_pose,
            callback_group=self.callback_group,
        )

        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.control_loop,
            callback_group=self.callback_group,
        )
        self.add_on_set_parameters_callback(self.on_parameters_changed)

    def on_current_pose(self, msg):
        with self.state_condition:
            self.current_pose = msg
            self.current_yaw = self.yaw_from_quaternion(msg.pose.orientation)
            self.state_condition.notify_all()

    def handle_move_to_pose(self, request, response):
        with self.service_lock:
            with self.state_condition:
                if self.current_pose is None:
                    response.success = False
                    response.message = 'Pose feedback unavailable'
                    return response

                goal = {
                    'x': request.x,
                    'y': request.y,
                    'yaw': self.normalize_angle(request.yaw),
                    'position_tolerance': (
                        request.position_tolerance
                        if request.position_tolerance > 0.0
                        else self.position_tolerance
                    ),
                    'yaw_tolerance': (
                        request.yaw_tolerance
                        if request.yaw_tolerance > 0.0
                        else self.yaw_tolerance
                    ),
                }
                timeout_sec = (
                    request.timeout_sec
                    if request.timeout_sec > 0.0
                    else self.default_timeout_sec
                )

                self.active_goal = goal
                self.goal_completed = False
                self.reset_controllers()
                self.state_condition.notify_all()

            deadline = time.monotonic() + timeout_sec
            while rclpy.ok():
                should_return = False
                success = False
                message = ''
                goal_snapshot = goal

                with self.state_condition:
                    goal_snapshot = self.active_goal if self.active_goal else goal
                    if self.goal_completed:
                        should_return = True
                        success = True
                        message = 'Goal reached'
                    else:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0.0:
                            self.active_goal = None
                            self.goal_completed = False
                            self.reset_controllers()
                            self.state_condition.notify_all()
                            should_return = True
                            success = False
                            message = 'MoveToPose timeout'
                        else:
                            self.state_condition.wait(timeout=remaining)

                if should_return:
                    if not success:
                        self.publish_zero_twist()
                    self.fill_move_response(
                        response,
                        success,
                        message,
                        goal_snapshot,
                    )
                    return response

            with self.state_condition:
                self.active_goal = None
                self.goal_completed = False
                self.reset_controllers()
                self.state_condition.notify_all()

            self.publish_zero_twist()
            self.fill_move_response(
                response,
                False,
                'ROS shutdown while waiting for goal',
                goal,
            )
            return response

    def control_loop(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now

        with self.state_condition:
            current_pose = self.current_pose
            goal = self.active_goal

        if current_pose is None or goal is None:
            return

        _, _, _, body_error_x, body_error_y, yaw_error = (
            self.compute_goal_errors(current_pose, goal)
        )

        if abs(yaw_error) <= goal['yaw_tolerance']:
            self.yaw_stable_cycle_count += 1
        else:
            self.yaw_stable_cycle_count = 0

        if (
            abs(body_error_x) <= goal['position_tolerance'] and
            abs(body_error_y) <= goal['position_tolerance'] and
            self.yaw_stable_cycle_count >= self.yaw_stable_cycles_required
        ):
            self.reset_controllers()
            self.publish_zero_twist()
            with self.state_condition:
                if self.active_goal is not None:
                    self.active_goal = None
                    self.goal_completed = True
                    self.state_condition.notify_all()
            return

        cmd_vel = Twist()
        cmd_vel.linear.x = self.x_pid.update(body_error_x, dt)
        cmd_vel.linear.y = self.y_pid.update(body_error_y, dt)
        cmd_vel.angular.z = self.yaw_pid.update(yaw_error, dt)
        self.cmd_vel_publisher.publish(cmd_vel)

    def fill_move_response(self, response, success, message, goal):
        final_x, final_y, final_yaw, position_error, yaw_error = (
            self.get_goal_status(goal)
        )
        response.success = success
        response.message = message
        response.final_x = final_x
        response.final_y = final_y
        response.final_yaw = final_yaw
        response.position_error = position_error
        response.yaw_error = yaw_error

    def get_goal_status(self, goal):
        with self.state_condition:
            if self.current_pose is None:
                return 0.0, 0.0, 0.0, float('inf'), float('inf')
            current_pose = self.current_pose.pose

        final_x = current_pose.position.x
        final_y = current_pose.position.y
        final_yaw = self.yaw_from_quaternion(current_pose.orientation)
        dx_world = goal['x'] - final_x
        dy_world = goal['y'] - final_y
        position_error = math.hypot(dx_world, dy_world)
        yaw_error = abs(self.normalize_angle(goal['yaw'] - final_yaw))
        return final_x, final_y, final_yaw, position_error, yaw_error

    def compute_goal_errors(self, current_pose_msg, goal):
        current_pose = current_pose_msg.pose
        current_yaw = self.yaw_from_quaternion(current_pose.orientation)
        dx_world = goal['x'] - current_pose.position.x
        dy_world = goal['y'] - current_pose.position.y
        yaw_error = self.normalize_angle(goal['yaw'] - current_yaw)

        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        body_error_x = cos_yaw * dx_world + sin_yaw * dy_world
        body_error_y = -sin_yaw * dx_world + cos_yaw * dy_world
        return (
            dx_world,
            dy_world,
            current_yaw,
            body_error_x,
            body_error_y,
            yaw_error,
        )

    def reset_controllers(self):
        self.x_pid.reset()
        self.y_pid.reset()
        self.yaw_pid.reset()
        self.yaw_stable_cycle_count = 0

    def publish_zero_twist(self):
        self.cmd_vel_publisher.publish(Twist())

    def on_parameters_changed(self, params):
        values = {}
        for param in params:
            values[param.name] = param.value

        position_tolerance = values.get(
            'position_tolerance', self.position_tolerance)
        yaw_tolerance = values.get('yaw_tolerance', self.yaw_tolerance)
        yaw_stable_cycles = values.get(
            'yaw_stable_cycles', self.yaw_stable_cycles_required)
        default_timeout_sec = values.get(
            'default_timeout_sec', self.default_timeout_sec)
        current_period = self.timer.timer_period_ns / 1e9
        current_rate = 1.0 / current_period
        publish_rate = values.get('publish_rate', current_rate)

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
        if (
            not isinstance(yaw_stable_cycles, int) or
            isinstance(yaw_stable_cycles, bool) or
            yaw_stable_cycles <= 0
        ):
            return SetParametersResult(
                successful=False,
                reason='yaw_stable_cycles must be a positive integer',
            )
        if default_timeout_sec <= 0.0:
            return SetParametersResult(
                successful=False,
                reason='default_timeout_sec must be greater than zero',
            )
        if publish_rate <= 0.0:
            return SetParametersResult(
                successful=False,
                reason='publish_rate must be greater than zero',
            )

        self.position_tolerance = position_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.yaw_stable_cycles_required = yaw_stable_cycles
        self.default_timeout_sec = default_timeout_sec

        self.x_pid.configure(
            values.get('x_kp', self.x_pid.kp),
            values.get('x_ki', self.x_pid.ki),
            values.get('x_kd', self.x_pid.kd),
            values.get('x_integral_limit', self.x_pid.integral_limit),
            values.get('x_output_limit', self.x_pid.output_limit),
        )
        self.y_pid.configure(
            values.get('y_kp', self.y_pid.kp),
            values.get('y_ki', self.y_pid.ki),
            values.get('y_kd', self.y_pid.kd),
            values.get('y_integral_limit', self.y_pid.integral_limit),
            values.get('y_output_limit', self.y_pid.output_limit),
        )
        self.yaw_pid.configure(
            values.get('yaw_kp', self.yaw_pid.kp),
            values.get('yaw_ki', self.yaw_pid.ki),
            values.get('yaw_kd', self.yaw_pid.kd),
            values.get('yaw_integral_limit', self.yaw_pid.integral_limit),
            values.get('yaw_output_limit', self.yaw_pid.output_limit),
        )

        new_period = 1.0 / publish_rate
        if abs(new_period - current_period) > 1e-9:
            self.timer.cancel()
            self.timer = self.create_timer(
                new_period,
                self.control_loop,
                callback_group=self.callback_group,
            )

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
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero_twist()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
