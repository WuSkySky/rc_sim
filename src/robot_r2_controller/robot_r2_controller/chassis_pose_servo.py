import math
import threading
import time

from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.srv import MoveRelative, MoveToPose


SERIAL_SOURCE = 'serial'
ODIN_SOURCE = 'odin'
POSE_SOURCES = (SERIAL_SOURCE, ODIN_SOURCE)

SERIAL_POSE_TOPIC = '/r2/pose_feedback'
ODIN_POSE_TOPIC = '/r2/pose_feedback_odin'
CMD_VEL_TOPIC = '/r2/cmd_vel'
MOVE_TO_POSE_SERVICE = '/r2/move_to_pose'
MOVE_RELATIVE_SERVICE = '/r2/move_relative'


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_pose_goal(current_pose, forward, left, yaw_delta):
    current_x, current_y, current_yaw = current_pose
    cos_yaw = math.cos(current_yaw)
    sin_yaw = math.sin(current_yaw)
    return (
        current_x + cos_yaw * forward - sin_yaw * left,
        current_y + sin_yaw * forward + cos_yaw * left,
        normalize_angle(current_yaw + yaw_delta),
    )


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

    def update(
        self,
        error,
        dt,
        proportional_gain_multiplier=1.0,
        output_limit=None,
    ):
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
            self.kp * proportional_gain_multiplier * error +
            self.ki * self.integral +
            self.kd * derivative
        )
        self.last_error = error
        self.has_last_error = True

        effective_output_limit = (
            self.output_limit
            if output_limit is None
            else abs(float(output_limit))
        )
        if effective_output_limit > 0.0:
            output = max(
                -effective_output_limit,
                min(output, effective_output_limit),
            )
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
        self.service_callback_group = MutuallyExclusiveCallbackGroup()

        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('position_tolerance', 0.02)
        self.declare_parameter('yaw_tolerance', 0.03)
        self.declare_parameter('yaw_stable_cycles', 10)
        self.declare_parameter('completion_wait_sec', 0.3)
        self.declare_parameter('initial_pose_timeout_sec', 2.0)
        self.declare_parameter('default_timeout_sec', 20.0)

        self.declare_parameter('x_kp', 2.5)
        self.declare_parameter('x_ki', 0.0)
        self.declare_parameter('x_kd', 0.2)
        self.declare_parameter('x_integral_limit', 0.5)
        self.declare_parameter('x_output_limit', 0.675)

        self.declare_parameter('y_kp', 2.5)
        self.declare_parameter('y_ki', 0.0)
        self.declare_parameter('y_kd', 0.2)
        self.declare_parameter('y_integral_limit', 0.5)
        self.declare_parameter('y_output_limit', 0.675)

        self.declare_parameter('yaw_kp', 3.0)
        self.declare_parameter('yaw_ki', 0.0)
        self.declare_parameter('yaw_kd', 0.2)
        self.declare_parameter('yaw_integral_limit', 0.5)
        self.declare_parameter('yaw_output_limit', 2.0)
        self.declare_parameter('yaw_small_error_gain_multiplier', 2.0)

        publish_rate = self._positive_parameter('publish_rate')
        self.position_tolerance = self._non_negative_parameter(
            'position_tolerance')
        self.yaw_tolerance = self._non_negative_parameter('yaw_tolerance')
        self.yaw_stable_cycles_required = int(
            self.get_parameter('yaw_stable_cycles').value)
        if self.yaw_stable_cycles_required <= 0:
            raise ValueError('yaw_stable_cycles must be greater than zero')
        self.completion_wait_sec = self._non_negative_parameter(
            'completion_wait_sec')
        self.initial_pose_timeout_sec = self._positive_parameter(
            'initial_pose_timeout_sec')
        self.default_timeout_sec = self._positive_parameter(
            'default_timeout_sec')

        self.x_pid = self._make_pid('x')
        self.y_pid = self._make_pid('y')
        self.yaw_pid = self._make_pid('yaw')
        self.yaw_small_error_gain_multiplier = self._finite_parameter(
            'yaw_small_error_gain_multiplier')
        if self.yaw_small_error_gain_multiplier < 1.0:
            raise ValueError(
                'yaw_small_error_gain_multiplier must be at least 1.0')

        self.state_condition = threading.Condition()
        self.service_lock = threading.Lock()
        self.current_poses = {source: None for source in POSE_SOURCES}
        self.pose_sequences = {source: 0 for source in POSE_SOURCES}
        self.active_goal = None
        self.goal_completed = False
        self.yaw_stable_cycle_count = 0
        self.last_tick = time.monotonic()

        self.cmd_vel_publisher = self.create_publisher(
            Twist, CMD_VEL_TOPIC, 10)
        self.serial_pose_subscriber = self.create_subscription(
            PoseStamped,
            SERIAL_POSE_TOPIC,
            lambda message: self.on_current_pose(SERIAL_SOURCE, message),
            10,
            callback_group=self.callback_group,
        )
        self.odin_pose_subscriber = self.create_subscription(
            PoseStamped,
            ODIN_POSE_TOPIC,
            lambda message: self.on_current_pose(ODIN_SOURCE, message),
            10,
            callback_group=self.callback_group,
        )
        self.move_to_pose_service = self.create_service(
            MoveToPose,
            MOVE_TO_POSE_SERVICE,
            self.handle_move_to_pose,
            callback_group=self.service_callback_group,
        )
        self.move_relative_service = self.create_service(
            MoveRelative,
            MOVE_RELATIVE_SERVICE,
            self.handle_move_relative,
            callback_group=self.service_callback_group,
        )
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.control_loop,
            callback_group=self.callback_group,
        )
        self.add_on_set_parameters_callback(self.on_parameters_changed)

    def _finite_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
        return value

    def _positive_parameter(self, name):
        value = self._finite_parameter(name)
        if value <= 0.0:
            raise ValueError(f'{name} must be greater than zero')
        return value

    def _non_negative_parameter(self, name):
        value = self._finite_parameter(name)
        if value < 0.0:
            raise ValueError(f'{name} must be non-negative')
        return value

    def _make_pid(self, prefix):
        return PidAxis(
            self._finite_parameter(f'{prefix}_kp'),
            self._finite_parameter(f'{prefix}_ki'),
            self._finite_parameter(f'{prefix}_kd'),
            self._finite_parameter(f'{prefix}_integral_limit'),
            self._finite_parameter(f'{prefix}_output_limit'),
        )

    @staticmethod
    def validate_pose_source(pose_source):
        if pose_source not in POSE_SOURCES:
            raise ValueError(
                "pose_source must be 'serial' or 'odin', "
                f'got {pose_source!r}')
        return pose_source

    def on_current_pose(self, pose_source, message):
        pose = message.pose
        values = (
            float(pose.position.x),
            float(pose.position.y),
            self.yaw_from_quaternion(pose.orientation),
        )
        if not all(math.isfinite(value) for value in values):
            return
        with self.state_condition:
            self.current_poses[pose_source] = message
            self.pose_sequences[pose_source] += 1
            self.state_condition.notify_all()

    def wait_for_fresh_pose(self, pose_source):
        deadline = time.monotonic() + self.initial_pose_timeout_sec
        with self.state_condition:
            initial_sequence = self.pose_sequences[pose_source]
            while self.pose_sequences[pose_source] <= initial_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RuntimeError(
                        f"Pose feedback unavailable for source "
                        f"'{pose_source}'")
                self.state_condition.wait(timeout=remaining)
            return self.current_poses[pose_source]

    @staticmethod
    def _validate_finite_request(values):
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError('motion request values must be finite')

    def _request_settings(self, request):
        linear_speed_limit_value = float(request.linear_speed_limit)
        if not math.isfinite(linear_speed_limit_value):
            raise ValueError('linear_speed_limit must be finite')
        if linear_speed_limit_value < 0.0:
            raise ValueError('linear_speed_limit must be non-negative')
        self._validate_finite_request((
            request.position_tolerance,
            request.yaw_tolerance,
            request.timeout_sec,
        ))
        linear_speed_limit = (
            linear_speed_limit_value
            if linear_speed_limit_value > 0.0
            else None
        )
        position_tolerance = (
            request.position_tolerance
            if request.position_tolerance > 0.0
            else self.position_tolerance
        )
        yaw_tolerance = (
            request.yaw_tolerance
            if request.yaw_tolerance > 0.0
            else self.yaw_tolerance
        )
        timeout_sec = (
            request.timeout_sec
            if request.timeout_sec > 0.0
            else self.default_timeout_sec
        )
        return (
            linear_speed_limit,
            position_tolerance,
            yaw_tolerance,
            timeout_sec,
        )

    def _absolute_goal(self, request, pose_source):
        self._validate_finite_request((request.x, request.y, request.yaw))
        (
            linear_speed_limit,
            position_tolerance,
            yaw_tolerance,
            timeout_sec,
        ) = self._request_settings(request)
        return {
            'pose_source': pose_source,
            'x': float(request.x),
            'y': float(request.y),
            'yaw': normalize_angle(float(request.yaw)),
            'linear_speed_limit': linear_speed_limit,
            'position_tolerance': position_tolerance,
            'yaw_tolerance': yaw_tolerance,
        }, timeout_sec

    def _relative_goal(self, request, pose_source, current_pose_message):
        self._validate_finite_request(
            (request.forward, request.left, request.yaw_delta))
        (
            linear_speed_limit,
            position_tolerance,
            yaw_tolerance,
            timeout_sec,
        ) = self._request_settings(request)
        pose = current_pose_message.pose
        current_pose = (
            float(pose.position.x),
            float(pose.position.y),
            self.yaw_from_quaternion(pose.orientation),
        )
        target_x, target_y, target_yaw = relative_pose_goal(
            current_pose,
            float(request.forward),
            float(request.left),
            float(request.yaw_delta),
        )
        return {
            'pose_source': pose_source,
            'x': target_x,
            'y': target_y,
            'yaw': target_yaw,
            'linear_speed_limit': linear_speed_limit,
            'position_tolerance': position_tolerance,
            'yaw_tolerance': yaw_tolerance,
        }, timeout_sec

    def handle_move_to_pose(self, request, response):
        return self._handle_motion(
            request,
            response,
            'MoveToPose',
            lambda source, _pose: self._absolute_goal(request, source),
        )

    def handle_move_relative(self, request, response):
        return self._handle_motion(
            request,
            response,
            'MoveRelative',
            lambda source, pose: self._relative_goal(
                request, source, pose),
        )

    def _handle_motion(self, request, response, description, make_goal):
        with self.service_lock:
            try:
                pose_source = self.validate_pose_source(request.pose_source)
                current_pose = self.wait_for_fresh_pose(pose_source)
                goal, timeout_sec = make_goal(pose_source, current_pose)
            except (RuntimeError, TypeError, ValueError) as error:
                self.publish_zero_twist()
                self.fill_unavailable_response(response, str(error))
                return response
            return self.execute_goal(
                goal, timeout_sec, response, description)

    def execute_goal(self, goal, timeout_sec, response, description):
        with self.state_condition:
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
                goal_snapshot = self.active_goal or goal
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
                        message = f'{description} timeout'
                    else:
                        self.state_condition.wait(timeout=remaining)

            if should_return:
                if success:
                    self.wait_after_completion()
                else:
                    self.publish_zero_twist()
                self.fill_move_response(
                    response, success, message, goal_snapshot)
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

    def wait_after_completion(self):
        deadline = time.monotonic() + self.completion_wait_sec
        while rclpy.ok():
            self.publish_zero_twist()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            time.sleep(min(remaining, 0.02))

    def control_loop(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now
        command = None
        completed = False

        with self.state_condition:
            goal = self.active_goal
            if goal is None:
                return
            current_pose = self.current_poses[goal['pose_source']]
            if current_pose is None:
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
                self.yaw_stable_cycle_count >=
                    self.yaw_stable_cycles_required
            ):
                self.reset_controllers()
                self.active_goal = None
                self.goal_completed = True
                self.state_condition.notify_all()
                completed = True
            else:
                command = Twist()
                linear_speed_limit = goal.get('linear_speed_limit')
                command.linear.x = self.x_pid.update(
                    body_error_x,
                    dt,
                    output_limit=linear_speed_limit,
                )
                command.linear.y = self.y_pid.update(
                    body_error_y,
                    dt,
                    output_limit=linear_speed_limit,
                )
                command.angular.z = self.yaw_pid.update(
                    yaw_error,
                    dt,
                    proportional_gain_multiplier=(
                        self.get_yaw_gain_multiplier(yaw_error)),
                )

        if completed:
            self.publish_zero_twist()
        elif command is not None:
            self.cmd_vel_publisher.publish(command)

    def get_yaw_gain_multiplier(self, yaw_error):
        normalized_error = min(abs(yaw_error) / math.pi, 1.0)
        return 1.0 + (
            self.yaw_small_error_gain_multiplier - 1.0
        ) * (1.0 - normalized_error)

    @staticmethod
    def fill_unavailable_response(response, message):
        response.success = False
        response.message = message
        response.final_x = 0.0
        response.final_y = 0.0
        response.final_yaw = 0.0
        response.position_error = float('inf')
        response.yaw_error = float('inf')

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
            current_pose_message = self.current_poses[goal['pose_source']]
            if current_pose_message is None:
                return 0.0, 0.0, 0.0, float('inf'), float('inf')
            current_pose = current_pose_message.pose

        final_x = current_pose.position.x
        final_y = current_pose.position.y
        final_yaw = self.yaw_from_quaternion(current_pose.orientation)
        position_error = math.hypot(
            goal['x'] - final_x, goal['y'] - final_y)
        yaw_error = abs(normalize_angle(goal['yaw'] - final_yaw))
        return final_x, final_y, final_yaw, position_error, yaw_error

    def compute_goal_errors(self, current_pose_message, goal):
        current_pose = current_pose_message.pose
        current_yaw = self.yaw_from_quaternion(current_pose.orientation)
        dx_world = goal['x'] - current_pose.position.x
        dy_world = goal['y'] - current_pose.position.y
        yaw_error = normalize_angle(goal['yaw'] - current_yaw)

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

    @staticmethod
    def _finite_value(name, value):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'{name} must be a number')
        result = float(value)
        if not math.isfinite(result):
            raise ValueError(f'{name} must be finite')
        return result

    def on_parameters_changed(self, params):
        values = {param.name: param.value for param in params}
        current_period = self.timer.timer_period_ns / 1e9
        current_rate = 1.0 / current_period

        try:
            position_tolerance = self._finite_value(
                'position_tolerance', values.get(
                    'position_tolerance', self.position_tolerance))
            yaw_tolerance = self._finite_value(
                'yaw_tolerance', values.get(
                    'yaw_tolerance', self.yaw_tolerance))
            completion_wait_sec = self._finite_value(
                'completion_wait_sec', values.get(
                    'completion_wait_sec', self.completion_wait_sec))
            initial_pose_timeout_sec = self._finite_value(
                'initial_pose_timeout_sec', values.get(
                    'initial_pose_timeout_sec',
                    self.initial_pose_timeout_sec))
            default_timeout_sec = self._finite_value(
                'default_timeout_sec', values.get(
                    'default_timeout_sec', self.default_timeout_sec))
            publish_rate = self._finite_value(
                'publish_rate', values.get('publish_rate', current_rate))
            yaw_gain_multiplier = self._finite_value(
                'yaw_small_error_gain_multiplier', values.get(
                    'yaw_small_error_gain_multiplier',
                    self.yaw_small_error_gain_multiplier))
            yaw_stable_cycles = values.get(
                'yaw_stable_cycles', self.yaw_stable_cycles_required)

            pid_values = {}
            for prefix, pid in (
                ('x', self.x_pid),
                ('y', self.y_pid),
                ('yaw', self.yaw_pid),
            ):
                for suffix, current in (
                    ('kp', pid.kp),
                    ('ki', pid.ki),
                    ('kd', pid.kd),
                    ('integral_limit', pid.integral_limit),
                    ('output_limit', pid.output_limit),
                ):
                    name = f'{prefix}_{suffix}'
                    pid_values[name] = self._finite_value(
                        name, values.get(name, current))
        except ValueError as error:
            return SetParametersResult(successful=False, reason=str(error))

        if position_tolerance < 0.0 or yaw_tolerance < 0.0:
            return SetParametersResult(
                successful=False,
                reason='position and yaw tolerances must be non-negative',
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
        if completion_wait_sec < 0.0:
            return SetParametersResult(
                successful=False,
                reason='completion_wait_sec must be non-negative',
            )
        if initial_pose_timeout_sec <= 0.0:
            return SetParametersResult(
                successful=False,
                reason='initial_pose_timeout_sec must be greater than zero',
            )
        if default_timeout_sec <= 0.0 or publish_rate <= 0.0:
            return SetParametersResult(
                successful=False,
                reason='timeouts and publish_rate must be greater than zero',
            )
        if yaw_gain_multiplier < 1.0:
            return SetParametersResult(
                successful=False,
                reason=(
                    'yaw_small_error_gain_multiplier must be at least 1.0'),
            )

        with self.state_condition:
            self.position_tolerance = position_tolerance
            self.yaw_tolerance = yaw_tolerance
            self.yaw_stable_cycles_required = yaw_stable_cycles
            self.completion_wait_sec = completion_wait_sec
            self.initial_pose_timeout_sec = initial_pose_timeout_sec
            self.default_timeout_sec = default_timeout_sec
            self.yaw_small_error_gain_multiplier = yaw_gain_multiplier
            for prefix, pid in (
                ('x', self.x_pid),
                ('y', self.y_pid),
                ('yaw', self.yaw_pid),
            ):
                pid.configure(
                    pid_values[f'{prefix}_kp'],
                    pid_values[f'{prefix}_ki'],
                    pid_values[f'{prefix}_kd'],
                    pid_values[f'{prefix}_integral_limit'],
                    pid_values[f'{prefix}_output_limit'],
                )

        new_period = 1.0 / publish_rate
        if abs(new_period - current_period) > 1e-9:
            old_timer = self.timer
            old_timer.cancel()
            self.timer = self.create_timer(
                new_period,
                self.control_loop,
                callback_group=self.callback_group,
            )
            self.destroy_timer(old_timer)
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
        return normalize_angle(angle)


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
