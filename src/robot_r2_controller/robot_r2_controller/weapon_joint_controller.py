import math
import threading
import time
from dataclasses import dataclass

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from robot_r2_common import AbortMonitor, PositionAbortMixin
from robot_r2_interfaces.srv import SetJointPosition
from std_msgs.msg import Float64


@dataclass(frozen=True)
class WeaponJointProfile:
    node_name: str
    command_topic: str
    feedback_topic: str
    service_name: str
    label: str
    default_min_position: float
    default_max_position: float
    default_tolerance: float


@dataclass(frozen=True)
class WeaponJointConfig:
    min_position: float
    max_position: float
    default_tolerance: float
    default_timeout_sec: float


ROTATE_PROFILE = WeaponJointProfile(
    node_name='weapon_rotate',
    command_topic='/r2/weapon/rotate_cmd',
    feedback_topic='/r2/weapon/rotate_feedback',
    service_name='/r2/weapon/set_rotate',
    label='Weapon rotate',
    default_min_position=0.0,
    default_max_position=math.radians(200.0),
    default_tolerance=0.01,
)

GRIP_PROFILE = WeaponJointProfile(
    node_name='weapon_grip',
    command_topic='/r2/weapon/grip_cmd',
    feedback_topic='/r2/weapon/grip_feedback',
    service_name='/r2/weapon/set_grip',
    label='Weapon grip',
    default_min_position=0.0,
    default_max_position=0.03,
    default_tolerance=0.001,
)


class WeaponJointServiceController(PositionAbortMixin, Node):
    def __init__(self, profile):
        super().__init__(profile.node_name)
        self.profile = profile
        self.callback_group = ReentrantCallbackGroup()
        self.service_lock = threading.Lock()
        self.state_condition = threading.Condition()
        self.config_lock = threading.RLock()
        self.current_position = None

        self.declare_parameter(
            'min_position', profile.default_min_position)
        self.declare_parameter(
            'max_position', profile.default_max_position)
        self.declare_parameter(
            'default_tolerance', profile.default_tolerance)
        self.declare_parameter('default_timeout_sec', 10.0)
        self._config = self._read_config()

        self.command_publisher = self.create_publisher(
            Float64, profile.command_topic, 10)
        self.feedback_subscription = self.create_subscription(
            Float64,
            profile.feedback_topic,
            self.on_feedback,
            10,
            callback_group=self.callback_group,
        )
        self.service = self.create_service(
            SetJointPosition,
            profile.service_name,
            self.handle_set_position,
            callback_group=self.callback_group,
        )
        self.parameter_callback = self.add_on_set_parameters_callback(
            self.on_parameters_changed)
        self.abort_monitor = AbortMonitor(
            self,
            callback_group=self.callback_group,
            on_abort=self.hold_current_position_on_abort,
        )

    @staticmethod
    def validate_config(config):
        values = (
            config.min_position,
            config.max_position,
            config.default_tolerance,
            config.default_timeout_sec,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('weapon joint configuration must be finite')
        if config.min_position >= config.max_position:
            raise ValueError('min_position must be less than max_position')
        if config.default_tolerance <= 0.0:
            raise ValueError('default_tolerance must be positive')
        if config.default_timeout_sec <= 0.0:
            raise ValueError('default_timeout_sec must be positive')

    @classmethod
    def config_from_values(cls, values):
        config = WeaponJointConfig(**values)
        cls.validate_config(config)
        return config

    def _read_config(self):
        values = {
            name: float(self.get_parameter(name).value)
            for name in WeaponJointConfig.__dataclass_fields__
        }
        return self.config_from_values(values)

    def config_snapshot(self):
        with self.config_lock:
            return self._config

    def on_parameters_changed(self, parameters):
        with self.config_lock:
            values = {
                name: getattr(self._config, name)
                for name in WeaponJointConfig.__dataclass_fields__
            }
            for parameter in parameters:
                if parameter.name not in values:
                    continue
                if parameter.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{parameter.name} must be a double',
                    )
                values[parameter.name] = float(parameter.value)
            try:
                candidate = self.config_from_values(values)
            except ValueError as exc:
                return SetParametersResult(
                    successful=False, reason=str(exc))
            self._config = candidate
        return SetParametersResult(successful=True)

    def on_feedback(self, message):
        position = float(message.data)
        if not math.isfinite(position):
            return
        with self.state_condition:
            self.current_position = position
            self.state_condition.notify_all()

    def handle_set_position(self, request, response):
        abort_scope = self.abort_scope()
        with self.service_lock, abort_scope:
            if self.abort_requested():
                return self.fill_aborted_position_response(request, response)
            config = self.config_snapshot()
            try:
                tolerance, timeout_sec = self.validate_request(
                    request, config)
            except ValueError as exc:
                return self.reject(response, str(exc))

            command = Float64()
            command.data = float(request.position)
            self.command_publisher.publish(command)
            if self.abort_requested():
                self.hold_current_position_on_abort()
                return self.fill_aborted_position_response(request, response)

            deadline = time.monotonic() + timeout_sec
            while rclpy.ok():
                if self.abort_requested():
                    return self.fill_aborted_position_response(
                        request, response)
                with self.state_condition:
                    if self.current_position is not None:
                        error = request.position - self.current_position
                        if abs(error) <= tolerance:
                            response.success = True
                            response.message = (
                                f'{self.profile.label} target reached')
                            response.final_position = self.current_position
                            response.position_error = error
                            return response
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        break
                    self.state_condition.wait(timeout=min(remaining, 0.05))

            with self.state_condition:
                final_position = (
                    self.current_position
                    if self.current_position is not None else 0.0)
            response.success = False
            response.message = f'{self.profile.label} timeout'
            response.final_position = final_position
            response.position_error = request.position - final_position
            return response

    def validate_request(self, request, config):
        position = float(request.position)
        requested_tolerance = float(request.tolerance)
        requested_timeout = float(request.timeout_sec)
        if not math.isfinite(position):
            raise ValueError(f'{self.profile.label} position must be finite')
        if not config.min_position <= position <= config.max_position:
            raise ValueError(
                f'{self.profile.label} position must be between '
                f'{config.min_position} and {config.max_position}')
        if not math.isfinite(requested_tolerance) or requested_tolerance < 0.0:
            raise ValueError('tolerance must be finite and non-negative')
        if not math.isfinite(requested_timeout) or requested_timeout < 0.0:
            raise ValueError('timeout_sec must be finite and non-negative')
        tolerance = (
            requested_tolerance
            if requested_tolerance > 0.0 else config.default_tolerance)
        timeout_sec = (
            requested_timeout
            if requested_timeout > 0.0 else config.default_timeout_sec)
        return tolerance, timeout_sec

    def reject(self, response, message):
        with self.state_condition:
            final_position = (
                self.current_position
                if self.current_position is not None else 0.0)
        response.success = False
        response.message = message
        response.final_position = final_position
        response.position_error = 0.0
        return response


def run(profile, args=None):
    rclpy.init(args=args)
    node = WeaponJointServiceController(profile)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_rotate(args=None):
    run(ROTATE_PROFILE, args=args)


def main_grip(args=None):
    run(GRIP_PROFILE, args=args)
