import math
import threading

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from robot_r2_interfaces.srv import SetJointPosition
from std_msgs.msg import Float64


class GripperRotateServiceController(Node):
    def __init__(self):
        super().__init__('kfs_gripper_rotate')
        self.callback_group = ReentrantCallbackGroup()
        self.service_lock = threading.Lock()
        self.state_condition = threading.Condition()
        self.config_lock = threading.RLock()

        self.declare_parameter('command_topic', '/r2/gripper/rotate_cmd')
        self.declare_parameter('feedback_topic', '/r2/gripper/rotate_feedback')
        self.declare_parameter('service_name', '/r2/gripper/set_rotate')
        self.declare_parameter('min_position', -math.radians(15.0))
        self.declare_parameter('max_position', math.radians(135.0))
        self.declare_parameter('default_tolerance', 0.01)
        self.declare_parameter('default_timeout_sec', 10.0)

        command_topic = self.get_parameter('command_topic').value
        feedback_topic = self.get_parameter('feedback_topic').value
        service_name = self.get_parameter('service_name').value
        self.min_position = float(self.get_parameter('min_position').value)
        self.max_position = float(self.get_parameter('max_position').value)
        self.default_tolerance = float(
            self.get_parameter('default_tolerance').value)
        self.default_timeout_sec = float(
            self.get_parameter('default_timeout_sec').value)
        self._validate_config(
            self.min_position,
            self.max_position,
            self.default_tolerance,
            self.default_timeout_sec,
        )

        self.current_position = None

        self.command_publisher = self.create_publisher(
            Float64, command_topic, 10)
        self.feedback_subscription = self.create_subscription(
            Float64,
            feedback_topic,
            self.on_feedback,
            10,
            callback_group=self.callback_group,
        )
        self.service = self.create_service(
            SetJointPosition,
            service_name,
            self.handle_set_rotate,
            callback_group=self.callback_group,
        )
        self.parameter_callback = self.add_on_set_parameters_callback(
            self.on_parameters_changed)

    @staticmethod
    def _validate_config(
            min_position, max_position,
            default_tolerance, default_timeout_sec):
        values = (
            min_position,
            max_position,
            default_tolerance,
            default_timeout_sec,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError(
                'rotation limits, tolerance and timeout must be finite')
        if min_position >= max_position:
            raise ValueError('min_position must be less than max_position')
        if default_tolerance <= 0.0:
            raise ValueError('default_tolerance must be positive')
        if default_timeout_sec <= 0.0:
            raise ValueError('default_timeout_sec must be positive')

    def on_parameters_changed(self, parameters):
        configurable = {
            'min_position',
            'max_position',
            'default_tolerance',
            'default_timeout_sec',
        }
        with self.config_lock:
            updated = {
                'min_position': self.min_position,
                'max_position': self.max_position,
                'default_tolerance': self.default_tolerance,
                'default_timeout_sec': self.default_timeout_sec,
            }
            for parameter in parameters:
                if parameter.name not in configurable:
                    continue
                if parameter.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{parameter.name} must be a double',
                    )
                updated[parameter.name] = float(parameter.value)

            try:
                self._validate_config(**updated)
            except ValueError as exc:
                return SetParametersResult(
                    successful=False,
                    reason=str(exc),
                )

            self.min_position = updated['min_position']
            self.max_position = updated['max_position']
            self.default_tolerance = updated['default_tolerance']
            self.default_timeout_sec = updated['default_timeout_sec']

        return SetParametersResult(successful=True)

    def on_feedback(self, msg):
        with self.state_condition:
            self.current_position = msg.data
            self.state_condition.notify_all()

    def handle_set_rotate(self, request, response):
        with self.service_lock:
            with self.config_lock:
                min_position = self.min_position
                max_position = self.max_position
                default_tolerance = self.default_tolerance
                default_timeout_sec = self.default_timeout_sec

            if not math.isfinite(request.position):
                response.success = False
                response.message = 'Gripper root rotate position must be finite'
                return response
            if not min_position <= request.position <= max_position:
                response.success = False
                response.message = (
                    'Gripper root rotate position must be between '
                    f'{min_position} and {max_position}'
                )
                return response

            tolerance = (
                request.tolerance
                if request.tolerance > 0.0
                else default_tolerance
            )
            timeout_sec = (
                request.timeout_sec
                if request.timeout_sec > 0.0
                else default_timeout_sec
            )

            cmd = Float64()
            cmd.data = request.position
            self.command_publisher.publish(cmd)

            deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
            while rclpy.ok():
                should_wait = False
                with self.state_condition:
                    if self.current_position is None:
                        should_wait = True
                    else:
                        error = request.position - self.current_position
                        if abs(error) <= tolerance:
                            response.success = True
                            response.message = (
                                'Gripper rotate target reached')
                            response.final_position = self.current_position
                            response.position_error = error
                            return response
                        should_wait = True

                    remaining = deadline - (
                        self.get_clock().now().nanoseconds / 1e9)
                    if remaining <= 0.0:
                        break
                    if should_wait:
                        self.state_condition.wait(timeout=remaining)

            with self.state_condition:
                final_pos = (
                    self.current_position
                    if self.current_position is not None else 0.0)
                response.success = False
                response.message = 'Gripper root rotate timeout'
                response.final_position = final_pos
                response.position_error = request.position - final_pos
                return response


def main():
    rclpy.init()
    node = GripperRotateServiceController()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
