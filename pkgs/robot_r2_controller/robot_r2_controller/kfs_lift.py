import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.srv import SetKfsLift
from std_msgs.msg import Float64


class KfsLiftServiceController(Node):
    def __init__(self):
        super().__init__('kfs_lift')
        self.callback_group = ReentrantCallbackGroup()
        self.service_lock = threading.Lock()
        self.state_condition = threading.Condition()

        self.declare_parameter('command_topic', '/r2/kfs_lift/cmd')
        self.declare_parameter('feedback_topic', '/r2/kfs_lift/feedback')
        self.declare_parameter('service_name', '/r2/kfs_lift')
        self.declare_parameter('default_tolerance', 0.005)
        self.declare_parameter('default_timeout_sec', 10.0)

        command_topic = self.get_parameter('command_topic').value
        feedback_topic = self.get_parameter('feedback_topic').value
        service_name = self.get_parameter('service_name').value
        self.default_tolerance = float(
            self.get_parameter('default_tolerance').value)
        self.default_timeout_sec = float(
            self.get_parameter('default_timeout_sec').value)

        if self.default_tolerance <= 0.0:
            raise ValueError('default_tolerance must be positive')
        if self.default_timeout_sec <= 0.0:
            raise ValueError('default_timeout_sec must be positive')

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
            SetKfsLift,
            service_name,
            self.handle_set_lift,
            callback_group=self.callback_group,
        )

    def on_feedback(self, msg):
        with self.state_condition:
            self.current_position = msg.data
            self.state_condition.notify_all()

    def handle_set_lift(self, request, response):
        with self.service_lock:
            if not math.isfinite(request.position):
                response.success = False
                response.message = 'KFS lift position must be finite'
                response.final_position = self._last_position()
                response.position_error = 0.0
                return response

            tolerance = (
                request.tolerance
                if math.isfinite(request.tolerance)
                and request.tolerance > 0.0
                else self.default_tolerance
            )
            timeout_sec = (
                request.timeout_sec
                if math.isfinite(request.timeout_sec)
                and request.timeout_sec > 0.0
                else self.default_timeout_sec
            )

            command = Float64()
            command.data = request.position
            self.command_publisher.publish(command)

            deadline = time.monotonic() + timeout_sec
            while rclpy.ok():
                with self.state_condition:
                    if self.current_position is not None:
                        error = request.position - self.current_position
                        if abs(error) <= tolerance:
                            response.success = True
                            response.message = 'KFS lift target reached'
                            response.final_position = self.current_position
                            response.position_error = error
                            return response

                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        break
                    self.state_condition.wait(timeout=remaining)

            final_position = self._last_position()
            response.success = False
            response.message = 'SetKfsLift timeout'
            response.final_position = final_position
            response.position_error = request.position - final_position
            return response

    def _last_position(self):
        with self.state_condition:
            return (
                self.current_position
                if self.current_position is not None else 0.0
            )


def main():
    rclpy.init()
    node = KfsLiftServiceController()
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
