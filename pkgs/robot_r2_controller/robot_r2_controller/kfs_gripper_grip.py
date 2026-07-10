import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.srv import SetGripperGrip
from std_msgs.msg import Float64


class GripperGripServiceController(Node):
    def __init__(self):
        super().__init__('kfs_gripper_grip')
        self.callback_group = ReentrantCallbackGroup()
        self.service_lock = threading.Lock()
        self.state_condition = threading.Condition()

        self.declare_parameter('command_topic', '/r2/gripper/grip_cmd')
        self.declare_parameter('feedback_topic', '/r2/gripper/grip_feedback')
        self.declare_parameter(
            'gap_feedback_topic', '/r2/gripper/grip_gap_feedback')
        self.declare_parameter('service_name', '/r2/gripper/set_grip')
        self.declare_parameter('default_tolerance', 0.005)
        self.declare_parameter('default_timeout_sec', 10.0)
        self.declare_parameter('closed_gap_threshold', 0.38)

        command_topic = self.get_parameter('command_topic').value
        feedback_topic = self.get_parameter('feedback_topic').value
        gap_feedback_topic = self.get_parameter('gap_feedback_topic').value
        service_name = self.get_parameter('service_name').value
        self.default_tolerance = self.get_parameter('default_tolerance').value
        self.default_timeout_sec = self.get_parameter(
            'default_timeout_sec').value
        self.closed_gap_threshold = self.get_parameter(
            'closed_gap_threshold').value

        self.current_position = None
        self.current_gap = None

        self.command_publisher = self.create_publisher(
            Float64, command_topic, 10)
        self.feedback_subscription = self.create_subscription(
            Float64,
            feedback_topic,
            self.on_feedback,
            10,
            callback_group=self.callback_group,
        )
        self.gap_feedback_subscription = self.create_subscription(
            Float64,
            gap_feedback_topic,
            self.on_gap_feedback,
            10,
            callback_group=self.callback_group,
        )
        self.service = self.create_service(
            SetGripperGrip,
            service_name,
            self.handle_set_grip,
            callback_group=self.callback_group,
        )

    def on_feedback(self, msg):
        with self.state_condition:
            self.current_position = msg.data
            self.state_condition.notify_all()

    def on_gap_feedback(self, msg):
        with self.state_condition:
            self.current_gap = msg.data
            self.state_condition.notify_all()

    def handle_set_grip(self, request, response):
        with self.service_lock:
            tolerance = (
                request.tolerance
                if request.tolerance > 0.0
                else self.default_tolerance
            )
            timeout_sec = (
                request.timeout_sec
                if request.timeout_sec > 0.0
                else self.default_timeout_sec
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
                        is_closing = request.position > 0.0
                        close_reached = (
                            is_closing and
                            self.current_gap is not None and
                            self.current_gap < self.closed_gap_threshold
                        )
                        reached = (
                            close_reached
                            if is_closing
                            else abs(error) <= tolerance
                        )
                        if reached:
                            response.success = True
                            response.message = 'Gripper grip target reached'
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
                response.message = 'SetGripperGrip timeout'
                response.final_position = final_pos
                response.position_error = request.position - final_pos
                return response


def main():
    rclpy.init()
    node = GripperGripServiceController()
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
