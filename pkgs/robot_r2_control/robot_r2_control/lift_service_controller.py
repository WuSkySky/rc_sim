import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.msg import LiftCommand, LiftFeedback
from robot_r2_interfaces.srv import SetLift


class LiftServiceController(Node):
    def __init__(self):
        super().__init__('robot_r2_lift_service_controller')
        self.callback_group = ReentrantCallbackGroup()
        self.service_lock = threading.Lock()
        self.state_condition = threading.Condition()

        self.declare_parameter('command_topic', '/r2/lift/cmd_lift')
        self.declare_parameter(
            'position_feedback_topic', '/r2/lift/position_feedback')
        self.declare_parameter('set_lift_service', '/r2/lift/set')
        self.declare_parameter('default_tolerance', 0.008)
        self.declare_parameter('default_timeout_sec', 10.0)

        command_topic = self.get_parameter('command_topic').value
        position_feedback_topic = self.get_parameter(
            'position_feedback_topic').value
        service_name = self.get_parameter('set_lift_service').value
        self.default_tolerance = self.get_parameter('default_tolerance').value
        self.default_timeout_sec = self.get_parameter(
            'default_timeout_sec').value

        self.current_front_lift = None
        self.current_rear_lift = None
        self.command_publisher = self.create_publisher(
            LiftCommand,
            command_topic,
            10,
        )
        self.feedback_subscription = self.create_subscription(
            LiftFeedback,
            position_feedback_topic,
            self.on_feedback,
            10,
            callback_group=self.callback_group,
        )
        self.set_lift_service = self.create_service(
            SetLift,
            service_name,
            self.handle_set_lift,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            f'Lift service controller active: service={service_name}, '
            f'command={command_topic}, feedback={position_feedback_topic}')

    def on_feedback(self, msg):
        with self.state_condition:
            self.current_front_lift = msg.front_lift
            self.current_rear_lift = msg.rear_lift
            self.state_condition.notify_all()

    def handle_set_lift(self, request, response):
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

            command = LiftCommand()
            command.front_lift = request.front_lift
            command.rear_lift = request.rear_lift
            self.command_publisher.publish(command)

            deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
            while rclpy.ok():
                should_wait = False
                with self.state_condition:
                    if (
                        self.current_front_lift is None or
                        self.current_rear_lift is None
                    ):
                        should_wait = True
                    else:
                        front_error = request.front_lift - self.current_front_lift
                        rear_error = request.rear_lift - self.current_rear_lift
                        if (
                            abs(front_error) <= tolerance and
                            abs(rear_error) <= tolerance
                        ):
                            response.success = True
                            response.message = 'Lift target reached'
                            response.final_front_lift = self.current_front_lift
                            response.final_rear_lift = self.current_rear_lift
                            response.front_error = front_error
                            response.rear_error = rear_error
                            return response
                        should_wait = True

                    remaining = deadline - (self.get_clock().now().nanoseconds / 1e9)
                    if remaining <= 0.0:
                        break
                    if should_wait:
                        self.state_condition.wait(timeout=remaining)

            with self.state_condition:
                response.success = False
                response.message = 'SetLift timeout'
                response.final_front_lift = (
                    self.current_front_lift if self.current_front_lift is not None
                    else 0.0
                )
                response.final_rear_lift = (
                    self.current_rear_lift if self.current_rear_lift is not None
                    else 0.0
                )
                response.front_error = request.front_lift - response.final_front_lift
                response.rear_error = request.rear_lift - response.final_rear_lift
                return response


def main():
    rclpy.init()
    node = LiftServiceController()
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
