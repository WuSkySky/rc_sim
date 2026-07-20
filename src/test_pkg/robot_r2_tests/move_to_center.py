#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from robot_r2_interfaces.srv import MoveToPose


class MoveToPoseTestClient(Node):
    def __init__(self):
        super().__init__('move_to_pose_test_client')

        self.client = self.create_client(MoveToPose, '/r2/move_to_pose')

        self.platform_centers_xy = (
            (3.4, -3.0),
            (1.0, -5.4),
            (-1.4, -5.4),
            (-2.6, -4.2),
            (-2.6, -1.8),
            (-1.4, -0.6),

            (2.2, -4.2),
            (2.2, -3.0),
            (2.2, -1.8),

            (1.0, -4.2),
            (1.0, -3.0),
            (1.0, -1.8),

            (-0.2, -4.2),
            (-0.2, -3.0),
            (-0.2, -1.8),

            (-1.4, -4.2),
            (-1.4, -3.0),
            (-1.4, -1.8),
        )

        self.index = 0
        self.direction = 1
        self.yaw = 0.0
        self.yaw_step = math.pi / 2.0
        self.request_in_flight = False

        self.timer = self.create_timer(2.0, self.send_next_goal)

    def send_next_goal(self):
        if self.request_in_flight:
            return

        if not self.client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn('/r2/move_to_pose service unavailable')
            return

        x, y = self.platform_centers_xy[self.index]

        request = MoveToPose.Request()
        request.x = x
        request.y = y
        request.yaw = self.yaw
        request.position_tolerance = 0.03
        request.yaw_tolerance = 0.05
        request.timeout_sec = 20.0

        self.request_in_flight = True
        future = self.client.call_async(request)
        future.add_done_callback(self.on_goal_done)

        order = 'forward' if self.direction == 1 else 'reverse'
        self.get_logger().info(
            f'Requested target index={self.index}, order={order}, '
            f'x={x:.2f}, y={y:.2f}, yaw={math.degrees(self.yaw):.1f} deg'
        )

        self.yaw = self.normalize_angle(self.yaw + self.yaw_step)
        self.index += self.direction

        if self.index >= len(self.platform_centers_xy):
            self.direction = -1
            self.index = len(self.platform_centers_xy) - 1
        elif self.index < 0:
            self.direction = 1
            self.index = 0

    def on_goal_done(self, future):
        self.request_in_flight = False
        response = future.result()
        if response is None:
            self.get_logger().error('MoveToPose service call failed')
            return

        if response.success:
            self.get_logger().info(
                f'MoveToPose succeeded: '
                f'position_error={response.position_error:.4f}, '
                f'yaw_error={response.yaw_error:.4f}'
            )
        else:
            self.get_logger().error(
                f'MoveToPose failed: {response.message} '
                f'(position_error={response.position_error:.4f}, '
                f'yaw_error={response.yaw_error:.4f})'
            )

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main():
    rclpy.init()
    node = MoveToPoseTestClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
