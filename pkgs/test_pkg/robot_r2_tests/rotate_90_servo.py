#!/usr/bin/env python3

import math
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from robot_r2_interfaces.srv import MoveToPose


class RotateNinetyServoTest(Node):
    def __init__(self):
        super().__init__('rotate_90_servo_test')

        self.declare_parameter('current_pose_topic', '/r2/pose_feedback')
        self.declare_parameter('move_to_pose_service', '/r2/move_to_pose')
        self.declare_parameter('wait_after_rotation_sec', 1.0)
        self.declare_parameter('move_timeout_sec', 20.0)

        pose_topic = str(self.get_parameter('current_pose_topic').value)
        service_name = str(
            self.get_parameter('move_to_pose_service').value)
        self.wait_after_rotation_sec = float(
            self.get_parameter('wait_after_rotation_sec').value)
        self.move_timeout_sec = float(
            self.get_parameter('move_timeout_sec').value)
        if self.wait_after_rotation_sec < 0.0:
            raise ValueError(
                'wait_after_rotation_sec must be non-negative')
        if self.move_timeout_sec <= 0.0:
            raise ValueError('move_timeout_sec must be positive')

        self.current_pose = None
        self.fixed_position = None
        self.target_yaw = None
        self.pending_yaw = None
        self.request_in_flight = False
        self.next_request_time = 0.0

        self.pose_subscription = self.create_subscription(
            PoseStamped, pose_topic, self.on_pose_feedback, 10)
        self.move_client = self.create_client(MoveToPose, service_name)
        self.timer = self.create_timer(0.05, self.try_send_next_goal)

    def on_pose_feedback(self, message):
        pose = message.pose
        yaw = self.yaw_from_quaternion(pose.orientation)
        values = (pose.position.x, pose.position.y, yaw)
        if not all(math.isfinite(value) for value in values):
            return
        self.current_pose = values

    def try_send_next_goal(self):
        if self.request_in_flight or time.monotonic() < self.next_request_time:
            return
        if self.current_pose is None:
            return
        if not self.move_client.service_is_ready():
            return

        if self.fixed_position is None:
            self.fixed_position = self.current_pose[:2]
            self.target_yaw = self.current_pose[2]
            self.get_logger().info(
                f'Locked rotation center at x={self.fixed_position[0]:.3f}, '
                f'y={self.fixed_position[1]:.3f}')

        self.pending_yaw = self.normalize_angle(
            self.target_yaw + math.pi / 2.0)

        request = MoveToPose.Request()
        request.x = self.fixed_position[0]
        request.y = self.fixed_position[1]
        request.yaw = self.pending_yaw
        request.position_tolerance = 0.0
        request.yaw_tolerance = 0.0
        request.timeout_sec = self.move_timeout_sec

        self.request_in_flight = True
        future = self.move_client.call_async(request)
        future.add_done_callback(self.on_rotation_complete)
        self.get_logger().info(
            f'Commanding yaw={math.degrees(self.pending_yaw):.1f} deg')

    def on_rotation_complete(self, future):
        self.request_in_flight = False
        self.next_request_time = (
            time.monotonic() + self.wait_after_rotation_sec)

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'MoveToPose call raised: {exc}')
            return
        if response is None:
            self.get_logger().error('MoveToPose service call failed')
            return

        if not response.success:
            self.get_logger().error(
                f'Rotation failed: {response.message}, '
                f'yaw_error={math.degrees(response.yaw_error):.3f} deg')
            return

        self.target_yaw = self.pending_yaw
        self.get_logger().info(
            f'Rotation completed: final_yaw='
            f'{math.degrees(response.final_yaw):.3f} deg, '
            f'yaw_error={math.degrees(response.yaw_error):.3f} deg; '
            f'waiting {self.wait_after_rotation_sec:.1f} s')

    @staticmethod
    def yaw_from_quaternion(quaternion):
        sin_yaw = 2.0 * (
            quaternion.w * quaternion.z +
            quaternion.x * quaternion.y
        )
        cos_yaw = 1.0 - 2.0 * (
            quaternion.y * quaternion.y +
            quaternion.z * quaternion.z
        )
        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main():
    rclpy.init()
    node = RotateNinetyServoTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
