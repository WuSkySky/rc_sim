#!/usr/bin/env python3
"""KFS alignment controller.

Subscribes /r2/detection/processed → uses center_offset_x as error
PID controls chassis lateral velocity (linear.y on /r2/cmd_vel).
Service /r2/align_to_kfs blocks until alignment within tolerance.
"""

from __future__ import annotations

import threading
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.msg import KfsProcessedDetection
from robot_r2_interfaces.srv import AlignToKFS


class KfsAlignmentController(Node):
    def __init__(self):
        super().__init__('kfs_alignment')

        self.service_lock = threading.Lock()
        self.state_condition = threading.Condition()

        self._declare_parameters()
        self._load_parameters()

        self._sub = self.create_subscription(
            KfsProcessedDetection,
            self._detection_topic,
            self._on_detection,
            10,
        )
        self._pub_cmd = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._srv = self.create_service(
            AlignToKFS,
            self._align_service,
            self._handle_align,
            callback_group=ReentrantCallbackGroup(),
        )

        self._latest_offset_x: int | None = None
        self._latest_kfs_type: str = "none"

    # ---- parameters ------------------------------------------------

    def _declare_parameters(self) -> None:
        self.declare_parameter('detection_topic', '/r2/detection/processed')
        self.declare_parameter('cmd_vel_topic', '/r2/cmd_vel')
        self.declare_parameter('align_service', '/r2/align_to_kfs')
        self.declare_parameter('pixel_tolerance', 5)
        self.declare_parameter('default_timeout_sec', 10.0)
        self.declare_parameter('control_rate', 50.0)

        self.declare_parameter('kp', 0.008)
        self.declare_parameter('ki', 0.0003)
        self.declare_parameter('kd', 0.001)
        self.declare_parameter('integral_limit', 0.5)
        self.declare_parameter('output_limit', 1.0)

    def _load_parameters(self) -> None:
        self._detection_topic = str(
            self.get_parameter('detection_topic').value)
        self._cmd_vel_topic = str(
            self.get_parameter('cmd_vel_topic').value)
        self._align_service = str(
            self.get_parameter('align_service').value)
        self._pixel_tolerance = int(
            self.get_parameter('pixel_tolerance').value)
        self._default_timeout_sec = float(
            self.get_parameter('default_timeout_sec').value)
        self._control_rate = float(
            self.get_parameter('control_rate').value)

        self._kp = float(self.get_parameter('kp').value)
        self._ki = float(self.get_parameter('ki').value)
        self._kd = float(self.get_parameter('kd').value)
        self._integral_limit = abs(
            float(self.get_parameter('integral_limit').value))
        self._output_limit = abs(
            float(self.get_parameter('output_limit').value))

    # ---- detection callback ---------------------------------------

    def _on_detection(self, msg: KfsProcessedDetection) -> None:
        with self.state_condition:
            self._latest_offset_x = msg.center_offset_x
            self._latest_kfs_type = msg.kfs_type
            self.state_condition.notify_all()

    # ---- PID ------------------------------------------------------

    def _pid_reset(self) -> None:
        self._integral = 0.0
        self._last_error = 0.0
        self._has_last_error = False

    def _pid_update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        self._integral += error * dt
        if self._integral_limit > 0.0:
            self._integral = max(
                -self._integral_limit,
                min(self._integral, self._integral_limit),
            )

        derivative = 0.0
        if self._has_last_error:
            derivative = (error - self._last_error) / dt

        output = (
            self._kp * error
            + self._ki * self._integral
            + self._kd * derivative
        )

        self._last_error = error
        self._has_last_error = True

        if self._output_limit > 0.0:
            output = max(-self._output_limit, min(output, self._output_limit))

        return output

    # ---- service handler ------------------------------------------

    def _handle_align(self, request, response):
        with self.service_lock:
            tolerance = (
                int(request.pixel_tolerance)
                if request.pixel_tolerance > 0
                else self._pixel_tolerance
            )
            timeout_sec = (
                float(request.timeout_sec)
                if request.timeout_sec > 0.0
                else self._default_timeout_sec
            )
            period = 1.0 / self._control_rate

            self._pid_reset()
            last_tick = time.monotonic()
            deadline = time.monotonic() + timeout_sec

            while rclpy.ok():
                with self.state_condition:
                    offset = self._latest_offset_x
                    kfs_type = self._latest_kfs_type

                # No detection yet — wait
                if offset is None or kfs_type == "none":
                    if time.monotonic() > deadline:
                        response.success = False
                        response.message = 'Alignment timeout: no detection'
                        response.final_offset_x = (
                            offset if offset is not None else 0)
                        self._stop()
                        return response
                    time.sleep(period)
                    continue

                # Within tolerance — done
                if abs(offset) <= tolerance:
                    self._stop()
                    response.success = True
                    response.message = (
                        f'Alignment complete: offset={offset}px '
                        f'≤ tolerance={tolerance}px'
                    )
                    response.final_offset_x = offset
                    return response

                # Timeout check
                if time.monotonic() > deadline:
                    self._stop()
                    response.success = False
                    response.message = (
                        f'Alignment timeout: final offset={offset}px '
                        f'> tolerance={tolerance}px'
                    )
                    response.final_offset_x = offset
                    return response

                # PID control
                now = time.monotonic()
                dt = now - last_tick
                if dt <= 0.0:
                    dt = period
                last_tick = now

                # sign convention: positive offset → target is right →
                # need to move left  → negative linear.y
                output = self._pid_update(-float(offset), dt)

                cmd = Twist()
                cmd.linear.y = output
                self._pub_cmd.publish(cmd)

                time.sleep(period)

            # ROS shutdown
            self._stop()
            response.success = False
            response.message = 'Alignment aborted: ROS shutdown'
            response.final_offset_x = (
                self._latest_offset_x
                if self._latest_offset_x is not None else 0)
            return response

    # ---- helpers --------------------------------------------------

    def _stop(self) -> None:
        self._pub_cmd.publish(Twist())


def main():
    rclpy.init()
    node = KfsAlignmentController()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
