#!/usr/bin/env python3
"""Align the chassis using ROI center feedback from the vision node."""

from __future__ import annotations

import math
import threading
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.msg import KfsRoiDetection
from robot_r2_interfaces.srv import AlignToKFS


class KfsAlignmentController(Node):
    """Run only the alignment control loop; vision lives in kfs_roi."""

    def __init__(self) -> None:
        super().__init__("kfs_alignment")
        self.service_lock = threading.Lock()
        self.state_condition = threading.Condition()
        self._feedback_callback_group = MutuallyExclusiveCallbackGroup()
        self._service_callback_group = ReentrantCallbackGroup()

        self._declare_parameters()
        self._load_parameters()

        self._frame_sequence = 0
        self._latest_offset_x: int | None = None
        self._latest_frame_received_at = 0.0
        self._alignment_active = False

        self._subscription = self.create_subscription(
            KfsRoiDetection,
            self._roi_topic,
            self._on_roi,
            1,
            callback_group=self._feedback_callback_group,
        )
        self._pub_cmd = self.create_publisher(
            Twist,
            self._cmd_vel_topic,
            10,
        )
        self._srv = self.create_service(
            AlignToKFS,
            self._align_service,
            self._handle_align,
            callback_group=self._service_callback_group,
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("roi_topic", "/r2/kfs/roi")
        self.declare_parameter("cmd_vel_topic", "/r2/cmd_vel")
        self.declare_parameter("align_service", "/r2/align_to_kfs")
        self.declare_parameter("pixel_tolerance", 5)
        self.declare_parameter("stable_cycles", 10)
        self.declare_parameter("default_timeout_sec", 10.0)
        self.declare_parameter("kp", 0.008)
        self.declare_parameter("ki", 0.0003)
        self.declare_parameter("kd", 0.001)
        self.declare_parameter("integral_limit", 0.5)
        self.declare_parameter("output_limit", 1.0)

    def _load_parameters(self) -> None:
        self._roi_topic = str(self.get_parameter("roi_topic").value)
        self._cmd_vel_topic = str(
            self.get_parameter("cmd_vel_topic").value
        )
        self._align_service = str(
            self.get_parameter("align_service").value
        )
        self._pixel_tolerance = int(
            self.get_parameter("pixel_tolerance").value
        )
        self._stable_cycles = int(
            self.get_parameter("stable_cycles").value
        )
        self._default_timeout_sec = float(
            self.get_parameter("default_timeout_sec").value
        )
        if self._pixel_tolerance < 0:
            raise ValueError("pixel_tolerance must be non-negative")
        if self._stable_cycles <= 0:
            raise ValueError("stable_cycles must be greater than zero")
        if (
            not math.isfinite(self._default_timeout_sec)
            or self._default_timeout_sec <= 0.0
        ):
            raise ValueError(
                "default_timeout_sec must be finite and positive"
            )

        self._kp = self._finite_parameter("kp")
        self._ki = self._finite_parameter("ki")
        self._kd = self._finite_parameter("kd")
        self._integral_limit = abs(
            self._finite_parameter("integral_limit")
        )
        self._output_limit = abs(
            self._finite_parameter("output_limit")
        )

    def _finite_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        return value

    def _on_roi(self, msg: KfsRoiDetection) -> None:
        received_at = time.monotonic()
        with self.state_condition:
            if not self._alignment_active:
                return
            self._frame_sequence += 1
            self._latest_offset_x = (
                int(msg.center_offset_x) if msg.valid else None
            )
            self._latest_frame_received_at = received_at
            self.state_condition.notify_all()

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
            output = max(
                -self._output_limit,
                min(output, self._output_limit),
            )
        return output

    def _handle_align(self, request, response):
        with self.service_lock:
            requested_tolerance = float(request.pixel_tolerance)
            requested_timeout = float(request.timeout_sec)
            if not math.isfinite(requested_tolerance):
                return self._failure_response(
                    response,
                    "pixel_tolerance must be finite",
                    None,
                )
            if not math.isfinite(requested_timeout):
                return self._failure_response(
                    response,
                    "timeout_sec must be finite",
                    None,
                )

            tolerance = (
                requested_tolerance
                if requested_tolerance > 0.0
                else float(self._pixel_tolerance)
            )
            timeout_sec = (
                requested_timeout
                if requested_timeout > 0.0
                else self._default_timeout_sec
            )
            with self.state_condition:
                self._alignment_active = True
            try:
                return self._execute_alignment(
                    response,
                    tolerance,
                    timeout_sec,
                )
            finally:
                with self.state_condition:
                    self._alignment_active = False

    def _execute_alignment(self, response, tolerance, timeout_sec):
        self._pid_reset()
        stable_cycle_count = 0
        last_frame_time: float | None = None
        last_offset: int | None = None
        saw_detection = False
        service_started_at = time.monotonic()
        deadline = service_started_at + timeout_sec

        with self.state_condition:
            handled_sequence = self._frame_sequence

        while rclpy.ok():
            with self.state_condition:
                while (
                    self._frame_sequence <= handled_sequence
                    and rclpy.ok()
                ):
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        break
                    self.state_condition.wait(
                        timeout=min(remaining, 0.1)
                    )

                if self._frame_sequence > handled_sequence:
                    handled_sequence = self._frame_sequence
                    offset = self._latest_offset_x
                    frame_time = self._latest_frame_received_at
                else:
                    offset = None
                    frame_time = 0.0

            if time.monotonic() >= deadline:
                if not saw_detection:
                    message = (
                        "Alignment timeout: no KFS ROI detected"
                    )
                elif last_offset is None:
                    message = "Alignment timeout: target lost"
                else:
                    message = (
                        f"Alignment timeout: final offset={last_offset}px, "
                        f"stable={stable_cycle_count}/"
                        f"{self._stable_cycles}"
                    )
                return self._failure_response(
                    response,
                    message,
                    last_offset,
                )

            if frame_time < service_started_at:
                continue
            if offset is None:
                stable_cycle_count = 0
                last_offset = None
                last_frame_time = None
                self._pid_reset()
                self._stop()
                continue

            saw_detection = True
            last_offset = offset
            if abs(offset) <= tolerance:
                stable_cycle_count += 1
                last_frame_time = frame_time
                self._pid_reset()
                self._stop()
                if stable_cycle_count >= self._stable_cycles:
                    response.success = True
                    response.message = (
                        f"Alignment complete: offset={offset}px, "
                        f"stable={stable_cycle_count}/"
                        f"{self._stable_cycles}, "
                        f"tolerance={tolerance:g}px"
                    )
                    response.final_offset_x = offset
                    return response
                continue

            stable_cycle_count = 0
            if last_frame_time is None:
                dt = max(frame_time - service_started_at, 1e-6)
            else:
                dt = max(frame_time - last_frame_time, 1e-6)
            last_frame_time = frame_time

            # Positive image offset means the target is to the right.
            output = self._pid_update(-float(offset), dt)
            cmd = Twist()
            cmd.linear.y = output
            self._pub_cmd.publish(cmd)

        return self._failure_response(
            response,
            "Alignment aborted: ROS shutdown",
            last_offset,
        )

    def _failure_response(self, response, message, offset):
        self._stop()
        response.success = False
        response.message = message
        response.final_offset_x = offset if offset is not None else 0
        return response

    def _stop(self) -> None:
        self._pub_cmd.publish(Twist())


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
