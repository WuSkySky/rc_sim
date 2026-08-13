#!/usr/bin/env python3
"""Align the chassis using ROI center feedback from the vision node."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from robot_r2_interfaces.msg import AlignmentDetection
from robot_r2_interfaces.srv import Align


DETECTION_TOPIC = "/r2/alignment/detection"
CMD_VEL_TOPIC = "/r2/alignment/cmd_vel"
ALIGN_SERVICE = "/r2/alignment/align"


@dataclass(frozen=True)
class AlignmentConfig:
    pixel_tolerance: int
    stable_cycles: int
    default_timeout_sec: float
    kp: float
    ki: float
    kd: float
    integral_limit: float
    output_limit: float


class AlignmentController(Node):
    """Align the chassis from generic image-space target feedback."""

    def __init__(self) -> None:
        super().__init__("alignment")
        self.service_lock = threading.Lock()
        self.state_condition = threading.Condition()
        self._config_lock = threading.Lock()
        self._feedback_callback_group = MutuallyExclusiveCallbackGroup()
        self._service_callback_group = ReentrantCallbackGroup()

        self._declare_parameters()
        self._config = self._read_config()

        self._frame_sequence = 0
        self._latest_offset_x: int | None = None
        self._latest_frame_received_at = 0.0
        self._alignment_active = False
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._subscription = self.create_subscription(
            AlignmentDetection,
            DETECTION_TOPIC,
            self._on_detection,
            image_qos,
            callback_group=self._feedback_callback_group,
        )
        self._pub_cmd = self.create_publisher(
            Twist,
            CMD_VEL_TOPIC,
            10,
        )
        self._srv = self.create_service(
            Align,
            ALIGN_SERVICE,
            self._handle_align,
            callback_group=self._service_callback_group,
        )
        self._parameter_callback = self.add_on_set_parameters_callback(
            self._on_parameters_changed
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("pixel_tolerance", 5)
        self.declare_parameter("stable_cycles", 10)
        self.declare_parameter("default_timeout_sec", 10.0)
        self.declare_parameter("kp", 0.004)
        self.declare_parameter("ki", 0.00015)
        self.declare_parameter("kd", 0.0005)
        self.declare_parameter("integral_limit", 0.5)
        self.declare_parameter("output_limit", 0.1)

    @staticmethod
    def _validate_config(config: AlignmentConfig) -> None:
        if (
            not isinstance(config.pixel_tolerance, int)
            or isinstance(config.pixel_tolerance, bool)
        ):
            raise ValueError("pixel_tolerance must be an integer")
        if (
            not isinstance(config.stable_cycles, int)
            or isinstance(config.stable_cycles, bool)
        ):
            raise ValueError("stable_cycles must be an integer")
        if config.pixel_tolerance < 0:
            raise ValueError("pixel_tolerance must be non-negative")
        if config.stable_cycles <= 0:
            raise ValueError("stable_cycles must be greater than zero")
        if (
            not math.isfinite(config.default_timeout_sec)
            or config.default_timeout_sec <= 0.0
        ):
            raise ValueError(
                "default_timeout_sec must be finite and positive"
            )
        for name in ("kp", "ki", "kd"):
            if not math.isfinite(getattr(config, name)):
                raise ValueError(f"{name} must be finite")
        if (
            not math.isfinite(config.integral_limit)
            or config.integral_limit < 0.0
        ):
            raise ValueError(
                "integral_limit must be finite and non-negative"
            )
        if (
            not math.isfinite(config.output_limit)
            or config.output_limit <= 0.0
        ):
            raise ValueError("output_limit must be finite and positive")

    @classmethod
    def _config_from_values(cls, values) -> AlignmentConfig:
        config = AlignmentConfig(
            pixel_tolerance=values["pixel_tolerance"],
            stable_cycles=values["stable_cycles"],
            default_timeout_sec=values["default_timeout_sec"],
            kp=values["kp"],
            ki=values["ki"],
            kd=values["kd"],
            integral_limit=values["integral_limit"],
            output_limit=values["output_limit"],
        )
        cls._validate_config(config)
        return config

    def _read_config(self) -> AlignmentConfig:
        names = AlignmentConfig.__dataclass_fields__
        values = {
            name: self.get_parameter(name).value
            for name in names
        }
        return self._config_from_values(values)

    def _config_snapshot(self) -> AlignmentConfig:
        with self._config_lock:
            return self._config

    def _on_parameters_changed(self, parameters) -> SetParametersResult:
        with self._config_lock:
            current = self._config
            values = {
                name: getattr(current, name)
                for name in AlignmentConfig.__dataclass_fields__
            }
            for parameter in parameters:
                if parameter.name in values:
                    values[parameter.name] = parameter.value
            try:
                candidate = self._config_from_values(values)
            except (TypeError, ValueError) as error:
                return SetParametersResult(
                    successful=False,
                    reason=str(error),
                )
            self._config = candidate
        return SetParametersResult(successful=True)

    def _on_detection(self, msg: AlignmentDetection) -> None:
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

    def _pid_update(
        self,
        error: float,
        dt: float,
        config: AlignmentConfig,
    ) -> float:
        if dt <= 0.0:
            return 0.0

        self._integral += error * dt
        if config.integral_limit > 0.0:
            self._integral = max(
                -config.integral_limit,
                min(self._integral, config.integral_limit),
            )

        derivative = 0.0
        if self._has_last_error:
            derivative = (error - self._last_error) / dt
        output = (
            config.kp * error
            + config.ki * self._integral
            + config.kd * derivative
        )
        self._last_error = error
        self._has_last_error = True
        if config.output_limit > 0.0:
            output = max(
                -config.output_limit,
                min(output, config.output_limit),
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

            tolerance_override = (
                requested_tolerance
                if requested_tolerance > 0.0
                else None
            )
            timeout_override = (
                requested_timeout
                if requested_timeout > 0.0
                else None
            )
            with self.state_condition:
                self._alignment_active = True
            try:
                return self._execute_alignment(
                    response,
                    tolerance_override,
                    timeout_override,
                )
            finally:
                with self.state_condition:
                    self._alignment_active = False

    def _execute_alignment(
        self,
        response,
        tolerance_override,
        timeout_override,
    ):
        self._pid_reset()
        stable_cycle_count = 0
        last_frame_time: float | None = None
        last_offset: int | None = None
        saw_detection = False
        service_started_at = time.monotonic()

        with self.state_condition:
            handled_sequence = self._frame_sequence

        while rclpy.ok():
            with self.state_condition:
                while (
                    self._frame_sequence <= handled_sequence
                    and rclpy.ok()
                ):
                    wait_config = self._config_snapshot()
                    timeout_sec = (
                        timeout_override
                        if timeout_override is not None
                        else wait_config.default_timeout_sec
                    )
                    deadline = service_started_at + timeout_sec
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

            config = self._config_snapshot()
            tolerance = (
                tolerance_override
                if tolerance_override is not None
                else float(config.pixel_tolerance)
            )
            timeout_sec = (
                timeout_override
                if timeout_override is not None
                else config.default_timeout_sec
            )
            deadline = service_started_at + timeout_sec
            if time.monotonic() >= deadline:
                if not saw_detection:
                    message = (
                        "Alignment timeout: no target detected"
                    )
                elif last_offset is None:
                    message = "Alignment timeout: target lost"
                else:
                    message = (
                        f"Alignment timeout: final offset={last_offset}px, "
                        f"stable={stable_cycle_count}/"
                        f"{config.stable_cycles}"
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
                if stable_cycle_count >= config.stable_cycles:
                    response.success = True
                    response.message = (
                        f"Alignment complete: offset={offset}px, "
                        f"stable={stable_cycle_count}/"
                        f"{config.stable_cycles}, "
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
            output = self._pid_update(-float(offset), dt, config)
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
    node = AlignmentController()
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
