#!/usr/bin/env python3
"""Convert target-center error into safe lateral chassis commands."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time

from geometry_msgs.msg import Twist
import rclpy
from rcl_interfaces.msg import (
    FloatingPointRange,
    IntegerRange,
    ParameterDescriptor,
    SetParametersResult,
)
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from robot_r2_interfaces.msg import TargetDetection

from robot_r2_target_alignment.controller_core import (
    LateralPid,
    PidConfig,
    command_for_mode,
)


@dataclass(frozen=True)
class ControllerConfig:
    """One atomically replaceable runtime configuration."""

    enabled: bool
    test_mode: bool
    rate_hz: float
    pixel_tolerance: int
    stable_frames: int
    lost_timeout_sec: float
    log_rate_hz: float
    pid: PidConfig


def _float_descriptor(
    description: str,
    minimum: float,
    maximum: float,
) -> ParameterDescriptor:
    return ParameterDescriptor(
        description=description,
        floating_point_range=[
            FloatingPointRange(
                from_value=minimum,
                to_value=maximum,
                step=0.0,
            )
        ],
    )


def _integer_descriptor(
    description: str,
    minimum: int,
    maximum: int,
) -> ParameterDescriptor:
    return ParameterDescriptor(
        description=description,
        integer_range=[
            IntegerRange(
                from_value=minimum,
                to_value=maximum,
                step=1,
            )
        ],
    )


class TargetAlignmentController(Node):
    """Track one selected detection and drive chassis lateral velocity."""

    PARAMETER_NAMES = (
        "control.enabled",
        "test_mode",
        "control.rate_hz",
        "control.pixel_tolerance",
        "control.stable_frames",
        "control.lost_timeout_sec",
        "control.kp",
        "control.ki",
        "control.kd",
        "control.integral_limit",
        "control.output_limit",
        "control.minimum_output",
        "control.invert_output",
        "log.rate_hz",
    )

    def __init__(self) -> None:
        super().__init__("target_alignment_controller")
        self._config_lock = threading.RLock()
        self._state_lock = threading.Lock()
        self._declare_parameters()
        self._parameter_values = {
            name: self.get_parameter(name).value
            for name in self.PARAMETER_NAMES
        }
        self._config = self._make_config(self._parameter_values)
        self._pid = LateralPid(self._config.pid)

        self._last_message_at: float | None = None
        self._target_valid = False
        self._offset_x = 0
        self._normalized_error = 0.0
        self._class_name = ""
        self._confidence = 0.0
        self._stable_count = 0
        self._last_control_at: float | None = None
        self._next_control_at = 0.0
        self._last_log_at = 0.0
        self._last_status = ""

        detection_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            lifespan=Duration(seconds=0.2),
        )
        self._subscription = self.create_subscription(
            TargetDetection,
            "detections",
            self._on_detection,
            detection_qos,
        )
        self._publisher = self.create_publisher(Twist, "cmd_vel", command_qos)
        self._timer = self.create_timer(0.02, self._control_tick)
        self.add_on_set_parameters_callback(self._on_parameters_changed)

        mode = "TEST" if self._config.test_mode else "LIVE"
        self.get_logger().info(f"Alignment controller ready in {mode} mode")

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "control.enabled",
            True,
            ParameterDescriptor(description="Enable alignment control."),
        )
        self.declare_parameter(
            "test_mode",
            True,
            ParameterDescriptor(
                description="Compute commands but never publish them."
            ),
        )
        self.declare_parameter(
            "control.rate_hz",
            20.0,
            _float_descriptor("Control update rate.", 5.0, 50.0),
        )
        self.declare_parameter(
            "control.pixel_tolerance",
            20,
            _integer_descriptor(
                "Horizontal center tolerance in pixels.",
                0,
                4096,
            ),
        )
        self.declare_parameter(
            "control.stable_frames",
            8,
            _integer_descriptor("Frames required to declare alignment.", 1, 1000),
        )
        self.declare_parameter(
            "control.lost_timeout_sec",
            0.3,
            _float_descriptor(
                "Maximum detection age before stopping.",
                0.02,
                10.0,
            ),
        )
        self.declare_parameter(
            "control.kp",
            0.6,
            _float_descriptor(
                "Proportional gain for normalized error.",
                0.0,
                100.0,
            ),
        )
        self.declare_parameter(
            "control.ki",
            0.0,
            _float_descriptor(
                "Integral gain for normalized error.",
                0.0,
                100.0,
            ),
        )
        self.declare_parameter(
            "control.kd",
            0.05,
            _float_descriptor(
                "Derivative gain for normalized error.",
                0.0,
                100.0,
            ),
        )
        self.declare_parameter(
            "control.integral_limit",
            1.0,
            _float_descriptor("Absolute integral clamp.", 0.0, 100.0),
        )
        self.declare_parameter(
            "control.output_limit",
            0.4,
            _float_descriptor(
                "Maximum absolute linear.y command.",
                0.001,
                10.0,
            ),
        )
        self.declare_parameter(
            "control.minimum_output",
            0.04,
            _float_descriptor(
                "Minimum non-zero linear.y command.",
                0.0,
                10.0,
            ),
        )
        self.declare_parameter(
            "control.invert_output",
            True,
            ParameterDescriptor(
                description="Map positive image offset to negative linear.y."
            ),
        )
        self.declare_parameter(
            "log.rate_hz",
            2.0,
            _float_descriptor("Maximum repeated status log rate.", 0.1, 20.0),
        )

    @staticmethod
    def _make_config(values: dict[str, object]) -> ControllerConfig:
        config = ControllerConfig(
            enabled=bool(values["control.enabled"]),
            test_mode=bool(values["test_mode"]),
            rate_hz=float(values["control.rate_hz"]),
            pixel_tolerance=int(values["control.pixel_tolerance"]),
            stable_frames=int(values["control.stable_frames"]),
            lost_timeout_sec=float(values["control.lost_timeout_sec"]),
            log_rate_hz=float(values["log.rate_hz"]),
            pid=PidConfig(
                kp=float(values["control.kp"]),
                ki=float(values["control.ki"]),
                kd=float(values["control.kd"]),
                integral_limit=float(values["control.integral_limit"]),
                output_limit=float(values["control.output_limit"]),
                minimum_output=float(values["control.minimum_output"]),
                invert_output=bool(values["control.invert_output"]),
            ),
        )
        config.pid.validate()
        numeric = (config.rate_hz, config.lost_timeout_sec, config.log_rate_hz)
        if not all(math.isfinite(value) and value > 0.0 for value in numeric):
            raise ValueError("rates and timeouts must be finite and positive")
        if not 5.0 <= config.rate_hz <= 50.0:
            raise ValueError("control.rate_hz must be in [5, 50]")
        if config.pixel_tolerance < 0 or config.stable_frames <= 0:
            raise ValueError("tolerance and stable frame count are invalid")
        return config

    def _on_parameters_changed(self, parameters) -> SetParametersResult:
        proposed = dict(self._parameter_values)
        for parameter in parameters:
            if parameter.name in proposed:
                proposed[parameter.name] = parameter.value
        try:
            new_config = self._make_config(proposed)
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        with self._config_lock:
            old_config = self._config
            entering_safe_state = old_config.enabled and (
                not new_config.enabled
                or (not old_config.test_mode and new_config.test_mode)
            )
            if entering_safe_state and not old_config.test_mode:
                self._publisher.publish(Twist())
            self._config = new_config
            self._parameter_values = proposed
            self._pid.set_config(new_config.pid)
            self._next_control_at = 0.0
            with self._state_lock:
                self._stable_count = 0
                self._last_control_at = None
        return SetParametersResult(successful=True)

    def _on_detection(self, message: TargetDetection) -> None:
        now = time.monotonic()
        with self._config_lock:
            tolerance = self._config.pixel_tolerance
            with self._state_lock:
                previous_message_at = self._last_message_at
                self._last_message_at = now
                if (
                    previous_message_at is None
                    or now - previous_message_at
                    > self._config.lost_timeout_sec
                ):
                    self._stable_count = 0
                width = int(message.image_width)
                height = int(message.image_height)
                center_u = int(message.center_u)
                confidence = float(message.confidence)
                valid = (
                    bool(message.valid)
                    and width > 0
                    and height > 0
                    and 0 <= center_u < width
                    and math.isfinite(confidence)
                    and 0.0 <= confidence <= 1.0
                )
                if not valid:
                    self._target_valid = False
                    self._stable_count = 0
                    return
                offset_x = center_u - width // 2
                self._target_valid = True
                self._offset_x = offset_x
                self._normalized_error = offset_x / (float(width) * 0.5)
                self._class_name = str(message.class_name)
                self._confidence = confidence
                if abs(offset_x) <= tolerance:
                    self._stable_count += 1
                else:
                    self._stable_count = 0

    def _control_tick(self) -> None:
        now = time.monotonic()
        with self._config_lock:
            config = self._config
            if now < self._next_control_at:
                return
            self._next_control_at = now + 1.0 / config.rate_hz
            with self._state_lock:
                last_message_at = self._last_message_at
                target_valid = self._target_valid
                offset_x = self._offset_x
                error = self._normalized_error
                class_name = self._class_name
                confidence = self._confidence
                stable_count = self._stable_count

            if not config.enabled:
                self._pid.reset()
                self._log_status("DISABLED", "control is disabled", config, now)
                return

            age = math.inf if last_message_at is None else now - last_message_at
            if not target_valid or age > config.lost_timeout_sec:
                self._pid.reset()
                self._last_control_at = None
                self._send_command(0.0, config)
                state = "NO_TARGET" if last_message_at is None else "TARGET_LOST"
                detail = "waiting for detection"
                if math.isfinite(age):
                    detail = f"last detection age={age:.3f}s"
                self._log_status(state, detail, config, now)
                return

            if stable_count >= config.stable_frames:
                self._pid.reset()
                self._last_control_at = None
                self._send_command(0.0, config)
                detail = (
                    f"class={class_name} offset_x={offset_x:+d}px "
                    f"stable={stable_count}/{config.stable_frames}"
                )
                self._log_status("ALIGNED", detail, config, now)
                return

            if abs(offset_x) <= config.pixel_tolerance:
                self._pid.reset()
                self._last_control_at = None
                self._send_command(0.0, config)
                detail = (
                    f"class={class_name} offset_x={offset_x:+d}px "
                    f"stable={stable_count}/{config.stable_frames}"
                )
                self._log_status("STABILIZING", detail, config, now)
                return

            dt = (
                1.0 / config.rate_hz
                if self._last_control_at is None
                else max(now - self._last_control_at, 1e-6)
            )
            self._last_control_at = now
            command = self._pid.update(error, dt)
            self._send_command(command, config)
            detail = (
                f"class={class_name} conf={confidence:.3f} "
                f"offset_x={offset_x:+d}px candidate_vy={command:+.3f}m/s"
            )
            self._log_status("TRACKING", detail, config, now)

    def _send_command(
        self,
        lateral_velocity: float,
        config: ControllerConfig,
    ) -> None:
        output = command_for_mode(lateral_velocity, config.test_mode)
        if output is None:
            return
        command = Twist()
        command.linear.y = output
        self._publisher.publish(command)

    def _log_status(
        self,
        status: str,
        detail: str,
        config: ControllerConfig,
        now: float,
    ) -> None:
        changed = status != self._last_status
        due = now - self._last_log_at >= 1.0 / config.log_rate_hz
        if not changed and not due:
            return
        suffix = " [TEST]" if config.test_mode else ""
        self.get_logger().info(f"{status} {detail}{suffix}")
        self._last_status = status
        self._last_log_at = now

    def stop(self) -> None:
        """Send a final zero command when live output is enabled."""
        with self._config_lock:
            if self._config.enabled and not self._config.test_mode:
                self._publisher.publish(Twist())
            self._pid.reset()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TargetAlignmentController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
