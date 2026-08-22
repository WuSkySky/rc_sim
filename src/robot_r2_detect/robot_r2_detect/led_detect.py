#!/usr/bin/env python3
"""ROS 2 node for ArUco-guided LED target detection."""

from __future__ import annotations

import math
import threading
import time

import cv2
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_common import ABORT_MESSAGE, AbortMonitor, AbortableMixin
from robot_r2_interfaces.msg import CameraFrame, LedDetection
from robot_r2_interfaces.srv import DetectLed
from sensor_msgs.msg import Image

from .camera_frame import (
    bgr_to_image_message,
    camera_frame_header,
    camera_frame_to_bgr,
    camera_qos,
)
from .led_detection import (
    ArucoDetector,
    ArucoLedMapper,
    LedDetectionResult,
    LedStateDetector,
    TargetMatchTracker,
)


INPUT_IMAGE_TOPIC = "/r2/led_detection/image"
SERVICE_NAME = "/r2/led_detection/detect"
RESULT_TOPIC = "/r2/led_detection/result"
VISUALIZATION_TOPIC = "/r2/led_detection/debug"
MAX_PROCESSING_RATE_HZ = 15.0


def _advance_rate_limit(
    now: float, next_allowed: float, period: float
) -> tuple[bool, float]:
    """Return whether to process now and the next allowed monotonic time."""
    if now < next_allowed:
        return False, next_allowed
    return True, now + period


def _image_work_needed(
    continuous_detection: bool,
    visualization_enabled: bool,
    service_active: bool,
) -> bool:
    """Return whether the node must keep and process an image reader."""
    return continuous_detection or visualization_enabled or service_active


class LedDetectNode(AbortableMixin, Node):
    """Detect configurable LEDs and wait for a requested target state."""

    def __init__(self) -> None:
        super().__init__("led_detect")
        self._state_condition = threading.Condition()
        self._service_lock = threading.Lock()
        self._subscription_lock = threading.Lock()
        self._image_callback_group = MutuallyExclusiveCallbackGroup()
        self._service_callback_group = MutuallyExclusiveCallbackGroup()
        self._abort_callback_group = MutuallyExclusiveCallbackGroup()

        self._frame_sequence = 0
        self._service_active = False
        self._service_started_at = 0.0
        self._service_last_valid_states: tuple[bool, ...] | None = None
        self._service_last_reason = ""
        self._tracker: TargetMatchTracker | None = None
        self._next_processing_at = 0.0
        self._image_subscription = None

        self._declare_parameters()
        self._load_parameters()
        image_qos = camera_qos()

        self._image_qos = image_qos
        self._visualization_publisher = self.create_publisher(
            Image, VISUALIZATION_TOPIC, image_qos
        )
        self._result_publisher = self.create_publisher(
            LedDetection, RESULT_TOPIC, 10
        )
        self._service = self.create_service(
            DetectLed,
            SERVICE_NAME,
            self._handle_detect_led,
            callback_group=self._service_callback_group,
        )
        self.add_on_set_parameters_callback(self._on_parameters_changed)
        self._sync_image_subscription()
        self.abort_monitor = AbortMonitor(
            self,
            callback_group=self._abort_callback_group,
            on_abort=self._wake_service_on_abort,
        )

    def _wake_service_on_abort(self):
        with self._state_condition:
            self._state_condition.notify_all()

    def _declare_parameters(self) -> None:
        self.declare_parameter("visualization_enabled", False)
        self.declare_parameter("continuous_detection", False)
        self.declare_parameter("stable_match_frames", 5)
        self.declare_parameter("service_timeout_sec", 30.0)
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("target_marker_id", 4)
        self.declare_parameter("marker_size_mm", 100.0)
        self.declare_parameter("led_count", 1)
        self.declare_parameter(
            "led_positions_x_mm", [0.0]
        )
        self.declare_parameter(
            "led_positions_y_mm", [170.0]
        )
        self.declare_parameter("led_radius_mm", 10.0)
        self.declare_parameter("brightness_threshold", 200.0)
        self.declare_parameter("target_processing_rate", 15.0)

    def _load_parameters(self) -> None:
        self._visualization_enabled = bool(
            self.get_parameter("visualization_enabled").value
        )
        self._continuous_detection = bool(
            self.get_parameter("continuous_detection").value
        )
        stable_match_frames = int(
            self.get_parameter("stable_match_frames").value
        )
        service_timeout_sec = float(
            self.get_parameter("service_timeout_sec").value
        )
        aruco_dictionary = str(
            self.get_parameter("aruco_dictionary").value
        )
        target_marker_id = int(
            self.get_parameter("target_marker_id").value
        )
        marker_size_mm = float(
            self.get_parameter("marker_size_mm").value
        )
        led_count = int(self.get_parameter("led_count").value)
        positions_x = tuple(
            float(value)
            for value in self.get_parameter(
                "led_positions_x_mm"
            ).value
        )
        positions_y = tuple(
            float(value)
            for value in self.get_parameter(
                "led_positions_y_mm"
            ).value
        )
        led_radius_mm = float(
            self.get_parameter("led_radius_mm").value
        )
        brightness_threshold = float(
            self.get_parameter("brightness_threshold").value
        )
        target_processing_rate = float(
            self.get_parameter("target_processing_rate").value
        )

        if stable_match_frames <= 0:
            raise ValueError("stable_match_frames must be positive")
        if not math.isfinite(service_timeout_sec) or service_timeout_sec <= 0:
            raise ValueError("service_timeout_sec must be finite and positive")
        if not aruco_dictionary:
            raise ValueError("aruco_dictionary must not be empty")
        if target_marker_id < -1:
            raise ValueError(
                "target_marker_id must be -1 or non-negative"
            )
        if led_count <= 0:
            raise ValueError("led_count must be positive")
        if len(positions_x) != led_count or len(positions_y) != led_count:
            raise ValueError(
                "led_count must equal the lengths of "
                "led_positions_x_mm and led_positions_y_mm"
            )
        if not all(
            math.isfinite(value) for value in positions_x + positions_y
        ):
            raise ValueError("LED positions must be finite")
        if (
            not math.isfinite(target_processing_rate)
            or not 0.0 < target_processing_rate <= MAX_PROCESSING_RATE_HZ
        ):
            raise ValueError(
                "target_processing_rate must be finite and in (0, 15]"
            )

        mapper = ArucoLedMapper(
            marker_size_mm=marker_size_mm,
            led_positions_mm=tuple(zip(positions_x, positions_y)),
            led_radius_mm=led_radius_mm,
        )
        self._led_count = led_count
        self._stable_match_frames = stable_match_frames
        self._service_timeout_sec = service_timeout_sec
        self._target_processing_rate = target_processing_rate
        self._processing_deadline_sec = 1.0 / target_processing_rate
        self._processing_period_sec = 1.0 / target_processing_rate
        self._state_detector = LedStateDetector(
            detector=ArucoDetector(aruco_dictionary),
            mapper=mapper,
            target_marker_id=(
                None if target_marker_id == -1 else target_marker_id
            ),
            brightness_threshold=brightness_threshold,
        )

    def _on_parameters_changed(self, parameters) -> SetParametersResult:
        continuous_detection = None
        visualization_enabled = None
        processing_rate = None
        stable_match_frames = None
        service_timeout_sec = None
        for parameter in parameters:
            if parameter.name == "target_processing_rate":
                if isinstance(parameter.value, bool) or not isinstance(
                    parameter.value, (int, float)
                ):
                    return SetParametersResult(
                        successful=False,
                        reason="target_processing_rate must be a number",
                    )
                processing_rate = float(parameter.value)
                if (
                    not math.isfinite(processing_rate)
                    or not 0.0 < processing_rate <= MAX_PROCESSING_RATE_HZ
                ):
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "target_processing_rate must be finite and in "
                            "(0, 15]"
                        ),
                    )
                continue
            if parameter.name == "stable_match_frames":
                if isinstance(parameter.value, bool) or not isinstance(
                    parameter.value, int
                ):
                    return SetParametersResult(
                        successful=False,
                        reason="stable_match_frames must be an integer",
                    )
                stable_match_frames = parameter.value
                if stable_match_frames <= 0:
                    return SetParametersResult(
                        successful=False,
                        reason="stable_match_frames must be positive",
                    )
                continue
            if parameter.name == "service_timeout_sec":
                if isinstance(parameter.value, bool) or not isinstance(
                    parameter.value, (int, float)
                ):
                    return SetParametersResult(
                        successful=False,
                        reason="service_timeout_sec must be a number",
                    )
                service_timeout_sec = float(parameter.value)
                if (
                    not math.isfinite(service_timeout_sec)
                    or service_timeout_sec <= 0.0
                ):
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "service_timeout_sec must be finite and positive"
                        ),
                    )
                continue
            if parameter.name not in (
                "continuous_detection", "visualization_enabled"
            ):
                continue
            if not isinstance(parameter.value, bool):
                return SetParametersResult(
                    successful=False,
                    reason=f"{parameter.name} must be a boolean",
                )
            if parameter.name == "continuous_detection":
                continuous_detection = parameter.value
            else:
                visualization_enabled = parameter.value

        if continuous_detection is not None:
            with self._state_condition:
                self._continuous_detection = continuous_detection
            self._sync_image_subscription()
            state = "enabled" if continuous_detection else "disabled"
            self.get_logger().info(
                f"continuous LED detection {state}"
            )
        if visualization_enabled is not None:
            with self._state_condition:
                self._visualization_enabled = visualization_enabled
            self._sync_image_subscription()
            state = "enabled" if visualization_enabled else "disabled"
            self.get_logger().info(f"LED visualization {state}")
        if processing_rate is not None:
            with self._state_condition:
                self._target_processing_rate = processing_rate
                self._processing_deadline_sec = 1.0 / processing_rate
                self._processing_period_sec = 1.0 / processing_rate
                self._next_processing_at = 0.0
            self.get_logger().info(
                "LED processing rate limit changed to "
                f"{processing_rate:g} Hz"
            )
        if stable_match_frames is not None:
            with self._state_condition:
                self._stable_match_frames = stable_match_frames
            self.get_logger().info(
                "LED stable match requirement changed to "
                f"{stable_match_frames} frames"
            )
        if service_timeout_sec is not None:
            with self._state_condition:
                self._service_timeout_sec = service_timeout_sec
            self.get_logger().info(
                "LED service timeout changed to "
                f"{service_timeout_sec:g} seconds"
            )
        return SetParametersResult(successful=True)

    def _sync_image_subscription(self) -> None:
        """Match the image reader lifetime to detection or visualization."""
        with self._subscription_lock:
            with self._state_condition:
                needed = _image_work_needed(
                    self._continuous_detection,
                    self._visualization_enabled,
                    self._service_active,
                )
            if needed:
                if self._image_subscription is None:
                    self._image_subscription = self.create_subscription(
                        CameraFrame,
                        INPUT_IMAGE_TOPIC,
                        self._on_image,
                        self._image_qos,
                        callback_group=self._image_callback_group,
                    )
                    self.get_logger().info(
                        "LED image subscription enabled on "
                        f"{INPUT_IMAGE_TOPIC}"
                    )
                return
            if self._image_subscription is not None:
                subscription = self._image_subscription
                self._image_subscription = None
                self.destroy_subscription(subscription)
                self.get_logger().info(
                    "LED image subscription disabled while idle"
                )

    def _on_image(self, message: CameraFrame) -> None:
        started_at = time.monotonic()
        with self._state_condition:
            should_process = _image_work_needed(
                self._continuous_detection,
                self._visualization_enabled,
                self._service_active,
            )
            allowed, next_processing_at = _advance_rate_limit(
                started_at,
                self._next_processing_at,
                self._processing_period_sec,
            )
            if not should_process or not allowed:
                return
            # Drop excess frames before CameraFrame conversion and ArUco work.
            self._next_processing_at = next_processing_at

        image: np.ndarray | None = None
        try:
            image = camera_frame_to_bgr(message)
            result = self._state_detector.detect(image)
        except Exception as exc:
            result = LedDetectionResult(
                valid=False,
                reason=f"failed to process source image: {exc}",
            )

        with self._state_condition:
            self._frame_sequence += 1
            if (
                self._service_active
                and started_at >= self._service_started_at
                and self._tracker is not None
            ):
                states = result.states if result.valid else None
                self._tracker.update(states)
                if result.valid:
                    self._service_last_valid_states = result.states
                    self._service_last_reason = ""
                else:
                    self._service_last_reason = result.reason
            visualization_enabled = self._visualization_enabled
            continuous_detection = self._continuous_detection
            target = (
                self._tracker.target_states
                if self._tracker is not None
                else None
            )
            match_count = (
                self._tracker.count if self._tracker is not None else 0
            )
            required_frames = (
                self._tracker.required_frames
                if self._tracker is not None
                else self._stable_match_frames
            )
            self._state_condition.notify_all()

        if not result.valid:
            self.get_logger().debug(result.reason)

        if continuous_detection:
            result_message = self._make_result_message(message, result)
            with self._state_condition:
                continuous_detection = self._continuous_detection
            if continuous_detection:
                self._result_publisher.publish(result_message)

        if visualization_enabled and image is not None:
            visualization = self._make_visualization(
                image, result, target, match_count, required_frames
            )
            visualization_message = bgr_to_image_message(
                visualization,
                camera_frame_header(message),
            )
            with self._state_condition:
                visualization_enabled = self._visualization_enabled
            if visualization_enabled:
                self._visualization_publisher.publish(
                    visualization_message
                )

        processing_time = time.monotonic() - started_at
        if processing_time > self._processing_deadline_sec:
            self.get_logger().warn(
                "LED detection image processing overrun: "
                f"{processing_time * 1000.0:.2f} ms > "
                f"{self._processing_deadline_sec * 1000.0:.2f} ms "
                f"(target {self._target_processing_rate:g} Hz)"
            )

    def _handle_detect_led(self, request, response):
        abort_scope = self.abort_scope()
        target_states = tuple(bool(state) for state in request.target_states)
        if len(target_states) != self._led_count:
            response.success = False
            response.message = (
                f"target_states contains {len(target_states)} states; "
                f"expected {self._led_count}"
            )
            response.led_states = []
            return response

        with self._service_lock, abort_scope:
            if self.abort_requested():
                response.success = False
                response.message = ABORT_MESSAGE
                response.led_states = []
                return response
            started_at = time.monotonic()
            last_states: tuple[bool, ...] | None = None
            last_reason = ""
            stable_count = 0
            with self._state_condition:
                required_frames = self._stable_match_frames
                timeout_sec = self._service_timeout_sec
                deadline = started_at + timeout_sec
                handled_sequence = self._frame_sequence
                self._service_started_at = started_at
                self._service_last_valid_states = None
                self._service_last_reason = ""
                self._tracker = TargetMatchTracker(
                    target_states,
                    required_frames=required_frames,
                )
                self._service_active = True
                # Let a new request consume the next arriving frame instead of
                # waiting for a previous rate-limit slot.
                self._next_processing_at = started_at

            try:
                self._sync_image_subscription()
                while rclpy.ok():
                    if self.abort_requested():
                        response.success = False
                        response.message = ABORT_MESSAGE
                        response.led_states = (
                            list(last_states)
                            if last_states is not None else []
                        )
                        return response
                    with self._state_condition:
                        while (
                            self._frame_sequence <= handled_sequence
                            and rclpy.ok()
                        ):
                            remaining = deadline - time.monotonic()
                            if remaining <= 0.0:
                                break
                            self._state_condition.wait(
                                timeout=min(remaining, 0.05)
                            )

                        if self._frame_sequence > handled_sequence:
                            handled_sequence = self._frame_sequence
                        matched = (
                            self._tracker is not None
                            and self._tracker.count
                            >= required_frames
                        )
                        last_states = self._service_last_valid_states
                        last_reason = self._service_last_reason
                        stable_count = (
                            self._tracker.count
                            if self._tracker is not None
                            else 0
                        )

                    if matched:
                        response.success = True
                        response.message = (
                            "target LED states matched for "
                            f"{required_frames} consecutive frames"
                        )
                        response.led_states = list(target_states)
                        return response

                    if time.monotonic() >= deadline:
                        response.success = False
                        if last_states is None:
                            detail = (
                                f"; last result: {last_reason}"
                                if last_reason
                                else "; no image was processed"
                            )
                            response.message = (
                                "LED target detection timed out after "
                                f"{timeout_sec:g} seconds without "
                                f"a complete LED detection{detail}"
                            )
                        else:
                            target_text = self._format_states(target_states)
                            states_text = self._format_states(last_states)
                            detail = (
                                f", last_result={last_reason}"
                                if last_reason
                                else ""
                            )
                            response.message = (
                                "LED target detection timed out after "
                                f"{timeout_sec:g} seconds: "
                                f"target={target_text}, "
                                f"last={states_text}, "
                                f"stable={stable_count}/"
                                f"{required_frames}{detail}"
                            )
                        response.led_states = (
                            list(last_states)
                            if last_states is not None
                            else []
                        )
                        self.get_logger().warn(response.message)
                        return response

                response.success = False
                response.message = "LED target detection aborted: ROS shutdown"
                response.led_states = (
                    list(last_states) if last_states is not None else []
                )
                return response
            finally:
                with self._state_condition:
                    self._service_active = False
                    self._tracker = None
                    self._service_last_valid_states = None
                    self._service_last_reason = ""
                    self._state_condition.notify_all()
                self._sync_image_subscription()

    @staticmethod
    def _format_states(states) -> str:
        return "".join("1" if state else "0" for state in states)

    @staticmethod
    def _make_result_message(
        source: CameraFrame,
        result: LedDetectionResult,
    ) -> LedDetection:
        message = LedDetection()
        try:
            message.header = camera_frame_header(source)
        except ValueError as exc:
            message.valid = False
            message.led_states = []
            message.reason = f"failed to read source frame header: {exc}"
            return message

        message.valid = result.valid
        message.led_states = list(result.states) if result.valid else []
        message.reason = "" if result.valid else result.reason
        return message

    @staticmethod
    def _make_visualization(
        image: np.ndarray,
        result: LedDetectionResult,
        target: tuple[bool, ...] | None,
        match_count: int,
        required_frames: int,
    ) -> np.ndarray:
        visualization = image.copy()

        if result.marker is not None:
            corners = np.asarray(result.marker.corners, dtype=np.int32)
            cv2.polylines(
                visualization,
                [corners],
                isClosed=True,
                color=(255, 180, 0),
                thickness=2,
            )
            cv2.putText(
                visualization,
                f"ArUco {result.marker.marker_id}",
                tuple(corners[0]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 180, 0),
                2,
                cv2.LINE_AA,
            )

        for roi, brightness, state in zip(
            result.rois, result.brightness, result.states
        ):
            color = (0, 255, 0) if state else (0, 0, 255)
            cv2.circle(
                visualization,
                (roi.x_px, roi.y_px),
                roi.radius_px,
                color,
                2,
            )
            cv2.putText(
                visualization,
                f"LED{roi.index} {'ON' if state else 'OFF'} "
                f"{brightness:.1f}",
                (roi.x_px + roi.radius_px + 4, roi.y_px),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
                cv2.LINE_AA,
            )

        if target is None:
            status = "visualization active"
        else:
            target_text = "".join("1" if state else "0" for state in target)
            status = (
                f"target={target_text} "
                f"stable={match_count}/{required_frames}"
            )
        if not result.valid:
            status = f"{status} | {result.reason}"
        LedDetectNode._put_status_text(visualization, status)
        return visualization

    @staticmethod
    def _put_status_text(image: np.ndarray, text: str) -> None:
        cv2.putText(
            image,
            text,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            text,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LedDetectNode()
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


if __name__ == "__main__":
    main()
