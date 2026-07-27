#!/usr/bin/env python3
"""ROS 2 node for AprilTag-guided LED target detection."""

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
from robot_r2_interfaces.srv import DetectLed
from sensor_msgs.msg import Image

from .led_detection import (
    ApriltagDetector,
    AprilTagLedMapper,
    LedDetectionResult,
    LedStateDetector,
    TargetMatchTracker,
)


STABLE_MATCH_FRAMES = 3
SERVICE_TIMEOUT_SEC = 60.0


def image_message_to_bgr(message: Image) -> np.ndarray:
    """Convert a ROS image message into contiguous BGR data.

    Supported encodings: rgb8, bgr8, yuv422_yuy2.
    """
    encoding = message.encoding.lower()
    if message.height <= 0 or message.width <= 0:
        raise ValueError("image height and width must be positive")

    expected_size = int(message.height) * int(message.step)
    data = np.frombuffer(message.data, dtype=np.uint8)
    if data.size != expected_size:
        raise ValueError(
            f"image data has {data.size} bytes, expected {expected_size}"
        )

    if encoding in ("rgb8", "bgr8"):
        row_size = int(message.width) * 3
        if message.step < row_size:
            raise ValueError(
                f"image step {message.step} is smaller than row size {row_size}"
            )
        rows = data.reshape(int(message.height), int(message.step))
        image = rows[:, :row_size].reshape(
            int(message.height), int(message.width), 3
        )
        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(image)

    if encoding == "yuv422_yuy2":
        row_size = int(message.width) * 2
        if message.step < row_size:
            raise ValueError(
                f"image step {message.step} is smaller than row size {row_size}"
            )
        rows = data.reshape(int(message.height), int(message.step))
        yuv = rows[:, :row_size].reshape(
            int(message.height), int(message.width), 2
        )
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUYV)

    raise ValueError(f"unsupported image encoding: {message.encoding}")


class LedDetectNode(Node):
    """Detect configurable LEDs and wait for a requested target state."""

    def __init__(self) -> None:
        super().__init__("led_detect")
        self._state_condition = threading.Condition()
        self._service_lock = threading.Lock()
        self._image_callback_group = MutuallyExclusiveCallbackGroup()
        self._service_callback_group = MutuallyExclusiveCallbackGroup()

        self._frame_sequence = 0
        self._service_active = False
        self._service_started_at = 0.0
        self._service_last_valid_states: tuple[bool, ...] | None = None
        self._service_last_reason = ""
        self._tracker: TargetMatchTracker | None = None

        self._declare_parameters()
        self._load_parameters()

        self._image_subscription = self.create_subscription(
            Image,
            self._color_topic,
            self._on_image,
            1,
            callback_group=self._image_callback_group,
        )
        self._visualization_publisher = self.create_publisher(
            Image, self._visualization_topic, 1
        )
        self._service = self.create_service(
            DetectLed,
            self._service_name,
            self._handle_detect_led,
            callback_group=self._service_callback_group,
        )
        self.add_on_set_parameters_callback(self._on_parameters_changed)

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "color_topic", "/r2/front_camera/image_raw"
        )
        self.declare_parameter(
            "service_name", "/r2/led_detection/detect"
        )
        self.declare_parameter(
            "visualization_topic", "/r2/led_detection/viz"
        )
        self.declare_parameter("visualization_enabled", False)
        self.declare_parameter("continuous_detection", False)
        self.declare_parameter("tag_family", "tag36h11")
        self.declare_parameter("target_tag_id", 0)
        self.declare_parameter("tag_size_mm", 100.0)
        self.declare_parameter("led_count", 3)
        self.declare_parameter(
            "led_positions_x_mm", [0.0, 33.333333, 66.666667]
        )
        self.declare_parameter(
            "led_positions_y_mm", [110.0, 110.0, 110.0]
        )
        self.declare_parameter("led_radius_mm", 5.0)
        self.declare_parameter("brightness_threshold", 120.0)
        self.declare_parameter("target_processing_rate", 10.0)

    def _load_parameters(self) -> None:
        self._color_topic = str(
            self.get_parameter("color_topic").value
        )
        self._service_name = str(
            self.get_parameter("service_name").value
        )
        self._visualization_topic = str(
            self.get_parameter("visualization_topic").value
        )
        self._visualization_enabled = bool(
            self.get_parameter("visualization_enabled").value
        )
        self._continuous_detection = bool(
            self.get_parameter("continuous_detection").value
        )
        tag_family = str(self.get_parameter("tag_family").value)
        target_tag_id = int(self.get_parameter("target_tag_id").value)
        tag_size_mm = float(self.get_parameter("tag_size_mm").value)
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

        if not self._color_topic:
            raise ValueError("color_topic must not be empty")
        if not self._service_name:
            raise ValueError("service_name must not be empty")
        if not self._visualization_topic:
            raise ValueError("visualization_topic must not be empty")
        if not tag_family:
            raise ValueError("tag_family must not be empty")
        if target_tag_id < -1:
            raise ValueError("target_tag_id must be -1 or non-negative")
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
            or target_processing_rate <= 0.0
        ):
            raise ValueError(
                "target_processing_rate must be finite and positive"
            )

        mapper = AprilTagLedMapper(
            tag_size_mm=tag_size_mm,
            led_positions_mm=tuple(zip(positions_x, positions_y)),
            led_radius_mm=led_radius_mm,
        )
        self._led_count = led_count
        self._target_processing_rate = target_processing_rate
        self._processing_deadline_sec = 1.0 / target_processing_rate
        self._state_detector = LedStateDetector(
            detector=ApriltagDetector(tag_family),
            mapper=mapper,
            target_tag_id=None if target_tag_id == -1 else target_tag_id,
            brightness_threshold=brightness_threshold,
        )

    def _on_parameters_changed(self, parameters) -> SetParametersResult:
        continuous_detection = None
        visualization_enabled = None
        for parameter in parameters:
            if parameter.name not in (
                "continuous_detection",
                "visualization_enabled",
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
            state = "enabled" if continuous_detection else "disabled"
            self.get_logger().info(
                f"continuous LED detection {state}"
            )
        if visualization_enabled is not None:
            with self._state_condition:
                self._visualization_enabled = visualization_enabled
            state = "enabled" if visualization_enabled else "disabled"
            self.get_logger().info(f"LED visualization {state}")
        return SetParametersResult(successful=True)

    def _on_image(self, message: Image) -> None:
        with self._state_condition:
            should_process = (
                self._continuous_detection or self._service_active
            )
        if not should_process:
            return

        started_at = time.monotonic()
        image: np.ndarray | None = None
        try:
            image = image_message_to_bgr(message)
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
            target = (
                self._tracker.target_states
                if self._tracker is not None
                else None
            )
            match_count = (
                self._tracker.count if self._tracker is not None else 0
            )
            self._state_condition.notify_all()

        if not result.valid:
            self.get_logger().debug(result.reason)

        if visualization_enabled and image is not None:
            visualization = self._make_visualization(
                image, result, target, match_count
            )
            visualization_message = self._bgr_to_image_message(
                visualization, message
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
        target_states = tuple(bool(state) for state in request.target_states)
        if len(target_states) != self._led_count:
            response.success = False
            response.message = (
                f"target_states contains {len(target_states)} states; "
                f"expected {self._led_count}"
            )
            response.led_states = []
            return response

        with self._service_lock:
            started_at = time.monotonic()
            deadline = started_at + SERVICE_TIMEOUT_SEC
            last_states: tuple[bool, ...] | None = None
            last_reason = ""
            stable_count = 0
            with self._state_condition:
                handled_sequence = self._frame_sequence
                self._service_started_at = started_at
                self._service_last_valid_states = None
                self._service_last_reason = ""
                self._tracker = TargetMatchTracker(
                    target_states,
                    required_frames=STABLE_MATCH_FRAMES,
                )
                self._service_active = True

            try:
                while rclpy.ok():
                    with self._state_condition:
                        while (
                            self._frame_sequence <= handled_sequence
                            and rclpy.ok()
                        ):
                            remaining = deadline - time.monotonic()
                            if remaining <= 0.0:
                                break
                            self._state_condition.wait(
                                timeout=min(remaining, 0.1)
                            )

                        if self._frame_sequence > handled_sequence:
                            handled_sequence = self._frame_sequence
                        matched = (
                            self._tracker is not None
                            and self._tracker.count
                            >= STABLE_MATCH_FRAMES
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
                            f"{STABLE_MATCH_FRAMES} consecutive frames"
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
                                f"{SERVICE_TIMEOUT_SEC:g} seconds without "
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
                                f"{SERVICE_TIMEOUT_SEC:g} seconds: "
                                f"target={target_text}, "
                                f"last={states_text}, "
                                f"stable={stable_count}/"
                                f"{STABLE_MATCH_FRAMES}{detail}"
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

    @staticmethod
    def _format_states(states) -> str:
        return "".join("1" if state else "0" for state in states)

    @staticmethod
    def _make_visualization(
        image: np.ndarray,
        result: LedDetectionResult,
        target: tuple[bool, ...] | None,
        match_count: int,
    ) -> np.ndarray:
        visualization = image.copy()

        if result.tag is not None:
            corners = np.asarray(result.tag.corners, dtype=np.int32)
            cv2.polylines(
                visualization,
                [corners],
                isClosed=True,
                color=(255, 180, 0),
                thickness=2,
            )
            cv2.putText(
                visualization,
                f"Tag {result.tag.tag_id}",
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
            status = "continuous detection"
        else:
            target_text = "".join("1" if state else "0" for state in target)
            status = (
                f"target={target_text} "
                f"stable={match_count}/{STABLE_MATCH_FRAMES}"
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

    @staticmethod
    def _bgr_to_image_message(
        image: np.ndarray,
        source: Image,
    ) -> Image:
        image = np.ascontiguousarray(image, dtype=np.uint8)
        message = Image()
        message.header = source.header
        message.height = image.shape[0]
        message.width = image.shape[1]
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = image.shape[1] * 3
        message.data = image.tobytes()
        return message


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
