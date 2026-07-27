#!/usr/bin/env python3
"""Publish a KFS ROI and center offset from the front camera."""

from __future__ import annotations

import math
import time

import cv2
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from robot_r2_interfaces.msg import KfsRoiDetection
from sensor_msgs.msg import Image

from robot_r2_detect.kfs_roi_detection import (
    KfsRoiResult,
    extract_kfs_roi,
)


def image_message_to_bgr(msg: Image) -> np.ndarray:
    """Convert a ROS image message into contiguous BGR.

    Supported encodings: rgb8, bgr8, yuv422_yuy2.
    """
    encoding = msg.encoding.lower()
    if msg.height <= 0 or msg.width <= 0:
        raise ValueError("image height and width must be positive")

    expected_size = int(msg.height) * int(msg.step)
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if data.size != expected_size:
        raise ValueError(
            f"image data has {data.size} bytes, expected {expected_size}"
        )

    if encoding in ("rgb8", "bgr8"):
        row_size = int(msg.width) * 3
        if msg.step < row_size:
            raise ValueError(
                f"image step {msg.step} is smaller than row size {row_size}"
            )
        rows = data.reshape(int(msg.height), int(msg.step))
        image = rows[:, :row_size].reshape(
            int(msg.height),
            int(msg.width),
            3,
        )
        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(image)

    if encoding == "yuv422_yuy2":
        row_size = int(msg.width) * 2
        if msg.step < row_size:
            raise ValueError(
                f"image step {msg.step} is smaller than row size {row_size}"
            )
        rows = data.reshape(int(msg.height), int(msg.step))
        yuv = rows[:, :row_size].reshape(
            int(msg.height), int(msg.width), 2
        )
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUYV)

    raise ValueError(f"unsupported image encoding: {msg.encoding}")


def bgr_to_image_message(image: np.ndarray, header) -> Image:
    """Convert a BGR array into a ROS image with the supplied header."""
    image = np.ascontiguousarray(image, dtype=np.uint8)
    message = Image()
    message.header = header
    message.height = image.shape[0]
    message.width = image.shape[1]
    message.encoding = "bgr8"
    message.is_bigendian = 0
    message.step = image.shape[1] * 3
    message.data = image.tobytes()
    return message


def empty_image_message(header) -> Image:
    message = Image()
    message.header = header
    message.height = 0
    message.width = 0
    message.encoding = "bgr8"
    message.is_bigendian = 0
    message.step = 0
    message.data = b""
    return message


class KfsRoiNode(Node):
    """Own all KFS image processing used by alignment and classification."""

    def __init__(self) -> None:
        super().__init__("kfs_roi")
        self._declare_parameters()
        self._load_parameters()

        self._publisher = self.create_publisher(
            KfsRoiDetection,
            self._roi_topic,
            1,
        )
        self._visualization_publisher = self.create_publisher(
            Image,
            self._visualization_topic,
            1,
        )
        self._subscription = self.create_subscription(
            Image,
            self._color_topic,
            self._on_image,
            1,
        )
        self.add_on_set_parameters_callback(self._on_parameters_changed)

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "color_topic",
            "/r2/front_camera/image_raw",
        )
        self.declare_parameter("roi_topic", "/r2/kfs/roi")
        self.declare_parameter(
            "visualization_topic",
            "/r2/alignment/viz",
        )
        self.declare_parameter("target_processing_rate", 30.0)
        self.declare_parameter("visualization_enabled", False)
        self.declare_parameter("blue_hsv_lower", [90, 80, 60])
        self.declare_parameter("blue_hsv_upper", [130, 255, 255])
        self.declare_parameter("red_low_hsv_lower", [0, 80, 60])
        self.declare_parameter("red_low_hsv_upper", [10, 255, 255])
        self.declare_parameter("red_high_hsv_lower", [170, 80, 60])
        self.declare_parameter("red_high_hsv_upper", [179, 255, 255])
        self.declare_parameter("column_threshold_ratio", 0.8)

    def _load_parameters(self) -> None:
        self._color_topic = str(
            self.get_parameter("color_topic").value
        )
        self._roi_topic = str(self.get_parameter("roi_topic").value)
        self._visualization_topic = str(
            self.get_parameter("visualization_topic").value
        )
        target_rate = float(
            self.get_parameter("target_processing_rate").value
        )
        if not math.isfinite(target_rate) or target_rate <= 0.0:
            raise ValueError(
                "target_processing_rate must be finite and positive"
            )
        self._processing_deadline_sec = 1.0 / target_rate
        self._target_processing_rate = target_rate
        self._visualization_enabled = bool(
            self.get_parameter("visualization_enabled").value
        )

        self._blue_lower = self._hsv_parameter("blue_hsv_lower")
        self._blue_upper = self._hsv_parameter("blue_hsv_upper")
        self._red_low_lower = self._hsv_parameter(
            "red_low_hsv_lower"
        )
        self._red_low_upper = self._hsv_parameter(
            "red_low_hsv_upper"
        )
        self._red_high_lower = self._hsv_parameter(
            "red_high_hsv_lower"
        )
        self._red_high_upper = self._hsv_parameter(
            "red_high_hsv_upper"
        )
        self._validate_hsv_range(
            "blue_hsv",
            self._blue_lower,
            self._blue_upper,
        )
        self._validate_hsv_range(
            "red_low_hsv",
            self._red_low_lower,
            self._red_low_upper,
        )
        self._validate_hsv_range(
            "red_high_hsv",
            self._red_high_lower,
            self._red_high_upper,
        )
        self._column_threshold_ratio = float(
            self.get_parameter("column_threshold_ratio").value
        )
        if (
            not math.isfinite(self._column_threshold_ratio)
            or not 0.0 < self._column_threshold_ratio <= 1.0
        ):
            raise ValueError(
                "column_threshold_ratio must be finite and in (0, 1]"
            )

    def _hsv_parameter(self, name: str) -> np.ndarray:
        values = tuple(
            int(value) for value in self.get_parameter(name).value
        )
        if len(values) != 3:
            raise ValueError(f"{name} must contain [h, s, v]")
        hue, saturation, value = values
        if not 0 <= hue <= 179:
            raise ValueError(f"{name} hue must be in [0, 179]")
        if not 0 <= saturation <= 255 or not 0 <= value <= 255:
            raise ValueError(
                f"{name} saturation and value must be in [0, 255]"
            )
        return np.asarray(values, dtype=np.uint8)

    @staticmethod
    def _validate_hsv_range(
        name: str,
        lower: np.ndarray,
        upper: np.ndarray,
    ) -> None:
        if np.any(lower > upper):
            raise ValueError(
                f"{name} lower bounds must not exceed upper bounds"
            )

    def _on_parameters_changed(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name != "visualization_enabled":
                continue
            if not isinstance(parameter.value, bool):
                return SetParametersResult(
                    successful=False,
                    reason="visualization_enabled must be a boolean",
                )
            self._visualization_enabled = parameter.value
            state = "enabled" if parameter.value else "disabled"
            self.get_logger().info(
                f"KFS ROI visualization {state}"
            )
        return SetParametersResult(successful=True)

    def _on_image(self, msg: Image) -> None:
        started_at = time.monotonic()
        try:
            image = image_message_to_bgr(msg)
            result = extract_kfs_roi(
                image,
                self._blue_lower,
                self._blue_upper,
                self._red_low_lower,
                self._red_low_upper,
                self._red_high_lower,
                self._red_high_upper,
                self._column_threshold_ratio,
            )
        except (cv2.error, TypeError, ValueError) as exc:
            self.get_logger().error(f"Failed to extract KFS ROI: {exc}")
            return

        self._publisher.publish(
            self._make_roi_message(msg, image, result)
        )
        if self._visualization_enabled:
            visualization = self._make_visualization(image, result)
            if self._visualization_enabled:
                self._visualization_publisher.publish(
                    bgr_to_image_message(visualization, msg.header)
                )

        processing_time = time.monotonic() - started_at
        if processing_time > self._processing_deadline_sec:
            self.get_logger().warn(
                "KFS ROI processing overrun: "
                f"{processing_time * 1000.0:.2f} ms > "
                f"{self._processing_deadline_sec * 1000.0:.2f} ms "
                f"(target {self._target_processing_rate:g} Hz)"
            )

    @staticmethod
    def _make_roi_message(
        source: Image,
        image: np.ndarray,
        result: KfsRoiResult,
    ) -> KfsRoiDetection:
        message = KfsRoiDetection()
        message.header = source.header
        message.valid = result.valid
        message.image_width = image.shape[1]
        message.image_height = image.shape[0]
        if not result.valid or result.roi is None:
            message.roi = empty_image_message(source.header)
            return message

        message.roi = bgr_to_image_message(result.roi, source.header)
        message.x1 = result.x1
        message.y1 = result.y1
        message.x2 = result.x2
        message.y2 = result.y2
        message.center_u = result.center_u
        message.center_v = result.center_v
        message.center_offset_x = result.center_offset_x
        message.center_offset_y = result.center_offset_y
        return message

    @staticmethod
    def _make_visualization(
        image: np.ndarray,
        result: KfsRoiResult,
    ) -> np.ndarray:
        source_view = image.copy()
        mask_view = cv2.cvtColor(result.mask, cv2.COLOR_GRAY2BGR)
        if result.valid:
            for view in (source_view, mask_view):
                cv2.rectangle(
                    view,
                    (result.x1, result.y1),
                    (result.x2, result.y2),
                    (0, 255, 0),
                    2,
                )
                cv2.drawMarker(
                    view,
                    (result.center_u, result.center_v),
                    (0, 0, 255),
                    cv2.MARKER_CROSS,
                    18,
                    2,
                )
            status = (
                f"offset=({result.center_offset_x:+d},"
                f"{result.center_offset_y:+d}) px"
            )
        else:
            status = "ROI not found"

        cv2.putText(
            source_view,
            status,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            source_view,
            status,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return np.hstack((source_view, mask_view))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = KfsRoiNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
