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
from robot_r2_interfaces.msg import CameraFrame, KfsRoiDetection
from sensor_msgs.msg import Image

from robot_r2_detect.camera_frame import (
    bgr_to_image_message,
    camera_frame_header,
    camera_frame_to_bgr,
    camera_qos,
    clear_camera_frame,
    fill_bgr_camera_frame,
)
from robot_r2_detect.kfs_roi_detection import (
    KfsRoiResult,
    extract_kfs_roi,
)


class KfsRoiNode(Node):
    """Own all KFS image processing used by alignment and classification."""

    def __init__(self) -> None:
        super().__init__("kfs_roi")
        self._declare_parameters()
        self._load_parameters()
        image_qos = camera_qos()
        self._roi_message = KfsRoiDetection()

        self._publisher = self.create_publisher(
            KfsRoiDetection,
            self._roi_topic,
            image_qos,
        )
        self._visualization_publisher = self.create_publisher(
            Image,
            self._visualization_topic,
            image_qos,
        )
        self._standard_image_publisher = self.create_publisher(
            Image,
            self._standard_image_topic,
            image_qos,
        )
        self._subscription = self.create_subscription(
            CameraFrame,
            self._color_topic,
            self._on_image,
            image_qos,
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
            "/r2/alignment/debug",
        )
        self.declare_parameter(
            "standard_image_topic",
            "/r2/kfs/roi/debug",
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
        self._standard_image_topic = str(
            self.get_parameter("standard_image_topic").value
        )
        if not self._standard_image_topic:
            raise ValueError("standard_image_topic must not be empty")
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

    def _on_image(self, msg: CameraFrame) -> None:
        started_at = time.monotonic()
        try:
            image = camera_frame_to_bgr(msg)
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

        roi_message = self._make_roi_message(msg, image, result)
        self._publisher.publish(roi_message)
        if self._visualization_enabled:
            if result.valid and result.roi is not None:
                standard_image = bgr_to_image_message(
                    result.roi,
                    camera_frame_header(msg),
                )
            else:
                standard_image = Image()
                standard_image.header = camera_frame_header(msg)
                standard_image.encoding = "bgr8"
            self._standard_image_publisher.publish(standard_image)
            visualization = self._make_visualization(image, result)
            self._visualization_publisher.publish(
                bgr_to_image_message(
                    visualization,
                    camera_frame_header(msg),
                )
            )

        processing_time = time.monotonic() - started_at
        if processing_time > self._processing_deadline_sec:
            self.get_logger().warn(
                "KFS ROI processing overrun: "
                f"{processing_time * 1000.0:.2f} ms > "
                f"{self._processing_deadline_sec * 1000.0:.2f} ms "
                f"(target {self._target_processing_rate:g} Hz)"
            )

    def _make_roi_message(
        self,
        source: CameraFrame,
        image: np.ndarray,
        result: KfsRoiResult,
    ) -> KfsRoiDetection:
        message = self._roi_message
        message.valid = result.valid
        message.image_width = image.shape[1]
        message.image_height = image.shape[0]
        if not result.valid or result.roi is None:
            clear_camera_frame(message.roi, source)
            message.x1 = 0
            message.y1 = 0
            message.x2 = 0
            message.y2 = 0
            message.center_u = 0
            message.center_v = 0
            message.center_offset_x = 0
            message.center_offset_y = 0
            return message

        fill_bgr_camera_frame(message.roi, result.roi, source)
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
