#!/usr/bin/env python3
"""Bridge a USB camera (cv2.VideoCapture) into the CameraFrame transport.

Publishes CameraFrame images and an estimated CameraInfo so that
downstream ArUco pose estimation can run without a calibration file.
"""

from __future__ import annotations

import math
import threading
import time

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile
from robot_r2_interfaces.msg import CameraFrame
from sensor_msgs.msg import CameraInfo

from robot_r2_detect.camera_frame import camera_qos, fill_bgr_camera_frame


_FRAME_ID = "usb_camera_optical_frame"


def _estimate_camera_info(width: int, height: int, frame_id: str) -> CameraInfo:
    """Build a rough CameraInfo from image dimensions alone."""
    f = float(max(width, height))
    cx = float(width) / 2.0
    cy = float(height) / 2.0

    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = width
    msg.height = height
    msg.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
    msg.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.distortion_model = "plumb_bob"
    return msg


class UsbCameraBridge(Node):
    """Capture from a USB camera and publish CameraFrame + CameraInfo."""

    def __init__(self) -> None:
        super().__init__("usb_camera_bridge")

        self.declare_parameter("device", 0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("target_fps", 30.0)

        self._state_lock = threading.Lock()
        configuration = self._validate_configuration({
            name: self.get_parameter(name).value
            for name in ("device", "width", "height", "target_fps")
        })
        self._cap, actual_w, actual_h = self._open_capture(configuration)
        self._configuration = configuration
        self.get_logger().info(
            f"camera opened: {actual_w}x{actual_h}"
        )

        self._sequence = 0
        self._frame_id_bytes = list(_FRAME_ID.encode("utf-8"))
        if len(self._frame_id_bytes) > CameraFrame.FRAME_ID_CAPACITY:
            raise ValueError(
                f"frame_id '{_FRAME_ID}' exceeds "
                f"{CameraFrame.FRAME_ID_CAPACITY} bytes"
            )
        self._image_message = CameraFrame()

        image_qos = camera_qos()
        self._image_pub = self.create_publisher(
            CameraFrame, "image_raw", image_qos,
        )
        self._camera_info_pub = self.create_publisher(
            CameraInfo, "camera_info", QoSProfile(depth=1),
        )

        self._camera_info = _estimate_camera_info(
            actual_w, actual_h, _FRAME_ID
        )
        self._publish_camera_info(self.get_clock().now())
        self.get_logger().info("published estimated camera_info")

        self._timer = self.create_timer(
            1.0 / configuration["target_fps"], self._capture
        )
        self._last_camera_info_time = time.monotonic()
        self.add_on_set_parameters_callback(self._on_parameters_changed)

    @staticmethod
    def _validate_configuration(values: dict) -> dict:
        for name in ("device", "width", "height"):
            if isinstance(values[name], bool) or not isinstance(
                values[name], int
            ):
                raise ValueError(f"{name} must be an integer")
        if values["device"] < 0:
            raise ValueError("device must be non-negative")
        if values["width"] <= 0 or values["height"] <= 0:
            raise ValueError("width and height must be positive")
        target_fps = values["target_fps"]
        if isinstance(target_fps, bool) or not isinstance(
            target_fps, (int, float)
        ):
            raise ValueError("target_fps must be a number")
        target_fps = float(target_fps)
        if not math.isfinite(target_fps) or target_fps <= 0.0:
            raise ValueError("target_fps must be finite and positive")
        result = dict(values)
        result["target_fps"] = target_fps
        return result

    @staticmethod
    def _open_capture(configuration: dict):
        device = configuration["device"]
        capture = cv2.VideoCapture(device)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"cannot open video device {device}")
        width, height = UsbCameraBridge._configure_capture(
            capture, configuration
        )
        return capture, width, height

    @staticmethod
    def _configure_capture(capture, configuration: dict):
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(configuration["width"]))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(configuration["height"]))
        capture.set(cv2.CAP_PROP_FPS, configuration["target_fps"])
        capture.set(
            cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")
        )
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return width, height

    def _on_parameters_changed(self, parameters) -> SetParametersResult:
        with self._state_lock:
            candidate = dict(self._configuration)
        for parameter in parameters:
            if parameter.name not in candidate:
                return SetParametersResult(
                    successful=False,
                    reason=f"unknown parameter '{parameter.name}'",
                )
            candidate[parameter.name] = parameter.value
        try:
            configuration = self._validate_configuration(candidate)
        except (RuntimeError, TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        with self._state_lock:
            current_device = self._configuration["device"]
        new_capture = None
        if configuration["device"] != current_device:
            try:
                new_capture, width, height = self._open_capture(configuration)
            except RuntimeError as exc:
                return SetParametersResult(successful=False, reason=str(exc))
        else:
            with self._state_lock:
                width, height = self._configure_capture(
                    self._cap, configuration
                )

        new_timer = self.create_timer(
            1.0 / configuration["target_fps"], self._capture
        )
        new_camera_info = _estimate_camera_info(width, height, _FRAME_ID)
        with self._state_lock:
            old_capture = self._cap
            old_timer = self._timer
            if new_capture is not None:
                self._cap = new_capture
            self._timer = new_timer
            self._camera_info = new_camera_info
            self._configuration = configuration
        old_timer.cancel()
        self.destroy_timer(old_timer)
        if new_capture is not None:
            old_capture.release()
        self._publish_camera_info(self.get_clock().now())
        self.get_logger().info(
            f"camera configuration updated: {width}x{height} at "
            f"{configuration['target_fps']:g} Hz"
        )
        return SetParametersResult(successful=True)

    def _publish_camera_info(self, stamp) -> None:
        with self._state_lock:
            self._camera_info.header.stamp = stamp.to_msg()
            self._camera_info_pub.publish(self._camera_info)

    def _capture(self) -> None:
        with self._state_lock:
            ret, frame = self._cap.read()
        if not ret:
            self.get_logger().warn("failed to read camera frame")
            return

        bgr = np.ascontiguousarray(frame, dtype=np.uint8)

        now = self.get_clock().now()
        sec = now.nanoseconds // 1_000_000_000
        nsec = now.nanoseconds % 1_000_000_000

        msg = self._image_message
        msg.sequence = self._sequence
        self._sequence += 1
        msg.stamp_sec = int(sec)
        msg.stamp_nanosec = int(nsec)
        msg.layout_version = CameraFrame.LAYOUT_VERSION
        msg.encoding = CameraFrame.ENCODING_BGR8
        msg.is_bigendian = 0
        msg.frame_id_size = len(self._frame_id_bytes)
        msg.frame_id[:msg.frame_id_size] = self._frame_id_bytes

        fill_bgr_camera_frame(msg, bgr, msg)
        self._image_pub.publish(msg)

        if time.monotonic() - self._last_camera_info_time > 5.0:
            self._publish_camera_info(now)
            self._last_camera_info_time = time.monotonic()

    def destroy_node(self) -> None:
        with self._state_lock:
            self._cap.release()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = UsbCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
