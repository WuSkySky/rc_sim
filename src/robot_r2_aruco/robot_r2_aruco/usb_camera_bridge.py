#!/usr/bin/env python3
"""Bridge a USB camera (cv2.VideoCapture) into the CameraFrame transport.

Publishes CameraFrame images and an estimated CameraInfo so that
downstream ArUco pose estimation can run without a calibration file.
"""

from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from robot_r2_interfaces.msg import CameraFrame
from sensor_msgs.msg import CameraInfo

from robot_r2_detect.camera_frame import camera_qos, fill_bgr_camera_frame


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
        self.declare_parameter("frame_id", "usb_camera")
        self.declare_parameter("target_fps", 30.0)

        device = self.get_parameter("device").value
        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        frame_id = self.get_parameter("frame_id").value
        target_fps = self.get_parameter("target_fps").value

        self._period = 1.0 / float(target_fps)

        self._cap = cv2.VideoCapture(int(device))
        if not self._cap.isOpened():
            raise RuntimeError(f"cannot open video device {device}")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
        self._cap.set(cv2.CAP_PROP_FPS, float(target_fps))
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f"camera opened: {actual_w}x{actual_h}"
        )

        self._sequence = 0
        self._frame_id_bytes = list(frame_id.encode("utf-8"))
        if len(self._frame_id_bytes) > CameraFrame.FRAME_ID_CAPACITY:
            raise ValueError(
                f"frame_id '{frame_id}' exceeds {CameraFrame.FRAME_ID_CAPACITY} bytes"
            )

        image_qos = camera_qos()
        self._image_pub = self.create_publisher(
            CameraFrame, "image_raw", image_qos,
        )
        self._camera_info_pub = self.create_publisher(
            CameraInfo, "camera_info", image_qos,
        )

        camera_info = _estimate_camera_info(actual_w, actual_h, frame_id)
        camera_info.header.stamp = self.get_clock().now().to_msg()
        self._camera_info_pub.publish(camera_info)
        self.get_logger().info("published estimated camera_info")

        self._timer = self.create_timer(self._period, self._capture)
        self._last_camera_info_time = time.monotonic()

    def _capture(self) -> None:
        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().warn("failed to read camera frame")
            return

        bgr = np.ascontiguousarray(frame, dtype=np.uint8)

        now = self.get_clock().now()
        sec = now.nanoseconds // 1_000_000_000
        nsec = now.nanoseconds % 1_000_000_000

        msg = CameraFrame()
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
            camera_info = _estimate_camera_info(
                bgr.shape[1], bgr.shape[0],
                self.get_parameter("frame_id").value,
            )
            camera_info.header.stamp = now.to_msg()
            self._camera_info_pub.publish(camera_info)
            self._last_camera_info_time = time.monotonic()

    def destroy_node(self) -> None:
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
