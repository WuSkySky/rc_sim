"""Utilities for the bounded CameraFrame transport."""

from __future__ import annotations

import cv2
import numpy as np
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from robot_r2_interfaces.msg import CameraFrame
from sensor_msgs.msg import Image
from std_msgs.msg import Header


def camera_qos() -> QoSProfile:
    """Return the shared real-time image QoS."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def frame_id_text(message: CameraFrame) -> str:
    """Decode and validate a CameraFrame frame_id."""
    size = int(message.frame_id_size)
    if size > CameraFrame.FRAME_ID_CAPACITY:
        raise ValueError(
            f"frame_id_size {size} exceeds "
            f"{CameraFrame.FRAME_ID_CAPACITY}"
        )
    try:
        return bytes(message.frame_id[:size]).decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError("frame_id is not valid UTF-8") from exc


def camera_frame_header(message: CameraFrame) -> Header:
    """Build a standard Header for small results and debug images."""
    if int(message.stamp_nanosec) >= 1_000_000_000:
        raise ValueError("stamp_nanosec must be less than 1000000000")
    header = Header()
    header.stamp.sec = int(message.stamp_sec)
    header.stamp.nanosec = int(message.stamp_nanosec)
    header.frame_id = frame_id_text(message)
    return header


def camera_frame_to_bgr(message: CameraFrame) -> np.ndarray:
    """Validate a CameraFrame and create a BGR OpenCV view/conversion."""
    if int(message.layout_version) != CameraFrame.LAYOUT_VERSION:
        raise ValueError(
            f"unsupported CameraFrame layout {message.layout_version}"
        )
    if int(message.is_bigendian) not in (0, 1):
        raise ValueError("is_bigendian must be 0 or 1")
    width = int(message.width)
    height = int(message.height)
    step = int(message.step)
    data_size = int(message.data_size)
    if width <= 0 or height <= 0:
        raise ValueError("image height and width must be positive")

    if message.encoding in (
        CameraFrame.ENCODING_BGR8,
        CameraFrame.ENCODING_RGB8,
    ):
        channels = 3
    elif message.encoding == CameraFrame.ENCODING_MONO8:
        channels = 1
    else:
        raise ValueError(
            f"unsupported CameraFrame encoding {message.encoding}"
        )

    row_size = width * channels
    if step < row_size:
        raise ValueError(
            f"image step {step} is smaller than row size {row_size}"
        )
    expected_size = height * step
    if data_size != expected_size:
        raise ValueError(
            f"data_size {data_size} does not equal height * step "
            f"({expected_size})"
        )
    if data_size > CameraFrame.DATA_CAPACITY:
        raise ValueError(
            f"data_size {data_size} exceeds CameraFrame capacity "
            f"{CameraFrame.DATA_CAPACITY}"
        )
    if len(message.data) != data_size:
        raise ValueError(
            f"image data has {len(message.data)} bytes, expected {data_size}"
        )
    camera_frame_header(message)

    rows = np.frombuffer(
        message.data,
        dtype=np.uint8,
        count=data_size,
    ).reshape(height, step)
    if channels == 1:
        mono = rows[:, :row_size].reshape(height, width)
        return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

    image = rows[:, :row_size].reshape(height, width, channels)
    if message.encoding == CameraFrame.ENCODING_RGB8:
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image


def fill_bgr_camera_frame(
    target: CameraFrame,
    image: np.ndarray,
    source: CameraFrame,
) -> None:
    """Fill a reusable CameraFrame with a packed BGR image."""
    packed = np.ascontiguousarray(image, dtype=np.uint8)
    if packed.ndim != 3 or packed.shape[2] != 3:
        raise ValueError("BGR image must have shape (height, width, 3)")
    height, width = packed.shape[:2]
    data_size = int(packed.nbytes)
    if width <= 0 or height <= 0:
        raise ValueError("image height and width must be positive")
    if data_size > CameraFrame.DATA_CAPACITY:
        raise ValueError(
            f"image has {data_size} bytes, capacity is "
            f"{CameraFrame.DATA_CAPACITY}"
        )

    target.sequence = int(source.sequence)
    target.stamp_sec = int(source.stamp_sec)
    target.stamp_nanosec = int(source.stamp_nanosec)
    target.width = width
    target.height = height
    target.step = width * 3
    target.data_size = data_size
    target.encoding = CameraFrame.ENCODING_BGR8
    target.is_bigendian = 0
    target.layout_version = CameraFrame.LAYOUT_VERSION
    target.frame_id_size = int(source.frame_id_size)
    target.frame_id[:] = source.frame_id
    del target.data[:]
    target.data.frombytes(memoryview(packed).cast("B"))


def clear_camera_frame(
    target: CameraFrame,
    source: CameraFrame,
) -> None:
    """Reset a reusable derived frame while preserving source identity."""
    target.sequence = int(source.sequence)
    target.stamp_sec = int(source.stamp_sec)
    target.stamp_nanosec = int(source.stamp_nanosec)
    target.width = 0
    target.height = 0
    target.step = 0
    target.data_size = 0
    target.encoding = CameraFrame.ENCODING_BGR8
    target.is_bigendian = 0
    target.layout_version = CameraFrame.LAYOUT_VERSION
    target.frame_id_size = int(source.frame_id_size)
    target.frame_id[:] = source.frame_id
    del target.data[:]


def bgr_to_image_message(image: np.ndarray, header: Header) -> Image:
    """Convert BGR pixels to a standard debug Image."""
    packed = np.ascontiguousarray(image, dtype=np.uint8)
    message = Image()
    message.header = header
    message.height = packed.shape[0]
    message.width = packed.shape[1]
    message.encoding = "bgr8"
    message.is_bigendian = 0
    message.step = packed.shape[1] * 3
    message.data = packed.tobytes()
    return message


def camera_frame_to_image(message: CameraFrame) -> Image:
    """Convert a valid CameraFrame into a standard BGR debug Image."""
    return bgr_to_image_message(
        camera_frame_to_bgr(message),
        camera_frame_header(message),
    )
