"""Conversion helpers for the repository's bounded CameraFrame message."""

from __future__ import annotations

from array import array

import cv2
import numpy as np
from robot_r2_interfaces.msg import CameraFrame
from sensor_msgs.msg import Image
from std_msgs.msg import Header


def message_header(message: CameraFrame) -> Header:
    """Convert and validate the compact CameraFrame header."""
    if int(message.stamp_nanosec) >= 1_000_000_000:
        raise ValueError("stamp_nanosec must be less than 1000000000")
    frame_id_size = int(message.frame_id_size)
    if frame_id_size > CameraFrame.FRAME_ID_CAPACITY:
        raise ValueError("frame_id_size exceeds CameraFrame capacity")
    try:
        frame_id = bytes(message.frame_id[:frame_id_size]).decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError("frame_id is not valid UTF-8") from exc
    header = Header()
    header.stamp.sec = int(message.stamp_sec)
    header.stamp.nanosec = int(message.stamp_nanosec)
    header.frame_id = frame_id
    return header


def message_to_bgr(message: CameraFrame) -> np.ndarray:
    """Validate a bounded CameraFrame and return a BGR image."""
    if int(message.layout_version) != CameraFrame.LAYOUT_VERSION:
        raise ValueError("unsupported CameraFrame layout version")
    width = int(message.width)
    height = int(message.height)
    step = int(message.step)
    data_size = int(message.data_size)
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    if int(message.is_bigendian) not in (0, 1):
        raise ValueError("is_bigendian must be zero or one")

    if message.encoding in (
        CameraFrame.ENCODING_BGR8,
        CameraFrame.ENCODING_RGB8,
    ):
        channels = 3
    elif message.encoding == CameraFrame.ENCODING_MONO8:
        channels = 1
    else:
        raise ValueError("unsupported CameraFrame encoding")

    row_size = width * channels
    if step < row_size:
        raise ValueError("image step is smaller than its packed row size")
    if data_size != height * step:
        raise ValueError("image data_size does not equal height * step")
    if data_size > CameraFrame.DATA_CAPACITY or len(message.data) != data_size:
        raise ValueError("image data length is invalid")
    message_header(message)

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


def bgr_to_image(image: np.ndarray, header: Header) -> Image:
    """Create a standard sensor_msgs/Image for visualization only."""
    packed = np.ascontiguousarray(image, dtype=np.uint8)
    message = Image()
    message.header = header
    message.height = packed.shape[0]
    message.width = packed.shape[1]
    message.encoding = "bgr8"
    message.is_bigendian = 0
    message.step = packed.shape[1] * 3
    # Humble's generated uint8[] setter validates bytes one element at a time.
    # Supplying its native array type takes the direct assignment fast path.
    message.data = array("B", packed.tobytes())
    return message
