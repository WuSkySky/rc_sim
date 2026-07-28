import array

import numpy as np
import pytest
from robot_r2_interfaces.msg import CameraFrame

from robot_r2_detect.camera_frame import (
    camera_frame_header,
    camera_frame_to_bgr,
    clear_camera_frame,
    fill_bgr_camera_frame,
)


def make_frame(
    *,
    width=2,
    height=2,
    step=8,
    encoding=CameraFrame.ENCODING_BGR8,
) -> CameraFrame:
    message = CameraFrame()
    message.sequence = 7
    message.stamp_sec = 12
    message.stamp_nanosec = 34
    message.width = width
    message.height = height
    message.step = step
    message.data_size = height * step
    message.encoding = encoding
    message.layout_version = CameraFrame.LAYOUT_VERSION
    frame_id = b"camera"
    message.frame_id_size = len(frame_id)
    message.frame_id[:len(frame_id)] = np.frombuffer(
        frame_id, dtype=np.uint8)
    message.data = array.array("B", range(message.data_size))
    return message


def test_camera_frame_view_ignores_row_padding():
    message = make_frame()

    image = camera_frame_to_bgr(message)

    assert image.shape == (2, 2, 3)
    assert image[0].reshape(-1).tolist() == list(range(6))
    assert image[1].reshape(-1).tolist() == list(range(8, 14))
    assert camera_frame_header(message).frame_id == "camera"


def test_rgb_frame_is_converted_to_bgr():
    message = make_frame(width=1, height=1, step=3)
    message.encoding = CameraFrame.ENCODING_RGB8
    message.data_size = 3
    message.data = array.array("B", [1, 2, 3])

    image = camera_frame_to_bgr(message)

    assert image[0, 0].tolist() == [3, 2, 1]


def test_camera_frame_rejects_inconsistent_data_size():
    message = make_frame()
    message.data_size -= 1

    with pytest.raises(ValueError, match="height \\* step"):
        camera_frame_to_bgr(message)


def test_derived_frame_preserves_source_identity():
    source = make_frame()
    target = CameraFrame()
    roi = np.full((3, 4, 3), 42, dtype=np.uint8)

    fill_bgr_camera_frame(target, roi, source)

    assert target.sequence == source.sequence
    assert target.stamp_sec == source.stamp_sec
    assert target.stamp_nanosec == source.stamp_nanosec
    assert (target.width, target.height, target.step) == (4, 3, 12)
    assert len(target.data) == target.data_size == 36

    clear_camera_frame(target, source)
    assert target.sequence == source.sequence
    assert (target.width, target.height, target.data_size) == (0, 0, 0)
    assert len(target.data) == 0
