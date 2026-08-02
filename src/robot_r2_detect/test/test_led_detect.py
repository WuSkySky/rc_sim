"""Tests for LED detection result message construction."""

import numpy as np

from robot_r2_detect.led_detect import LedDetectNode
from robot_r2_detect.led_detection import LedDetectionResult
from robot_r2_interfaces.msg import CameraFrame


def make_camera_frame() -> CameraFrame:
    frame = CameraFrame()
    frame.sequence = 7
    frame.stamp_sec = 12
    frame.stamp_nanosec = 345
    frame_id = b"left_camera"
    frame.frame_id_size = len(frame_id)
    frame.frame_id[:len(frame_id)] = np.frombuffer(
        frame_id, dtype=np.uint8
    )
    return frame


def test_valid_result_message_contains_states_and_source_header():
    result = LedDetectionResult(
        valid=True,
        states=(True, False),
    )

    message = LedDetectNode._make_result_message(
        make_camera_frame(), result
    )

    assert message.valid
    assert list(message.led_states) == [True, False]
    assert message.reason == ""
    assert message.header.stamp.sec == 12
    assert message.header.stamp.nanosec == 345
    assert message.header.frame_id == "left_camera"


def test_invalid_result_message_contains_reason_and_no_states():
    result = LedDetectionResult(
        valid=False,
        states=(True,),
        reason="no matching ArUco marker detected",
    )

    message = LedDetectNode._make_result_message(
        make_camera_frame(), result
    )

    assert not message.valid
    assert list(message.led_states) == []
    assert message.reason == "no matching ArUco marker detected"


def test_invalid_source_header_becomes_invalid_result():
    frame = make_camera_frame()
    frame.stamp_nanosec = 1_000_000_000
    result = LedDetectionResult(valid=True, states=(True,))

    message = LedDetectNode._make_result_message(frame, result)

    assert not message.valid
    assert list(message.led_states) == []
    assert "failed to read source frame header" in message.reason
