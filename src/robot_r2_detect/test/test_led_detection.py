"""Tests for the ROS-independent LED detection logic."""

import numpy as np

from robot_r2_detect.led_detection import (
    ArucoDetection,
    ArucoLedMapper,
    LedStateDetector,
    TargetMatchTracker,
)


MARKER_CORNERS = (
    (10.0, 10.0),
    (110.0, 10.0),
    (110.0, 110.0),
    (10.0, 110.0),
)


class FakeArucoDetector:
    def __init__(self, detections):
        self._detections = detections

    def detect(self, _image):
        return list(self._detections)


def make_marker(marker_id=4, area_px=10000.0):
    return ArucoDetection(
        marker_id=marker_id,
        corners=MARKER_CORNERS,
        center=(60.0, 60.0),
        area_px=area_px,
    )


def test_mapper_projects_marker_relative_positions():
    mapper = ArucoLedMapper(
        marker_size_mm=100.0,
        led_positions_mm=[(10.0, 10.0)],
        led_radius_mm=2.0,
    )

    rois, homography, reason = mapper.map_rois(
        MARKER_CORNERS, (140, 140)
    )

    assert homography is not None
    assert reason == ""
    assert len(rois) == 1
    assert (rois[0].x_px, rois[0].y_px) == (20, 20)
    assert rois[0].radius_px == 2


def test_mapper_rejects_roi_crossing_image_boundary():
    mapper = ArucoLedMapper(
        marker_size_mm=100.0,
        led_positions_mm=[(-10.0, -10.0)],
        led_radius_mm=5.0,
    )

    rois, homography, reason = mapper.map_rois(
        MARKER_CORNERS, (140, 140)
    )

    assert homography is not None
    assert rois == ()
    assert "not fully inside" in reason


def test_detector_returns_each_led_state_in_configured_order():
    mapper = ArucoLedMapper(
        marker_size_mm=100.0,
        led_positions_mm=[(10.0, 10.0), (30.0, 10.0)],
        led_radius_mm=2.0,
    )
    detector = LedStateDetector(
        detector=FakeArucoDetector([make_marker()]),
        mapper=mapper,
        target_marker_id=4,
        brightness_threshold=120.0,
    )
    image = np.zeros((140, 140, 3), dtype=np.uint8)
    image[18:23, 18:23] = 255

    result = detector.detect(image)

    assert result.valid
    assert result.states == (True, False)
    assert result.brightness[0] == 255.0
    assert result.brightness[1] == 0.0


def test_detector_selects_largest_matching_marker():
    mapper = ArucoLedMapper(
        marker_size_mm=100.0,
        led_positions_mm=[(10.0, 10.0)],
        led_radius_mm=2.0,
    )
    small_marker = make_marker(marker_id=4, area_px=100.0)
    large_marker = make_marker(marker_id=4, area_px=400.0)
    detector = LedStateDetector(
        detector=FakeArucoDetector([small_marker, large_marker]),
        mapper=mapper,
        target_marker_id=4,
        brightness_threshold=120.0,
    )

    result = detector.detect(
        np.zeros((140, 140, 3), dtype=np.uint8)
    )

    assert result.valid
    assert result.marker == large_marker


def test_detector_reports_target_marker_id_mismatch():
    mapper = ArucoLedMapper(
        marker_size_mm=100.0,
        led_positions_mm=[(10.0, 10.0)],
        led_radius_mm=2.0,
    )
    detector = LedStateDetector(
        detector=FakeArucoDetector([make_marker(marker_id=7)]),
        mapper=mapper,
        target_marker_id=4,
        brightness_threshold=120.0,
    )

    result = detector.detect(
        np.zeros((140, 140, 3), dtype=np.uint8)
    )

    assert not result.valid
    assert result.states == ()
    assert "target ArUco marker 4 not found" in result.reason
    assert "[7]" in result.reason


def test_detector_reports_no_marker():
    mapper = ArucoLedMapper(
        marker_size_mm=100.0,
        led_positions_mm=[(10.0, 10.0)],
        led_radius_mm=2.0,
    )
    detector = LedStateDetector(
        detector=FakeArucoDetector([]),
        mapper=mapper,
        target_marker_id=4,
        brightness_threshold=120.0,
    )

    result = detector.detect(
        np.zeros((140, 140, 3), dtype=np.uint8)
    )

    assert not result.valid
    assert result.reason == "no matching ArUco marker detected"


def test_target_match_requires_five_consecutive_valid_frames():
    tracker = TargetMatchTracker([True, False])

    assert not tracker.update([True, False])
    assert not tracker.update([True, False])
    assert not tracker.update([False, False])
    assert tracker.count == 0
    assert not tracker.update([True, False])
    assert not tracker.update([True, False])
    assert not tracker.update([True, False])
    assert not tracker.update([True, False])
    assert tracker.update([True, False])
    assert tracker.count == 5


def test_invalid_detection_resets_target_match_count():
    tracker = TargetMatchTracker([False], required_frames=3)

    tracker.update([False])
    tracker.update([False])
    assert not tracker.update(None)
    assert tracker.count == 0
