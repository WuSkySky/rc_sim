"""Tests for the ROS-independent LED detection logic."""

import numpy as np

from robot_r2_detect.led_detection import (
    AprilTagLedMapper,
    LedStateDetector,
    TagDetection,
    TargetMatchTracker,
)


TAG_CORNERS = (
    (0.0, 0.0),
    (100.0, 0.0),
    (100.0, 100.0),
    (0.0, 100.0),
)


class FakeTagDetector:
    def __init__(self, detections):
        self._detections = detections

    def detect(self, _image):
        return list(self._detections)


def make_tag(tag_id=0, decision_margin=10.0):
    return TagDetection(
        tag_id=tag_id,
        corners=TAG_CORNERS,
        center=(50.0, 50.0),
        decision_margin=decision_margin,
    )


def test_mapper_projects_black_border_relative_positions():
    mapper = AprilTagLedMapper(
        tag_size_mm=80.0,
        led_positions_mm=[(10.0, 10.0)],
        led_radius_mm=2.0,
    )

    rois, homography, reason = mapper.map_rois(
        TAG_CORNERS, (100, 100)
    )

    assert homography is not None
    assert reason == ""
    assert len(rois) == 1
    assert (rois[0].x_px, rois[0].y_px) == (20, 20)
    assert rois[0].radius_px == 2


def test_mapper_rejects_roi_crossing_image_boundary():
    mapper = AprilTagLedMapper(
        tag_size_mm=80.0,
        led_positions_mm=[(-10.0, -10.0)],
        led_radius_mm=5.0,
    )

    rois, homography, reason = mapper.map_rois(
        TAG_CORNERS, (100, 100)
    )

    assert homography is not None
    assert rois == ()
    assert "not fully inside" in reason


def test_detector_returns_each_led_state_in_configured_order():
    mapper = AprilTagLedMapper(
        tag_size_mm=80.0,
        led_positions_mm=[(10.0, 10.0), (30.0, 10.0)],
        led_radius_mm=2.0,
    )
    detector = LedStateDetector(
        detector=FakeTagDetector([make_tag()]),
        mapper=mapper,
        target_tag_id=0,
        brightness_threshold=120.0,
    )
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    image[18:23, 18:23] = 255

    result = detector.detect(image)

    assert result.valid
    assert result.states == (True, False)
    assert result.brightness[0] == 255.0
    assert result.brightness[1] == 0.0


def test_detector_selects_highest_margin_matching_tag():
    mapper = AprilTagLedMapper(
        tag_size_mm=80.0,
        led_positions_mm=[(10.0, 10.0)],
        led_radius_mm=2.0,
    )
    low_margin = make_tag(tag_id=0, decision_margin=1.0)
    high_margin = make_tag(tag_id=0, decision_margin=20.0)
    detector = LedStateDetector(
        detector=FakeTagDetector([low_margin, high_margin]),
        mapper=mapper,
        target_tag_id=0,
        brightness_threshold=120.0,
    )

    result = detector.detect(
        np.zeros((100, 100, 3), dtype=np.uint8)
    )

    assert result.valid
    assert result.tag == high_margin


def test_target_match_requires_three_consecutive_valid_frames():
    tracker = TargetMatchTracker([True, False], required_frames=3)

    assert not tracker.update([True, False])
    assert not tracker.update([True, False])
    assert not tracker.update([False, False])
    assert tracker.count == 0
    assert not tracker.update([True, False])
    assert not tracker.update([True, False])
    assert tracker.update([True, False])
    assert tracker.count == 3


def test_invalid_detection_resets_target_match_count():
    tracker = TargetMatchTracker([False], required_frames=3)

    tracker.update([False])
    tracker.update([False])
    assert not tracker.update(None)
    assert tracker.count == 0
