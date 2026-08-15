from robot_r2_target_alignment.detector_core import (
    DetectionCandidate,
    inference_wait_seconds,
    parse_id_filter,
    parse_name_filter,
    select_target,
)


def candidate(class_id, name, confidence, center_x, center_y=50.0):
    return DetectionCandidate(
        class_id=class_id,
        class_name=name,
        confidence=confidence,
        x1=center_x - 10.0,
        y1=center_y - 10.0,
        x2=center_x + 10.0,
        y2=center_y + 10.0,
    )


def test_filters_are_parsed():
    assert parse_name_filter("person, ball") == ("person", "ball")
    assert parse_id_filter("0, 3") == (0, 3)


def test_first_inference_starts_without_waiting():
    assert inference_wait_seconds(15.0, None, 100.0) == 0.0


def test_inference_rate_is_limited_start_to_start():
    wait_seconds = inference_wait_seconds(10.0, 100.0, 100.04)
    assert abs(wait_seconds - 0.06) < 1e-9


def test_slow_inference_does_not_add_an_extra_period():
    assert inference_wait_seconds(15.0, 100.0, 100.08) == 0.0


def test_highest_confidence_allowed_target_is_selected():
    detections = [
        candidate(0, "person", 0.9, 20.0),
        candidate(1, "ball", 0.8, 80.0),
    ]
    selected = select_target(
        detections,
        ("ball",),
        (),
        None,
        100,
        100,
        0.2,
        0.15,
        "confidence",
    )
    assert selected is detections[1]


def test_center_mode_selects_target_closest_to_alignment_point():
    left = candidate(0, "target", 0.95, 20.0)
    centered = candidate(0, "target", 0.60, 48.0)
    selected = select_target(
        [left, centered],
        (),
        (),
        None,
        100,
        100,
        0.2,
        0.15,
        "center",
    )
    assert selected is centered


def test_tracking_hysteresis_keeps_nearby_previous_target():
    previous = candidate(0, "person", 0.8, 20.0)
    nearby = candidate(0, "person", 0.80, 22.0)
    challenger = candidate(0, "person", 0.90, 80.0)
    selected = select_target(
        [nearby, challenger],
        (),
        (),
        previous,
        100,
        100,
        0.2,
        0.15,
        "confidence",
    )
    assert selected is nearby
