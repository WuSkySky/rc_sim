import numpy as np
import pytest

from robot_r2_detect.kfs_detect_yolo import (
    PUBLIC_CLASS_IDS,
    _aggregate_probabilities,
    _processing_overrun_message,
    _public_class_name,
    _select_most_frequent_class,
    _validate_model_class_names,
)


CLASS_NAMES = tuple(
    [f"fake_{index}" for index in range(15)]
    + ["r1"]
    + [f"r2_{index}" for index in range(15)]
)


def test_public_class_mapping():
    assert _public_class_name("fake_14") == "fake"
    assert _public_class_name("r2_0") == "r2"
    assert _public_class_name("r1") == "r1"
    with pytest.raises(ValueError, match="Unsupported"):
        _public_class_name("unknown")


def test_model_class_names_require_expected_groups():
    _validate_model_class_names(CLASS_NAMES)
    with pytest.raises(ValueError, match="31 classes"):
        _validate_model_class_names(CLASS_NAMES[:-1])


def test_probabilities_are_aggregated_into_public_classes():
    probabilities = np.zeros(31, dtype=np.float32)
    probabilities[0] = 0.20
    probabilities[1] = 0.25
    probabilities[15] = 0.10
    probabilities[16] = 0.15
    probabilities[17] = 0.30

    class_id, class_name, confidence, detailed_name = (
        _aggregate_probabilities(probabilities, CLASS_NAMES)
    )

    assert class_id == PUBLIC_CLASS_IDS["r2"]
    assert class_name == "r2"
    assert confidence == pytest.approx(0.45)
    assert detailed_name == "r2_1"


def test_vote_tie_prefers_latest_sample():
    assert _select_most_frequent_class(["r2", "fake"]) == "fake"


def test_processing_overrun_message():
    config = (30.0, 1.0 / 30.0)
    assert _processing_overrun_message(config[1], config) is None
    assert _processing_overrun_message(0.04, config) == (
        "KFS detection ROI processing overrun: "
        "40.00 ms > 33.33 ms (target 30 Hz)"
    )
