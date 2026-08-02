from robot_r2_detect.kfs_detect_resnet import _processing_overrun_message


def test_processing_at_deadline_does_not_warn():
    processing_config = (30.0, 1.0 / 30.0)

    assert _processing_overrun_message(
        processing_config[1], processing_config
    ) is None


def test_processing_over_deadline_reports_rate_and_durations():
    processing_config = (30.0, 1.0 / 30.0)

    message = _processing_overrun_message(0.04, processing_config)

    assert message == (
        "KFS detection image processing overrun: "
        "40.00 ms > 33.33 ms (target 30 Hz)"
    )
