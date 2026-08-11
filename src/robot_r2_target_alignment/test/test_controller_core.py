import math

from robot_r2_target_alignment.controller_core import (
    LateralPid,
    PidConfig,
    command_for_mode,
)


def config(**overrides):
    values = {
        "kp": 1.0,
        "ki": 0.0,
        "kd": 0.0,
        "integral_limit": 1.0,
        "output_limit": 0.4,
        "minimum_output": 0.05,
        "invert_output": True,
    }
    values.update(overrides)
    return PidConfig(**values)


def test_positive_image_error_commands_negative_lateral_velocity():
    controller = LateralPid(config())
    assert controller.update(0.2, 0.05) == -0.2


def test_output_is_limited():
    controller = LateralPid(config())
    assert controller.update(1.0, 0.05) == -0.4


def test_minimum_nonzero_output_is_applied():
    controller = LateralPid(config())
    assert controller.update(0.01, 0.05) == -0.05


def test_invalid_timing_returns_zero():
    controller = LateralPid(config())
    assert controller.update(0.2, 0.0) == 0.0
    assert controller.update(0.2, math.nan) == 0.0


def test_test_mode_suppresses_the_command_entirely():
    assert command_for_mode(0.2, test_mode=True) is None
    assert command_for_mode(0.2, test_mode=False) == 0.2
