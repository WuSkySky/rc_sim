import threading
from types import SimpleNamespace

import pytest
from rclpy.parameter import Parameter

from robot_r2_controller.weapon_joint_controller import (
    GRIP_PROFILE,
    ROTATE_PROFILE,
    WeaponJointConfig,
    WeaponJointServiceController,
)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_controller(profile, config):
    controller = WeaponJointServiceController.__new__(
        WeaponJointServiceController)
    controller.profile = profile
    controller.service_lock = threading.Lock()
    controller.state_condition = threading.Condition()
    controller.config_lock = threading.RLock()
    controller.current_position = None
    controller.command_publisher = FakePublisher()
    controller._config = config
    return controller


def rotate_config():
    return WeaponJointConfig(0.0, 3.5, 0.01, 10.0)


def test_request_override_and_default_values_are_resolved():
    controller = make_controller(ROTATE_PROFILE, rotate_config())

    assert controller.validate_request(
        SimpleNamespace(position=1.0, tolerance=0.02, timeout_sec=4.0),
        controller._config,
    ) == pytest.approx((0.02, 4.0))
    assert controller.validate_request(
        SimpleNamespace(position=1.0, tolerance=0.0, timeout_sec=0.0),
        controller._config,
    ) == pytest.approx((0.01, 10.0))


def test_position_outside_joint_limits_is_rejected():
    controller = make_controller(GRIP_PROFILE, WeaponJointConfig(
        0.0, 0.03, 0.001, 10.0))

    with pytest.raises(ValueError, match='between'):
        controller.validate_request(
            SimpleNamespace(position=0.031, tolerance=0.0, timeout_sec=0.0),
            controller._config,
        )


def test_feedback_inside_tolerance_completes_service(monkeypatch):
    controller = make_controller(ROTATE_PROFILE, rotate_config())
    controller.current_position = 1.005
    monkeypatch.setattr(
        'robot_r2_controller.weapon_joint_controller.rclpy.ok',
        lambda: True,
    )
    request = SimpleNamespace(position=1.0, tolerance=0.01, timeout_sec=1.0)
    response = SimpleNamespace()

    result = controller.handle_set_position(request, response)

    assert result.success
    assert result.position_error == pytest.approx(-0.005)
    assert controller.command_publisher.messages[0].data == 1.0


def test_service_reports_timeout_without_feedback(monkeypatch):
    controller = make_controller(GRIP_PROFILE, WeaponJointConfig(
        0.0, 0.03, 0.001, 10.0))
    monkeypatch.setattr(
        'robot_r2_controller.weapon_joint_controller.rclpy.ok',
        lambda: False,
    )
    request = SimpleNamespace(position=0.028, tolerance=0.001, timeout_sec=1.0)
    response = SimpleNamespace()

    result = controller.handle_set_position(request, response)

    assert not result.success
    assert result.position_error == pytest.approx(0.028)
    assert 'timeout' in result.message


def test_dynamic_limits_and_defaults_are_validated_atomically():
    controller = make_controller(ROTATE_PROFILE, rotate_config())
    original = controller.config_snapshot()

    result = controller.on_parameters_changed([
        Parameter('max_position', value=3.6),
        Parameter('default_tolerance', value=0.02),
    ])

    assert result.successful
    assert original.max_position == pytest.approx(3.5)
    assert controller.config_snapshot().max_position == pytest.approx(3.6)
    assert controller.config_snapshot().default_tolerance == pytest.approx(0.02)

    rejected = controller.on_parameters_changed([
        Parameter('min_position', value=4.0),
    ])
    assert not rejected.successful
    assert controller.config_snapshot().max_position == pytest.approx(3.6)
