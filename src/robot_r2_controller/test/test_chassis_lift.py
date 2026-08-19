import threading
from types import SimpleNamespace

import pytest

from robot_r2_controller.chassis_lift import LiftServiceController


class FakeClock:
    class Instant:
        nanoseconds = 0

    def now(self):
        return self.Instant()


class FakePublisher:
    def publish(self, _message):
        pass


class UpdatingCondition:
    def __init__(self, controller):
        self.controller = controller

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        return False

    def wait(self, timeout):
        del timeout
        self.controller.current_front_left_lift = 0.009
        self.controller.current_front_right_lift = 0.009
        self.controller.current_rear_left_lift = 0.009
        self.controller.current_rear_right_lift = 0.009


def test_request_tolerance_overrides_default(monkeypatch):
    controller = LiftServiceController.__new__(LiftServiceController)
    controller.service_lock = threading.Lock()
    controller.default_tolerance = 0.01
    controller.default_timeout_sec = 10.0
    controller.current_front_left_lift = 0.007
    controller.current_front_right_lift = 0.007
    controller.current_rear_left_lift = 0.007
    controller.current_rear_right_lift = 0.007
    controller.command_publisher = FakePublisher()
    controller.get_clock = lambda: FakeClock()
    controller.state_condition = UpdatingCondition(controller)
    monkeypatch.setattr(
        'robot_r2_controller.chassis_lift.rclpy.ok', lambda: True)
    request = SimpleNamespace(
        front_lift=0.01,
        rear_lift=0.01,
        tolerance=0.002,
        timeout_sec=1.0,
    )
    response = SimpleNamespace()

    result = controller.handle_set_lift(request, response)

    assert result.success
    assert result.front_error == pytest.approx(0.001)
