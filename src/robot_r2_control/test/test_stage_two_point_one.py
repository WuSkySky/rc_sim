from types import SimpleNamespace

import pytest

from robot_r2_control.stage_two_point_one import StageTwoPointOneController
from robot_r2_interfaces.srv import MoveToPose


class ImmediateFuture:
    def __init__(self, response):
        self.response = response

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self.response


class FakeClient:
    def __init__(self, success=True):
        self.requests = []
        self.success = success

    def call_async(self, request):
        self.requests.append(request)
        return ImmediateFuture(SimpleNamespace(
            success=self.success,
            message='ok' if self.success else 'failed',
        ))


def make_controller():
    controller = StageTwoPointOneController.__new__(
        StageTwoPointOneController)
    controller.move_client = FakeClient()
    controller.move_timeout_sec = 35.0
    return controller


def test_move_to_pose_uses_odin_source_and_absolute_values():
    controller = make_controller()

    controller.move_to_pose((1.5, -2.0, 3.141592653589793))

    request = controller.move_client.requests[0]
    assert request.pose_source == MoveToPose.Request.ODIN
    assert request.x == pytest.approx(1.5)
    assert request.y == pytest.approx(-2.0)
    assert request.yaw == pytest.approx(3.141592653589793)
    assert request.position_tolerance == pytest.approx(0.0)
    assert request.yaw_tolerance == pytest.approx(0.0)
    assert request.timeout_sec == pytest.approx(35.0)


def test_move_to_pose_failure_raises():
    controller = make_controller()
    controller.move_client = FakeClient(success=False)

    with pytest.raises(RuntimeError, match='MoveToPose failed'):
        controller.move_to_pose((0.0, 0.0, 0.0))
