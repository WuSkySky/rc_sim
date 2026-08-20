from types import SimpleNamespace

import pytest

from robot_r2_control.stage_two_control import StageTwoController
from robot_r2_interfaces.srv import StageTwo, StageTwoPointOne, StageTwoPointTwo


class ImmediateFuture:
    def __init__(self, response):
        self.response = response

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self.response


class FakeClient:
    def __init__(self, loaded_count):
        self.loaded_count = loaded_count
        self.requests = []

    def call_async(self, request):
        self.requests.append(request)
        return ImmediateFuture(SimpleNamespace(
            success=True,
            message='ok',
            loaded_count=self.loaded_count,
        ))


def make_controller():
    controller = StageTwoController.__new__(StageTwoController)
    controller.loaded_count = 0
    controller.point_one_timeout_sec = 10.0
    controller.point_two_timeout_sec = 10.0
    controller.point_one_client = FakeClient(2)
    controller.point_two_client = FakeClient(3)
    return controller


@pytest.mark.parametrize('team', [StageTwo.Request.RED, StageTwo.Request.BLUE])
def test_full_stage_two_forwards_team_to_both_subtasks(team):
    controller = make_controller()

    controller.run_point_one(team)
    controller.run_point_two(team, StageTwo.Request.LEFT)

    point_one_request = controller.point_one_client.requests[0]
    assert isinstance(point_one_request, StageTwoPointOne.Request)
    assert point_one_request.team == team
    assert point_one_request.loaded_count == 0

    point_two_request = controller.point_two_client.requests[0]
    assert isinstance(point_two_request, StageTwoPointTwo.Request)
    assert point_two_request.team == team
    assert point_two_request.fake_kfs_decision == StageTwo.Request.LEFT
    assert point_two_request.loaded_count == 2
    assert controller.loaded_count == 3


def test_full_stage_two_rejects_unknown_team():
    with pytest.raises(ValueError, match='team must be red or blue'):
        StageTwoController.validate_team('green')
