import threading
from types import SimpleNamespace

import pytest

from robot_r2_control.stage_two_control import StageTwoController
from robot_r2_interfaces.msg import CellIndex
from robot_r2_interfaces.srv import StageTwo, StageTwoPointOne, StageTwoPointTwo


class ImmediateFuture:
    def __init__(self, response):
        self.response = response

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self.response


class FakeClient:
    def __init__(self, loaded_count, success=True):
        self.loaded_count = loaded_count
        self.success = success
        self.requests = []
        self.wait_timeouts = []

    def call_async(self, request):
        self.requests.append(request)
        return ImmediateFuture(SimpleNamespace(
            success=self.success,
            message='ok' if self.success else 'failed',
            loaded_count=self.loaded_count,
        ))

    def wait_for_service(self, timeout_sec):
        self.wait_timeouts.append(timeout_sec)
        return True


def cell(forward_index, lateral_index):
    return CellIndex(
        forward_index=forward_index,
        lateral_index=lateral_index,
    )


def indices(messages):
    return [
        (message.forward_index, message.lateral_index)
        for message in messages
    ]


def default_move_cells():
    return [cell(4, 2), cell(3, 2), cell(2, 2), cell(1, 2), cell(1, 1)]


def make_controller():
    controller = StageTwoController.__new__(StageTwoController)
    controller.loaded_count = 0
    controller.dependency_timeout_sec = 2.0
    controller.point_one_timeout_sec = 10.0
    controller.point_two_timeout_sec = 20.0
    controller.point_one_client = FakeClient(2)
    controller.point_two_client = FakeClient(3)
    controller.service_lock = threading.Lock()
    controller.config_lock = threading.Lock()
    return controller


@pytest.mark.parametrize('team', [StageTwo.Request.RED, StageTwo.Request.BLUE])
def test_standard_mode_forwards_team_and_ignores_overall_arrays(team):
    controller = make_controller()
    controller.run_point_one(
        team, StageTwo.Request.STANDARD, (), controller.point_one_timeout_sec)
    controller.run_point_two(
        team,
        StageTwo.Request.LEFT,
        StageTwo.Request.STANDARD,
        (),
        (),
        controller.point_two_timeout_sec,
    )

    point_one_request = controller.point_one_client.requests[0]
    assert isinstance(point_one_request, StageTwoPointOne.Request)
    assert point_one_request.team == team
    assert point_one_request.loaded_count == 0
    assert point_one_request.mode == StageTwoPointOne.Request.STANDARD
    assert list(point_one_request.route_cells) == []

    point_two_request = controller.point_two_client.requests[0]
    assert isinstance(point_two_request, StageTwoPointTwo.Request)
    assert point_two_request.team == team
    assert point_two_request.fake_kfs_decision == StageTwo.Request.LEFT
    assert point_two_request.loaded_count == 2
    assert point_two_request.mode == StageTwoPointTwo.Request.STANDARD
    assert list(point_two_request.move_cells) == []
    assert list(point_two_request.load_cells) == []
    assert controller.loaded_count == 3


@pytest.mark.parametrize('mode', [StageTwo.Request.STANDARD, StageTwo.Request.SKIP])
def test_non_route_modes_silently_ignore_arrays(mode):
    result = StageTwoController.validate_and_split_request(
        mode,
        [cell(99, 99)],
        [cell(99, 99)],
    )

    assert result == ((), (), ())


def test_route_splits_entry_kfs_from_remaining_grid():
    move_cells, point_one_route, point_two_loads = (
        StageTwoController.validate_and_split_request(
            StageTwo.Request.ROUTE,
            default_move_cells(),
            [cell(2, 3), cell(4, 1), cell(1, 1), cell(4, 3), cell(4, 2)],
        ))

    assert move_cells == ((4, 2), (3, 2), (2, 2), (1, 2), (1, 1))
    assert point_one_route == (3, 1, 2)
    assert point_two_loads == ((1, 1), (2, 3))


def test_route_without_entry_kfs_has_empty_point_one_route():
    _, point_one_route, point_two_loads = (
        StageTwoController.validate_and_split_request(
            StageTwo.Request.ROUTE,
            default_move_cells(),
            [cell(2, 3)],
        ))

    assert point_one_route == ()
    assert point_two_loads == ((2, 3),)


def test_only_blue_point_one_route_is_mirrored():
    assert StageTwoController.point_one_route_for_team(
        (1,), StageTwo.Request.RED) == (1,)
    assert StageTwoController.point_one_route_for_team(
        (1,), StageTwo.Request.BLUE) == (3,)


def test_blue_point_one_fix_keeps_point_two_route_and_load_cells_unchanged():
    controller = make_controller()
    request = SimpleNamespace(
        team=StageTwo.Request.BLUE,
        fake_kfs_decision=0,
        mode=StageTwo.Request.ROUTE,
        move_cells=default_move_cells(),
        kfs_cells=[cell(4, 1), cell(2, 3)],
    )
    response = SimpleNamespace(success=None, message='', loaded_count=0)

    result = controller.handle_stage_two(request, response)

    assert result.success
    assert list(controller.point_one_client.requests[0].route_cells) == [3]
    point_two_request = controller.point_two_client.requests[0]
    assert indices(point_two_request.move_cells) == indices(
        default_move_cells())
    assert indices(point_two_request.load_cells) == [(2, 3)]


@pytest.mark.parametrize(
    'mode, moves, loads, message',
    [
        (99, [], [], 'unknown StageTwo mode'),
        (StageTwo.Request.ROUTE, [], [], 'must not be empty'),
        (
            StageTwo.Request.ROUTE,
            [cell(4, 2), cell(4, 2)],
            [],
            'move_cells must not contain duplicates',
        ),
        (
            StageTwo.Request.ROUTE,
            default_move_cells(),
            [cell(3, 1), cell(3, 1)],
            'kfs_cells must not contain duplicates',
        ),
        (
            StageTwo.Request.ROUTE,
            [cell(5, 2)],
            [],
            'must start at',
        ),
        (
            StageTwo.Request.ROUTE,
            default_move_cells(),
            [cell(2, 4)],
            'lateral lanes 1..3',
        ),
        (
            StageTwo.Request.ROUTE,
            [cell(4, 2), cell(2, 2), cell(1, 1)],
            [],
            'must be adjacent',
        ),
    ],
)
def test_invalid_overall_route_is_rejected(mode, moves, loads, message):
    with pytest.raises(ValueError, match=message):
        StageTwoController.validate_and_split_request(mode, moves, loads)


def test_route_requests_are_forwarded_to_each_substage():
    controller = make_controller()
    controller.run_point_one(
        StageTwo.Request.RED,
        StageTwo.Request.ROUTE,
        (3, 1),
        controller.point_one_timeout_sec,
    )
    controller.run_point_two(
        StageTwo.Request.RED,
        0,
        StageTwo.Request.ROUTE,
        ((4, 2), (3, 2), (2, 2), (1, 2), (1, 1)),
        ((1, 1), (2, 3)),
        controller.point_two_timeout_sec,
    )

    point_one_request = controller.point_one_client.requests[0]
    assert point_one_request.mode == StageTwoPointOne.Request.ROUTE
    assert list(point_one_request.route_cells) == [3, 1]
    point_two_request = controller.point_two_client.requests[0]
    assert point_two_request.mode == StageTwoPointTwo.Request.ROUTE
    assert indices(point_two_request.move_cells) == indices(default_move_cells())
    assert indices(point_two_request.load_cells) == [(1, 1), (2, 3)]


def test_route_without_entry_kfs_skips_point_one_service():
    controller = make_controller()
    request = SimpleNamespace(
        team=StageTwo.Request.RED,
        fake_kfs_decision=0,
        mode=StageTwo.Request.ROUTE,
        move_cells=default_move_cells(),
        kfs_cells=[cell(2, 3)],
    )
    response = SimpleNamespace(success=None, message='', loaded_count=0)

    result = controller.handle_stage_two(request, response)

    assert result.success
    assert controller.point_one_client.requests == []
    assert controller.point_one_client.wait_timeouts == []
    assert controller.point_two_client.requests[0].loaded_count == 0
    assert indices(controller.point_two_client.requests[0].load_cells) == [
        (2, 3)]
    assert '2.1 skipped' in result.message


def test_point_one_failure_stops_before_point_two():
    controller = make_controller()
    controller.point_one_client = FakeClient(1, success=False)
    request = SimpleNamespace(
        team=StageTwo.Request.RED,
        fake_kfs_decision=StageTwo.Request.LEFT,
        mode=StageTwo.Request.STANDARD,
        move_cells=[cell(99, 99)],
        kfs_cells=[cell(99, 99)],
    )
    response = SimpleNamespace(success=None, message='', loaded_count=0)

    result = controller.handle_stage_two(request, response)

    assert not result.success
    assert result.loaded_count == 1
    assert controller.point_two_client.requests == []


def test_full_stage_two_rejects_unknown_team():
    with pytest.raises(ValueError, match='team must be red or blue'):
        StageTwoController.validate_team('green')


def test_timeout_parameters_update_atomically():
    controller = make_controller()
    parameters = [
        SimpleNamespace(name='dependency_timeout_sec', value=3.0),
        SimpleNamespace(name='stage_two_point_two_timeout_sec', value=30.0),
    ]

    result = controller._on_parameters_changed(parameters)

    assert result.successful
    assert controller.config_snapshot() == (3.0, 10.0, 30.0)


def test_invalid_timeout_update_keeps_all_previous_values():
    controller = make_controller()
    parameters = [
        SimpleNamespace(name='dependency_timeout_sec', value=3.0),
        SimpleNamespace(name='stage_two_point_one_timeout_sec', value=0.0),
    ]

    result = controller._on_parameters_changed(parameters)

    assert not result.successful
    assert controller.config_snapshot() == (2.0, 10.0, 20.0)
