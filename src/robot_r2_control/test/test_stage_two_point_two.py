import math
import threading
from types import SimpleNamespace

import pytest

from robot_r2_control.stage_two_point_two import StageTwoPointTwoController
from robot_r2_interfaces.srv import MoveToPose, StageTwoPointTwo


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

    def wait_for_service(self, timeout_sec):
        return True


class FakeLogger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass


def make_controller():
    controller = StageTwoPointTwoController.__new__(
        StageTwoPointTwoController)
    controller.forward_x = [-2.6, -1.4, -0.2, 1.0, 2.2, 3.4]
    controller.lateral_y = [-4.2, -3.0, -1.8]
    controller.cell_heights = (
        0.0, float('nan'), 0.0,
        1.0, 2.0, 1.0,
        2.0, 3.0, 2.0,
        1.0, 2.0, 3.0,
        2.0, 1.0, 2.0,
        float('nan'), 0.0, float('nan'),
    )
    controller.chassis_front_offset = 0.35
    controller.team = StageTwoPointTwo.Request.RED
    controller.initial_index = (5, 2)
    controller.terminal_forward_index = 0
    controller.cell_detection_results = [
        [None for _ in controller.lateral_y]
        for _ in controller.forward_x
    ]
    controller.move_client = FakeClient()
    controller.move_timeout_sec = 35.0
    controller.dependency_timeout_sec = 2.0
    controller.service_lock = threading.Lock()
    controller.config_lock = threading.Lock()
    controller.exit_cell_0_0_pose = (-2.6, -5.4, math.pi)
    controller.exit_x_offset = 2.9
    return controller


@pytest.mark.parametrize(
    'team, expected_y',
    [
        (StageTwoPointTwo.Request.RED, -3.0),
        (StageTwoPointTwo.Request.BLUE, 3.0),
    ],
)
def test_get_cell_only_mirrors_y_for_blue_team(team, expected_y):
    controller = make_controller()
    controller.team = team

    assert controller.get_cell((5, 2)) == pytest.approx(
        (3.4, expected_y, 0.0))


def test_invalid_team_is_rejected():
    with pytest.raises(ValueError, match='team must be red or blue'):
        StageTwoPointTwoController.validate_team('green')


def test_move_to_pose_uses_odin_source_and_absolute_values():
    controller = make_controller()

    controller.move_to_pose(1.5, -2.0, math.pi)

    request = controller.move_client.requests[0]
    assert request.pose_source == MoveToPose.Request.ODIN
    assert request.x == pytest.approx(1.5)
    assert request.y == pytest.approx(-2.0)
    assert request.yaw == pytest.approx(math.pi)
    assert request.position_tolerance == pytest.approx(0.0)
    assert request.yaw_tolerance == pytest.approx(0.0)
    assert request.timeout_sec == pytest.approx(35.0)


def test_move_to_pose_failure_raises():
    controller = make_controller()
    controller.move_client = FakeClient(success=False)

    with pytest.raises(RuntimeError, match='MoveToPose failed'):
        controller.move_to_pose(0.0, 0.0, 0.0)


@pytest.mark.parametrize(
    'team, expected_y',
    [
        (StageTwoPointTwo.Request.RED, -5.4),
        (StageTwoPointTwo.Request.BLUE, 5.4),
    ],
)
def test_exit_targets_mirror_only_y(team, expected_y):
    controller = make_controller()

    first, second = controller.exit_targets(team)

    assert first == pytest.approx((-2.6, expected_y, math.pi))
    assert second == pytest.approx((-5.5, expected_y, math.pi))


def test_exit_service_runs_both_absolute_targets_in_order():
    controller = make_controller()
    moves = []
    controller.move_to_pose = lambda *target: moves.append(target)
    response = SimpleNamespace(success=None, message='')

    result = controller.handle_exit(
        SimpleNamespace(team=StageTwoPointTwo.Request.RED), response)

    assert result.success
    assert moves == pytest.approx([
        (-2.6, -5.4, math.pi),
        (-5.5, -5.4, math.pi),
    ])


def test_exit_service_stops_after_first_move_failure():
    controller = make_controller()
    moves = []

    def fail_first(*target):
        moves.append(target)
        raise RuntimeError('first move failed')

    controller.move_to_pose = fail_first
    response = SimpleNamespace(success=None, message='')

    result = controller.handle_exit(
        SimpleNamespace(team=StageTwoPointTwo.Request.RED), response)

    assert not result.success
    assert result.message == 'first move failed'
    assert moves == [(-2.6, -5.4, math.pi)]


def test_exit_service_reports_second_move_failure():
    controller = make_controller()
    moves = []

    def fail_second(*target):
        moves.append(target)
        if len(moves) == 2:
            raise RuntimeError('second move failed')

    controller.move_to_pose = fail_second
    response = SimpleNamespace(success=None, message='')

    result = controller.handle_exit(
        SimpleNamespace(team=StageTwoPointTwo.Request.BLUE), response)

    assert not result.success
    assert result.message == 'second move failed'
    assert moves == [
        (-2.6, 5.4, math.pi),
        (-5.5, 5.4, math.pi),
    ]


def test_exit_parameters_update_atomically():
    controller = make_controller()

    result = controller._on_parameters_changed([
        SimpleNamespace(
            name='exit_cell_0_0_pose', value=[-3.0, -6.0, math.pi]),
        SimpleNamespace(name='exit_x_offset', value=1.5),
    ])

    assert result.successful
    assert controller.exit_targets(StageTwoPointTwo.Request.RED) == (
        pytest.approx((-3.0, -6.0, math.pi)),
        pytest.approx((-4.5, -6.0, math.pi)),
    )


def test_invalid_exit_parameter_update_keeps_all_previous_values():
    controller = make_controller()

    result = controller._on_parameters_changed([
        SimpleNamespace(
            name='exit_cell_0_0_pose', value=[-3.0, -6.0, math.pi]),
        SimpleNamespace(name='exit_x_offset', value=-0.1),
    ])

    assert not result.successful
    assert controller.exit_cell_0_0_pose == (-2.6, -5.4, math.pi)
    assert controller.exit_x_offset == 2.9


def test_exit_parameter_update_rejects_non_finite_derived_target():
    controller = make_controller()

    result = controller._on_parameters_changed([
        SimpleNamespace(
            name='exit_cell_0_0_pose', value=[-1e308, -6.0, math.pi]),
        SimpleNamespace(name='exit_x_offset', value=1e308),
    ])

    assert not result.successful
    assert 'must be finite' in result.reason
    assert controller.exit_cell_0_0_pose == (-2.6, -5.4, math.pi)
    assert controller.exit_x_offset == 2.9


def test_move_one_cell_computes_step_distance_from_odin_pose():
    controller = make_controller()
    controller.get_logger = lambda: FakeLogger()
    poses = iter([(3.3, -3.0, math.pi), (3.3, -3.0, math.pi)])
    controller.wait_for_pose = lambda: next(poses)

    moves = []
    controller.move_to_pose = (
        lambda x, y, yaw: moves.append(('move', x, y, yaw)))
    traverses = []
    controller.traverse_step = (
        lambda is_up, distance: traverses.append(
            ('traverse', is_up, distance)))

    controller.move_one_cell((5, 2), (4, 2))

    # 转向保持当前位置，只转 yaw；目标格心为绝对坐标。
    assert moves[0] == ('move', 3.3, -3.0, math.pi)
    # 边界 x=2.8，车头前沿 x=3.3-0.35=2.95，距离 = 2.8-2.95 = -0.15，
    # 沿行进方向 (-1, 0) 投影 => +0.15。
    assert traverses == [('traverse', True, pytest.approx(0.15))]
    assert moves[1] == ('move', 2.2, -3.0, math.pi)
    assert controller.arrival_direction == (-1.0, 0.0)


def test_execute_task_skip_mode_follows_forward_path():
    controller = make_controller()
    controller.get_logger = lambda: FakeLogger()
    controller._skip_kfs_detection = True
    controller.loaded_count = 0
    controller.arrival_direction = None
    moves = []
    controller.move_one_cell = (
        lambda source, target: moves.append((source, target)))

    final_index = controller.execute_task(StageTwoPointTwo.Request.LEFT)

    assert final_index == (0, 1)
    assert moves == [
        ((5, 2), (4, 2)),
        ((4, 2), (3, 2)),
        ((3, 2), (2, 2)),
        ((2, 2), (1, 2)),
        ((1, 2), (1, 1)),
        ((1, 1), (0, 1)),
    ]
    # 跳过识别：不产生任何检测缓存。
    assert all(
        cell is None
        for row in controller.cell_detection_results
        for cell in row
    )


def test_left_right_index_decisions_are_unchanged_for_blue_team():
    controller = make_controller()
    controller.team = StageTwoPointTwo.Request.BLUE

    assert controller.selected_lateral_delta(
        StageTwoPointTwo.Request.LEFT, (1, 2)) == controller.LEFT
    assert controller.selected_lateral_delta(
        StageTwoPointTwo.Request.RIGHT, (1, 2)) == controller.RIGHT
    assert controller.rotate_left(controller.FORWARD) == controller.LEFT
    assert controller.rotate_right(controller.FORWARD) == controller.RIGHT
