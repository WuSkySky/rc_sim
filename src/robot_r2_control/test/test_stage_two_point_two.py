import math
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
    controller.initial_index = (5, 2)
    controller.terminal_forward_index = 0
    controller.cell_detection_results = [
        [None for _ in controller.lateral_y]
        for _ in controller.forward_x
    ]
    controller.move_client = FakeClient()
    controller.move_timeout_sec = 35.0
    return controller


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
