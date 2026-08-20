import math
import threading
from types import SimpleNamespace

import pytest

from robot_r2_control.stage_two_point_two import StageTwoPointTwoController
from robot_r2_interfaces.srv import (
    MoveRelative,
    MoveToPose,
    SetLift,
    StageTwoPointTwo,
)


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
        self.wait_timeouts = []
        self.success = success

    def call_async(self, request):
        self.requests.append(request)
        return ImmediateFuture(SimpleNamespace(
            success=self.success,
            message='ok' if self.success else 'failed',
        ))

    def wait_for_service(self, timeout_sec):
        self.wait_timeouts.append(timeout_sec)
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
    # 基准坐标为蓝方（负 Y）；单队伍用例默认使用基准（蓝方）。
    controller.team = StageTwoPointTwo.Request.BLUE
    controller.initial_index = (5, 2)
    controller.terminal_forward_index = 0
    controller.cell_detection_results = [
        [None for _ in controller.lateral_y]
        for _ in controller.forward_x
    ]
    controller.move_client = FakeClient()
    controller.move_relative_client = FakeClient()
    controller.lift_client = FakeClient()
    controller.traverse_client = FakeClient()
    controller.kfs_action_client = FakeClient()
    controller.detection_client = FakeClient()
    controller.align_client = FakeClient()
    controller.move_timeout_sec = 35.0
    controller.pose_timeout_sec = 2.0
    controller.traverse_timeout_sec = 150.0
    controller.detection_timeout_sec = 10.0
    controller.align_timeout_sec = 15.0
    controller.load_timeout_sec = 70.0
    controller.release_timeout_sec = 70.0
    controller.dependency_timeout_sec = 2.0
    controller.higher_kfs_edge_offset = 0.2
    controller.lower_kfs_edge_offset = 0.4
    controller.release_edge_offset = 0.2
    controller.detection_sample_count = 10
    controller.mode = StageTwoPointTwo.Request.STANDARD
    controller.service_lock = threading.Lock()
    controller.config_lock = threading.Lock()
    controller.exit_cell_0_0_pose = (-2.6, -5.4, math.pi)
    controller.exit_x_offset = 2.9
    controller.exit_lift_height = 0.03
    controller.lift_timeout_sec = 15.0
    return controller


@pytest.mark.parametrize(
    'team, expected_y',
    [
        (StageTwoPointTwo.Request.BLUE, -3.0),
        (StageTwoPointTwo.Request.RED, 3.0),
    ],
)
def test_get_cell_keeps_blue_and_mirrors_red(team, expected_y):
    controller = make_controller()
    controller.team = team

    assert controller.get_cell((5, 2)) == pytest.approx(
        (3.4, expected_y, 0.0))


def test_invalid_team_is_rejected():
    with pytest.raises(ValueError, match='team must be red or blue'):
        StageTwoPointTwoController.validate_team('green')


def cell(forward_index, lateral_index):
    return SimpleNamespace(
        forward_index=forward_index,
        lateral_index=lateral_index,
    )


def default_route_messages(final_lateral=1):
    lateral_tail = (
        [cell(1, 2), cell(1, final_lateral)]
        if final_lateral != 2
        else [cell(1, 2)]
    )
    return [
        cell(4, 2),
        cell(3, 2),
        cell(2, 2),
        *lateral_tail,
    ]


def test_route_mode_validates_and_assigns_load_cells():
    controller = make_controller()

    move_cells, load_cells, load_targets = (
        controller.validate_mode_and_routes(
            StageTwoPointTwo.Request.ROUTE,
            default_route_messages(1),
            [cell(4, 1), cell(0, 1)],
        )
    )

    assert move_cells == ((4, 2), (3, 2), (2, 2), (1, 2), (1, 1))
    assert load_cells == ((4, 1), (0, 1))
    assert load_targets[0] == ((4, 1),)
    assert load_targets[-1] == ((0, 1),)


@pytest.mark.parametrize(
    'mode, moves, loads, message',
    [
        (99, [], [], 'unknown Stage 2.2 mode'),
        (
            StageTwoPointTwo.Request.STANDARD,
            [cell(4, 2)],
            [],
            'must be empty',
        ),
        (StageTwoPointTwo.Request.ROUTE, [], [], 'must not be empty'),
        (
            StageTwoPointTwo.Request.ROUTE,
            [cell(3, 2), cell(2, 2), cell(1, 1)],
            [],
            'must start',
        ),
        (
            StageTwoPointTwo.Request.ROUTE,
            [cell(4, 2), cell(3, 2), cell(2, 2), cell(1, 2)],
            [],
            'must end',
        ),
        (
            StageTwoPointTwo.Request.ROUTE,
            [cell(4, 2), cell(3, 2), cell(1, 2), cell(1, 1)],
            [],
            'must be adjacent',
        ),
        (
            StageTwoPointTwo.Request.ROUTE,
            [cell(4, 2), cell(3, 2), cell(4, 2), cell(1, 1)],
            [],
            'duplicates',
        ),
    ],
)
def test_route_mode_rejects_invalid_routes(
        mode, moves, loads, message):
    controller = make_controller()

    with pytest.raises(ValueError, match=message):
        controller.validate_mode_and_routes(mode, moves, loads)


def test_route_mode_rejects_unreachable_load_cell():
    controller = make_controller()

    with pytest.raises(ValueError, match='not front/left/right'):
        controller.validate_mode_and_routes(
            StageTwoPointTwo.Request.ROUTE,
            default_route_messages(1),
            [cell(0, 3)],
        )


def test_route_mode_rejects_duplicate_load_cells():
    controller = make_controller()

    with pytest.raises(ValueError, match='load_cells.*duplicates'):
        controller.validate_mode_and_routes(
            StageTwoPointTwo.Request.ROUTE,
            default_route_messages(1),
            [cell(4, 1), cell(4, 1)],
        )


def test_route_mode_rejects_same_height_load_cell():
    controller = make_controller()
    heights = list(controller.cell_heights)
    heights[4 * 3] = 1.0
    controller.cell_heights = tuple(heights)

    with pytest.raises(ValueError, match='same height'):
        controller.validate_mode_and_routes(
            StageTwoPointTwo.Request.ROUTE,
            default_route_messages(1),
            [cell(4, 1)],
        )


def test_route_load_order_puts_next_movement_cell_last():
    controller = make_controller()

    _, _, load_targets = controller.validate_mode_and_routes(
        StageTwoPointTwo.Request.ROUTE,
        default_route_messages(1),
        [cell(3, 2), cell(4, 3), cell(4, 1)],
    )

    assert load_targets[0] == ((4, 1), (4, 3), (3, 2))


def test_route_mode_rejects_wrong_height_difference():
    controller = make_controller()
    heights = list(controller.cell_heights)
    heights[3 * 3 + 1] = 1.0
    controller.cell_heights = tuple(heights)

    with pytest.raises(ValueError, match='exactly one height level'):
        controller.validate_mode_and_routes(
            StageTwoPointTwo.Request.ROUTE,
            default_route_messages(1),
            [],
        )


def test_route_mode_rejects_untraversable_load_cell():
    controller = make_controller()

    with pytest.raises(ValueError, match='not traversable'):
        controller.validate_mode_and_routes(
            StageTwoPointTwo.Request.ROUTE,
            default_route_messages(1),
            [cell(5, 1)],
        )


def test_route_mode_rejects_untraversable_move_cell():
    controller = make_controller()

    with pytest.raises(ValueError, match='not traversable'):
        controller.validate_mode_and_routes(
            StageTwoPointTwo.Request.ROUTE,
            [
                cell(4, 2),
                cell(4, 1),
                cell(5, 1),
                cell(1, 1),
            ],
            [],
        )


@pytest.mark.parametrize('final_lateral', [1, 3])
def test_execute_route_appends_matching_terminal_cell(final_lateral):
    controller = make_controller()
    controller.get_logger = lambda: FakeLogger()
    move_messages = default_route_messages(final_lateral)
    (
        controller.move_cells,
        controller.load_cells,
        controller.route_load_targets,
    ) = controller.validate_mode_and_routes(
        StageTwoPointTwo.Request.ROUTE,
        move_messages,
        [],
    )
    moves = []
    controller.move_one_cell = (
        lambda source, target: moves.append((source, target)))

    final_index = controller.execute_route_task()

    assert final_index == (0, final_lateral)
    assert moves[0] == ((5, 2), (4, 2))
    assert moves[-1] == (
        (1, final_lateral),
        (0, final_lateral),
    )


def test_route_load_targets_keep_next_cell_at_edge():
    controller = make_controller()
    calls = []
    controller.pickup_kfs = lambda current, target, return_to_center: (
        calls.append((current, target, return_to_center)))

    controller.load_route_targets(
        (4, 2),
        ((4, 1), (3, 2)),
        (3, 2),
    )

    assert calls == [
        ((4, 2), (4, 1), True),
        ((4, 2), (3, 2), False),
    ]


@pytest.mark.parametrize(
    'mode, detection_waits, alignment_waits',
    [
        (StageTwoPointTwo.Request.STANDARD, [2.0], [2.0]),
        (StageTwoPointTwo.Request.SKIP, [], []),
        (StageTwoPointTwo.Request.ROUTE, [], [2.0]),
    ],
)
def test_mode_dependencies(mode, detection_waits, alignment_waits):
    controller = make_controller()
    controller.mode = mode

    controller.wait_for_dependencies()

    assert controller.detection_client.wait_timeouts == detection_waits
    assert controller.align_client.wait_timeouts == alignment_waits


def test_standard_mode_cache_reset_clears_previous_task_results():
    controller = make_controller()
    controller.cell_detection_results[2][1] = 'true'

    controller.reset_detection_results()

    assert all(
        value is None
        for row in controller.cell_detection_results
        for value in row
    )


@pytest.mark.parametrize(
    'current, target, loaded_count, expected_mode',
    [
        ((4, 2), (4, 1), 0, 1),
        ((4, 2), (4, 1), 2, 2),
        ((3, 2), (3, 1), 0, 3),
        ((3, 2), (3, 1), 1, 5),
        ((3, 2), (3, 1), 2, 4),
    ],
)
def test_route_pickup_preserves_alignment_and_load_modes(
        current, target, loaded_count, expected_mode):
    controller = make_controller()
    controller.get_logger = lambda: FakeLogger()
    controller.loaded_count = loaded_count
    alignments = []
    load_modes = []
    controller.move_to_pose = lambda *pose: None
    controller.align_kfs = lambda: alignments.append(True)
    controller.load_kfs = lambda mode: load_modes.append(mode)

    controller.pickup_kfs(current, target)

    assert alignments == [True]
    assert load_modes == [expected_mode]


@pytest.mark.parametrize(
    'current, target, expected_offset',
    [
        ((4, 2), (4, 1), 0.2),
        ((3, 2), (3, 1), 0.4),
    ],
)
def test_pickup_approaches_high_and_low_kfs_relative_to_aligned_pose(
        current, target, expected_offset):
    controller = make_controller()
    controller.get_logger = lambda: FakeLogger()
    absolute_moves = []
    controller.move_to_pose = lambda *pose: absolute_moves.append(pose)
    controller.align_kfs = lambda: None
    controller.load_kfs = lambda _mode: None
    controller.loaded_count = 0

    controller.pickup_kfs(current, target)

    current_cell = controller.get_cell(current)
    assert len(absolute_moves) == 1
    assert absolute_moves[0][:2] == pytest.approx(current_cell[:2])
    assert absolute_moves[0][2] == pytest.approx(-math.pi / 2.0)
    assert [
        request.forward
        for request in controller.move_relative_client.requests
    ] == pytest.approx([expected_offset, -expected_offset])
    assert all(
        request.pose_source == MoveRelative.Request.ODIN
        for request in controller.move_relative_client.requests
    )


def test_route_pickup_preserves_full_load_release_behavior():
    controller = make_controller()
    controller.get_logger = lambda: FakeLogger()
    controller.loaded_count = 3
    controller.arrival_direction = (-1.0, 0.0)
    controller.move_to_pose = lambda *pose: None
    controller.align_kfs = lambda: None
    releases = []
    load_modes = []

    def release():
        releases.append(True)
        controller.loaded_count -= 1

    controller.release_kfs = release
    controller.load_kfs = lambda mode: load_modes.append(mode)

    controller.pickup_kfs((4, 2), (4, 1))

    assert releases == [True]
    assert load_modes == [2]
    assert [
        request.forward
        for request in controller.move_relative_client.requests
    ] == pytest.approx([0.2, -0.2, 0.2, -0.2, 0.2, -0.2])
    assert [
        request.yaw_delta
        for request in controller.move_relative_client.requests
    ] == pytest.approx([
        0.0,
        math.pi / 2.0,
        0.0,
        -math.pi / 2.0,
        0.0,
        0.0,
    ])


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


def test_move_relative_uses_odin_source_and_relative_values():
    controller = make_controller()

    controller.move_relative(0.2, -0.1, math.pi / 2.0)

    request = controller.move_relative_client.requests[0]
    assert request.pose_source == MoveRelative.Request.ODIN
    assert request.forward == pytest.approx(0.2)
    assert request.left == pytest.approx(-0.1)
    assert request.yaw_delta == pytest.approx(math.pi / 2.0)
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
        (StageTwoPointTwo.Request.BLUE, -3.0),
        (StageTwoPointTwo.Request.RED, 3.0),
    ],
)
def test_servo_to_initial_cell_uses_team_grid_pose(team, expected_y):
    controller = make_controller()
    controller.team = team
    targets = []
    controller.move_to_pose = lambda *target: targets.append(target)
    controller.get_logger = lambda: FakeLogger()

    controller.servo_to_initial_cell()

    assert targets == [pytest.approx((3.4, expected_y, math.pi))]


def test_initial_servo_failure_prevents_stage_path_execution():
    controller = make_controller()
    controller.loaded_count = 0
    controller.get_logger = lambda: FakeLogger()
    controller.move_to_pose = lambda *_target: (_ for _ in ()).throw(
        RuntimeError('initial servo failed'))
    executed = []
    controller.execute_task = lambda decision: executed.append(decision)
    response = SimpleNamespace(success=None, message='', loaded_count=0)
    request = SimpleNamespace(
        team=StageTwoPointTwo.Request.RED,
        loaded_count=0,
        mode=StageTwoPointTwo.Request.SKIP,
        fake_kfs_decision=StageTwoPointTwo.Request.LEFT,
        move_cells=[],
        load_cells=[],
    )

    result = controller.handle_task(request, response)

    assert not result.success
    assert result.message == 'initial servo failed'
    assert executed == []


@pytest.mark.parametrize(
    'mode, decision, moves',
    [
        (StageTwoPointTwo.Request.STANDARD, StageTwoPointTwo.Request.LEFT, []),
        (StageTwoPointTwo.Request.SKIP, StageTwoPointTwo.Request.RIGHT, []),
        (StageTwoPointTwo.Request.ROUTE, 0, default_route_messages(1)),
    ],
)
def test_all_task_modes_servo_before_executing_path(mode, decision, moves):
    controller = make_controller()
    actions = []
    controller.wait_for_dependencies = lambda: actions.append('dependencies')
    controller.servo_to_initial_cell = lambda: actions.append('servo')
    controller.execute_task = lambda value: (
        actions.append(('execute', value)) or (0, 1))
    response = SimpleNamespace(success=None, message='', loaded_count=0)
    request = SimpleNamespace(
        team=StageTwoPointTwo.Request.RED,
        loaded_count=0,
        mode=mode,
        fake_kfs_decision=decision,
        move_cells=moves,
        load_cells=[],
    )

    result = controller.handle_task(request, response)

    assert result.success
    assert actions == ['dependencies', 'servo', ('execute', decision)]


@pytest.mark.parametrize(
    'team, expected_y',
    [
        (StageTwoPointTwo.Request.BLUE, -5.4),
        (StageTwoPointTwo.Request.RED, 5.4),
    ],
)
def test_exit_targets_keep_blue_and_mirror_red(team, expected_y):
    controller = make_controller()

    first, second = controller.exit_targets(team)

    assert first == pytest.approx((-2.6, expected_y, math.pi))
    assert second == pytest.approx((-5.5, expected_y, math.pi))


def test_exit_service_runs_both_absolute_targets_in_order():
    controller = make_controller()
    actions = []
    controller.set_lift = lambda height, timeout: actions.append(
        ('lift', height, timeout))
    controller.move_to_pose = lambda *target: actions.append(
        ('move', *target))
    response = SimpleNamespace(success=None, message='')

    result = controller.handle_exit(
        SimpleNamespace(team=StageTwoPointTwo.Request.BLUE), response)

    assert result.success
    assert actions[0] == ('lift', pytest.approx(0.03), pytest.approx(15.0))
    assert [action[0] for action in actions[1:]] == ['move', 'move']
    assert actions[1][1:] == pytest.approx((-2.6, -5.4, math.pi))
    assert actions[2][1:] == pytest.approx((-5.5, -5.4, math.pi))


def test_exit_lift_uses_same_absolute_height_for_front_and_rear():
    controller = make_controller()

    controller.set_lift(0.03, 15.0)

    request = controller.lift_client.requests[0]
    assert isinstance(request, SetLift.Request)
    assert request.front_lift == pytest.approx(0.03)
    assert request.rear_lift == pytest.approx(0.03)
    assert request.tolerance == pytest.approx(0.0)
    assert request.timeout_sec == pytest.approx(15.0)


def test_exit_service_stops_before_moves_when_lift_fails():
    controller = make_controller()
    controller.lift_client = FakeClient(success=False)
    moves = []
    controller.move_to_pose = lambda *target: moves.append(target)
    response = SimpleNamespace(success=None, message='')

    result = controller.handle_exit(
        SimpleNamespace(team=StageTwoPointTwo.Request.RED), response)

    assert not result.success
    assert result.message == 'SetLift failed: failed'
    assert moves == []


def test_exit_service_stops_after_first_move_failure():
    controller = make_controller()
    moves = []

    def fail_first(*target):
        moves.append(target)
        raise RuntimeError('first move failed')

    controller.move_to_pose = fail_first
    response = SimpleNamespace(success=None, message='')

    result = controller.handle_exit(
        SimpleNamespace(team=StageTwoPointTwo.Request.BLUE), response)

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
        (-2.6, -5.4, math.pi),
        (-5.5, -5.4, math.pi),
    ]


def test_exit_parameters_update_atomically():
    controller = make_controller()

    result = controller._on_parameters_changed([
        SimpleNamespace(
            name='exit_cell_0_0_pose', value=[-3.0, -6.0, math.pi]),
        SimpleNamespace(name='exit_x_offset', value=1.5),
        SimpleNamespace(name='exit_lift_height', value=0.04),
        SimpleNamespace(name='lift_timeout_sec', value=20.0),
    ])

    assert result.successful
    assert controller.exit_targets(StageTwoPointTwo.Request.RED) == (
        pytest.approx((-3.0, 6.0, math.pi)),
        pytest.approx((-4.5, 6.0, math.pi)),
    )
    assert controller.exit_lift_height == pytest.approx(0.04)
    assert controller.lift_timeout_sec == pytest.approx(20.0)


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


def test_invalid_exit_lift_parameter_update_keeps_previous_config():
    controller = make_controller()

    result = controller._on_parameters_changed([
        SimpleNamespace(name='exit_lift_height', value=-0.01),
        SimpleNamespace(name='lift_timeout_sec', value=20.0),
    ])

    assert not result.successful
    assert controller.exit_lift_height == pytest.approx(0.03)
    assert controller.lift_timeout_sec == pytest.approx(15.0)


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


def test_all_runtime_parameters_update_and_geometry_resets_cache():
    controller = make_controller()
    controller.cell_detection_results[2][1] = 'true'

    result = controller._on_parameters_changed([
        SimpleNamespace(name='move_timeout_sec', value=40.0),
        SimpleNamespace(name='detection_sample_count', value=12),
        SimpleNamespace(name='higher_kfs_edge_offset', value=0.25),
        SimpleNamespace(name='forward_x', value=list(controller.forward_x)),
    ])

    assert result.successful
    assert controller.move_timeout_sec == pytest.approx(40.0)
    assert controller.detection_sample_count == 12
    assert controller.higher_kfs_edge_offset == pytest.approx(0.25)
    assert all(
        value is None
        for row in controller.cell_detection_results
        for value in row
    )


def test_invalid_geometry_update_rolls_back_all_values():
    controller = make_controller()
    original_timeout = controller.move_timeout_sec
    original_heights = controller.cell_heights

    result = controller._on_parameters_changed([
        SimpleNamespace(name='move_timeout_sec', value=40.0),
        SimpleNamespace(name='cell_heights', value=[0.0]),
    ])

    assert not result.successful
    assert controller.move_timeout_sec == pytest.approx(original_timeout)
    assert controller.cell_heights == original_heights


def test_grid_parameters_cannot_change_during_active_task():
    controller = make_controller()
    controller.service_lock.acquire()
    try:
        result = controller._on_parameters_changed([
            SimpleNamespace(
                name='forward_x',
                value=list(controller.forward_x),
            ),
        ])
    finally:
        controller.service_lock.release()

    assert not result.successful
    assert 'while a task is active' in result.reason


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
    controller.mode = StageTwoPointTwo.Request.SKIP
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
