import threading
from types import SimpleNamespace

import pytest

from robot_r2_control.stage_two_point_one import StageTwoPointOneController
from robot_r2_interfaces.srv import MoveToPose, StageTwoPointOne


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


def make_controller():
    controller = StageTwoPointOneController.__new__(
        StageTwoPointOneController)
    controller.config_lock = threading.Lock()
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


@pytest.mark.parametrize(
    'team, expected_y',
    [
        (StageTwoPointOne.Request.RED, -1.8),
        (StageTwoPointOne.Request.BLUE, 1.8),
    ],
)
def test_team_edge_pose_only_mirrors_y(team, expected_y):
    pose = (3.2, -1.8, 3.141592653589793)

    mirrored = StageTwoPointOneController.team_edge_pose(pose, team)

    assert mirrored == pytest.approx((3.2, expected_y, pose[2]))


def test_invalid_team_is_rejected():
    with pytest.raises(ValueError, match='team must be red or blue'):
        StageTwoPointOneController.validate_team('green')


@pytest.mark.parametrize(
    'mode, route, loaded_count, expected',
    [
        (StageTwoPointOne.Request.STANDARD, [], 0, (3, 1, 2)),
        (StageTwoPointOne.Request.SKIP, [], 0, (3, 1, 2)),
        (StageTwoPointOne.Request.ROUTE, [1, 3], 0, (1, 3)),
    ],
)
def test_validate_mode_and_route(mode, route, loaded_count, expected):
    assert StageTwoPointOneController.validate_mode_and_route(
        mode, route, loaded_count) == expected


@pytest.mark.parametrize(
    'mode, route, loaded_count, message',
    [
        (99, [], 0, 'unknown Stage 2.1 mode'),
        (StageTwoPointOne.Request.STANDARD, [1], 0, 'must be empty'),
        (StageTwoPointOne.Request.ROUTE, [], 0, 'must not be empty'),
        (StageTwoPointOne.Request.ROUTE, [1, 4], 0, 'only contain'),
        (StageTwoPointOne.Request.ROUTE, [1, 1], 0, 'duplicates'),
        (StageTwoPointOne.Request.ROUTE, [1], 3, 'cannot start'),
    ],
)
def test_validate_mode_and_route_rejects_invalid_request(
        mode, route, loaded_count, message):
    with pytest.raises(ValueError, match=message):
        StageTwoPointOneController.validate_mode_and_route(
            mode, route, loaded_count)


def make_dependency_controller(mode):
    controller = StageTwoPointOneController.__new__(
        StageTwoPointOneController)
    controller.config_lock = threading.Lock()
    controller.mode = mode
    controller.dependency_timeout_sec = 2.0
    controller.move_client = FakeClient()
    controller.lift_client = FakeClient()
    controller.kfs_action_client = FakeClient()
    controller.detection_client = FakeClient()
    controller.align_client = FakeClient()
    return controller


def test_standard_mode_waits_for_visual_dependencies():
    controller = make_dependency_controller(
        StageTwoPointOne.Request.STANDARD)

    controller.wait_for_dependencies()

    assert controller.detection_client.wait_timeouts == [2.0]
    assert controller.align_client.wait_timeouts == [2.0]


@pytest.mark.parametrize(
    'mode',
    [StageTwoPointOne.Request.SKIP, StageTwoPointOne.Request.ROUTE],
)
def test_non_visual_modes_do_not_wait_for_visual_dependencies(mode):
    controller = make_dependency_controller(mode)
    visual_waits = []
    controller.detection_client.wait_for_service = (
        lambda timeout_sec: visual_waits.append('detect'))
    controller.align_client.wait_for_service = (
        lambda timeout_sec: visual_waits.append('align'))

    controller.wait_for_dependencies()

    assert visual_waits == []


def make_execute_controller(route):
    controller = StageTwoPointOneController.__new__(
        StageTwoPointOneController)
    controller.config_lock = threading.Lock()
    controller.cell_5_1_edge_pose = (1.0, -1.0, 0.0)
    controller.cell_5_2_edge_pose = (2.0, -2.0, 0.0)
    controller.cell_5_3_edge_pose = (3.0, -3.0, 0.0)
    controller.lift_initial = (0.01, 0.01)
    controller.lift_up = (0.2, 0.2)
    controller.lift_down = (0.01, 0.01)
    controller.current_lift = None
    controller.route_cells = route
    actions = []
    controller.set_lift = lambda lift: actions.append(('lift', lift))
    controller.move_to_loading_edge = (
        lambda pose: actions.append(('move', pose[0])))
    controller.process_cell = lambda pose: actions.append(('process', pose[0]))
    return controller, actions


def test_high_to_high_route_does_not_lower_or_repeat_lift():
    controller, actions = make_execute_controller((1, 3))

    controller.execute_task(StageTwoPointOne.Request.RED)

    assert actions == [
        ('lift', (0.01, 0.01)),
        ('move', 1.0),
        ('lift', (0.2, 0.2)),
        ('process', 1.0),
        ('move', 3.0),
        ('process', 3.0),
    ]


def test_height_changes_after_arriving_at_low_cell():
    controller, actions = make_execute_controller((1, 2))

    controller.execute_task(StageTwoPointOne.Request.RED)

    assert actions == [
        ('lift', (0.01, 0.01)),
        ('move', 1.0),
        ('lift', (0.2, 0.2)),
        ('process', 1.0),
        ('move', 2.0),
        ('lift', (0.01, 0.01)),
        ('process', 2.0),
    ]


def test_process_cell_behaviors_are_mode_specific():
    controller = StageTwoPointOneController.__new__(
        StageTwoPointOneController)
    calls = []
    controller.detect_kfs_type = (
        lambda: StageTwoPointOneController.KFS_TRUE_CLASS)
    controller.load_at_cell = (
        lambda pose, align: calls.append((pose, align)))

    controller.mode = StageTwoPointOne.Request.SKIP
    controller.process_cell((1.0, 0.0, 0.0))
    controller.mode = StageTwoPointOne.Request.ROUTE
    controller.process_cell((2.0, 0.0, 0.0))
    controller.mode = StageTwoPointOne.Request.STANDARD
    controller.process_cell((3.0, 0.0, 0.0))

    assert calls == [
        ((2.0, 0.0, 0.0), False),
        ((3.0, 0.0, 0.0), True),
    ]


def test_standard_mode_does_not_load_non_true_kfs():
    controller = StageTwoPointOneController.__new__(
        StageTwoPointOneController)
    controller.mode = StageTwoPointOne.Request.STANDARD
    controller.detect_kfs_type = lambda: 'fake'
    loads = []
    controller.load_at_cell = (
        lambda pose, align: loads.append((pose, align)))

    controller.process_cell((1.0, 0.0, 0.0))

    assert loads == []


def make_parameter_controller():
    controller = StageTwoPointOneController.__new__(
        StageTwoPointOneController)
    controller.config_lock = threading.Lock()
    controller.move_timeout_sec = 35.0
    controller.cell_5_1_edge_pose = (3.2, -4.2, 3.14)
    controller.lift_up = (0.2, 0.2)
    return controller


def test_dynamic_parameters_update_atomically():
    controller = make_parameter_controller()
    parameters = [
        SimpleNamespace(name='lift_up_front', value=0.25),
        SimpleNamespace(name='lift_up_rear', value=0.3),
    ]

    result = controller._on_parameters_changed(parameters)

    assert result.successful
    assert controller.lift_up == pytest.approx((0.25, 0.3))


def test_invalid_dynamic_parameter_preserves_configuration():
    controller = make_parameter_controller()

    result = controller._on_parameters_changed([
        SimpleNamespace(name='move_timeout_sec', value=-1.0),
    ])

    assert not result.successful
    assert controller.move_timeout_sec == pytest.approx(35.0)


def test_dynamic_pose_parameter_validates_before_update():
    controller = make_parameter_controller()
    original = controller.cell_5_1_edge_pose

    invalid_result = controller._on_parameters_changed([
        SimpleNamespace(
            name='cell_5_1_high_kfs_edge_pose',
            value=[1.0, 2.0],
        ),
    ])

    assert not invalid_result.successful
    assert controller.cell_5_1_edge_pose == pytest.approx(original)

    valid_result = controller._on_parameters_changed([
        SimpleNamespace(
            name='cell_5_1_high_kfs_edge_pose',
            value=[1.0, 2.0, 3.0],
        ),
    ])

    assert valid_result.successful
    assert controller.cell_5_1_edge_pose == pytest.approx((1.0, 2.0, 3.0))
