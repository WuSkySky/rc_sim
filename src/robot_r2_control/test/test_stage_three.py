import math
import threading
from types import SimpleNamespace

import pytest

from robot_r2_control.stage_three import StageThreeController
from robot_r2_interfaces.srv import (
    KfsAction,
    MoveRelative,
    MoveToPose,
    SetJointPosition,
    StageThree,
)


class ImmediateFuture:
    def __init__(self, response):
        self.response = response

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self.response


class FakeClient:
    def __init__(self, name, actions, success=True, available=True):
        self.name = name
        self.actions = actions
        self.success = success
        self.available = available
        self.requests = []

    def wait_for_service(self, timeout_sec):
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        self.actions.append((self.name, request))
        return ImmediateFuture(SimpleNamespace(
            success=self.success,
            message='ok' if self.success else 'failed',
        ))


def default_config():
    return {
        'dependency_timeout_sec': 2.0,
        'relocalization_timeout_sec': 5.0,
        'move_timeout_sec': 35.0,
        'pop_timeout_sec': 70.0,
        'kfs_lift_timeout_sec': 15.0,
        'stage_two_exit_endpoint_x': -5.5,
        'first_target_x_offset': 0.21,
        'target_x_spacing': 0.54,
        'target_y_offset': 4.63,
        'intermediate_backoff_distance': 0.25,
        'standard_final_backoff_distance': 3.0,
        'single_final_backoff_distance': 2.0,
        'kfs_lift_height': 0.35,
        'kfs_lift_tolerance': 0.005,
    }


def make_controller():
    controller = StageThreeController.__new__(StageThreeController)
    controller.service_lock = threading.Lock()
    controller.config_lock = threading.Lock()
    controller.config = default_config()
    controller.blue_relocalization_pose = (
        -5.69, 5.53, 0.0, 0.0, 0.0, math.pi / 2.0)
    controller.actions = []
    controller.set_base_pose_client = FakeClient(
        'relocalize', controller.actions)
    controller.move_to_pose_client = FakeClient(
        'move_to', controller.actions)
    controller.move_relative_client = FakeClient(
        'move_relative', controller.actions)
    controller.kfs_action_client = FakeClient(
        'pop', controller.actions)
    controller.kfs_lift_client = FakeClient(
        'lift', controller.actions)
    return controller


@pytest.mark.parametrize(
    'team, expected_relocalization_y, expected_target_y, expected_yaw',
    [
        (StageThree.Request.BLUE, 5.53, 10.16, math.pi / 2.0),
        (StageThree.Request.RED, -5.53, -10.16, -math.pi / 2.0),
    ],
)
def test_task_config_mirrors_y_and_yaw(
        team, expected_relocalization_y, expected_target_y, expected_yaw):
    controller = make_controller()

    config = controller.task_config(team, 3)

    assert config['relocalization_pose'] == pytest.approx(
        (-5.69, expected_relocalization_y, 0.0, 0.0, 0.0, expected_yaw))
    expected_targets = (
        (-5.29, expected_target_y, expected_yaw),
        (-4.75, expected_target_y, expected_yaw),
        (-4.21, expected_target_y, expected_yaw),
    )
    for actual, expected in zip(config['targets'], expected_targets):
        assert actual == pytest.approx(expected)


@pytest.mark.parametrize(
    'loaded_count, expected_modes, expected_lifts, expected_backoffs',
    [
        (3, (1, 2, 2), (False, False, True), (0.25, 0.25, 3.0)),
        (2, (2, 2), (False, True), (0.25, 3.0)),
        (1, (2,), (True,), (2.0,)),
    ],
)
def test_loaded_count_selects_pop_suffix_and_backoffs(
        loaded_count, expected_modes, expected_lifts, expected_backoffs):
    controller = make_controller()

    config = controller.task_config(
        StageThree.Request.BLUE, loaded_count)

    assert tuple(step[0] for step in config['pop_plan']) == expected_modes
    assert tuple(step[1] for step in config['pop_plan']) == expected_lifts
    assert config['backoff_distances'] == expected_backoffs
    assert len(config['targets']) == loaded_count


@pytest.mark.parametrize('loaded_count', [0, 4])
def test_invalid_loaded_count_is_rejected_before_relocalization(loaded_count):
    controller = make_controller()
    response = SimpleNamespace(success=None, message='')

    result = controller.handle_task(SimpleNamespace(
        team=StageThree.Request.BLUE,
        loaded_count=loaded_count,
    ), response)

    assert not result.success
    assert 'loaded_count must be 1, 2 or 3' in result.message
    assert controller.actions == []


def test_invalid_team_is_rejected_before_relocalization():
    controller = make_controller()
    response = SimpleNamespace(success=None, message='')

    result = controller.handle_task(SimpleNamespace(
        team='green',
        loaded_count=3,
    ), response)

    assert not result.success
    assert 'team must be red or blue' in result.message
    assert controller.actions == []


@pytest.mark.parametrize(
    'loaded_count, expected_order, expected_modes, expected_backoffs',
    [
        (
            3,
            (
                'relocalize',
                'move_to', 'pop', 'move_relative',
                'move_to', 'pop', 'move_relative',
                'move_to', 'lift', 'pop', 'move_relative',
            ),
            (1, 2, 2),
            (-0.25, -0.25, -3.0),
        ),
        (
            2,
            (
                'relocalize',
                'move_to', 'pop', 'move_relative',
                'move_to', 'lift', 'pop', 'move_relative',
            ),
            (2, 2),
            (-0.25, -3.0),
        ),
        (
            1,
            ('relocalize', 'move_to', 'lift', 'pop', 'move_relative'),
            (2,),
            (-2.0,),
        ),
    ],
)
def test_service_executes_expected_action_sequence(
        loaded_count, expected_order, expected_modes, expected_backoffs):
    controller = make_controller()
    response = SimpleNamespace(success=None, message='')

    result = controller.handle_task(SimpleNamespace(
        team=StageThree.Request.BLUE,
        loaded_count=loaded_count,
    ), response)

    assert result.success
    assert tuple(action[0] for action in controller.actions) == expected_order
    assert tuple(
        request.mode
        for request in controller.kfs_action_client.requests
    ) == expected_modes
    assert all(
        request.action == KfsAction.Request.POP
        for request in controller.kfs_action_client.requests
    )
    assert tuple(
        request.forward
        for request in controller.move_relative_client.requests
    ) == expected_backoffs
    assert all(
        request.pose_source == MoveRelative.Request.ODIN
        for request in controller.move_relative_client.requests
    )
    assert all(
        request.pose_source == MoveToPose.Request.ODIN
        for request in controller.move_to_pose_client.requests
    )

    if controller.kfs_lift_client.requests:
        lift_request = controller.kfs_lift_client.requests[0]
        assert isinstance(lift_request, SetJointPosition.Request)
        assert lift_request.position == pytest.approx(0.35)
        assert lift_request.tolerance == pytest.approx(0.005)


def test_service_uses_only_first_two_absolute_targets_for_two_kfs():
    controller = make_controller()
    response = SimpleNamespace(success=None, message='')

    controller.handle_task(SimpleNamespace(
        team=StageThree.Request.RED,
        loaded_count=2,
    ), response)

    targets = tuple(
        (request.x, request.y, request.yaw)
        for request in controller.move_to_pose_client.requests
    )
    expected_targets = (
        (-5.29, -10.16, -math.pi / 2.0),
        (-4.75, -10.16, -math.pi / 2.0),
    )
    for actual, expected in zip(targets, expected_targets):
        assert actual == pytest.approx(expected)


def test_action_dependency_failure_happens_after_relocalization():
    controller = make_controller()
    controller.move_to_pose_client.available = False
    response = SimpleNamespace(success=None, message='')

    result = controller.handle_task(SimpleNamespace(
        team=StageThree.Request.BLUE,
        loaded_count=3,
    ), response)

    assert not result.success
    assert result.message == 'MoveToPose service unavailable'
    assert tuple(action[0] for action in controller.actions) == (
        'relocalize',)


def test_pop_failure_stops_remaining_actions():
    controller = make_controller()
    controller.kfs_action_client.success = False
    response = SimpleNamespace(success=None, message='')

    result = controller.handle_task(SimpleNamespace(
        team=StageThree.Request.BLUE,
        loaded_count=3,
    ), response)

    assert not result.success
    assert result.message == 'KfsAction pop failed: failed'
    assert tuple(action[0] for action in controller.actions) == (
        'relocalize', 'move_to', 'pop')


def test_dynamic_parameter_update_is_atomic_and_validated():
    controller = make_controller()

    result = controller._on_parameters_changed([
        SimpleNamespace(name='target_x_spacing', value=0.6),
        SimpleNamespace(name='kfs_lift_height', value=0.36),
    ])

    assert result.successful
    assert controller.config['target_x_spacing'] == pytest.approx(0.6)
    assert controller.config['kfs_lift_height'] == pytest.approx(0.36)

    original = dict(controller.config)
    result = controller._on_parameters_changed([
        SimpleNamespace(name='target_x_spacing', value=-0.1),
        SimpleNamespace(name='kfs_lift_height', value=0.37),
    ])
    assert not result.successful
    assert controller.config == original
