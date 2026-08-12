import threading
from types import SimpleNamespace

import pytest

from robot_r2_control.kfs_loader import (
    KfsLoaderController,
    MotionStep,
    TRAJECTORY_PARAMETER_NAMES,
    parse_sequence,
)
from robot_r2_interfaces.srv import KfsAction


class ImmediateFuture:
    def __init__(self, response):
        self.response = response

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self.response


class FakeClient:
    def __init__(self, responses=None):
        self.requests = []
        self.responses = list(responses or [])

    def wait_for_service(self, timeout_sec):
        del timeout_sec
        return True

    def call_async(self, request):
        self.requests.append(request)
        response = (
            self.responses.pop(0)
            if self.responses
            else SimpleNamespace(success=True, message='ok')
        )
        return ImmediateFuture(response)


def make_controller():
    controller = KfsLoaderController.__new__(KfsLoaderController)
    controller.operation_lock = threading.Lock()
    controller.config_lock = threading.Lock()
    controller.service_timeout_sec = 10.0
    controller.sequences = {
        name: parse_sequence(
            name,
            [0.0, 0.0, 0.1, 0.01, 0.01, 0.005],
        )
        for name in TRAJECTORY_PARAMETER_NAMES
    }
    controller.root_rotate_client = FakeClient()
    controller.tip_rotate_client = FakeClient()
    controller.grip_client = FakeClient()
    return controller


@pytest.mark.parametrize(
    'action, mode, method, expected',
    [
        (
            KfsAction.Request.LOAD,
            KfsAction.Request.FRONT,
            KfsAction.Request.STANDARD,
            'front_standard_sequence',
        ),
        (
            KfsAction.Request.LOAD,
            KfsAction.Request.FRONT,
            KfsAction.Request.TRANSFER,
            'front_transfer_sequence',
        ),
        (
            KfsAction.Request.LOAD,
            KfsAction.Request.TOP,
            KfsAction.Request.STANDARD,
            'top_standard_sequence',
        ),
        (
            KfsAction.Request.LOAD,
            KfsAction.Request.TOP,
            KfsAction.Request.TRANSFER,
            'top_transfer_sequence',
        ),
        (KfsAction.Request.RELEASE, 0, 0, 'release_sequence'),
        (KfsAction.Request.POP, 0, 0, 'pop_sequence'),
    ],
)
def test_action_request_selects_complete_trajectory(
    action, mode, method, expected
):
    request = SimpleNamespace(
        action=action,
        mode=mode,
        load_method=method,
    )

    assert KfsLoaderController.action_sequence_name(request) == expected


def test_unknown_action_is_rejected_without_executing_a_trajectory():
    controller = make_controller()
    request = SimpleNamespace(action='unknown', mode=0, load_method=0)
    response = SimpleNamespace(success=None, message='')
    executed = []
    controller.execute_sequence = lambda *args: executed.append(args)

    result = controller.handle_kfs_action(request, response)

    assert not result.success
    assert 'unsupported KFS action' in result.message
    assert executed == []


def test_parse_sequence_groups_each_six_values_into_one_step():
    sequence = parse_sequence(
        'test_sequence',
        [
            1.0, -1.0, 0.2, 0.01, 0.02, 0.003,
            0.0, 0.0, 0.1, 0.04, 0.05, 0.006,
        ],
    )

    assert sequence == (
        MotionStep(1.0, -1.0, 0.2, 0.01, 0.02, 0.003),
        MotionStep(0.0, 0.0, 0.1, 0.04, 0.05, 0.006),
    )


@pytest.mark.parametrize(
    'values, message',
    [
        ([], 'must not be empty'),
        ([0.0] * 5, 'multiple of 6'),
        ([0.0, 0.0, 0.1, 0.01, 0.01, float('nan')], 'must be finite'),
        ([2.4, 0.0, 0.1, 0.01, 0.01, 0.005], 'root position'),
        ([0.0, 0.1, 0.1, 0.01, 0.01, 0.005], 'tip position'),
        ([0.0, 0.0, 0.21, 0.01, 0.01, 0.005], 'grip position'),
        ([0.0, 0.0, 0.1, 0.0, 0.01, 0.005], 'greater than zero'),
    ],
)
def test_parse_sequence_rejects_invalid_trajectory(values, message):
    with pytest.raises(ValueError, match=message):
        parse_sequence('test_sequence', values)


def test_execute_sequence_sends_all_three_requests_before_waiting():
    controller = make_controller()
    sequence = (
        MotionStep(1.0, -1.0, 0.2, 0.01, 0.02, 0.003),
    )
    calls_seen_when_waiting = []

    def observe_futures(futures, timeout_sec, description):
        del futures, timeout_sec, description
        calls_seen_when_waiting.append((
            len(controller.root_rotate_client.requests),
            len(controller.tip_rotate_client.requests),
            len(controller.grip_client.requests),
        ))

    controller.wait_for_step_futures = observe_futures

    controller.execute_sequence('test_sequence', sequence, 3.0)

    assert calls_seen_when_waiting == [(1, 1, 1)]
    assert controller.root_rotate_client.requests[0].position == 1.0
    assert controller.tip_rotate_client.requests[0].position == -1.0
    assert controller.grip_client.requests[0].position == 0.2
    assert controller.root_rotate_client.requests[0].tolerance == 0.01
    assert controller.tip_rotate_client.requests[0].tolerance == 0.02
    assert controller.grip_client.requests[0].tolerance == 0.003


def test_repeated_target_is_sent_again_on_the_next_step():
    controller = make_controller()
    repeated_step = MotionStep(1.0, -1.0, 0.2, 0.01, 0.01, 0.005)

    controller.execute_sequence(
        'test_sequence',
        (repeated_step, repeated_step),
        3.0,
    )

    assert len(controller.root_rotate_client.requests) == 2
    assert len(controller.tip_rotate_client.requests) == 2
    assert len(controller.grip_client.requests) == 2


def test_motor_failure_aborts_remaining_steps():
    controller = make_controller()
    controller.tip_rotate_client = FakeClient([
        SimpleNamespace(success=False, message='stalled'),
    ])
    sequence = (
        MotionStep(1.0, -1.0, 0.2, 0.01, 0.01, 0.005),
        MotionStep(0.0, 0.0, 0.1, 0.01, 0.01, 0.005),
    )

    with pytest.raises(RuntimeError, match='step 1 failed.*tip: stalled'):
        controller.execute_sequence('test_sequence', sequence, 3.0)

    assert len(controller.root_rotate_client.requests) == 1
    assert len(controller.tip_rotate_client.requests) == 1
    assert len(controller.grip_client.requests) == 1


def test_valid_parameter_update_replaces_only_future_snapshot():
    controller = make_controller()
    old_sequence, _ = controller.snapshot_config('release_sequence')
    new_values = [0.5, -0.5, 0.2, 0.02, 0.02, 0.004]

    result = controller.on_parameters_changed([
        SimpleNamespace(name='release_sequence', value=new_values),
        SimpleNamespace(name='service_timeout_sec', value=4.0),
    ])

    new_sequence, new_timeout = controller.snapshot_config(
        'release_sequence')
    assert result.successful
    assert new_sequence != old_sequence
    assert old_sequence == (
        MotionStep(0.0, 0.0, 0.1, 0.01, 0.01, 0.005),
    )
    assert new_sequence == parse_sequence('release_sequence', new_values)
    assert new_timeout == 4.0


@pytest.mark.parametrize(
    'parameter',
    [
        SimpleNamespace(name='service_timeout_sec', value=0.0),
        SimpleNamespace(name='service_timeout_sec', value=float('inf')),
        SimpleNamespace(name='front_standard_sequence', value=[]),
    ],
)
def test_invalid_parameter_update_preserves_configuration(parameter):
    controller = make_controller()
    old_sequences = dict(controller.sequences)
    old_timeout = controller.service_timeout_sec

    result = controller.on_parameters_changed([parameter])

    assert not result.successful
    assert controller.sequences == old_sequences
    assert controller.service_timeout_sec == old_timeout


def test_multi_parameter_update_is_atomic_when_one_value_is_invalid():
    controller = make_controller()
    old_sequences = dict(controller.sequences)

    result = controller.on_parameters_changed([
        SimpleNamespace(
            name='release_sequence',
            value=[0.5, -0.5, 0.2, 0.02, 0.02, 0.004],
        ),
        SimpleNamespace(name='service_timeout_sec', value=-1.0),
    ])

    assert not result.successful
    assert controller.sequences == old_sequences
    assert controller.service_timeout_sec == 10.0


def test_all_six_trajectory_parameters_accept_valid_sequences():
    assert 'pop_sequence' in TRAJECTORY_PARAMETER_NAMES
    assert len(TRAJECTORY_PARAMETER_NAMES) == 6
    for parameter_name in TRAJECTORY_PARAMETER_NAMES:
        assert parse_sequence(
            parameter_name,
            [0.0, 0.0, 0.1, 0.01, 0.01, 0.005],
        )
