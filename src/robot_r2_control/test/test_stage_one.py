import math
import threading
from types import SimpleNamespace

import pytest
from rclpy.parameter import Parameter

from robot_r2_control.stage_one import (
    PARAMETER_DEFAULTS,
    StageOneController,
)
from robot_r2_interfaces.srv import MoveRelative, StageOne


class FakeLogger:
    def __init__(self):
        self.warnings = []

    def info(self, _message):
        pass

    def warn(self, message):
        self.warnings.append(message)


class ImmediateFuture:
    def __init__(self, response):
        self.response = response

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self.response


class FakeClient:
    def __init__(self, name, call_order, response=None):
        self.name = name
        self.call_order = call_order
        self.requests = []
        self.response = response or SimpleNamespace(
            success=True, message='ok')

    def call_async(self, request):
        self.call_order.append(self.name)
        self.requests.append(request)
        return ImmediateFuture(self.response)


class DependencyClient:
    def __init__(self, available):
        self.available = available

    def wait_for_service(self, timeout_sec):
        del timeout_sec
        return self.available


def default_config():
    return StageOneController.config_from_values(
        dict(PARAMETER_DEFAULTS))


def make_sequence_controller():
    controller = StageOneController.__new__(StageOneController)
    controller.logger = FakeLogger()
    controller.get_logger = lambda: controller.logger
    controller.weapon_rotate_client = 'rotate_client'
    controller.weapon_grip_client = 'grip_client'
    controller.calls = []
    controller.set_lift = (
        lambda height, _config, allow_timeout=False:
        controller.calls.append(('lift', height)))
    controller.move_relative = (
        lambda forward, left, yaw, _config, timeout_sec=None,
        allow_timeout=False: controller.calls.append(
            ('move', forward, left, yaw)))
    controller.align_tip = lambda _config: controller.calls.append(('align',))
    controller.set_weapon_pair = (
        lambda rotate, grip, _config: controller.calls.append(
            ('weapon_pair', rotate, grip)))
    controller.move_and_set_weapon_pair = (
        lambda forward, left, yaw, rotate, grip, _config:
        controller.calls.append(
            ('move_and_weapon_pair', forward, left, yaw, rotate, grip)))
    controller.set_weapon_joint = (
        lambda client, _description, position, tolerance, _config:
        controller.calls.append(
            ('weapon_joint', client, position, tolerance)))
    controller.wait_before_release = lambda config: controller.calls.append(
        ('release_delay', config.final_release_delay_sec))
    return controller


def test_red_team_runs_all_numbered_actions_in_order():
    controller = make_sequence_controller()
    config = default_config()

    controller.execute_task(config, StageOne.Request.RED)

    assert controller.calls == [
        ('lift', 0.01),
        (
            'move_and_weapon_pair',
            -0.887,
            0.781,
            0.0,
            math.pi / 2.0,
            0.028,
        ),
        ('lift', 0.14),
        ('align',),
        ('move', -0.10, 0.0, 0.0),
        ('weapon_joint', 'grip_client', 0.0, 0.001),
        ('move', 0.03, 0.0, 0.0),
        ('lift', 0.17),
        (
            'weapon_joint',
            'rotate_client',
            math.radians(142.0),
            0.01,
        ),
        ('move', 0.05, 0.0, 0.0),
        ('lift', 0.01),
        ('move', 0.20, 0.0, 0.0),
        ('move', 0.0, 0.0, math.pi),
        ('release_delay', 25.0),
        ('weapon_joint', 'grip_client', 0.028, 0.001),
    ]


def test_action_failure_stops_later_actions_and_reports_number():
    controller = make_sequence_controller()
    config = default_config()
    lift_call_count = 0

    def fail_second_lift(height, _config):
        nonlocal lift_call_count
        lift_call_count += 1
        controller.calls.append(('lift', height))
        if lift_call_count == 2:
            raise RuntimeError('lift rejected')

    controller.set_lift = fail_second_lift

    with pytest.raises(RuntimeError, match=r'Action 3 .*lift rejected'):
        controller.execute_task(config, StageOne.Request.RED)

    assert controller.calls == [
        ('lift', 0.01),
        (
            'move_and_weapon_pair',
            -0.887,
            0.781,
            0.0,
            math.pi / 2.0,
            0.028,
        ),
        ('lift', 0.14),
    ]


def test_alignment_timeout_continues_remaining_actions_and_records_warning():
    controller = make_sequence_controller()

    def time_out_alignment(_config):
        controller.calls.append(('align',))
        return 'Alignment timeout: no target detected'

    controller.align_tip = time_out_alignment
    warnings = []

    result = controller.execute_task(
        default_config(), StageOne.Request.RED, warnings)

    assert len(controller.calls) == 15
    assert controller.calls[3] == ('align',)
    assert controller.calls[-1] == (
        'weapon_joint', 'grip_client', 0.028, 0.001)
    assert result == tuple(warnings)
    assert len(warnings) == 1
    assert 'Alignment timeout: no target detected' in warnings[0]
    assert 'remaining actions continued' in warnings[0]
    assert controller.logger.warnings == warnings


def test_backward_timeout_uses_ten_seconds_and_continues():
    controller = make_sequence_controller()
    timeout_calls = []

    def time_out_backward(
        forward,
        left,
        yaw,
        _config,
        timeout_sec=None,
        allow_timeout=False,
    ):
        controller.calls.append(('move', forward, left, yaw))
        if allow_timeout and forward < 0.0:
            timeout_calls.append(timeout_sec)
            return 'MoveRelative timeout'
        return None

    controller.move_relative = time_out_backward
    warnings = []

    result = controller.execute_task(
        default_config(), StageOne.Request.RED, warnings)

    assert timeout_calls == [pytest.approx(10.0)]
    assert len(controller.calls) == 15
    assert controller.calls[4] == ('move', -0.10, 0.0, 0.0)
    assert controller.calls[5] == (
        'weapon_joint', 'grip_client', 0.0, 0.001)
    assert controller.calls[-1] == (
        'weapon_joint', 'grip_client', 0.028, 0.001)
    assert result == tuple(warnings)
    assert len(warnings) == 1
    assert 'Action 5' in warnings[0]
    assert 'MoveRelative timeout' in warnings[0]
    assert 'remaining actions continued' in warnings[0]
    assert controller.logger.warnings == warnings


def test_backward_non_timeout_failure_still_stops_stage_one():
    controller = make_sequence_controller()

    def reject_backward(
        forward,
        left,
        yaw,
        _config,
        timeout_sec=None,
        allow_timeout=False,
    ):
        controller.calls.append(('move', forward, left, yaw))
        if allow_timeout and forward < 0.0:
            raise RuntimeError('MoveRelative failed: invalid pose')

    controller.move_relative = reject_backward

    with pytest.raises(
        RuntimeError,
        match=r'Action 5 .*MoveRelative failed: invalid pose',
    ):
        controller.execute_task(default_config(), StageOne.Request.RED)

    assert controller.calls[-1] == ('move', -0.10, 0.0, 0.0)


def test_pre_lift_move_timeout_uses_five_seconds_and_continues():
    controller = make_sequence_controller()
    timeout_calls = []

    def time_out_pre_lift_move(
        forward,
        left,
        yaw,
        _config,
        timeout_sec=None,
        allow_timeout=False,
    ):
        controller.calls.append(('move', forward, left, yaw))
        if allow_timeout and forward > 0.0:
            timeout_calls.append(timeout_sec)
            return 'MoveRelative timeout'
        return None

    controller.move_relative = time_out_pre_lift_move
    warnings = []

    result = controller.execute_task(
        default_config(), StageOne.Request.RED, warnings)

    assert timeout_calls == [pytest.approx(5.0)]
    assert len(controller.calls) == 15
    assert controller.calls[6] == ('move', 0.03, 0.0, 0.0)
    assert controller.calls[7] == ('lift', 0.17)
    assert controller.calls[-1] == (
        'weapon_joint', 'grip_client', 0.028, 0.001)
    assert result == tuple(warnings)
    assert len(warnings) == 1
    assert 'Action 7' in warnings[0]
    assert 'MoveRelative timeout' in warnings[0]
    assert 'remaining actions continued' in warnings[0]
    assert controller.logger.warnings == warnings


def test_pre_lift_move_non_timeout_failure_still_stops_stage_one():
    controller = make_sequence_controller()

    def reject_pre_lift_move(
        forward,
        left,
        yaw,
        _config,
        timeout_sec=None,
        allow_timeout=False,
    ):
        controller.calls.append(('move', forward, left, yaw))
        if allow_timeout and forward > 0.0:
            raise RuntimeError('MoveRelative failed: invalid pose')

    controller.move_relative = reject_pre_lift_move

    with pytest.raises(
        RuntimeError,
        match=r'Action 7 .*MoveRelative failed: invalid pose',
    ):
        controller.execute_task(default_config(), StageOne.Request.RED)

    assert controller.calls[-1] == ('move', 0.03, 0.0, 0.0)


def test_incremental_lift_timeout_continues_remaining_actions_with_warning():
    controller = make_sequence_controller()

    def time_out_incremental_lift(height, _config, allow_timeout=False):
        controller.calls.append(('lift', height))
        if allow_timeout:
            return 'SetLift timeout'
        return None

    controller.set_lift = time_out_incremental_lift
    warnings = []

    result = controller.execute_task(
        default_config(), StageOne.Request.RED, warnings)

    assert len(controller.calls) == 15
    assert controller.calls[7] == ('lift', 0.17)
    assert controller.calls[8] == (
        'weapon_joint', 'rotate_client', math.radians(142.0), 0.01)
    assert controller.calls[-1] == (
        'weapon_joint', 'grip_client', 0.028, 0.001)
    assert result == tuple(warnings)
    assert len(warnings) == 1
    assert 'Action 8' in warnings[0]
    assert 'SetLift timeout' in warnings[0]
    assert 'remaining actions continued' in warnings[0]
    assert controller.logger.warnings == warnings


def test_incremental_lift_non_timeout_failure_still_stops_stage_one():
    controller = make_sequence_controller()

    def reject_incremental_lift(height, _config, allow_timeout=False):
        controller.calls.append(('lift', height))
        if allow_timeout:
            raise RuntimeError('SetLift failed: invalid feedback')

    controller.set_lift = reject_incremental_lift

    with pytest.raises(
        RuntimeError,
        match=r'Action 8 .*SetLift failed: invalid feedback',
    ):
        controller.execute_task(default_config(), StageOne.Request.RED)

    assert controller.calls[-1] == ('lift', 0.17)


def test_set_lift_allows_only_explicit_service_timeout():
    timeout_controller = StageOneController.__new__(StageOneController)
    timeout_controller.lift_client = FakeClient(
        'lift', [],
        response=SimpleNamespace(
            success=False,
            message='SetLift timeout',
        ),
    )

    warning = timeout_controller.set_lift(
        0.17, default_config(), allow_timeout=True)

    assert warning == 'SetLift timeout'
    request = timeout_controller.lift_client.requests[0]
    assert request.front_lift == pytest.approx(0.17)
    assert request.rear_lift == pytest.approx(0.17)

    strict_controller = StageOneController.__new__(StageOneController)
    strict_controller.lift_client = FakeClient(
        'lift', [],
        response=SimpleNamespace(
            success=False,
            message='SetLift timeout',
        ),
    )

    with pytest.raises(
        RuntimeError,
        match='SetLift failed: SetLift timeout',
    ):
        strict_controller.set_lift(0.14, default_config())

    rejected_controller = StageOneController.__new__(StageOneController)
    rejected_controller.lift_client = FakeClient(
        'lift', [],
        response=SimpleNamespace(
            success=False,
            message='invalid feedback',
        ),
    )

    with pytest.raises(
        RuntimeError,
        match='SetLift failed: invalid feedback',
    ):
        rejected_controller.set_lift(
            0.17, default_config(), allow_timeout=True)


def test_align_tip_returns_timeout_message_instead_of_failing():
    controller = StageOneController.__new__(StageOneController)
    controller.align_client = FakeClient(
        'align', [],
        response=SimpleNamespace(
            success=False,
            message='Alignment timeout: target lost',
        ),
    )

    warning = controller.align_tip(default_config())

    assert warning == 'Alignment timeout: target lost'
    request = controller.align_client.requests[0]
    assert request.pixel_tolerance == pytest.approx(5.0)
    assert request.timeout_sec == pytest.approx(15.0)


def test_align_tip_non_timeout_failure_still_fails_stage_one():
    controller = StageOneController.__new__(StageOneController)
    controller.align_client = FakeClient(
        'align', [],
        response=SimpleNamespace(
            success=False,
            message='aborted by /r2/system/abort',
        ),
    )

    with pytest.raises(
        RuntimeError,
        match='AlignToTip failed: aborted by /r2/system/abort',
    ):
        controller.align_tip(default_config())


def test_response_message_includes_alignment_warning_on_success_or_failure():
    warnings = [
        'Action 4 (align weapon tip) timed out; remaining actions continued: '
        'Alignment timeout: no target detected',
    ]

    success_message = StageOneController.response_message(
        'Stage 1 completed for red team', warnings)
    failure_message = StageOneController.response_message(
        'Action 5 (move backward) failed: rejected', warnings)

    assert success_message.startswith('Stage 1 completed for red team;')
    assert 'Alignment timeout: no target detected' in success_message
    assert failure_message.startswith('Action 5 (move backward) failed:')
    assert 'Alignment timeout: no target detected' in failure_message


def make_handle_controller(execute_task):
    controller = StageOneController.__new__(StageOneController)
    controller.service_lock = threading.Lock()
    controller.config_lock = threading.RLock()
    controller._config = default_config()
    controller.wait_for_dependencies = lambda _timeout: None
    controller.execute_task = execute_task
    return controller


def test_handle_task_succeeds_and_reports_alignment_timeout_warning():
    warning = (
        'Action 4 (align weapon tip) timed out; remaining actions continued: '
        'Alignment timeout: target lost'
    )

    def execute(_config, _team, warnings):
        warnings.append(warning)

    controller = make_handle_controller(execute)
    response = controller.handle_task(
        SimpleNamespace(team=StageOne.Request.RED),
        SimpleNamespace(success=False, message=''),
    )

    assert response.success
    assert response.message.startswith('Stage 1 completed for red team;')
    assert warning in response.message


def test_handle_task_failure_keeps_earlier_alignment_timeout_warning():
    warning = (
        'Action 4 (align weapon tip) timed out; remaining actions continued: '
        'Alignment timeout: no target detected'
    )

    def execute(_config, _team, warnings):
        warnings.append(warning)
        raise RuntimeError('Action 5 (move backward) failed: rejected')

    controller = make_handle_controller(execute)
    response = controller.handle_task(
        SimpleNamespace(team=StageOne.Request.BLUE),
        SimpleNamespace(success=True, message=''),
    )

    assert not response.success
    assert response.message.startswith('Action 5 (move backward) failed:')
    assert warning in response.message


def test_weapon_pair_dispatches_both_requests_before_waiting():
    controller = StageOneController.__new__(StageOneController)
    call_order = []
    controller.weapon_rotate_client = FakeClient('rotate', call_order)
    controller.weapon_grip_client = FakeClient('grip', call_order)

    controller.set_weapon_pair(math.pi / 2.0, 0.028, default_config())

    assert call_order == ['rotate', 'grip']
    rotate_request = controller.weapon_rotate_client.requests[0]
    grip_request = controller.weapon_grip_client.requests[0]
    assert rotate_request.position == pytest.approx(math.pi / 2.0)
    assert rotate_request.tolerance == pytest.approx(0.01)
    assert grip_request.position == pytest.approx(0.028)
    assert grip_request.tolerance == pytest.approx(0.001)


def test_initial_move_and_weapon_pair_dispatch_all_requests_together():
    controller = StageOneController.__new__(StageOneController)
    call_order = []
    controller.move_client = FakeClient('move', call_order)
    controller.weapon_rotate_client = FakeClient('rotate', call_order)
    controller.weapon_grip_client = FakeClient('grip', call_order)

    controller.move_and_set_weapon_pair(
        -0.887,
        0.781,
        0.0,
        math.pi / 2.0,
        0.028,
        default_config(),
    )

    assert call_order == ['move', 'rotate', 'grip']
    move_request = controller.move_client.requests[0]
    assert move_request.pose_source == MoveRelative.Request.SERIAL
    assert move_request.forward == pytest.approx(-0.887)
    assert move_request.left == pytest.approx(0.781)
    assert controller.weapon_rotate_client.requests[0].position == (
        pytest.approx(math.pi / 2.0))
    assert controller.weapon_grip_client.requests[0].position == (
        pytest.approx(0.028))


@pytest.mark.parametrize(
    'name,value',
    [
        ('action_2_left_m', -0.1),
        ('action_4_pixel_tolerance_px', 0.0),
        ('action_6_backward_timeout_sec', 0.0),
        ('action_8_pre_lift_forward_m', -0.1),
        ('action_8_pre_lift_forward_timeout_sec', 0.0),
        ('action_8_lift_increment_m', -0.1),
        ('final_release_delay_sec', 0.0),
        ('weapon_timeout_sec', math.inf),
    ],
)
def test_invalid_config_is_rejected(name, value):
    values = dict(PARAMETER_DEFAULTS)
    values[name] = value

    with pytest.raises(ValueError, match=name):
        StageOneController.config_from_values(values)


def test_parameter_update_atomically_replaces_config_snapshot():
    controller = StageOneController.__new__(StageOneController)
    controller.config_lock = threading.RLock()
    controller._config = default_config()
    original = controller.config_snapshot()

    result = controller.on_parameters_changed([
        Parameter('action_6_backward_m', value=0.15),
        Parameter('position_tolerance_m', value=0.006),
    ])

    assert result.successful
    assert original.action_6_backward_m == pytest.approx(0.10)
    assert original.position_tolerance_m == pytest.approx(0.005)
    updated = controller.config_snapshot()
    assert updated.action_6_backward_m == pytest.approx(0.15)
    assert updated.position_tolerance_m == pytest.approx(0.006)


def test_parameter_update_accepts_release_delay_and_final_grip_position():
    controller = StageOneController.__new__(StageOneController)
    controller.config_lock = threading.RLock()
    controller._config = default_config()

    result = controller.on_parameters_changed([
        Parameter('final_release_delay_sec', value=30.0),
        Parameter('final_weapon_grip_m', value=0.025),
    ])

    assert result.successful
    updated = controller.config_snapshot()
    assert updated.final_release_delay_sec == pytest.approx(30.0)
    assert updated.final_weapon_grip_m == pytest.approx(0.025)


def test_move_relative_uses_serial_source_and_forwards_request_values():
    controller = StageOneController.__new__(StageOneController)
    controller.move_client = FakeClient('move', [])

    controller.move_relative(-0.1, 0.2, math.pi, default_config())

    request = controller.move_client.requests[0]
    assert request.pose_source == MoveRelative.Request.SERIAL
    assert request.forward == pytest.approx(-0.1)
    assert request.left == pytest.approx(0.2)
    assert request.yaw_delta == pytest.approx(math.pi)
    assert request.position_tolerance == pytest.approx(0.005)
    assert request.yaw_tolerance == pytest.approx(0.01)
    assert request.timeout_sec == pytest.approx(35.0)


def test_move_relative_allows_only_explicit_service_timeout():
    timeout_controller = StageOneController.__new__(StageOneController)
    timeout_controller.move_client = FakeClient(
        'move', [],
        response=SimpleNamespace(
            success=False,
            message='MoveRelative timeout',
        ),
    )

    warning = timeout_controller.move_relative(
        0.03,
        0.0,
        0.0,
        default_config(),
        timeout_sec=5.0,
        allow_timeout=True,
    )

    assert warning == 'MoveRelative timeout'
    request = timeout_controller.move_client.requests[0]
    assert request.forward == pytest.approx(0.03)
    assert request.timeout_sec == pytest.approx(5.0)

    strict_controller = StageOneController.__new__(StageOneController)
    strict_controller.move_client = FakeClient(
        'move', [],
        response=SimpleNamespace(
            success=False,
            message='MoveRelative timeout',
        ),
    )

    with pytest.raises(
        RuntimeError,
        match='MoveRelative failed: MoveRelative timeout',
    ):
        strict_controller.move_relative(
            0.03, 0.0, 0.0, default_config(), timeout_sec=5.0)

    rejected_controller = StageOneController.__new__(StageOneController)
    rejected_controller.move_client = FakeClient(
        'move', [],
        response=SimpleNamespace(
            success=False,
            message='invalid pose',
        ),
    )

    with pytest.raises(
        RuntimeError,
        match='MoveRelative failed: invalid pose',
    ):
        rejected_controller.move_relative(
            0.03,
            0.0,
            0.0,
            default_config(),
            timeout_sec=5.0,
            allow_timeout=True,
        )


def test_unavailable_dependency_aborts_before_actions():
    controller = StageOneController.__new__(StageOneController)
    controller.move_client = DependencyClient(True)
    controller.lift_client = DependencyClient(True)
    controller.align_client = DependencyClient(False)
    controller.weapon_rotate_client = DependencyClient(True)
    controller.weapon_grip_client = DependencyClient(True)

    with pytest.raises(RuntimeError, match='AlignToTip service unavailable'):
        controller.wait_for_dependencies(0.1)


def test_wait_for_dependencies_does_not_require_led_service():
    controller = StageOneController.__new__(StageOneController)
    controller.move_client = DependencyClient(True)
    controller.lift_client = DependencyClient(True)
    controller.align_client = DependencyClient(True)
    controller.weapon_rotate_client = DependencyClient(True)
    controller.weapon_grip_client = DependencyClient(True)

    controller.wait_for_dependencies(0.1)


def test_blue_team_reverses_only_lateral_translation():
    controller = make_sequence_controller()

    controller.execute_task(default_config(), StageOne.Request.BLUE)

    assert controller.calls[1] == (
        'move_and_weapon_pair',
        -0.887,
        -0.781,
        0.0,
        math.pi / 2.0,
        0.028,
    )
    assert controller.calls[4] == ('move', -0.10, 0.0, 0.0)
    assert controller.calls[6] == ('move', 0.03, 0.0, 0.0)
    assert controller.calls[9] == ('move', 0.05, 0.0, 0.0)
    assert controller.calls[11] == ('move', 0.20, 0.0, 0.0)
    assert controller.calls[12] == ('move', 0.0, 0.0, math.pi)


def test_release_delay_waits_for_configured_duration():
    controller = StageOneController.__new__(StageOneController)
    waits = []
    controller.wait_for_event_or_abort = (
        lambda event, timeout: waits.append((event, timeout)) or False)

    controller.wait_before_release(default_config())

    assert len(waits) == 1
    event, timeout = waits[0]
    assert isinstance(event, threading.Event)
    assert not event.is_set()
    assert timeout == pytest.approx(25.0)


def test_release_delay_abort_keeps_weapon_gripper_closed():
    controller = make_sequence_controller()

    def abort_release_delay(config):
        controller.calls.append(
            ('release_delay', config.final_release_delay_sec))
        raise RuntimeError('aborted by /r2/system/abort')

    controller.wait_before_release = abort_release_delay

    with pytest.raises(
        RuntimeError,
        match=r'Action 14 .*aborted by /r2/system/abort',
    ):
        controller.execute_task(default_config(), StageOne.Request.RED)

    assert controller.calls[-1] == ('release_delay', 25.0)


@pytest.mark.parametrize('team', ['', 'RED', 'green'])
def test_invalid_team_is_rejected(team):
    with pytest.raises(ValueError, match='team must be red or blue'):
        StageOneController.validate_team(team)
