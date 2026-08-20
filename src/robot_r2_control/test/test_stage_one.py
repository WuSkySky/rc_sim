import math
import threading
from types import SimpleNamespace

import pytest
from rclpy.parameter import Parameter

from robot_r2_control.stage_one import (
    PARAMETER_DEFAULTS,
    StageOneController,
)
from robot_r2_interfaces.srv import DetectLed, MoveRelative, StageOne


class FakeLogger:
    def info(self, _message):
        pass


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
    controller.get_logger = lambda: FakeLogger()
    controller.weapon_rotate_client = 'rotate_client'
    controller.weapon_grip_client = 'grip_client'
    controller.calls = []
    controller.set_lift = lambda height, _config: controller.calls.append(
        ('lift', height))
    controller.move_relative = (
        lambda forward, left, yaw, _config: controller.calls.append(
            ('move', forward, left, yaw)))
    controller.align_tip = lambda _config: controller.calls.append(('align',))
    controller.set_weapon_pair = (
        lambda rotate, grip, _config: controller.calls.append(
            ('weapon_pair', rotate, grip)))
    controller.set_weapon_joint = (
        lambda client, _description, position, tolerance, _config:
        controller.calls.append(
            ('weapon_joint', client, position, tolerance)))
    controller.detect_led = lambda config: controller.calls.append(
        ('led_detect', config.led_target_states))
    return controller


def test_red_team_runs_all_numbered_actions_in_order():
    controller = make_sequence_controller()
    config = default_config()

    controller.execute_task(config, StageOne.Request.RED)

    assert controller.calls == [
        ('lift', 0.01),
        ('move', -0.887, 0.781, 0.0),
        ('lift', 0.14),
        ('align',),
        ('weapon_pair', math.pi / 2.0, 0.028),
        ('move', -0.10, 0.0, 0.0),
        ('weapon_joint', 'grip_client', 0.0, 0.001),
        ('lift', 0.21),
        (
            'weapon_joint',
            'rotate_client',
            math.radians(142.0),
            0.01,
        ),
        ('lift', 0.01),
        ('move', 0.20, 0.0, 0.0),
        ('move', 0.0, 0.0, math.pi),
        ('led_detect', (True,)),
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
        ('move', -0.887, 0.781, 0.0),
        ('lift', 0.14),
    ]


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


@pytest.mark.parametrize(
    'name,value',
    [
        ('action_2_left_m', -0.1),
        ('action_4_pixel_tolerance_px', 0.0),
        ('action_8_pre_lift_height_m', -0.1),
        ('weapon_timeout_sec', math.inf),
        ('led_target_states', []),
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


def test_parameter_update_accepts_led_targets_and_final_grip_position():
    controller = StageOneController.__new__(StageOneController)
    controller.config_lock = threading.RLock()
    controller._config = default_config()

    result = controller.on_parameters_changed([
        Parameter('led_target_states', value=[False, True]),
        Parameter('final_weapon_grip_m', value=0.025),
    ])

    assert result.successful
    updated = controller.config_snapshot()
    assert updated.led_target_states == (False, True)
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


def test_unavailable_dependency_aborts_before_actions():
    controller = StageOneController.__new__(StageOneController)
    controller.move_client = DependencyClient(True)
    controller.lift_client = DependencyClient(True)
    controller.align_client = DependencyClient(False)
    controller.weapon_rotate_client = DependencyClient(True)
    controller.weapon_grip_client = DependencyClient(True)
    controller.led_detect_client = DependencyClient(True)

    with pytest.raises(RuntimeError, match='AlignToTip service unavailable'):
        controller.wait_for_dependencies(0.1)


def test_unavailable_led_dependency_aborts_before_actions():
    controller = StageOneController.__new__(StageOneController)
    controller.move_client = DependencyClient(True)
    controller.lift_client = DependencyClient(True)
    controller.align_client = DependencyClient(True)
    controller.weapon_rotate_client = DependencyClient(True)
    controller.weapon_grip_client = DependencyClient(True)
    controller.led_detect_client = DependencyClient(False)

    with pytest.raises(RuntimeError, match='LedDetect service unavailable'):
        controller.wait_for_dependencies(0.1)


def test_blue_team_reverses_only_lateral_translation():
    controller = make_sequence_controller()

    controller.execute_task(default_config(), StageOne.Request.BLUE)

    assert controller.calls[1] == ('move', -0.887, -0.781, 0.0)
    assert controller.calls[5] == ('move', -0.10, 0.0, 0.0)
    assert controller.calls[10] == ('move', 0.20, 0.0, 0.0)
    assert controller.calls[11] == ('move', 0.0, 0.0, math.pi)


def test_led_detection_request_uses_configured_target_states():
    controller = StageOneController.__new__(StageOneController)
    controller.led_detect_client = FakeClient('led_detect', [])
    values = dict(PARAMETER_DEFAULTS)
    values['led_target_states'] = [True, False, True]

    controller.detect_led(StageOneController.config_from_values(values))

    request = controller.led_detect_client.requests[0]
    assert isinstance(request, DetectLed.Request)
    assert request.target_states == [True, False, True]


def test_led_detection_rejection_is_reported():
    controller = StageOneController.__new__(StageOneController)
    controller.led_detect_client = FakeClient(
        'led_detect', [],
        response=SimpleNamespace(
            success=False,
            message='target did not stabilize',
        ),
    )

    with pytest.raises(
        RuntimeError,
        match='LedDetect failed: target did not stabilize',
    ):
        controller.detect_led(default_config())


def test_led_detection_failure_keeps_weapon_gripper_closed():
    controller = make_sequence_controller()

    def fail_led_detection(config):
        controller.calls.append(('led_detect', config.led_target_states))
        raise RuntimeError('target not detected')

    controller.detect_led = fail_led_detection

    with pytest.raises(
        RuntimeError,
        match=r'Action 13 .*target not detected',
    ):
        controller.execute_task(default_config(), StageOne.Request.RED)

    assert controller.calls[-1] == ('led_detect', (True,))


@pytest.mark.parametrize('team', ['', 'RED', 'green'])
def test_invalid_team_is_rejected(team):
    with pytest.raises(ValueError, match='team must be red or blue'):
        StageOneController.validate_team(team)
