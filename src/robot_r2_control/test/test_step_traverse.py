import threading
from types import SimpleNamespace

import pytest

from robot_r2_control.step_traverse import StepTraverseController
from robot_r2_interfaces.srv import MoveRelative


class ImmediateFuture:
    def __init__(self, response):
        self.response = response

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self.response


class FakeClient:
    def __init__(self):
        self.requests = []

    def call_async(self, request):
        self.requests.append(request)
        return ImmediateFuture(SimpleNamespace(success=True, message='ok'))


def make_controller():
    controller = StepTraverseController.__new__(StepTraverseController)
    controller.config_lock = threading.Lock()
    controller.up_pre_lift_clearance = 0.05
    controller.a1 = 0.05
    controller.a1_backoff = 0.015
    controller.a2 = 0.42
    controller.a2_backoff = 0.015
    controller.a3 = 0.35
    controller.a3_linear_speed_limit = 1.125
    controller.b1 = 0.22
    controller.b2 = 0.44
    controller.b3 = 0.16
    controller.lift_all = (0.20, 0.20)
    controller.lift_front_only = (0.20, 0.01)
    controller.lift_rear_only = (0.01, 0.20)
    controller.lift_down = (0.01, 0.01)
    return controller


def test_up_step_prepositions_before_lifting_and_moves_relative_segments():
    controller = make_controller()
    calls = []
    controller.move_relative = (
        lambda distance, linear_speed_limit=0.0: calls.append(
            ('move', distance, linear_speed_limit)))
    controller.set_lift = lambda positions: calls.append(
        ('lift', positions))

    controller.run_up_step(0.20)

    assert calls == [
        ('move', pytest.approx(0.15), 0.0),
        ('lift', controller.lift_all),
        ('move', pytest.approx(0.05), 0.0),
        ('move', pytest.approx(-0.015), 0.0),
        ('lift', controller.lift_rear_only),
        ('move', pytest.approx(0.42), 0.0),
        ('move', pytest.approx(-0.015), 0.0),
        ('lift', controller.lift_down),
        ('move', pytest.approx(0.35), 1.125),
    ]


def test_down_step_moves_relative_segments_with_lifts_between():
    controller = make_controller()
    calls = []
    controller.move_relative = lambda distance: calls.append(
        ('move', distance))
    controller.set_lift = lambda positions: calls.append(
        ('lift', positions))

    controller.run_down_step(0.20)

    assert calls == [
        ('move', pytest.approx(0.42)),
        ('lift', controller.lift_front_only),
        ('move', pytest.approx(0.44)),
        ('lift', controller.lift_all),
        ('move', pytest.approx(0.16)),
        ('lift', controller.lift_down),
    ]


def test_up_step_reverses_to_clearance_when_already_too_close():
    controller = make_controller()
    moves = []
    controller.move_relative = (
        lambda distance, linear_speed_limit=0.0: moves.append(distance))
    controller.set_lift = lambda positions: None

    controller.run_up_step(0.03)

    assert moves[0] == pytest.approx(-0.02)


def test_up_step_does_not_lift_when_prepositioning_fails():
    controller = make_controller()
    lift_calls = []

    def fail_move(_distance):
        raise RuntimeError('preposition failed')

    controller.move_relative = fail_move
    controller.set_lift = lambda positions: lift_calls.append(positions)

    with pytest.raises(RuntimeError, match='preposition failed'):
        controller.run_up_step(0.20)
    assert lift_calls == []


def test_move_relative_uses_serial_source_and_forwards_distance():
    controller = StepTraverseController.__new__(StepTraverseController)
    controller.move_client = FakeClient()
    controller.move_timeout_sec = 35.0

    controller.move_relative(-0.02)

    request = controller.move_client.requests[0]
    assert request.pose_source == MoveRelative.Request.SERIAL
    assert request.forward == pytest.approx(-0.02)
    assert request.left == pytest.approx(0.0)
    assert request.yaw_delta == pytest.approx(0.0)
    assert request.linear_speed_limit == pytest.approx(0.0)
    assert request.position_tolerance == pytest.approx(0.0)
    assert request.yaw_tolerance == pytest.approx(0.0)
    assert request.timeout_sec == pytest.approx(35.0)


def test_move_relative_forwards_linear_speed_limit():
    controller = StepTraverseController.__new__(StepTraverseController)
    controller.move_client = FakeClient()
    controller.move_timeout_sec = 35.0

    controller.move_relative(0.35, 1.125)

    request = controller.move_client.requests[0]
    assert request.linear_speed_limit == pytest.approx(1.125)


@pytest.mark.parametrize('value', [-0.01, float('inf'), float('nan'), True])
def test_invalid_pre_lift_clearance_update_is_rejected(value):
    controller = make_controller()
    parameter = SimpleNamespace(
        name='up_pre_lift_clearance',
        value=value,
    )

    result = controller.on_parameters_changed([parameter])

    assert not result.successful
    assert controller.up_pre_lift_clearance == 0.05


def test_valid_pre_lift_clearance_update_is_applied():
    controller = make_controller()
    parameter = SimpleNamespace(
        name='up_pre_lift_clearance',
        value=0.08,
    )

    result = controller.on_parameters_changed([parameter])

    assert result.successful
    assert controller.up_pre_lift_clearance == 0.08


@pytest.mark.parametrize('name, original, updated', [
    ('a1', 0.05, 0.12),
    ('a1_backoff', 0.015, 0.02),
    ('a2', 0.42, 0.50),
    ('a2_backoff', 0.015, 0.03),
    ('a3', 0.35, 0.30),
    ('b1', 0.22, 0.26),
    ('b2', 0.44, 0.40),
    ('b3', 0.16, 0.20),
])
def test_move_distance_parameters_update_at_runtime(name, original, updated):
    controller = make_controller()

    result = controller.on_parameters_changed([
        SimpleNamespace(name=name, value=updated),
    ])

    assert result.successful
    assert getattr(controller, name) == updated


@pytest.mark.parametrize('name', [
    'a1', 'a1_backoff', 'a2', 'a2_backoff', 'a3',
    'b1', 'b2', 'b3', 'up_pre_lift_clearance'])
@pytest.mark.parametrize('value', [-0.01, float('inf'), float('nan'), True])
def test_invalid_move_distance_update_is_rejected(name, value):
    controller = make_controller()
    original = getattr(controller, name)

    result = controller.on_parameters_changed([
        SimpleNamespace(name=name, value=value),
    ])

    assert not result.successful
    assert getattr(controller, name) == original


def test_a3_linear_speed_limit_updates_at_runtime():
    controller = make_controller()

    result = controller.on_parameters_changed([
        SimpleNamespace(name='a3_linear_speed_limit', value=0.9),
    ])

    assert result.successful
    assert controller.a3_linear_speed_limit == 0.9


@pytest.mark.parametrize('value', [-0.01, float('inf'), float('nan'), True])
def test_invalid_a3_linear_speed_limit_is_rejected(value):
    controller = make_controller()

    result = controller.on_parameters_changed([
        SimpleNamespace(name='a3_linear_speed_limit', value=value),
    ])

    assert not result.successful
    assert controller.a3_linear_speed_limit == 1.125


def test_lift_pair_parameters_update_at_runtime():
    controller = make_controller()

    result = controller.on_parameters_changed([
        SimpleNamespace(name='lift_all_front', value=0.30),
        SimpleNamespace(name='lift_rear_only_rear', value=0.25),
    ])

    assert result.successful
    assert controller.lift_all == (0.30, 0.20)
    assert controller.lift_rear_only == (0.01, 0.25)


@pytest.mark.parametrize('value', [float('inf'), float('nan'), True])
def test_invalid_lift_parameter_update_is_rejected(value):
    controller = make_controller()

    result = controller.on_parameters_changed([
        SimpleNamespace(name='lift_down_front', value=value),
    ])

    assert not result.successful
    assert controller.lift_down == (0.01, 0.01)


def test_parameter_update_is_atomic_when_any_value_invalid():
    controller = make_controller()

    result = controller.on_parameters_changed([
        SimpleNamespace(name='a1', value=0.12),
        SimpleNamespace(name='b1', value=-1.0),
    ])

    assert not result.successful
    assert controller.a1 == 0.05
    assert controller.b1 == 0.22


def test_unrelated_parameter_update_is_ignored():
    controller = make_controller()

    result = controller.on_parameters_changed([
        SimpleNamespace(name='unrelated', value=42),
    ])

    assert result.successful
    assert controller.a1 == 0.05
