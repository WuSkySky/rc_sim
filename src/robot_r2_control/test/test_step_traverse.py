import threading
from types import SimpleNamespace

import pytest

from robot_r2_control.step_traverse import StepTraverseController


def make_controller():
    controller = StepTraverseController.__new__(StepTraverseController)
    controller.config_lock = threading.Lock()
    controller.up_pre_lift_clearance = 0.05
    controller.a1 = 0.05
    controller.a2 = 0.42
    controller.a3 = 0.35
    controller.lift_all = (0.20, 0.20)
    controller.lift_rear_only = (0.0, 0.20)
    controller.lift_down = (0.0, 0.0)
    return controller


def test_up_step_prepositions_before_lifting_and_keeps_existing_targets():
    controller = make_controller()
    calls = []
    controller.move_from_start = lambda pose, distance: calls.append(
        ('move', pose, distance))
    controller.set_lift = lambda positions: calls.append(
        ('lift', positions))
    start_pose = (1.0, 2.0, 0.5)

    controller.run_up_step(start_pose, 0.20)

    assert calls == [
        ('move', start_pose, pytest.approx(0.15)),
        ('lift', controller.lift_all),
        ('move', start_pose, pytest.approx(0.25)),
        ('lift', controller.lift_rear_only),
        ('move', start_pose, pytest.approx(0.67)),
        ('lift', controller.lift_down),
        ('move', start_pose, pytest.approx(1.02)),
    ]


def test_up_step_reverses_to_clearance_when_already_too_close():
    controller = make_controller()
    moves = []
    controller.move_from_start = lambda pose, distance: moves.append(distance)
    controller.set_lift = lambda positions: None

    controller.run_up_step((0.0, 0.0, 0.0), 0.03)

    assert moves[0] == pytest.approx(-0.02)


def test_up_step_does_not_lift_when_prepositioning_fails():
    controller = make_controller()
    lift_calls = []

    def fail_move(_pose, _distance):
        raise RuntimeError('preposition failed')

    controller.move_from_start = fail_move
    controller.set_lift = lambda positions: lift_calls.append(positions)

    with pytest.raises(RuntimeError, match='preposition failed'):
        controller.run_up_step((0.0, 0.0, 0.0), 0.20)
    assert lift_calls == []


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
