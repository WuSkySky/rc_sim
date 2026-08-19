import math
import threading
import time
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
import pytest
from rclpy.parameter import Parameter

from robot_r2_controller.chassis_pose_servo import (
    ODIN_SOURCE,
    SERIAL_SOURCE,
    PidAxis,
    PoseServo,
    relative_pose_goal,
)


def make_pose(x, y, yaw):
    message = PoseStamped()
    message.pose.position.x = float(x)
    message.pose.position.y = float(y)
    message.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def make_controller_state():
    controller = PoseServo.__new__(PoseServo)
    controller.state_condition = threading.Condition()
    controller.current_poses = {
        SERIAL_SOURCE: None,
        ODIN_SOURCE: None,
    }
    controller.pose_sequences = {
        SERIAL_SOURCE: 0,
        ODIN_SOURCE: 0,
    }
    controller.position_tolerance = 0.005
    controller.yaw_tolerance = 0.01
    controller.default_timeout_sec = 20.0
    controller.initial_pose_timeout_sec = 0.001
    return controller


def make_request(**overrides):
    values = {
        'pose_source': SERIAL_SOURCE,
        'x': 0.0,
        'y': 0.0,
        'yaw': 0.0,
        'forward': 0.0,
        'left': 0.0,
        'yaw_delta': 0.0,
        'position_tolerance': 0.0,
        'yaw_tolerance': 0.0,
        'timeout_sec': 0.0,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_relative_pose_goal_uses_starting_body_heading():
    target = relative_pose_goal(
        (1.0, 2.0, math.pi / 2.0),
        forward=-0.887,
        left=0.781,
        yaw_delta=0.25,
    )

    assert target == pytest.approx(
        (1.0 - 0.781, 2.0 - 0.887, math.pi / 2.0 + 0.25))


@pytest.mark.parametrize('source', ['', 'SERIAL', 'camera'])
def test_unknown_pose_source_is_rejected(source):
    with pytest.raises(ValueError, match='pose_source'):
        PoseServo.validate_pose_source(source)


def test_pose_callbacks_keep_serial_and_odin_independent():
    controller = make_controller_state()

    controller.on_current_pose(SERIAL_SOURCE, make_pose(1.0, 2.0, 0.1))
    controller.on_current_pose(ODIN_SOURCE, make_pose(9.0, 8.0, -0.2))

    assert controller.current_poses[SERIAL_SOURCE].pose.position.x == 1.0
    assert controller.current_poses[ODIN_SOURCE].pose.position.x == 9.0
    assert controller.pose_sequences == {
        SERIAL_SOURCE: 1,
        ODIN_SOURCE: 1,
    }


def test_relative_goal_uses_only_selected_source_pose():
    controller = make_controller_state()
    request = make_request(
        pose_source=SERIAL_SOURCE,
        forward=0.1,
        left=0.0,
        yaw_delta=0.0,
    )

    goal, timeout_sec = controller._relative_goal(
        request,
        SERIAL_SOURCE,
        make_pose(1.0, 2.0, math.pi / 2.0),
    )

    assert goal['pose_source'] == SERIAL_SOURCE
    assert (goal['x'], goal['y'], goal['yaw']) == pytest.approx(
        (1.0, 2.1, math.pi / 2.0))
    assert timeout_sec == pytest.approx(20.0)


def test_serial_relative_then_odin_absolute_requires_no_continuity():
    controller = make_controller_state()
    serial_goal, _ = controller._relative_goal(
        make_request(forward=0.1),
        SERIAL_SOURCE,
        make_pose(10.0, 0.0, 0.0),
    )
    odin_goal, _ = controller._absolute_goal(
        make_request(
            pose_source=ODIN_SOURCE,
            x=2.0,
            y=3.0,
            yaw=0.5,
        ),
        ODIN_SOURCE,
    )

    assert serial_goal['x'] == pytest.approx(10.1)
    assert serial_goal['pose_source'] == SERIAL_SOURCE
    assert odin_goal['x'] == pytest.approx(2.0)
    assert odin_goal['y'] == pytest.approx(3.0)
    assert odin_goal['pose_source'] == ODIN_SOURCE


def test_goal_status_comes_from_selected_source():
    controller = make_controller_state()
    controller.current_poses[SERIAL_SOURCE] = make_pose(100.0, 100.0, 1.0)
    controller.current_poses[ODIN_SOURCE] = make_pose(2.0, 3.0, 0.5)
    goal = {
        'pose_source': ODIN_SOURCE,
        'x': 2.0,
        'y': 3.0,
        'yaw': 0.5,
    }

    status = controller.get_goal_status(goal)

    assert status == pytest.approx((2.0, 3.0, 0.5, 0.0, 0.0))


def test_wait_for_fresh_pose_times_out_for_only_selected_source():
    controller = make_controller_state()
    controller.current_poses[SERIAL_SOURCE] = make_pose(1.0, 0.0, 0.0)
    controller.pose_sequences[SERIAL_SOURCE] = 1

    with pytest.raises(RuntimeError, match="source 'odin'"):
        controller.wait_for_fresh_pose(ODIN_SOURCE)


class FakeTimer:
    timer_period_ns = 20_000_000


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def add_control_state(controller):
    controller.completion_wait_sec = 0.0
    controller.yaw_stable_cycles_required = 10
    controller.yaw_stable_cycle_count = 0
    controller.yaw_small_error_gain_multiplier = 11.0
    controller.x_pid = PidAxis(1.0, 0.0, 0.03, 0.3, 0.3)
    controller.y_pid = PidAxis(1.0, 0.0, 0.03, 0.3, 0.3)
    controller.yaw_pid = PidAxis(0.3, 0.0, 0.02, 10.0, 0.6)
    controller.cmd_vel_publisher = FakePublisher()


def test_control_loop_ignores_unselected_pose_source():
    controller = make_controller_state()
    add_control_state(controller)
    controller.current_poses[SERIAL_SOURCE] = make_pose(0.0, 0.0, 0.0)
    controller.current_poses[ODIN_SOURCE] = make_pose(100.0, 0.0, 0.0)
    controller.active_goal = {
        'pose_source': SERIAL_SOURCE,
        'x': 1.0,
        'y': 0.0,
        'yaw': 0.0,
        'position_tolerance': 0.005,
        'yaw_tolerance': 0.01,
    }
    controller.goal_completed = False
    controller.last_tick = time.monotonic() - 0.1

    controller.control_loop()

    assert controller.cmd_vel_publisher.messages[-1].linear.x > 0.0


def test_execute_goal_stops_at_request_timeout(monkeypatch):
    controller = make_controller_state()
    add_control_state(controller)
    controller.current_poses[SERIAL_SOURCE] = make_pose(0.0, 0.0, 0.0)
    response = SimpleNamespace()
    goal = {
        'pose_source': SERIAL_SOURCE,
        'x': 1.0,
        'y': 0.0,
        'yaw': 0.0,
        'position_tolerance': 0.005,
        'yaw_tolerance': 0.01,
    }
    monkeypatch.setattr(
        'robot_r2_controller.chassis_pose_servo.rclpy.ok', lambda: True)

    result = controller.execute_goal(
        goal, 0.001, response, 'MoveRelative')

    assert not result.success
    assert result.message == 'MoveRelative timeout'
    assert controller.active_goal is None
    assert controller.cmd_vel_publisher.messages


def test_initial_pose_timeout_supports_valid_dynamic_update():
    controller = make_controller_state()
    controller.timer = FakeTimer()
    add_control_state(controller)

    result = controller.on_parameters_changed([
        Parameter('initial_pose_timeout_sec', value=3.0),
    ])

    assert result.successful
    assert controller.initial_pose_timeout_sec == pytest.approx(3.0)


def test_initial_pose_timeout_rejects_invalid_dynamic_update():
    controller = make_controller_state()
    controller.timer = FakeTimer()
    add_control_state(controller)
    original = controller.initial_pose_timeout_sec

    result = controller.on_parameters_changed([
        Parameter('initial_pose_timeout_sec', value=0.0),
    ])

    assert not result.successful
    assert controller.initial_pose_timeout_sec == original
