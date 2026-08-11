import math
import threading
from types import SimpleNamespace

import pytest

from robot_r2_control.gui_control import (
    GuiControlNode,
    manual_twist_components,
    motion_control_text,
    normalize_angle,
    normalize_motion_key,
    relative_pose_goal,
    velocity_test_twist_components,
)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeFuture:
    def __init__(self):
        self.callback = None
        self.response = None

    def add_done_callback(self, callback):
        self.callback = callback

    def result(self):
        return self.response

    def complete(self, response):
        self.response = response
        self.callback(self)


class FakeClient:
    def __init__(self, ready=True):
        self.ready = ready
        self.requests = []
        self.future = FakeFuture()

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return self.future


def make_node_stub():
    node = GuiControlNode.__new__(GuiControlNode)
    node.state_lock = threading.RLock()
    node.active_manual_keys = set()
    node.velocity_test_kind = None
    node.velocity_test_deadline = None
    node.pose_request_in_flight = False
    node.kfs_alignment_request_in_flight = False
    node.motion_config = dict(GuiControlNode.MOTION_PARAMETER_DEFAULTS)
    node.cmd_vel_publisher = FakePublisher()
    node.kfs_alignment_client = FakeClient()
    node.status_events = []
    return node


@pytest.mark.parametrize(
    'key, expected',
    [
        ('w', (0.2, 0.0, 0.0)),
        ('s', (-0.2, 0.0, 0.0)),
        ('a', (0.0, 0.2, 0.0)),
        ('d', (0.0, -0.2, 0.0)),
        ('q', (0.0, 0.0, 1.0)),
        ('e', (0.0, 0.0, -1.0)),
    ],
)
def test_manual_key_mapping_matches_teleop(key, expected):
    assert manual_twist_components({key}, 0.2, 1.0) == expected


@pytest.mark.parametrize('keysym', ['w', 'W', 'a', 'S', 'd', 'Q', 'e'])
def test_normalize_motion_key_accepts_supported_keys(keysym):
    assert normalize_motion_key(keysym) == keysym.lower()


def test_normalize_motion_key_ignores_other_keys():
    assert normalize_motion_key('space') is None


def test_manual_key_components_combine_and_cancel_opposites():
    assert manual_twist_components({'w', 'a', 'q'}, 0.2, 1.0) == (
        0.2, 0.2, 1.0)
    assert manual_twist_components(
        {'w', 's', 'a', 'd', 'q', 'e'}, 0.2, 1.0) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize(
    'kind, expected',
    [
        ('forward', (0.5, 0.0, 0.0)),
        ('left', (0.0, 0.5, 0.0)),
        ('rotate_left', (0.0, 0.0, 1.57)),
    ],
)
def test_velocity_test_mapping(kind, expected):
    assert velocity_test_twist_components(kind, 0.5, 1.57) == expected


def test_motion_control_text_uses_parameter_values():
    config = dict(GuiControlNode.MOTION_PARAMETER_DEFAULTS)
    config.update({
        'manual_linear_speed': 0.35,
        'manual_angular_speed': 1.2,
        'velocity_test_linear_speed': 0.75,
        'velocity_test_angular_speed': 2.5,
        'velocity_test_duration_sec': 1.25,
        'pose_test_linear_distance': 0.8,
        'pose_test_yaw': 0.785,
    })

    text = motion_control_text(config)

    assert '0.35 m/s' in text['keyboard_hint']
    assert '1.2 rad/s' in text['keyboard_hint']
    assert text['velocity']['forward'] == '0.75 m/s 前进 1.25 s'
    assert text['velocity']['rotate_left'] == (
        '2.5 rad/s 逆时针旋转 1.25 s')
    assert text['pose']['forward'] == '位置伺服前进 0.8 m'
    assert text['pose']['rotate_left'] == '位置伺服逆时针旋转 0.785 rad'
    assert text['kfs_alignment'] == 'KFS 对齐（容忍 10 px，超时 3 s）'


def test_kfs_alignment_uses_configured_tolerance_and_timeout():
    node = make_node_stub()

    success, message = node.request_kfs_alignment()

    assert success
    assert '容忍 10 px' in message
    assert '超时 3 s' in message
    request = node.kfs_alignment_client.requests[0]
    assert request.pixel_tolerance == pytest.approx(10.0)
    assert request.timeout_sec == pytest.approx(3.0)
    assert node.kfs_alignment_request_in_flight
    command = node.cmd_vel_publisher.messages[-1]
    assert command.linear.x == 0.0
    assert command.linear.y == 0.0
    assert command.angular.z == 0.0

    node.kfs_alignment_client.future.complete(SimpleNamespace(
        success=True,
        message='ok',
        final_offset_x=4,
    ))
    assert not node.kfs_alignment_request_in_flight
    assert node.pop_status_events() == ['KFS 对齐完成：最终偏差 4 px']


def test_kfs_alignment_reports_unavailable_service():
    node = make_node_stub()
    node.kfs_alignment_client = FakeClient(ready=False)

    success, message = node.request_kfs_alignment()

    assert not success
    assert message == '/r2/align_to_kfs 服务不可用'
    assert not node.kfs_alignment_request_in_flight
    assert not node.cmd_vel_publisher.messages


def test_repeated_key_press_is_ignored_and_release_publishes_zero():
    node = make_node_stub()

    assert node.press_manual_key('w')
    assert not node.press_manual_key('w')
    assert len(node.cmd_vel_publisher.messages) == 1
    assert node.release_manual_key('w')
    command = node.cmd_vel_publisher.messages[-1]
    assert command.linear.x == 0.0
    assert command.linear.y == 0.0
    assert command.angular.z == 0.0


def test_focus_loss_clears_keys_and_publishes_zero_once():
    node = make_node_stub()
    node.press_manual_key('w')
    node.press_manual_key('a')

    assert node.release_all_manual_keys()
    assert not node.active_manual_keys
    assert not node.release_all_manual_keys()
    assert len(node.cmd_vel_publisher.messages) == 3


def test_keyboard_press_cancels_velocity_test():
    node = make_node_stub()
    node.velocity_test_kind = 'forward'
    node.velocity_test_deadline = math.inf

    assert node.press_manual_key('a')
    assert node.velocity_test_kind is None
    assert node.velocity_test_deadline is None


def test_keyboard_is_ignored_during_pose_servo():
    node = make_node_stub()
    node.pose_request_in_flight = True

    assert not node.press_manual_key('w')
    assert not node.active_manual_keys
    assert not node.cmd_vel_publisher.messages


def test_relative_forward_uses_current_body_heading():
    target = relative_pose_goal((1.0, 2.0, math.pi / 2.0), 0.5, 0.0, 0.0)
    assert target == pytest.approx((1.0, 2.5, math.pi / 2.0))


def test_relative_left_uses_current_body_heading():
    target = relative_pose_goal((1.0, 2.0, math.pi / 2.0), 0.0, 0.5, 0.0)
    assert target == pytest.approx((0.5, 2.0, math.pi / 2.0))


def test_relative_rotation_is_counterclockwise_and_normalized():
    target = relative_pose_goal((1.0, 2.0, 3.0), 0.0, 0.0, 1.57)
    assert target[:2] == (1.0, 2.0)
    assert target[2] == pytest.approx(normalize_angle(4.57))


@pytest.mark.parametrize(
    'name, value',
    [
        ('motion_publish_rate', 0.0),
        ('manual_linear_speed', -0.1),
        ('velocity_test_duration_sec', math.inf),
        ('kfs_alignment_pixel_tolerance', 0.0),
        ('kfs_alignment_timeout_sec', -1.0),
    ],
)
def test_invalid_motion_parameters_are_rejected(name, value):
    config = dict(GuiControlNode.MOTION_PARAMETER_DEFAULTS)
    config[name] = value
    assert GuiControlNode._validate_motion_config(config)
