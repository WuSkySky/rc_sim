import math
import threading
from types import SimpleNamespace

import pytest

from robot_r2_control.gui_control import (
    GuiControlNode,
    make_parameter_load_command,
    manual_twist_components,
    motion_control_text,
    normalize_angle,
    normalize_motion_key,
    parse_relocalization_values,
    relative_pose_goal,
    resolve_kfs_loader_source_config,
    summarize_parameter_load_result,
    velocity_test_twist_components,
)
from robot_r2_interfaces.srv import KfsAction, StageTwoPointTwo, TraverseStep


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
    node.kfs_load_feedback_generation = 0
    node.kfs_load_motor_feedback = {
        name: None for name in GuiControlNode.KFS_LOAD_MOTOR_FEEDBACK_TOPICS
    }
    node.active_manual_keys = set()
    node.velocity_test_kind = None
    node.velocity_test_deadline = None
    node.pose_request_in_flight = False
    node.kfs_alignment_request_in_flight = False
    node.tip_alignment_request_in_flight = False
    node.relocalization_request_in_flight = False
    node.step_test_request_in_flight = False
    node.step_test_direction = None
    node.stage_two_point_one_request_in_flight = False
    node.stage_two_point_one_skip = None
    node.stage_two_point_two_request_in_flight = False
    node.stage_two_point_two_skip = None
    node.kfs_action_request_in_flight = False
    node.motion_config = dict(GuiControlNode.MOTION_PARAMETER_DEFAULTS)
    node.cmd_vel_publisher = FakePublisher()
    node.kfs_alignment_client = FakeClient()
    node.tip_alignment_client = FakeClient()
    node.kfs_action_client = FakeClient()
    node.set_base_pose_client = FakeClient()
    node.step_traverse_client = FakeClient()
    node.stage_two_point_one_client = FakeClient()
    node.stage_two_point_two_client = FakeClient()
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


def test_kfs_loader_config_is_resolved_from_workspace_source(tmp_path):
    package_share = (
        tmp_path / 'install/robot_r2_control/share/robot_r2_control')
    installed_config = package_share / 'config/kfs_loader.yaml'
    installed_config.parent.mkdir(parents=True)
    installed_config.write_text('installed', encoding='utf-8')
    source_config = (
        tmp_path / 'src/robot_r2_control/config/kfs_loader.yaml')
    source_config.parent.mkdir(parents=True)
    source_config.write_text('source', encoding='utf-8')

    result = resolve_kfs_loader_source_config(package_share)

    assert result == str(source_config)


def test_parameter_load_command_does_not_use_ros_daemon():
    command = make_parameter_load_command('/workspace/config.yaml')

    assert command == [
        'ros2', 'param', 'load',
        '--no-daemon', '--spin-time', '2.0',
        '/kfs_loader_control', '/workspace/config.yaml',
    ]


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
    assert text['kfs_alignment'] == 'KFS 对齐'
    assert text['tip_alignment'] == '端头对齐'


def test_kfs_alignment_defers_tolerance_to_node_parameters():
    node = make_node_stub()

    success, message = node.request_kfs_alignment()

    assert success
    assert message == '已发送 KFS 对齐请求'
    request = node.kfs_alignment_client.requests[0]
    assert request.pixel_tolerance == 0.0
    assert request.timeout_sec == 0.0
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


def test_tip_alignment_defers_tolerance_to_node_parameters():
    node = make_node_stub()

    success, message = node.request_tip_alignment()

    assert success
    assert message == '已发送端头对齐请求'
    request = node.tip_alignment_client.requests[0]
    assert request.pixel_tolerance == 0.0
    assert request.timeout_sec == 0.0
    assert node.tip_alignment_request_in_flight
    command = node.cmd_vel_publisher.messages[-1]
    assert command.linear.x == 0.0
    assert command.linear.y == 0.0
    assert command.angular.z == 0.0

    node.tip_alignment_client.future.complete(SimpleNamespace(
        success=True,
        message='ok',
        final_offset_x=4,
    ))
    assert not node.tip_alignment_request_in_flight
    assert node.pop_status_events() == ['端头对齐完成：最终偏差 4 px']


def test_tip_alignment_reports_unavailable_service():
    node = make_node_stub()
    node.tip_alignment_client = FakeClient(ready=False)

    success, message = node.request_tip_alignment()

    assert not success
    assert message == '/r2/align_to_tip 服务不可用'
    assert not node.tip_alignment_request_in_flight
    assert not node.cmd_vel_publisher.messages


def test_kfs_alignment_is_blocked_while_tip_alignment_runs():
    node = make_node_stub()
    node.tip_alignment_request_in_flight = True

    success, message = node.request_kfs_alignment()

    assert not success
    assert message == '端头对齐正在执行'
    assert not node.cmd_vel_publisher.messages


def test_tip_alignment_is_blocked_while_kfs_alignment_runs():
    node = make_node_stub()
    node.kfs_alignment_request_in_flight = True

    success, message = node.request_tip_alignment()

    assert not success
    assert message == 'KFS 对齐正在执行'
    assert not node.cmd_vel_publisher.messages


def test_relocalization_sends_six_pose_values():
    node = make_node_stub()

    success, message = node.request_relocalization(
        ('1.5', '-0.5', '0.1', '0.2', '-0.3', '1.57'))

    assert success
    assert 'x=1.500 m' in message
    request = node.set_base_pose_client.requests[0]
    assert (
        request.x,
        request.y,
        request.z,
        request.roll,
        request.pitch,
        request.yaw,
    ) == pytest.approx((1.5, -0.5, 0.1, 0.2, -0.3, 1.57))
    assert node.relocalization_request_in_flight

    node.set_base_pose_client.future.complete(SimpleNamespace(
        success=True,
        message='base_link pose updated',
    ))
    assert not node.relocalization_request_in_flight
    assert node.pop_status_events() == [
        '重定位完成：base_link pose updated',
    ]


def test_relocalization_reports_service_failure():
    node = make_node_stub()

    success, _ = node.request_relocalization(('0',) * 6)
    assert success
    node.set_base_pose_client.future.complete(SimpleNamespace(
        success=False,
        message='failed to read current base_link pose',
    ))

    assert not node.relocalization_request_in_flight
    assert node.pop_status_events() == [
        '重定位失败：failed to read current base_link pose',
    ]


def test_relocalization_reports_unavailable_service():
    node = make_node_stub()
    node.set_base_pose_client = FakeClient(ready=False)

    success, message = node.request_relocalization(('0',) * 6)

    assert not success
    assert message == '/r2/set_base_pose 服务不可用'
    assert not node.set_base_pose_client.requests


@pytest.mark.parametrize(
    'values, error',
    [
        (('', '0', '0', '0', '0', '0'), 'x 必须是数值'),
        (('nan', '0', '0', '0', '0', '0'), 'x 必须是有限数值'),
        (('0', '0', '0', '0', 'inf', '0'), 'pitch 必须是有限数值'),
    ],
)
def test_invalid_relocalization_values_are_rejected(values, error):
    node = make_node_stub()

    success, message = node.request_relocalization(values)

    assert not success
    assert error in message
    assert not node.set_base_pose_client.requests


def test_up_step_test_traverses_at_zero_distance():
    node = make_node_stub()

    success, message = node.request_up_step_test()

    assert success
    assert message == '上台阶测试：正在跨越（距离 0.0 m）'
    assert not node.set_base_pose_client.requests
    assert node.step_test_request_in_flight

    step_request = node.step_traverse_client.requests[0]
    assert step_request.direction == TraverseStep.Request.UP
    assert step_request.distance_to_step == 0.0

    node.step_traverse_client.future.complete(SimpleNamespace(
        success=True,
        message='up step traversal completed',
    ))
    assert not node.step_test_request_in_flight
    assert node.pop_status_events() == [
        '上台阶测试完成：up step traversal completed',
    ]


def test_up_step_test_reports_step_failure_and_releases_busy_state():
    node = make_node_stub()

    success, _ = node.request_up_step_test()
    assert success
    node.step_traverse_client.future.complete(SimpleNamespace(
        success=False,
        message='Pose feedback unavailable',
    ))

    assert not node.step_test_request_in_flight
    assert node.pop_status_events()[-1] == (
        '上台阶失败：Pose feedback unavailable')


def test_stage_two_point_one_relocalizes_to_middle_edge_then_calls_service():
    node = make_node_stub()

    success, message = node.request_stage_two_point_one(skip=True)

    assert success
    assert message == '2.1 skip测试：正在重定位到中间台阶边缘'
    request = node.set_base_pose_client.requests[0]
    assert (request.x, request.y, request.yaw) == (3.15, -3.0, math.pi)
    assert request.z == 0.0
    assert request.roll == 0.0
    assert request.pitch == 0.0
    assert node.stage_two_point_one_request_in_flight
    assert not node.stage_two_point_one_client.requests

    node.set_base_pose_client.future.complete(SimpleNamespace(
        success=True,
        message='base_link pose updated',
    ))
    step_request = node.stage_two_point_one_client.requests[0]
    assert step_request.loaded_count == 0
    assert step_request.skip_kfs_detection is True
    assert node.stage_two_point_one_request_in_flight

    node.stage_two_point_one_client.future.complete(SimpleNamespace(
        success=True,
        message='Stage 2.1 completed',
    ))
    assert not node.stage_two_point_one_request_in_flight
    assert node.pop_status_events() == [
        '2.1 skip测试：重定位完成，正在调用 2.1',
        '2.1 skip测试完成：Stage 2.1 completed',
    ]


def test_stage_two_point_one_normal_passes_skip_false():
    node = make_node_stub()

    success, _ = node.request_stage_two_point_one(skip=False)
    assert success

    node.set_base_pose_client.future.complete(SimpleNamespace(
        success=True,
        message='base_link pose updated',
    ))
    step_request = node.stage_two_point_one_client.requests[0]
    assert step_request.loaded_count == 0
    assert step_request.skip_kfs_detection is False


def test_stage_two_point_two_relocalizes_then_calls_service():
    node = make_node_stub()

    success, message = node.request_stage_two_point_two(skip=True)

    assert success
    assert message == '2.2 skip测试：正在重定位到 (5,2)-(4,2) 边界'
    request = node.set_base_pose_client.requests[0]
    assert (request.x, request.y, request.yaw) == (3.15, -3.0, math.pi)
    assert node.stage_two_point_two_request_in_flight
    assert not node.stage_two_point_two_client.requests

    node.set_base_pose_client.future.complete(SimpleNamespace(
        success=True,
        message='base_link pose updated',
    ))
    step_request = node.stage_two_point_two_client.requests[0]
    assert step_request.fake_kfs_decision == StageTwoPointTwo.Request.LEFT
    assert step_request.loaded_count == 0
    assert step_request.skip_kfs_detection is True

    node.stage_two_point_two_client.future.complete(SimpleNamespace(
        success=True,
        message='Stage 2.2 completed at (0, 1)',
    ))
    assert not node.stage_two_point_two_request_in_flight
    assert node.pop_status_events() == [
        '2.2 skip测试：重定位完成，正在调用 2.2',
        '2.2 skip测试完成：Stage 2.2 completed at (0, 1)',
    ]


def test_parse_relocalization_values_requires_six_values():
    with pytest.raises(ValueError, match='需要 6 个参数'):
        parse_relocalization_values(('0',) * 5)


@pytest.mark.parametrize(
    'action, expected_mode, label',
    [
        (
            KfsAction.Request.LOAD,
            KfsAction.Request.MODE_1,
            '模式 1：前方装载（当前数量 0，装载到车上）',
        ),
        (
            KfsAction.Request.LOAD,
            KfsAction.Request.MODE_2,
            '模式 2：前方装载（当前数量 2，留在夹爪）',
        ),
        (
            KfsAction.Request.LOAD,
            KfsAction.Request.MODE_3,
            '模式 3：上方装载（当前数量 0，装载到车上）',
        ),
        (
            KfsAction.Request.LOAD,
            KfsAction.Request.MODE_4,
            '模式 4：上方装载（当前数量 2，留在夹爪）',
        ),
        (
            KfsAction.Request.LOAD,
            KfsAction.Request.MODE_5,
            '模式 5：上方装载（当前数量 1）',
        ),
        (KfsAction.Request.RELEASE, 0, '释放'),
        (
            KfsAction.Request.POP,
            KfsAction.Request.MODE_1,
            '弹出模式 1：从夹爪直接放置',
        ),
        (
            KfsAction.Request.POP,
            KfsAction.Request.MODE_2,
            '弹出模式 2：从车上拿取并放置',
        ),
    ],
)
def test_kfs_action_sends_expected_request(
        action, expected_mode, label):
    node = make_node_stub()

    success, message = node.request_kfs_action(action, expected_mode)

    assert success
    assert message == f'已发送 KFS {label}请求'
    request = node.kfs_action_client.requests[0]
    assert request.action == action
    assert request.mode == expected_mode
    assert node.kfs_action_request_in_flight

    node.kfs_action_client.future.complete(SimpleNamespace(
        success=True,
        message='completed',
    ))
    assert not node.kfs_action_request_in_flight
    assert node.pop_status_events() == [
        f'KFS {label}完成：completed',
    ]


def test_kfs_action_rejects_concurrent_request():
    node = make_node_stub()
    node.kfs_action_request_in_flight = True

    success, message = node.request_kfs_action(
        KfsAction.Request.LOAD,
        KfsAction.Request.MODE_1,
    )

    assert not success
    assert message == 'KFS 动作正在执行'
    assert not node.kfs_action_client.requests


@pytest.mark.parametrize('mode', [0, 3])
def test_kfs_action_rejects_unknown_pop_mode(mode):
    node = make_node_stub()

    with pytest.raises(ValueError, match='Unknown KFS pop mode'):
        node.request_kfs_action(KfsAction.Request.POP, mode)

    assert not node.kfs_action_client.requests


def test_kfs_action_reports_unavailable_service():
    node = make_node_stub()
    node.kfs_action_client = FakeClient(ready=False)

    success, message = node.request_kfs_action(KfsAction.Request.RELEASE)

    assert not success
    assert message == '/r2/kfs/action 服务不可用'
    assert not node.kfs_action_request_in_flight


@pytest.mark.parametrize(
    'motor_name, value',
    [
        ('root_rotate', 1.25),
        ('tip_rotate', -2.5),
        ('grip', 0.145),
    ],
)
def test_kfs_load_motor_feedback_is_saved(motor_name, value):
    node = make_node_stub()

    node._on_kfs_load_motor_feedback(
        motor_name, SimpleNamespace(data=value))

    generation, feedback = node.get_kfs_load_feedback_snapshot()
    assert generation == 1
    assert feedback[motor_name] == pytest.approx(value)


def test_non_finite_kfs_load_motor_feedback_is_ignored():
    node = make_node_stub()

    node._on_kfs_load_motor_feedback(
        'root_rotate', SimpleNamespace(data=math.nan))

    generation, feedback = node.get_kfs_load_feedback_snapshot()
    assert generation == 0
    assert feedback['root_rotate'] is None


def test_parameter_load_result_reports_all_parameters_written():
    output = '\n'.join([
        'Set parameter service_timeout_sec successful',
        'Set parameter mode_1_sequence successful',
        'Set parameter mode_2_sequence successful',
        'Set parameter mode_3_sequence successful',
        'Set parameter mode_4_sequence successful',
        'Set parameter mode_5_sequence successful',
        'Set parameter release_sequence successful',
        'Set parameter pop_1_sequence successful',
        'Set parameter pop_2_sequence successful',
    ])

    success, message = summarize_parameter_load_result(0, output, '')

    assert success
    assert message == 'KFS Load 参数写入成功：共 9 项'


def test_parameter_load_result_reports_partial_failure():
    stdout = 'Set parameter service_timeout_sec successful'
    stderr = (
        'Set parameter mode_1_sequence failed: '
        'length must be a multiple of 6'
    )

    success, message = summarize_parameter_load_result(0, stdout, stderr)

    assert not success
    assert 'mode_1_sequence failed' in message
    assert 'multiple of 6' in message


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
    ],
)
def test_invalid_motion_parameters_are_rejected(name, value):
    config = dict(GuiControlNode.MOTION_PARAMETER_DEFAULTS)
    config[name] = value
    assert GuiControlNode._validate_motion_config(config)
