import math
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

from robot_r2_control.gui_control import (
    GuiControlApp,
    GuiControlNode,
    STAGE_ONE_PARAMETER_NAMES,
    STAGE_TWO_POINT_ONE_PARAMETER_NAMES,
    STAGE_TWO_POINT_TWO_PARAMETER_NAMES,
    STEP_TRAVERSE_PARAMETER_NAMES,
    make_parameter_load_command,
    make_stage_one_parameter_load_command,
    manual_twist_components,
    motion_control_text,
    normalize_motion_key,
    parse_relocalization_values,
    parse_stage_two_point_one_route,
    resolve_kfs_loader_source_config,
    resolve_stage_one_source_config,
    summarize_named_parameter_load_result,
    summarize_parameter_load_result,
    summarize_stage_one_parameter_load_result,
    velocity_test_twist_components,
)
from robot_r2_control.stage_two_grid_gui import StageTwoGridModel
from robot_r2_interfaces.srv import (
    KfsAction,
    MoveRelative,
    StageOne,
    StageTwo,
    StageTwoPointOne,
    StageTwoPointTwo,
    StageTwoPointTwoExit,
    StageThree,
    TraverseStep,
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
        self.error = None

    def add_done_callback(self, callback):
        self.callback = callback

    def result(self):
        if self.error is not None:
            raise self.error
        return self.response

    def complete(self, response):
        self.response = response
        self.callback(self)

    def fail(self, error):
        self.error = error
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
    node.stage_one_request_in_flight = False
    node.stage_one_team = None
    node.stage_two_request_in_flight = False
    node.stage_two_team = None
    node.stage_two_point_one_request_in_flight = False
    node.stage_two_point_one_mode = None
    node.stage_two_point_one_route = ()
    node.stage_two_point_one_team = None
    node.stage_two_point_two_request_in_flight = False
    node.stage_two_point_two_mode = None
    node.stage_two_point_two_team = None
    node.stage_two_point_two_exit_request_in_flight = False
    node.stage_two_point_two_exit_team = None
    node.stage_three_request_in_flight = False
    node.stage_three_team = None
    node.stage_three_loaded_count = None
    node.kfs_action_request_in_flight = False
    node.kfs_parameter_load_in_flight = False
    node.stage_one_parameter_load_in_flight = False
    node.step_two_parameter_load_in_flight = {
        key: False for key in GuiControlNode.STEP_TWO_PARAMETER_LOAD_TARGETS
    }
    node.step_two_parameter_load_config_paths = {
        key: '/nonexistent/step_two.yaml'
        for key in GuiControlNode.STEP_TWO_PARAMETER_LOAD_TARGETS
    }
    node.config_generation = 0
    node.lift_min = 0.0
    node.lift_max = 0.376
    node.float_control_ranges = {
        control_name: (
            float(definition['minimum'][1]),
            float(definition['maximum'][1]),
        )
        for control_name, definition in (
            GuiControlNode.FLOAT_CONTROL_PARAMETERS.items())
    }
    node.motion_config = dict(GuiControlNode.MOTION_PARAMETER_DEFAULTS)
    node.cmd_vel_publisher = FakePublisher()
    node.move_relative_client = FakeClient()
    node.kfs_alignment_client = FakeClient()
    node.tip_alignment_client = FakeClient()
    node.kfs_action_client = FakeClient()
    node.set_base_pose_client = FakeClient()
    node.set_base_pose_odin_client = FakeClient()
    node.step_traverse_client = FakeClient()
    node.stage_one_client = FakeClient()
    node.stage_two_client = FakeClient()
    node.stage_two_point_one_client = FakeClient()
    node.stage_two_point_two_client = FakeClient()
    node.stage_two_point_two_exit_client = FakeClient()
    node.stage_three_client = FakeClient()
    node.stage_one_relocalization_pose = (0.0,) * 6
    node.stage_two_relocalization_pose = (
        GuiControlNode.STEP_TWO_RELOCALIZATION_DEFAULT)
    node.stage_two_point_one_relocalization_pose = (
        GuiControlNode.STEP_TWO_POINT_ONE_RELOCALIZATION_DEFAULT)
    node.stage_two_point_two_relocalization_pose = (
        GuiControlNode.STEP_TWO_POINT_TWO_RELOCALIZATION_DEFAULT)
    node.stage_two_point_two_exit_relocalization_pose = (
        GuiControlNode.STEP_TWO_POINT_TWO_EXIT_RELOCALIZATION_DEFAULT)
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


def test_stage_one_config_is_resolved_from_workspace_source(tmp_path):
    package_share = (
        tmp_path / 'install/robot_r2_control/share/robot_r2_control')
    installed_config = package_share / 'config/stage_one.yaml'
    installed_config.parent.mkdir(parents=True)
    installed_config.write_text('installed', encoding='utf-8')
    source_config = (
        tmp_path / 'src/robot_r2_control/config/stage_one.yaml')
    source_config.parent.mkdir(parents=True)
    source_config.write_text('source', encoding='utf-8')

    result = resolve_stage_one_source_config(package_share)

    assert result == str(source_config)


def test_stage_one_parameter_load_command_targets_stage_one_node():
    command = make_stage_one_parameter_load_command(
        '/workspace/stage_one.yaml')

    assert command == [
        'ros2', 'param', 'load',
        '--no-daemon', '--spin-time', '2.0',
        '/stage_one', '/workspace/stage_one.yaml',
    ]


def test_stage_one_source_yaml_targets_absolute_node_name():
    config_path = (
        Path(__file__).parents[1] / 'config' / 'stage_one.yaml')

    assert config_path.read_text(encoding='utf-8').startswith('/stage_one:')


@pytest.mark.parametrize(
    'file_name, expected_node',
    [
        ('step_traverse.yaml', '/step_traverse'),
        ('stage_two_point_one.yaml', '/stage_two_point_one'),
        ('stage_two_point_two.yaml', '/stage_two_point_two'),
    ],
)
def test_step_two_source_yaml_targets_absolute_node_name(
        file_name, expected_node):
    config_path = (
        Path(__file__).parents[1] / 'config' / file_name)

    assert config_path.read_text(encoding='utf-8').startswith(
        f'{expected_node}:')


@pytest.mark.parametrize(
    'key, node_name',
    [
        ('step_traverse', '/step_traverse'),
        ('stage_two_point_one', '/stage_two_point_one'),
        ('stage_two_point_two', '/stage_two_point_two'),
    ],
)
def test_step_two_parameter_load_command_targets_node(key, node_name):
    command = make_parameter_load_command(
        '/workspace/step_two.yaml', node_name)

    assert command == [
        'ros2', 'param', 'load',
        '--no-daemon', '--spin-time', '2.0',
        node_name, '/workspace/step_two.yaml',
    ]
    assert key in GuiControlNode.STEP_TWO_PARAMETER_LOAD_TARGETS


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
    assert text['pose']['serial']['forward'] == '位置伺服前进 0.8 m（下位机）'
    assert text['pose']['serial']['rotate_left'] == (
        '位置伺服逆时针旋转 0.785 rad（下位机）')
    assert text['pose']['odin']['forward'] == '位置伺服前进 0.8 m（Odin）'
    assert text['pose']['odin']['rotate_left'] == (
        '位置伺服逆时针旋转 0.785 rad（Odin）')
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


@pytest.mark.parametrize(
    'team,label',
    [
        (StageOne.Request.RED, '红方'),
        (StageOne.Request.BLUE, '蓝方'),
    ],
)
def test_stage_one_relocalizes_then_calls_team_service(team, label):
    node = make_node_stub()
    node.stage_one_relocalization_pose = (
        1.0, -2.0, 0.1, 0.0, 0.0, math.pi)

    success, message = node.request_stage_one(team)

    assert success
    assert message == f'Step1 {label}：正在重定位'
    relocalization = node.set_base_pose_client.requests[0]
    assert (
        relocalization.x,
        relocalization.y,
        relocalization.z,
        relocalization.roll,
        relocalization.pitch,
        relocalization.yaw,
    ) == pytest.approx(node.stage_one_relocalization_pose)
    assert node.stage_one_request_in_flight
    assert node.stage_one_team == team
    assert not node.stage_one_client.requests

    node.set_base_pose_client.future.complete(SimpleNamespace(
        success=True,
        message='base_link pose updated',
    ))
    stage_request = node.stage_one_client.requests[0]
    assert stage_request.team == team
    assert node.stage_one_request_in_flight

    node.stage_one_client.future.complete(SimpleNamespace(
        success=True,
        message=f'Stage 1 completed for {team} team',
    ))
    assert not node.stage_one_request_in_flight
    assert node.stage_one_team is None
    assert node.pop_status_events() == [
        f'Step1 {label}：重定位完成，正在调用 Step1',
        f'Step1 {label}完成：Stage 1 completed for {team} team',
    ]


def test_stage_one_relocalization_failure_does_not_call_stage_service():
    node = make_node_stub()

    success, _ = node.request_stage_one(StageOne.Request.RED)
    assert success
    node.set_base_pose_client.future.complete(SimpleNamespace(
        success=False,
        message='relocalization rejected',
    ))

    assert not node.stage_one_request_in_flight
    assert not node.stage_one_client.requests
    assert node.pop_status_events() == [
        'Step1 红方重定位失败：relocalization rejected',
    ]


def test_stage_one_reports_unavailable_service_before_relocalizing():
    node = make_node_stub()
    node.stage_one_client = FakeClient(ready=False)

    success, message = node.request_stage_one(StageOne.Request.BLUE)

    assert not success
    assert message == '/r2/stage_one 服务不可用'
    assert not node.set_base_pose_client.requests


def test_stage_one_relocalization_pose_supports_dynamic_update():
    node = make_node_stub()
    parameter = SimpleNamespace(
        name='stage_one_relocalization_pose',
        value=[1.0, 2.0, 0.0, 0.0, 0.0, math.pi],
    )

    result = node._on_parameters_changed([parameter])

    assert result.successful
    assert node.stage_one_relocalization_pose == pytest.approx(
        (1.0, 2.0, 0.0, 0.0, 0.0, math.pi))


def test_invalid_stage_one_relocalization_pose_is_rejected():
    node = make_node_stub()
    original = node.stage_one_relocalization_pose
    parameter = SimpleNamespace(
        name='stage_one_relocalization_pose',
        value=[0.0] * 5,
    )

    result = node._on_parameters_changed([parameter])

    assert not result.successful
    assert node.stage_one_relocalization_pose == original


def test_stage_one_rejects_unknown_team_without_relocalizing():
    node = make_node_stub()

    success, message = node.request_stage_one('green')

    assert not success
    assert message == 'Unknown Stage 1 team: green'
    assert not node.set_base_pose_client.requests


@pytest.mark.parametrize(
    'team,label,expected_y,expected_route,expected_kfs',
    [
        (
            StageTwo.Request.RED,
            '红方',
            3.0,
            [(4, 2), (3, 2), (2, 2), (1, 2), (1, 3)],
            [(2, 3), (4, 1)],
        ),
        (
            StageTwo.Request.BLUE,
            '蓝方',
            -3.0,
            [(4, 2), (3, 2), (2, 2), (1, 2), (1, 3)],
            [(2, 3), (4, 1)],
        ),
    ],
)
def test_complete_stage_two_relocalizes_to_cell_five_two_then_calls_service(
        team, label, expected_y, expected_route, expected_kfs):
    node = make_node_stub()
    route = ((4, 2), (3, 2), (2, 2), (1, 2), (1, 3))

    success, message = node.request_stage_two(
        team, route, ((4, 1), (2, 3)))

    assert success
    assert message == (
        f'完整 Step2 {label}：正在重定位到 (5,2) 格心')
    pose_request = node.set_base_pose_odin_client.requests[0]
    assert (pose_request.x, pose_request.y, pose_request.yaw) == pytest.approx(
        (3.4, expected_y, math.pi))
    assert not node.stage_two_client.requests

    node.set_base_pose_odin_client.future.complete(SimpleNamespace(
        success=True,
        message='base_link pose updated',
    ))
    stage_request = node.stage_two_client.requests[0]
    assert stage_request.team == team
    assert stage_request.mode == StageTwo.Request.ROUTE
    assert stage_request.fake_kfs_decision == 0
    assert [
        (cell.forward_index, cell.lateral_index)
        for cell in stage_request.move_cells
    ] == expected_route
    assert [
        (cell.forward_index, cell.lateral_index)
        for cell in stage_request.kfs_cells
    ] == expected_kfs

    node.stage_two_client.future.complete(SimpleNamespace(
        success=True,
        message='stage two completed',
        loaded_count=2,
    ))
    assert not node.stage_two_request_in_flight
    assert node.stage_two_team is None
    assert node.pop_status_events() == [
        f'完整 Step2 {label}：重定位完成，正在调用阶段服务',
        f'完整 Step2 {label}完成：stage two completed；已装载 2 个 KFS',
    ]


def test_complete_stage_two_relocalization_failure_stops_service_call():
    node = make_node_stub()
    route = ((4, 2), (3, 2), (2, 2), (1, 2), (1, 1))

    success, _ = node.request_stage_two(
        StageTwo.Request.RED, route, ())
    assert success
    node.set_base_pose_odin_client.future.complete(SimpleNamespace(
        success=False,
        message='relocalization rejected',
    ))

    assert not node.stage_two_client.requests
    assert not node.stage_two_request_in_flight
    assert node.pop_status_events() == [
        '完整 Step2 红方重定位失败：relocalization rejected',
    ]


def test_complete_stage_two_requires_both_services_before_relocalizing():
    node = make_node_stub()
    node.stage_two_client = FakeClient(ready=False)
    route = ((4, 2), (3, 2), (2, 2), (1, 2), (1, 1))

    success, message = node.request_stage_two(
        StageTwo.Request.BLUE, route, ())

    assert not success
    assert message == '/r2/stage_two 服务不可用'
    assert not node.set_base_pose_odin_client.requests


def test_debug_gui_invalid_complete_route_does_not_request_stage_two():
    messages = []
    app = GuiControlApp.__new__(GuiControlApp)
    app.stage_two_grid_editor = SimpleNamespace(model=StageTwoGridModel())
    app.team_value = SimpleNamespace(get=lambda: StageTwo.Request.RED)
    app.status_text = SimpleNamespace(set=messages.append)
    app.node = SimpleNamespace(
        request_stage_two=lambda *_args: pytest.fail(
            'invalid route must not call StageTwo'))

    app._start_stage_two()

    assert messages == ['Step2 路线不能为空']


def test_complete_stage_two_relocalization_exception_releases_interlock():
    node = make_node_stub()
    route = ((4, 2), (3, 2), (2, 2), (1, 2), (1, 1))
    node.request_stage_two(StageTwo.Request.BLUE, route, ())

    node.set_base_pose_odin_client.future.fail(RuntimeError('odin failed'))

    assert not node.stage_two_request_in_flight
    assert not node.stage_two_client.requests
    assert 'odin failed' in node.pop_status_events()[-1]


def test_complete_stage_two_pose_update_is_atomic_with_other_pose_updates():
    node = make_node_stub()
    new_stage_two_pose = [3.5, -3.1, 0.0, 0.0, 0.0, math.pi]

    result = node._on_parameters_changed([
        SimpleNamespace(
            name='stage_two_relocalization_pose',
            value=new_stage_two_pose,
        ),
        SimpleNamespace(
            name='stage_one_relocalization_pose',
            value=[0.0] * 5,
        ),
    ])

    assert not result.successful
    assert node.stage_two_relocalization_pose == pytest.approx(
        GuiControlNode.STEP_TWO_RELOCALIZATION_DEFAULT)


def test_complete_stage_two_pose_supports_dynamic_update():
    node = make_node_stub()
    updated_pose = [3.5, -3.1, 0.0, 0.0, 0.0, math.pi]

    result = node._on_parameters_changed([
        SimpleNamespace(
            name='stage_two_relocalization_pose',
            value=updated_pose,
        ),
    ])

    assert result.successful
    assert node.stage_two_relocalization_pose == pytest.approx(updated_pose)


@pytest.mark.parametrize(
    'team,label,expected_y',
    [
        (StageTwoPointOne.Request.RED, '红方', 2.2),
        (StageTwoPointOne.Request.BLUE, '蓝方', -2.2),
    ],
)
def test_stage_two_point_one_relocalizes_then_calls_team_service(
        team, label, expected_y):
    node = make_node_stub()

    success, message = node.request_stage_two_point_one(
        team, StageTwoPointOne.Request.SKIP)

    assert success
    assert message == f'2.1 {label}skip测试：正在重定位到测试起点'
    request = node.set_base_pose_odin_client.requests[0]
    assert (request.x, request.y, request.yaw) == (
        5.568, expected_y, math.pi)
    assert request.z == 0.0
    assert request.roll == 0.0
    assert request.pitch == 0.0
    assert node.stage_two_point_one_request_in_flight
    assert not node.stage_two_point_one_client.requests

    node.set_base_pose_odin_client.future.complete(SimpleNamespace(
        success=True,
        message='base_link pose updated',
    ))
    step_request = node.stage_two_point_one_client.requests[0]
    assert step_request.team == team
    assert step_request.loaded_count == 0
    assert step_request.mode == StageTwoPointOne.Request.SKIP
    assert list(step_request.route_cells) == []
    assert node.stage_two_point_one_request_in_flight

    node.stage_two_point_one_client.future.complete(SimpleNamespace(
        success=True,
        message='Stage 2.1 completed',
    ))
    assert not node.stage_two_point_one_request_in_flight
    assert node.pop_status_events() == [
        f'2.1 {label}skip测试：重定位完成，正在调用 2.1',
        f'2.1 {label}skip测试完成：Stage 2.1 completed',
    ]


def test_stage_two_point_one_normal_passes_standard_mode():
    node = make_node_stub()

    success, _ = node.request_stage_two_point_one(
        StageTwoPointOne.Request.RED, StageTwoPointOne.Request.STANDARD)
    assert success

    node.set_base_pose_odin_client.future.complete(SimpleNamespace(
        success=True,
        message='base_link pose updated',
    ))
    step_request = node.stage_two_point_one_client.requests[0]
    assert step_request.team == StageTwoPointOne.Request.RED
    assert step_request.loaded_count == 0
    assert step_request.mode == StageTwoPointOne.Request.STANDARD
    assert list(step_request.route_cells) == []


def test_stage_two_point_one_route_is_validated_before_relocalization():
    node = make_node_stub()

    success, message = node.request_stage_two_point_one(
        StageTwoPointOne.Request.RED,
        StageTwoPointOne.Request.ROUTE,
        (1, 1),
    )

    assert not success
    assert message == '2.1 路线不能包含重复格子'
    assert not node.set_base_pose_odin_client.requests


def test_stage_two_point_one_route_is_forwarded_after_relocalization():
    node = make_node_stub()

    success, message = node.request_stage_two_point_one(
        StageTwoPointOne.Request.RED,
        StageTwoPointOne.Request.ROUTE,
        (1, 3),
    )

    assert success
    assert message == (
        '2.1 红方路线[1,3]测试：正在重定位到测试起点')
    node.set_base_pose_odin_client.future.complete(SimpleNamespace(
        success=True,
        message='base_link pose updated',
    ))
    step_request = node.stage_two_point_one_client.requests[0]
    assert step_request.mode == StageTwoPointOne.Request.ROUTE
    assert list(step_request.route_cells) == [1, 3]


@pytest.mark.parametrize(
    'raw, expected',
    [('2,1,3', (2, 1, 3)), (' 1, 3 ', (1, 3))],
)
def test_parse_stage_two_point_one_route(raw, expected):
    assert parse_stage_two_point_one_route(raw) == expected


@pytest.mark.parametrize('raw', ['', '1,,2', '1,1', '4', '1,a'])
def test_parse_stage_two_point_one_route_rejects_invalid_input(raw):
    with pytest.raises(ValueError):
        parse_stage_two_point_one_route(raw)


@pytest.mark.parametrize(
    'team,label,expected_y',
    [
        (StageTwoPointOne.Request.RED, '红方', 3.0),
        (StageTwoPointOne.Request.BLUE, '蓝方', -3.0),
    ],
)
def test_stage_two_point_two_relocalizes_then_calls_team_service(
        team, label, expected_y):
    node = make_node_stub()

    success, message = node.request_stage_two_point_two(
        team, StageTwoPointTwo.Request.SKIP)

    assert success
    assert message == f'2.2 {label}skip测试：正在重定位到测试起点'
    request = node.set_base_pose_odin_client.requests[0]
    assert (request.x, request.y, request.yaw) == (
        3.4, expected_y, math.pi)
    assert node.stage_two_point_two_request_in_flight
    assert not node.stage_two_point_two_client.requests

    node.set_base_pose_odin_client.future.complete(SimpleNamespace(
        success=True,
        message='base_link pose updated',
    ))
    step_request = node.stage_two_point_two_client.requests[0]
    assert step_request.team == team
    assert step_request.fake_kfs_decision == StageTwoPointTwo.Request.LEFT
    assert step_request.loaded_count == 0
    assert step_request.mode == StageTwoPointTwo.Request.SKIP
    assert list(step_request.move_cells) == []
    assert list(step_request.load_cells) == []

    node.stage_two_point_two_client.future.complete(SimpleNamespace(
        success=True,
        message='Stage 2.2 completed at (0, 1)',
    ))
    assert not node.stage_two_point_two_request_in_flight
    assert node.pop_status_events() == [
        f'2.2 {label}skip测试：重定位完成，正在调用 2.2',
        f'2.2 {label}skip测试完成：Stage 2.2 completed at (0, 1)',
    ]


def test_stage_two_point_two_normal_passes_standard_mode():
    node = make_node_stub()

    success, message = node.request_stage_two_point_two(
        StageTwoPointOne.Request.RED,
        StageTwoPointTwo.Request.STANDARD,
    )

    assert success
    assert message == '2.2 红方正常测试：正在重定位到测试起点'
    node.set_base_pose_odin_client.future.complete(SimpleNamespace(
        success=True,
        message='base_link_odin pose updated',
    ))
    step_request = node.stage_two_point_two_client.requests[0]
    assert step_request.team == StageTwoPointOne.Request.RED
    assert step_request.fake_kfs_decision == StageTwoPointTwo.Request.LEFT
    assert step_request.loaded_count == 0
    assert step_request.mode == StageTwoPointTwo.Request.STANDARD
    assert list(step_request.move_cells) == []
    assert list(step_request.load_cells) == []


@pytest.mark.parametrize(
    'team,label,expected_y',
    [
        (StageTwoPointTwoExit.Request.RED, '红方', 1.8),
        (StageTwoPointTwoExit.Request.BLUE, '蓝方', -1.8),
    ],
)
def test_stage_two_point_two_exit_relocalizes_then_calls_team_service(
        team, label, expected_y):
    node = make_node_stub()

    success, message = node.request_stage_two_point_two_exit(team)

    assert success
    assert message == f'2.2 后续动作 {label}：正在重定位到 (0,3)'
    request = node.set_base_pose_odin_client.requests[0]
    assert (request.x, request.y, request.yaw) == pytest.approx(
        (-2.6, expected_y, math.pi))
    assert node.stage_two_point_two_exit_request_in_flight
    assert not node.stage_two_point_two_exit_client.requests

    node.set_base_pose_odin_client.future.complete(SimpleNamespace(
        success=True,
        message='base_link pose updated',
    ))
    exit_request = node.stage_two_point_two_exit_client.requests[0]
    assert exit_request.team == team

    node.stage_two_point_two_exit_client.future.complete(SimpleNamespace(
        success=True,
        message='exit completed',
    ))
    assert not node.stage_two_point_two_exit_request_in_flight
    assert node.pop_status_events() == [
        f'2.2 后续动作 {label}：重定位完成，正在调用离场服务',
        f'2.2 后续动作 {label}完成：exit completed',
    ]


def test_stage_two_point_two_exit_does_not_call_service_when_relocalizing_fails():
    node = make_node_stub()

    success, _ = node.request_stage_two_point_two_exit(
        StageTwoPointTwoExit.Request.RED)
    assert success

    node.set_base_pose_odin_client.future.complete(SimpleNamespace(
        success=False,
        message='odin unavailable',
    ))

    assert not node.stage_two_point_two_exit_client.requests
    assert not node.stage_two_point_two_exit_request_in_flight
    assert node.pop_status_events() == [
        '2.2 后续动作 红方重定位失败：odin unavailable',
    ]


def test_stage_two_point_two_exit_rejects_unknown_team():
    node = make_node_stub()

    success, message = node.request_stage_two_point_two_exit('green')

    assert not success
    assert message == 'Unknown Stage 2 team: green'
    assert not node.set_base_pose_odin_client.requests


@pytest.mark.parametrize(
    'loaded_count',
    [1, 2, 3],
)
def test_stage_three_buttons_pass_team_and_loaded_count(loaded_count):
    node = make_node_stub()

    success, message = node.request_stage_three(
        StageThree.Request.BLUE, loaded_count)

    assert success
    assert message == (
        f'Step3 蓝方（已有 {loaded_count} 个 KFS）：正在执行')
    request = node.stage_three_client.requests[0]
    assert request.team == StageThree.Request.BLUE
    assert request.loaded_count == loaded_count
    assert node.stage_three_request_in_flight

    node.stage_three_client.future.complete(SimpleNamespace(
        success=True,
        message='stage three completed',
    ))
    assert not node.stage_three_request_in_flight
    assert node.stage_three_team is None
    assert node.stage_three_loaded_count is None
    assert node.pop_status_events() == [
        f'Step3 蓝方（已有 {loaded_count} 个 KFS）完成：'
        'stage three completed',
    ]


def test_stage_three_rejects_invalid_loaded_count():
    node = make_node_stub()

    success, message = node.request_stage_three(
        StageThree.Request.RED, 0)

    assert not success
    assert message == 'Stage 3 loaded_count must be 1, 2 or 3, got 0'
    assert not node.stage_three_client.requests


def test_stage_three_rejects_unknown_team():
    node = make_node_stub()

    success, message = node.request_stage_three('green', 3)

    assert not success
    assert message == 'Unknown Stage 3 team: green'
    assert not node.stage_three_client.requests


def test_stage_three_reports_service_failure_and_releases_busy_state():
    node = make_node_stub()

    success, _ = node.request_stage_three(StageThree.Request.RED, 2)
    assert success
    node.stage_three_client.future.complete(SimpleNamespace(
        success=False,
        message='pop failed',
    ))

    assert not node.stage_three_request_in_flight
    assert node.pop_status_events() == [
        'Step3 红方（已有 2 个 KFS）失败：pop failed',
    ]


def test_stage_three_and_manual_kfs_action_are_mutually_exclusive():
    node = make_node_stub()
    node.kfs_action_request_in_flight = True

    success, message = node.request_stage_three(StageThree.Request.RED, 3)

    assert not success
    assert message == 'KFS 动作正在执行'
    assert not node.stage_three_client.requests

    node.kfs_action_request_in_flight = False
    node.stage_three_request_in_flight = True
    success, message = node.request_kfs_action(
        KfsAction.Request.POP, KfsAction.Request.MODE_1)
    assert not success
    assert message == 'Step3 正在执行'
    assert not node.kfs_action_client.requests


@pytest.mark.parametrize(
    'method_name, mode_or_skip',
    [
        ('request_stage_two_point_one', StageTwoPointOne.Request.SKIP),
        ('request_stage_two_point_two', StageTwoPointTwo.Request.SKIP),
    ],
)
def test_stage_two_rejects_unknown_team_before_relocalizing(
        method_name, mode_or_skip):
    node = make_node_stub()

    success, message = getattr(node, method_name)('green', mode_or_skip)

    assert not success
    assert message == 'Unknown Stage 2 team: green'
    assert not node.set_base_pose_odin_client.requests


@pytest.mark.parametrize(
    'parameter_name,attribute_name',
    [
        (
            'stage_two_point_one_relocalization_pose',
            'stage_two_point_one_relocalization_pose',
        ),
        (
            'stage_two_point_two_relocalization_pose',
            'stage_two_point_two_relocalization_pose',
        ),
        (
            'stage_two_point_two_exit_relocalization_pose',
            'stage_two_point_two_exit_relocalization_pose',
        ),
    ],
)
def test_stage_two_relocalization_poses_support_dynamic_update(
        parameter_name, attribute_name):
    node = make_node_stub()
    updated = [1.0, -2.0, 0.0, 0.0, 0.0, math.pi]

    result = node._on_parameters_changed([
        SimpleNamespace(name=parameter_name, value=updated),
    ])

    assert result.successful
    assert getattr(node, attribute_name) == pytest.approx(tuple(updated))


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
        ('weapon_rotate', 1.431),
        ('weapon_grip', 0.0),
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


def test_stage_one_parameter_load_result_checks_every_yaml_parameter():
    output = '\n'.join(
        f'Set parameter {name} successful'
        for name in sorted(STAGE_ONE_PARAMETER_NAMES)
    )

    success, message = summarize_stage_one_parameter_load_result(
        0, output, '')

    assert success
    assert message == 'Step1 参数写入成功：共 28 项'


def test_stage_one_parameter_load_result_reports_missing_parameter():
    successful_names = STAGE_ONE_PARAMETER_NAMES - {'move_timeout_sec'}
    output = '\n'.join(
        f'Set parameter {name} successful'
        for name in sorted(successful_names)
    )

    success, message = summarize_stage_one_parameter_load_result(
        0, output, '')

    assert not success
    assert '未确认写入：move_timeout_sec' in message


@pytest.mark.parametrize(
    'key, names, display_name',
    [
        (
            'step_traverse',
            STEP_TRAVERSE_PARAMETER_NAMES,
            '台阶跨越',
        ),
        (
            'stage_two_point_one',
            STAGE_TWO_POINT_ONE_PARAMETER_NAMES,
            '2.1',
        ),
        (
            'stage_two_point_two',
            STAGE_TWO_POINT_TWO_PARAMETER_NAMES,
            '2.2',
        ),
    ],
)
def test_step_two_parameter_load_result_checks_every_yaml_parameter(
        key, names, display_name):
    target = GuiControlNode.STEP_TWO_PARAMETER_LOAD_TARGETS[key]
    assert target['parameter_names'] == names
    output = '\n'.join(
        f'Set parameter {name} successful'
        for name in sorted(names)
    )

    success, message = summarize_named_parameter_load_result(
        display_name + ' ', names, 0, output, '')

    assert success
    assert message == f'{display_name} 参数写入成功：共 {len(names)} 项'


def test_step_two_parameter_load_result_reports_missing_parameter():
    successful_names = STEP_TRAVERSE_PARAMETER_NAMES - {'a1'}
    output = '\n'.join(
        f'Set parameter {name} successful'
        for name in sorted(successful_names)
    )

    success, message = summarize_named_parameter_load_result(
        '台阶跨越 ', STEP_TRAVERSE_PARAMETER_NAMES, 0, output, '')

    assert not success
    assert '未确认写入：a1' in message


def test_step_two_parameter_load_reports_missing_config_file():
    node = make_node_stub()

    success, message = node.request_step_two_parameter_load('step_traverse')

    assert not success
    assert '台阶跨越参数文件不存在' in message
    assert not node.step_two_parameter_load_in_flight['step_traverse']


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


@pytest.mark.parametrize(
    'pose_source, source_label',
    [
        (MoveRelative.Request.SERIAL, '下位机'),
        (MoveRelative.Request.ODIN, 'Odin'),
    ],
)
def test_request_relative_pose_uses_selected_source(
        pose_source, source_label):
    node = make_node_stub()

    success, message = node.request_relative_pose('forward', pose_source)

    assert success
    assert message == (
        f'已发送 {source_label} 相对移动：'
        '前 0.500 m，左 0.000 m，旋转 0.000 rad')
    request = node.move_relative_client.requests[0]
    assert request.pose_source == pose_source
    assert request.forward == pytest.approx(0.5)
    assert request.left == pytest.approx(0.0)
    assert request.yaw_delta == pytest.approx(0.0)
    assert request.position_tolerance == pytest.approx(0.0)
    assert request.yaw_tolerance == pytest.approx(0.0)
    assert request.timeout_sec == pytest.approx(20.0)
    assert node.pose_request_in_flight


def test_request_relative_pose_rotate_uses_yaw_parameter():
    node = make_node_stub()
    node.motion_config['pose_test_yaw'] = 0.785

    success, _ = node.request_relative_pose(
        'rotate_left', MoveRelative.Request.ODIN)

    assert success
    request = node.move_relative_client.requests[0]
    assert request.pose_source == MoveRelative.Request.ODIN
    assert request.forward == pytest.approx(0.0)
    assert request.left == pytest.approx(0.0)
    assert request.yaw_delta == pytest.approx(0.785)


@pytest.mark.parametrize('pose_source', ['', 'gps', None])
def test_request_relative_pose_rejects_unknown_source(pose_source):
    node = make_node_stub()

    success, message = node.request_relative_pose('forward', pose_source)

    assert not success
    assert '未知位姿来源' in message
    assert not node.move_relative_client.requests


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
