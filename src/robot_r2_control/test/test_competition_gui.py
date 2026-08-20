import math
from types import SimpleNamespace
import threading

import pytest

from robot_r2_control.competition_gui import (
    CompetitionGuiApp,
    CompetitionGuiEvent,
    CompetitionGuiNode,
)
from robot_r2_control.stage_two_grid_gui import (
    GRID_CELLS,
    StageTwoGridEditor,
    StageTwoGridModel,
    gui_cell_to_service_cell,
    grid_display_row,
)
from robot_r2_control.gui_control import GuiControlApp, GuiControlNode
from robot_r2_control.joint_control_gui import (
    JointControlGuiMixin,
    JointControlNodeMixin,
)
from robot_r2_control.stage_two_control import StageTwoController
from robot_r2_interfaces.srv import StageOne, StageThree, StageTwo


class DeferredFuture:
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
        self.futures = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        future = DeferredFuture()
        self.requests.append(request)
        self.futures.append(future)
        return future


class FakeVariable:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class FakeWidget:
    def __init__(self):
        self.states = []

    def configure(self, **kwargs):
        self.states.append(kwargs)


def make_node():
    node = CompetitionGuiNode.__new__(CompetitionGuiNode)
    node.state_lock = threading.RLock()
    node.status_events = []
    node.stage_request_in_flight = False
    node.active_stage = None
    node.set_base_pose_client = FakeClient()
    node.set_base_pose_odin_client = FakeClient()
    node.stage_one_client = FakeClient()
    node.stage_two_client = FakeClient()
    node.stage_three_client = FakeClient()
    node.stage_one_relocalization_pose = (
        node.STAGE_ONE_RELOCALIZATION_DEFAULT)
    node.stage_two_relocalization_pose = (
        node.STAGE_TWO_RELOCALIZATION_DEFAULT)
    node.config_generation = 0
    node.lift_min = 0.0
    node.lift_max = 0.376
    node.float_control_ranges = {
        name: (definition['minimum'][1], definition['maximum'][1])
        for name, definition in node.FLOAT_CONTROL_PARAMETERS.items()
    }
    return node


def message_indices(messages):
    return [
        (message.forward_index, message.lateral_index)
        for message in messages
    ]


def valid_route():
    return ((4, 2), (3, 2), (2, 2), (1, 2), (1, 1))


def complete_relocalization(client, success=True, message='ok'):
    client.futures[0].complete(SimpleNamespace(
        success=success,
        message=message,
    ))


def test_grid_layout_has_expected_coordinate_set():
    assert len(GRID_CELLS) == 12
    assert set(GRID_CELLS) == {
        (forward_index, lateral_index)
        for forward_index in range(1, 5)
        for lateral_index in range(1, 4)
    }


def test_exit_rows_are_above_entry_rows_in_matrix():
    assert grid_display_row(1) == 1
    assert grid_display_row(4) == 4


@pytest.mark.parametrize(
    'team, gui_cell, service_cell',
    [
        (StageTwo.Request.BLUE, (2, 1), (2, 1)),
        (StageTwo.Request.BLUE, (2, 2), (2, 2)),
        (StageTwo.Request.BLUE, (2, 3), (2, 3)),
        (StageTwo.Request.RED, (2, 1), (2, 3)),
        (StageTwo.Request.RED, (2, 2), (2, 2)),
        (StageTwo.Request.RED, (2, 3), (2, 1)),
    ],
)
def test_gui_cells_are_converted_to_team_service_coordinates(
        team, gui_cell, service_cell):
    assert gui_cell_to_service_cell(team, gui_cell) == service_cell


def test_highest_service_cell_appears_on_expected_team_side():
    assert gui_cell_to_service_cell(
        StageTwo.Request.BLUE, (3, 3)) == (3, 3)
    assert gui_cell_to_service_cell(
        StageTwo.Request.RED, (3, 1)) == (3, 3)


def test_route_toggle_preserves_click_order_and_reindexes_after_removal():
    model = StageTwoGridModel()
    for cell in ((4, 2), (3, 2), (2, 2), (1, 2)):
        model.toggle_route(cell)

    model.toggle_route((3, 2))

    assert model.route_cells == [(4, 2), (2, 2), (1, 2)]
    model.toggle_route((3, 2))
    assert model.route_cells == [(4, 2), (2, 2), (1, 2), (3, 2)]


def test_route_and_kfs_are_independent_and_clear_independently():
    model = StageTwoGridModel()
    model.toggle_route((4, 2))
    model.toggle_kfs((4, 2))
    model.toggle_kfs((1, 3))

    assert model.route_cells == [(4, 2)]
    assert model.sorted_kfs_cells() == ((1, 3), (4, 2))

    model.clear_route()
    assert model.route_cells == []
    assert model.sorted_kfs_cells() == ((1, 3), (4, 2))
    model.clear_kfs()
    assert model.sorted_kfs_cells() == ()


def test_route_button_labels_show_only_current_sequence():
    editor = StageTwoGridEditor.__new__(StageTwoGridEditor)
    editor.model = StageTwoGridModel()
    editor.model.route_cells = [(4, 2), (3, 2)]
    editor.route_buttons = {
        (4, 2): FakeWidget(),
        (3, 2): FakeWidget(),
        (2, 2): FakeWidget(),
    }

    editor.refresh_route_buttons()

    assert editor.route_buttons[(4, 2)].states[-1]['text'] == '第 1 步'
    assert editor.route_buttons[(3, 2)].states[-1]['text'] == '第 2 步'
    assert editor.route_buttons[(2, 2)].states[-1]['text'] == ''


def test_kfs_button_labels_do_not_expose_cell_coordinates():
    editor = StageTwoGridEditor.__new__(StageTwoGridEditor)
    editor.model = StageTwoGridModel()
    editor.model.kfs_cells = {(4, 2)}
    editor.kfs_buttons = {
        (4, 2): FakeWidget(),
        (3, 2): FakeWidget(),
    }

    editor.refresh_kfs_buttons()

    assert editor.kfs_buttons[(4, 2)].states[-1]['text'] == 'KFS'
    assert editor.kfs_buttons[(3, 2)].states[-1]['text'] == ''


@pytest.mark.parametrize(
    'route, message',
    [
        ((), '不能为空'),
        (((3, 2), (2, 2), (1, 1)), '必须从'),
        (((4, 2), (3, 2), (2, 2), (1, 2)), '结束'),
        (((4, 2), (2, 2), (1, 2), (1, 1)), '必须共边'),
    ],
)
def test_route_validation_rejects_invalid_sequences(route, message):
    model = StageTwoGridModel()
    model.route_cells = list(route)

    with pytest.raises(ValueError, match=message):
        model.validated_route()


def test_route_validation_accepts_stage_two_route():
    model = StageTwoGridModel()
    model.route_cells = list(valid_route())

    assert model.validated_route() == valid_route()


def test_invalid_route_does_not_call_stage_two_service():
    app = CompetitionGuiApp.__new__(CompetitionGuiApp)
    app.stage_two_grid_editor = SimpleNamespace(model=StageTwoGridModel())
    app.team_value = FakeVariable(StageTwo.Request.RED)
    app.status_text = FakeVariable('')
    app.node = SimpleNamespace(
        request_stage_two=lambda *_args: pytest.fail(
            'invalid route must not call StageTwo'))

    app._start_stage_two()

    assert '不能为空' in app.status_text.get()


def test_stage_one_relocalizes_before_sending_selected_team():
    node = make_node()

    accepted, _ = node.request_stage_one(StageOne.Request.BLUE)

    assert accepted
    assert not node.stage_one_client.requests
    pose_request = node.set_base_pose_client.requests[0]
    assert (
        pose_request.x,
        pose_request.y,
        pose_request.z,
        pose_request.roll,
        pose_request.pitch,
        pose_request.yaw,
    ) == pytest.approx(node.STAGE_ONE_RELOCALIZATION_DEFAULT)

    complete_relocalization(node.set_base_pose_client)

    request = node.stage_one_client.requests[0]
    assert request.team == StageOne.Request.BLUE


def test_red_stage_two_request_mirrors_route_and_sorts_service_kfs_cells():
    node = make_node()

    accepted, _ = node.request_stage_two(
        StageOne.Request.RED,
        valid_route(),
        ((4, 2), (1, 3)),
    )

    assert accepted
    assert not node.stage_two_client.requests
    pose_request = node.set_base_pose_odin_client.requests[0]
    assert (pose_request.x, pose_request.y, pose_request.yaw) == pytest.approx(
        (5.568, -2.2, math.pi))
    complete_relocalization(node.set_base_pose_odin_client)
    request = node.stage_two_client.requests[0]
    assert request.team == StageTwo.Request.RED
    assert request.mode == StageTwo.Request.ROUTE
    assert request.fake_kfs_decision == 0
    assert message_indices(request.move_cells) == [
        (4, 2), (3, 2), (2, 2), (1, 2), (1, 3)]
    assert message_indices(request.kfs_cells) == [(1, 1), (4, 2)]


def test_blue_stage_two_request_keeps_gui_coordinates():
    node = make_node()

    accepted, _ = node.request_stage_two(
        StageTwo.Request.BLUE,
        valid_route(),
        ((4, 2), (1, 3)),
    )

    assert accepted
    pose_request = node.set_base_pose_odin_client.requests[0]
    assert (pose_request.x, pose_request.y, pose_request.yaw) == pytest.approx(
        (5.568, 2.2, math.pi))
    complete_relocalization(node.set_base_pose_odin_client)
    request = node.stage_two_client.requests[0]
    assert message_indices(request.move_cells) == list(valid_route())
    assert message_indices(request.kfs_cells) == [(1, 3), (4, 2)]


def test_blue_gui_left_entry_kfs_is_routed_to_left_point_one_lane():
    node = make_node()
    node.request_stage_two(
        StageTwo.Request.BLUE,
        valid_route(),
        ((4, 1),),
    )
    complete_relocalization(node.set_base_pose_odin_client)
    request = node.stage_two_client.requests[0]

    _, point_one_route, point_two_loads = (
        StageTwoController.validate_and_split_request(
            request.mode,
            request.move_cells,
            request.kfs_cells,
        ))

    point_one_route = StageTwoController.point_one_route_for_team(
        point_one_route, StageTwo.Request.BLUE)
    assert point_one_route == (3,)
    assert point_two_loads == ()


def test_team_switch_keeps_gui_selection_but_changes_serialization():
    model = StageTwoGridModel()
    model.route_cells = list(valid_route())
    model.kfs_cells = {(3, 1)}
    original_route = list(model.route_cells)
    original_kfs = set(model.kfs_cells)
    red_node = make_node()
    blue_node = make_node()

    red_node.request_stage_two(
        StageTwo.Request.RED,
        model.validated_route(),
        model.sorted_kfs_cells(),
    )
    blue_node.request_stage_two(
        StageTwo.Request.BLUE,
        model.validated_route(),
        model.sorted_kfs_cells(),
    )
    complete_relocalization(red_node.set_base_pose_odin_client)
    complete_relocalization(blue_node.set_base_pose_odin_client)

    assert model.route_cells == original_route
    assert model.kfs_cells == original_kfs
    assert message_indices(red_node.stage_two_client.requests[0].kfs_cells) == [
        (3, 3)]
    assert message_indices(blue_node.stage_two_client.requests[0].kfs_cells) == [
        (3, 1)]


def test_stage_three_request_contains_selected_count():
    node = make_node()

    accepted, _ = node.request_stage_three(StageThree.Request.RED, 2)

    assert accepted
    request = node.stage_three_client.requests[0]
    assert request.team == StageThree.Request.RED
    assert request.loaded_count == 2


def test_stage_requests_are_mutually_exclusive_until_completion():
    node = make_node()
    accepted, _ = node.request_stage_one(StageOne.Request.RED)

    second_accepted, message = node.request_stage_three(
        StageThree.Request.RED, 3)

    assert accepted
    assert not second_accepted
    assert '正在执行' in message
    complete_relocalization(node.set_base_pose_client)
    node.stage_one_client.futures[0].complete(SimpleNamespace(
        success=True,
        message='ok',
    ))
    assert not node.is_stage_busy()


def test_stage_two_completion_reports_loaded_count():
    node = make_node()
    node.request_stage_two(StageTwo.Request.RED, valid_route(), ())

    complete_relocalization(node.set_base_pose_odin_client)
    node.stage_two_client.futures[0].complete(SimpleNamespace(
        success=True,
        message='ok',
        loaded_count=2,
    ))

    event = node.pop_status_events()[0]
    assert event.success
    assert event.stage == 2
    assert event.loaded_count == 2


def test_relocalization_failure_does_not_call_stage_service_and_unlocks():
    node = make_node()
    node.request_stage_two(StageTwo.Request.RED, valid_route(), ())

    complete_relocalization(
        node.set_base_pose_odin_client,
        success=False,
        message='no fix',
    )

    assert not node.stage_two_client.requests
    assert not node.is_stage_busy()
    event = node.pop_status_events()[0]
    assert not event.success
    assert '重定位失败' in event.message


def test_relocalization_exception_does_not_call_stage_service_and_unlocks():
    node = make_node()
    node.request_stage_one(StageOne.Request.RED)

    node.set_base_pose_client.futures[0].fail(RuntimeError('transport'))

    assert not node.stage_one_client.requests
    assert not node.is_stage_busy()
    event = node.pop_status_events()[0]
    assert not event.success
    assert '重定位调用异常' in event.message


def test_unavailable_relocalization_service_rejects_before_stage_call():
    node = make_node()
    node.set_base_pose_client.ready = False

    accepted, message = node.request_stage_one(StageOne.Request.RED)

    assert not accepted
    assert '重定位服务不可用' in message
    assert not node.stage_one_client.requests
    assert not node.is_stage_busy()


def test_stage_two_success_updates_stage_three_count():
    app = CompetitionGuiApp.__new__(CompetitionGuiApp)
    app.stage_three_count = FakeVariable(3)
    app.status_text = FakeVariable('')

    app._handle_event(CompetitionGuiEvent(
        message='Step2 完成',
        stage=2,
        success=True,
        loaded_count=1,
    ))

    assert app.stage_three_count.get() == 1
    assert '已更新为 1' in app.status_text.get()


def test_zero_loaded_count_keeps_manual_stage_three_selection():
    app = CompetitionGuiApp.__new__(CompetitionGuiApp)
    app.stage_three_count = FakeVariable(3)
    app.status_text = FakeVariable('')

    app._handle_event(CompetitionGuiEvent(
        message='Step2 完成',
        stage=2,
        success=True,
        loaded_count=0,
    ))

    assert app.stage_three_count.get() == 3
    assert '保持不变' in app.status_text.get()


def test_busy_state_disables_configuration_and_joint_widgets():
    app = CompetitionGuiApp.__new__(CompetitionGuiApp)
    app.interactive_widgets = [FakeWidget(), FakeWidget()]
    app.joint_control_widgets = [FakeWidget()]

    app._set_busy_state(True)
    app._set_busy_state(False)

    for widget in (*app.interactive_widgets, *app.joint_control_widgets):
        assert widget.states == [
            {'state': 'disabled'},
            {'state': 'normal'},
        ]


def test_joint_limits_update_atomically():
    node = make_node()

    result = node.on_parameters_changed([
        SimpleNamespace(name='lift_min', value=0.01),
        SimpleNamespace(name='lift_max', value=0.4),
        SimpleNamespace(name='grip_max', value=0.22),
    ])

    assert result.successful
    assert node.lift_min == pytest.approx(0.01)
    assert node.lift_max == pytest.approx(0.4)
    assert node.float_control_ranges['grip'][1] == pytest.approx(0.22)


def test_invalid_joint_limit_update_rolls_back_all_values():
    node = make_node()
    old_lift_min = node.lift_min
    old_ranges = dict(node.float_control_ranges)

    result = node.on_parameters_changed([
        SimpleNamespace(name='lift_min', value=0.02),
        SimpleNamespace(name='grip_min', value=0.3),
    ])

    assert not result.successful
    assert node.lift_min == old_lift_min
    assert node.float_control_ranges == old_ranges


def test_relocalization_poses_update_atomically():
    node = make_node()
    stage_one_pose = (1.0, 2.0, 0.0, 0.0, 0.0, 0.5)
    stage_two_pose = (5.0, -2.0, 0.0, 0.0, 0.0, math.pi)

    result = node.on_parameters_changed([
        SimpleNamespace(
            name='stage_one_relocalization_pose',
            value=list(stage_one_pose),
        ),
        SimpleNamespace(
            name='stage_two_relocalization_pose',
            value=list(stage_two_pose),
        ),
    ])

    assert result.successful
    assert node.stage_one_relocalization_pose == pytest.approx(stage_one_pose)
    assert node.stage_two_relocalization_pose == pytest.approx(stage_two_pose)


def test_invalid_pose_update_rolls_back_joint_and_pose_values():
    node = make_node()
    old_pose = node.stage_one_relocalization_pose
    old_lift_min = node.lift_min

    result = node.on_parameters_changed([
        SimpleNamespace(name='lift_min', value=0.01),
        SimpleNamespace(
            name='stage_one_relocalization_pose',
            value=[0.0, 0.0],
        ),
    ])

    assert not result.successful
    assert node.stage_one_relocalization_pose == old_pose
    assert node.lift_min == old_lift_min


def test_formal_gui_reuses_joint_components_without_debug_key_bindings():
    assert issubclass(GuiControlNode, JointControlNodeMixin)
    assert issubclass(GuiControlApp, JointControlGuiMixin)
    assert issubclass(CompetitionGuiNode, JointControlNodeMixin)
    assert issubclass(CompetitionGuiApp, JointControlGuiMixin)
    assert not hasattr(CompetitionGuiApp, '_on_key_press')
    assert not hasattr(CompetitionGuiNode, 'CMD_VEL_TOPIC')
