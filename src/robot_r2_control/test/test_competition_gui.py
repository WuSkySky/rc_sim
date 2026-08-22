from types import SimpleNamespace
import threading

import pytest

from robot_r2_control.competition_gui import (
    CompetitionGuiApp,
    CompetitionGuiConfigResult,
    CompetitionGuiNode,
)
from robot_r2_control.joint_control_gui import (
    JointControlGuiMixin,
    JointControlNodeMixin,
)
from robot_r2_control.stage_two_grid_gui import (
    GRID_CELLS,
    StageTwoGridEditor,
    StageTwoGridModel,
    grid_display_row,
    gui_cell_to_service_cell,
)
from robot_r2_interfaces.msg import AllStepStatus
from robot_r2_interfaces.srv import ConfigureAllStep, StageOne, StageTwo


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


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeConfigurationNode:
    def __init__(self):
        self.busy = False
        self.requests = []

    def is_stage_busy(self):
        return self.busy

    def send_configuration(self, request):
        self.requests.append(request)
        return True


def valid_route():
    return ((4, 2), (3, 2), (2, 2), (1, 2), (1, 1))


def message_indices(messages):
    return [
        (message.forward_index, message.lateral_index)
        for message in messages
    ]


def make_app():
    app = CompetitionGuiApp.__new__(CompetitionGuiApp)
    app.node = FakeConfigurationNode()
    app.team_value = FakeVariable(StageOne.Request.RED)
    app.selected_stage = FakeVariable(ConfigureAllStep.Request.STAGE_TWO)
    app.stage_three_count = FakeVariable(3)
    model = StageTwoGridModel()
    model.route_cells = list(valid_route())
    model.kfs_cells = {(4, 1), (1, 3)}
    app.stage_two_grid_editor = SimpleNamespace(model=model)
    app.status_text = FakeVariable('')
    app.configuration_dirty = True
    app.configuration_in_flight = False
    app.next_configuration_retry = 0.0
    return app


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
    'team, cell',
    [
        (StageTwo.Request.BLUE, (2, 1)),
        (StageTwo.Request.BLUE, (2, 3)),
        (StageTwo.Request.RED, (2, 1)),
        (StageTwo.Request.RED, (2, 3)),
    ],
)
def test_gui_cells_keep_team_local_lane_number(team, cell):
    assert gui_cell_to_service_cell(team, cell) == cell


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


def test_grid_editor_reports_every_change():
    changes = []
    editor = StageTwoGridEditor.__new__(StageTwoGridEditor)
    editor.change_callback = lambda: changes.append(True)

    editor._report_change()

    assert changes == [True]


def test_gui_builds_complete_all_step_configuration_snapshot():
    app = make_app()

    request = app._configuration_request()

    assert request.selected_stage == ConfigureAllStep.Request.STAGE_TWO
    assert request.team == StageOne.Request.RED
    assert message_indices(request.stage_two_move_cells) == list(valid_route())
    assert message_indices(request.stage_two_kfs_cells) == [(1, 3), (4, 1)]
    assert request.stage_three_loaded_count == 3


def test_configuration_changes_are_coalesced_while_request_is_in_flight():
    app = make_app()

    app._try_sync_configuration()
    assert len(app.node.requests) == 1
    assert app.configuration_in_flight
    assert not app.configuration_dirty

    app._configuration_changed()
    app._try_sync_configuration()
    assert len(app.node.requests) == 1

    app._handle_configuration_result(CompetitionGuiConfigResult(
        applied=True, ready=True, message='ok'))
    app._try_sync_configuration()
    assert len(app.node.requests) == 2


def test_unavailable_controller_keeps_configuration_dirty_for_retry():
    app = make_app()
    app.node.send_configuration = lambda _request: False

    app._try_sync_configuration()

    assert app.configuration_dirty
    assert not app.configuration_in_flight
    assert '不可用' in app.status_text.get()


def test_stage_two_success_updates_stage_three_count_and_resyncs():
    app = make_app()
    app.configuration_dirty = False
    status = AllStepStatus()
    status.state = AllStepStatus.SUCCEEDED
    status.selected_stage = ConfigureAllStep.Request.STAGE_TWO
    status.loaded_count = 2
    status.message = 'Step2 完成'

    app._handle_status(status)

    assert app.stage_three_count.get() == 2
    assert app.configuration_dirty
    assert '已更新为 2' in app.status_text.get()


def test_stage_two_zero_loaded_count_keeps_manual_stage_three_count():
    app = make_app()
    status = AllStepStatus()
    status.state = AllStepStatus.SUCCEEDED
    status.selected_stage = ConfigureAllStep.Request.STAGE_TWO
    status.loaded_count = 0
    status.message = 'Step2 完成，未装载 KFS'

    app._handle_status(status)

    assert app.stage_three_count.get() == 3
    assert app.status_text.get() == status.message


def test_busy_state_disables_configuration_and_joint_widgets_only():
    app = CompetitionGuiApp.__new__(CompetitionGuiApp)
    app.interactive_widgets = [FakeWidget(), FakeWidget()]
    app.joint_control_widgets = [FakeWidget()]
    app.abort_button = FakeWidget()

    app._set_busy_state(True)
    app._set_busy_state(False)

    for widget in (*app.interactive_widgets, *app.joint_control_widgets):
        assert widget.states == [
            {'state': 'disabled'},
            {'state': 'normal'},
        ]
    assert app.abort_button.states == []


def test_node_status_subscription_drives_busy_state():
    node = CompetitionGuiNode.__new__(CompetitionGuiNode)
    node.state_lock = threading.RLock()
    node.status_events = []
    node.stage_busy = False
    status = AllStepStatus()
    status.state = AllStepStatus.RUNNING

    node.on_all_step_status(status)

    assert node.is_stage_busy()
    assert node.pop_status_events() == [status]


def test_abort_current_task_stays_available():
    node = CompetitionGuiNode.__new__(CompetitionGuiNode)
    node.abort_publisher = FakePublisher()

    message = node.abort_current_task()

    assert message == '已发送取消当前任务请求'
    assert len(node.abort_publisher.messages) == 1


def test_formal_gui_no_longer_owns_stage_start_methods():
    assert issubclass(CompetitionGuiNode, JointControlNodeMixin)
    assert issubclass(CompetitionGuiApp, JointControlGuiMixin)
    assert not hasattr(CompetitionGuiApp, '_start_stage_one')
    assert not hasattr(CompetitionGuiApp, '_start_stage_two')
    assert not hasattr(CompetitionGuiApp, '_start_stage_three')
