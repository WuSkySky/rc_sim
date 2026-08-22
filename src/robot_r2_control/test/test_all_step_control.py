from types import SimpleNamespace
import threading

import pytest

import robot_r2_control.all_step_control as all_step_module
from robot_r2_control.all_step_control import AllStepConfig, AllStepControl
from robot_r2_interfaces.msg import AllStepStatus, CellIndex
from robot_r2_interfaces.srv import (
    ConfigureAllStep,
    StageOne,
    StageThree,
    StageTwo,
)


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


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeClock:
    def __init__(self, value=0.0):
        self.value = value

    def monotonic(self):
        return self.value


def valid_route():
    return ((4, 2), (3, 2), (2, 2), (1, 2), (1, 1))


def default_config():
    return AllStepConfig(
        selected_stage=ConfigureAllStep.Request.STAGE_ONE,
        team=StageOne.Request.RED,
        stage_two_move_cells=(),
        stage_two_kfs_cells=(),
        stage_three_loaded_count=3,
        ready=True,
        message='已准备红方 Step1',
    )


def make_control():
    node = AllStepControl.__new__(AllStepControl)
    node.state_lock = threading.RLock()
    node.busy = False
    node.button_blocked_until = 0.0
    node.button_ignore_sec = 5.0
    node.stage_one_relocalization_pose = (0.0,) * 6
    node.stage_two_relocalization_pose = (
        5.568, -2.2, 0.0, 0.0, 0.0, 3.141592653589793)
    node.config = default_config()
    node.status_publisher = FakePublisher()
    node.set_base_pose_client = FakeClient()
    node.set_base_pose_odin_client = FakeClient()
    node.stage_one_client = FakeClient()
    node.stage_two_client = FakeClient()
    node.stage_three_client = FakeClient()
    return node


def pressed():
    return SimpleNamespace(data=True)


def complete_relocalization(client, success=True, message='ok'):
    client.futures[-1].complete(SimpleNamespace(
        success=success, message=message))


def configure_request(stage, team=StageOne.Request.RED):
    request = ConfigureAllStep.Request()
    request.selected_stage = stage
    request.team = team
    request.stage_two_move_cells = []
    request.stage_two_kfs_cells = []
    request.stage_three_loaded_count = 3
    return request


def test_headless_default_button_starts_red_stage_one(monkeypatch):
    node = make_control()
    clock = FakeClock()
    monkeypatch.setattr(all_step_module.time, 'monotonic', clock.monotonic)

    node.on_button(pressed())

    assert node.busy
    assert len(node.set_base_pose_client.requests) == 1
    assert not node.stage_one_client.requests
    complete_relocalization(node.set_base_pose_client)
    assert node.stage_one_client.requests[0].team == StageOne.Request.RED


def test_button_ignores_five_seconds_then_recovers_without_release(
    monkeypatch,
):
    node = make_control()
    clock = FakeClock()
    monkeypatch.setattr(all_step_module.time, 'monotonic', clock.monotonic)

    node.on_button(pressed())
    complete_relocalization(node.set_base_pose_client)
    node.stage_one_client.futures[-1].complete(SimpleNamespace(
        success=True, message='done'))
    assert not node.busy

    clock.value = 4.999
    node.on_button(pressed())
    assert len(node.set_base_pose_client.requests) == 1

    clock.value = 5.0
    node.on_button(pressed())
    assert len(node.set_base_pose_client.requests) == 2


def test_button_remains_ignored_after_five_seconds_while_stage_is_busy(
    monkeypatch,
):
    node = make_control()
    clock = FakeClock()
    monkeypatch.setattr(all_step_module.time, 'monotonic', clock.monotonic)

    node.on_button(pressed())
    clock.value = 6.0
    node.on_button(pressed())
    assert len(node.set_base_pose_client.requests) == 1

    complete_relocalization(node.set_base_pose_client)
    node.stage_one_client.futures[-1].complete(SimpleNamespace(
        success=True, message='done'))
    clock.value = 7.0
    node.on_button(pressed())
    assert len(node.set_base_pose_client.requests) == 2


def test_invalid_latest_stage_two_route_replaces_old_ready_config():
    node = make_control()
    request = configure_request(ConfigureAllStep.Request.STAGE_TWO)
    response = node.handle_configure(
        request, ConfigureAllStep.Response())

    assert response.applied
    assert not response.ready
    assert not node.config.ready
    assert node.config.selected_stage == ConfigureAllStep.Request.STAGE_TWO
    assert 'must not be empty' in node.config.message


def test_configure_is_rejected_while_a_stage_is_running():
    node = make_control()
    node.busy = True
    request = configure_request(ConfigureAllStep.Request.STAGE_THREE)

    response = node.handle_configure(
        request, ConfigureAllStep.Response())

    assert not response.applied
    assert node.config.selected_stage == ConfigureAllStep.Request.STAGE_ONE


def test_red_stage_two_mirrors_relocalization_and_updates_loaded_count(
    monkeypatch,
):
    node = make_control()
    clock = FakeClock()
    monkeypatch.setattr(all_step_module.time, 'monotonic', clock.monotonic)
    node.config = AllStepConfig(
        selected_stage=ConfigureAllStep.Request.STAGE_TWO,
        team=StageTwo.Request.RED,
        stage_two_move_cells=valid_route(),
        stage_two_kfs_cells=((4, 1), (1, 3)),
        stage_three_loaded_count=3,
        ready=True,
        message='ready',
    )

    node.on_button(pressed())

    pose = node.set_base_pose_odin_client.requests[0]
    assert (pose.x, pose.y, pose.yaw) == pytest.approx(
        (5.568, 2.2, 3.141592653589793))
    complete_relocalization(node.set_base_pose_odin_client)
    request = node.stage_two_client.requests[0]
    assert request.team == StageTwo.Request.RED
    assert request.mode == StageTwo.Request.ROUTE
    assert [
        (cell.forward_index, cell.lateral_index)
        for cell in request.move_cells
    ] == list(valid_route())
    node.stage_two_client.futures[-1].complete(SimpleNamespace(
        success=True, message='done', loaded_count=2))

    assert not node.busy
    assert node.config.selected_stage == ConfigureAllStep.Request.STAGE_TWO
    assert node.config.stage_three_loaded_count == 2
    assert node.status_publisher.messages[-1].state == AllStepStatus.SUCCEEDED


def test_stage_three_uses_selected_team_and_loaded_count(monkeypatch):
    node = make_control()
    clock = FakeClock()
    monkeypatch.setattr(all_step_module.time, 'monotonic', clock.monotonic)
    node.config = AllStepConfig(
        selected_stage=ConfigureAllStep.Request.STAGE_THREE,
        team=StageThree.Request.BLUE,
        stage_two_move_cells=(),
        stage_two_kfs_cells=(),
        stage_three_loaded_count=1,
        ready=True,
        message='ready',
    )

    node.on_button(pressed())

    request = node.stage_three_client.requests[0]
    assert request.team == StageThree.Request.BLUE
    assert request.loaded_count == 1


def test_stage_two_zero_loaded_count_is_reported_but_keeps_manual_count():
    node = make_control()
    node.config = AllStepConfig(
        selected_stage=ConfigureAllStep.Request.STAGE_TWO,
        team=StageTwo.Request.RED,
        stage_two_move_cells=valid_route(),
        stage_two_kfs_cells=(),
        stage_three_loaded_count=3,
        ready=True,
        message='ready',
    )

    node.finish_stage(
        node.config,
        True,
        'done',
        loaded_count=0,
    )

    assert node.config.stage_three_loaded_count == 3
    assert node.status_publisher.messages[-1].loaded_count == 0


def test_unavailable_dependency_unlocks_and_publishes_failure(monkeypatch):
    node = make_control()
    node.set_base_pose_client.ready = False
    clock = FakeClock()
    monkeypatch.setattr(all_step_module.time, 'monotonic', clock.monotonic)

    node.on_button(pressed())

    assert not node.busy
    status = node.status_publisher.messages[-1]
    assert status.state == AllStepStatus.FAILED
    assert '不可用' in status.message


def test_dynamic_parameter_update_is_atomic_and_rejects_invalid_values():
    node = make_control()

    result = node.on_parameters_changed([
        SimpleNamespace(name='button_ignore_sec', value=6.0),
        SimpleNamespace(
            name='stage_one_relocalization_pose',
            value=[1.0, 2.0, 0.0, 0.0, 0.0, 0.5],
        ),
    ])
    assert result.successful
    assert node.button_ignore_sec == pytest.approx(6.0)
    assert node.stage_one_relocalization_pose == pytest.approx(
        (1.0, 2.0, 0.0, 0.0, 0.0, 0.5))

    invalid = node.on_parameters_changed([
        SimpleNamespace(name='button_ignore_sec', value=0.0),
        SimpleNamespace(
            name='stage_one_relocalization_pose',
            value=[9.0, 9.0, 0.0, 0.0, 0.0, 0.0],
        ),
    ])
    assert not invalid.successful
    assert node.button_ignore_sec == pytest.approx(6.0)
    assert node.stage_one_relocalization_pose == pytest.approx(
        (1.0, 2.0, 0.0, 0.0, 0.0, 0.5))


def test_cell_helper_keeps_exact_indices():
    message = CellIndex(forward_index=4, lateral_index=2)
    assert AllStepControl.cell_tuple(message) == (4, 2)
