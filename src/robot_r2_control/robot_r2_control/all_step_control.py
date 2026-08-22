from dataclasses import dataclass, replace
from functools import partial
import math
import threading
import time

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_r2_control.stage_two_control import StageTwoController
from robot_r2_interfaces.msg import AllStepStatus, CellIndex
from robot_r2_interfaces.srv import (
    ConfigureAllStep,
    SetBasePose,
    StageOne,
    StageThree,
    StageTwo,
)
from std_msgs.msg import Bool


@dataclass(frozen=True)
class AllStepConfig:
    selected_stage: int
    team: str
    stage_two_move_cells: tuple
    stage_two_kfs_cells: tuple
    stage_three_loaded_count: int
    ready: bool
    message: str


class AllStepControl(Node):
    BUTTON_TOPIC = '/r2/serial/button'
    CONFIGURE_SERVICE = '/r2/all_step/configure'
    STATUS_TOPIC = '/r2/all_step/status'
    SET_BASE_POSE_SERVICE = '/r2/set_base_pose'
    SET_BASE_POSE_ODIN_SERVICE = '/r2/set_base_pose_odin'
    STAGE_ONE_SERVICE = '/r2/stage_one'
    STAGE_TWO_SERVICE = '/r2/stage_two'
    STAGE_THREE_SERVICE = '/r2/stage_three'

    STAGE_ONE_RELOCALIZATION_DEFAULT = (0.0,) * 6
    STAGE_TWO_RELOCALIZATION_DEFAULT = (
        5.568, -2.2, 0.0, 0.0, 0.0, math.pi)

    def __init__(self):
        super().__init__('all_step_control')
        self.callback_group = ReentrantCallbackGroup()
        self.state_lock = threading.RLock()
        self.busy = False
        self.button_blocked_until = 0.0

        self.declare_parameter('button_ignore_sec', 5.0)
        self.declare_parameter(
            'stage_one_relocalization_pose',
            list(self.STAGE_ONE_RELOCALIZATION_DEFAULT),
        )
        self.declare_parameter(
            'stage_two_relocalization_pose',
            list(self.STAGE_TWO_RELOCALIZATION_DEFAULT),
        )
        self.button_ignore_sec = self.validate_positive_float(
            self.get_parameter('button_ignore_sec').value,
            'button_ignore_sec',
        )
        self.stage_one_relocalization_pose = self.validate_pose(
            self.get_parameter('stage_one_relocalization_pose').value,
            'stage_one_relocalization_pose',
        )
        self.stage_two_relocalization_pose = self.validate_pose(
            self.get_parameter('stage_two_relocalization_pose').value,
            'stage_two_relocalization_pose',
        )

        self.config = AllStepConfig(
            selected_stage=ConfigureAllStep.Request.STAGE_ONE,
            team=StageOne.Request.RED,
            stage_two_move_cells=(),
            stage_two_kfs_cells=(),
            stage_three_loaded_count=3,
            ready=True,
            message='已准备红方 Step1',
        )

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_publisher = self.create_publisher(
            AllStepStatus, self.STATUS_TOPIC, status_qos)
        self.button_subscription = self.create_subscription(
            Bool,
            self.BUTTON_TOPIC,
            self.on_button,
            10,
            callback_group=self.callback_group,
        )
        self.configure_service = self.create_service(
            ConfigureAllStep,
            self.CONFIGURE_SERVICE,
            self.handle_configure,
            callback_group=self.callback_group,
        )

        self.set_base_pose_client = self.create_client(
            SetBasePose,
            self.SET_BASE_POSE_SERVICE,
            callback_group=self.callback_group,
        )
        self.set_base_pose_odin_client = self.create_client(
            SetBasePose,
            self.SET_BASE_POSE_ODIN_SERVICE,
            callback_group=self.callback_group,
        )
        self.stage_one_client = self.create_client(
            StageOne,
            self.STAGE_ONE_SERVICE,
            callback_group=self.callback_group,
        )
        self.stage_two_client = self.create_client(
            StageTwo,
            self.STAGE_TWO_SERVICE,
            callback_group=self.callback_group,
        )
        self.stage_three_client = self.create_client(
            StageThree,
            self.STAGE_THREE_SERVICE,
            callback_group=self.callback_group,
        )
        self.add_on_set_parameters_callback(self.on_parameters_changed)
        self.publish_status(
            AllStepStatus.READY, self.config.message, self.config)

    @staticmethod
    def validate_positive_float(value, name):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'{name} must be numeric')
        converted = float(value)
        if not math.isfinite(converted) or converted <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return converted

    @staticmethod
    def validate_pose(values, name):
        if isinstance(values, (str, bytes)):
            raise ValueError(f'{name} must contain exactly 6 numbers')
        try:
            pose = tuple(values)
        except TypeError as exc:
            raise ValueError(
                f'{name} must contain exactly 6 numbers') from exc
        if len(pose) != 6:
            raise ValueError(f'{name} must contain exactly 6 numbers')
        if any(
            isinstance(value, bool) or not isinstance(value, (int, float))
            for value in pose
        ):
            raise ValueError(f'{name} values must be numeric')
        converted = tuple(float(value) for value in pose)
        if not all(math.isfinite(value) for value in converted):
            raise ValueError(f'{name} values must be finite')
        return converted

    def on_parameters_changed(self, parameters):
        updates = {parameter.name: parameter.value for parameter in parameters}
        try:
            with self.state_lock:
                ignore_sec = self.button_ignore_sec
                stage_one_pose = self.stage_one_relocalization_pose
                stage_two_pose = self.stage_two_relocalization_pose
            if 'button_ignore_sec' in updates:
                ignore_sec = self.validate_positive_float(
                    updates['button_ignore_sec'], 'button_ignore_sec')
            if 'stage_one_relocalization_pose' in updates:
                stage_one_pose = self.validate_pose(
                    updates['stage_one_relocalization_pose'],
                    'stage_one_relocalization_pose',
                )
            if 'stage_two_relocalization_pose' in updates:
                stage_two_pose = self.validate_pose(
                    updates['stage_two_relocalization_pose'],
                    'stage_two_relocalization_pose',
                )
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        with self.state_lock:
            self.button_ignore_sec = ignore_sec
            self.stage_one_relocalization_pose = stage_one_pose
            self.stage_two_relocalization_pose = stage_two_pose
        return SetParametersResult(successful=True)

    @staticmethod
    def cell_tuple(message):
        return int(message.forward_index), int(message.lateral_index)

    @staticmethod
    def cell_message(cell):
        return CellIndex(
            forward_index=int(cell[0]), lateral_index=int(cell[1]))

    @classmethod
    def stage_two_request(cls, config):
        request = StageTwo.Request()
        request.team = config.team
        request.fake_kfs_decision = 0
        request.mode = StageTwo.Request.ROUTE
        request.move_cells = [
            cls.cell_message(cell)
            for cell in config.stage_two_move_cells
        ]
        request.kfs_cells = [
            cls.cell_message(cell)
            for cell in config.stage_two_kfs_cells
        ]
        return request

    @classmethod
    def validate_config(cls, config):
        if config.selected_stage not in (
            ConfigureAllStep.Request.STAGE_ONE,
            ConfigureAllStep.Request.STAGE_TWO,
            ConfigureAllStep.Request.STAGE_THREE,
        ):
            raise ValueError(
                f'未知阶段：{config.selected_stage}')
        if config.team not in (StageOne.Request.RED, StageOne.Request.BLUE):
            raise ValueError('比赛队伍必须为 red 或 blue')
        if config.selected_stage == ConfigureAllStep.Request.STAGE_TWO:
            request = cls.stage_two_request(config)
            StageTwoController.validate_and_split_request(
                request.mode, request.move_cells, request.kfs_cells)
        if (
            config.selected_stage == ConfigureAllStep.Request.STAGE_THREE
            and config.stage_three_loaded_count not in (1, 2, 3)
        ):
            raise ValueError('Step3 KFS 数量必须是 1、2 或 3')

    @staticmethod
    def ready_message(stage):
        return f'已准备 Step{stage}，等待物理按钮'

    def config_from_request(self, request):
        candidate = AllStepConfig(
            selected_stage=int(request.selected_stage),
            team=str(request.team),
            stage_two_move_cells=tuple(
                self.cell_tuple(cell)
                for cell in request.stage_two_move_cells
            ),
            stage_two_kfs_cells=tuple(
                self.cell_tuple(cell)
                for cell in request.stage_two_kfs_cells
            ),
            stage_three_loaded_count=int(
                request.stage_three_loaded_count),
            ready=False,
            message='',
        )
        try:
            self.validate_config(candidate)
        except ValueError as exc:
            return replace(candidate, message=str(exc))
        return replace(
            candidate,
            ready=True,
            message=self.ready_message(candidate.selected_stage),
        )

    def handle_configure(self, request, response):
        candidate = self.config_from_request(request)
        with self.state_lock:
            if self.busy:
                response.applied = False
                response.ready = self.config.ready
                response.message = '阶段任务正在执行，配置未修改'
                return response
            self.config = candidate

        response.applied = True
        response.ready = candidate.ready
        response.message = candidate.message
        state = (
            AllStepStatus.READY
            if candidate.ready
            else AllStepStatus.NOT_READY
        )
        self.publish_status(state, candidate.message, candidate)
        return response

    def publish_status(
        self,
        state,
        message,
        config=None,
        loaded_count=None,
    ):
        if config is None:
            with self.state_lock:
                config = self.config
        status = AllStepStatus()
        status.state = int(state)
        status.selected_stage = int(config.selected_stage)
        status.config_ready = bool(config.ready)
        status.loaded_count = int(
            config.stage_three_loaded_count
            if loaded_count is None
            else loaded_count
        )
        status.message = str(message)
        self.status_publisher.publish(status)

    @staticmethod
    def base_pose_request(pose):
        request = SetBasePose.Request()
        (
            request.x,
            request.y,
            request.z,
            request.roll,
            request.pitch,
            request.yaw,
        ) = pose
        return request

    @staticmethod
    def stage_two_pose_for_team(pose, team):
        converted = list(pose)
        if team == StageTwo.Request.RED:
            converted[1] = -converted[1]
        return tuple(converted)

    def on_button(self, message):
        if not message.data:
            return
        now = time.monotonic()
        with self.state_lock:
            if now < self.button_blocked_until or self.busy:
                return
            self.button_blocked_until = now + self.button_ignore_sec
            config = self.config
            stage_one_pose = self.stage_one_relocalization_pose
            stage_two_pose = self.stage_two_relocalization_pose
            if not config.ready:
                state = AllStepStatus.NOT_READY
                status_message = config.message
            else:
                self.busy = True
                state = AllStepStatus.RUNNING
                status_message = f'Step{config.selected_stage}：正在启动'

        self.publish_status(state, status_message, config)
        if not config.ready:
            return

        if config.selected_stage == ConfigureAllStep.Request.STAGE_ONE:
            request = StageOne.Request()
            request.team = config.team
            self.begin_relocalized_stage(
                config,
                self.set_base_pose_client,
                stage_one_pose,
                self.stage_one_client,
                request,
            )
        elif config.selected_stage == ConfigureAllStep.Request.STAGE_TWO:
            request = self.stage_two_request(config)
            pose = self.stage_two_pose_for_team(stage_two_pose, config.team)
            self.begin_relocalized_stage(
                config,
                self.set_base_pose_odin_client,
                pose,
                self.stage_two_client,
                request,
            )
        else:
            request = StageThree.Request()
            request.team = config.team
            request.loaded_count = config.stage_three_loaded_count
            self.begin_stage(config, self.stage_three_client, request)

    def begin_relocalized_stage(
        self,
        config,
        relocalization_client,
        pose,
        stage_client,
        stage_request,
    ):
        description = f'Step{config.selected_stage}'
        if not relocalization_client.service_is_ready():
            self.finish_stage(
                config, False, f'{description} 重定位服务不可用')
            return
        if not stage_client.service_is_ready():
            self.finish_stage(config, False, f'{description} 服务不可用')
            return
        try:
            future = relocalization_client.call_async(
                self.base_pose_request(pose))
        except Exception as exc:
            self.finish_stage(
                config, False, f'{description} 重定位请求发送失败：{exc}')
            return
        future.add_done_callback(partial(
            self.on_relocalization_complete,
            config=config,
            stage_client=stage_client,
            stage_request=stage_request,
        ))
        self.publish_status(
            AllStepStatus.RUNNING,
            f'{description}：正在重定位',
            config,
        )

    def on_relocalization_complete(
        self,
        future,
        config,
        stage_client,
        stage_request,
    ):
        description = f'Step{config.selected_stage}'
        try:
            response = future.result()
        except Exception as exc:
            self.finish_stage(
                config, False, f'{description} 重定位调用异常：{exc}')
            return
        if response is None:
            self.finish_stage(
                config, False, f'{description} 重定位失败：无响应')
            return
        if not response.success:
            self.finish_stage(
                config,
                False,
                f'{description} 重定位失败：{response.message}',
            )
            return
        self.begin_stage(config, stage_client, stage_request)

    def begin_stage(self, config, client, request):
        description = f'Step{config.selected_stage}'
        if not client.service_is_ready():
            self.finish_stage(config, False, f'{description} 服务不可用')
            return
        try:
            future = client.call_async(request)
        except Exception as exc:
            self.finish_stage(
                config, False, f'{description} 请求发送失败：{exc}')
            return
        future.add_done_callback(partial(
            self.on_stage_complete, config=config))
        self.publish_status(
            AllStepStatus.RUNNING,
            f'{description}：正在执行',
            config,
        )

    def on_stage_complete(self, future, config):
        description = f'Step{config.selected_stage}'
        loaded_count = None
        try:
            response = future.result()
        except Exception as exc:
            self.finish_stage(
                config, False, f'{description} 调用异常：{exc}')
            return
        if response is None:
            self.finish_stage(config, False, f'{description} 失败：无响应')
            return
        if not response.success:
            self.finish_stage(
                config, False, f'{description} 失败：{response.message}')
            return
        if config.selected_stage == ConfigureAllStep.Request.STAGE_TWO:
            loaded_count = int(response.loaded_count)
        self.finish_stage(
            config,
            True,
            f'{description} 完成：{response.message}',
            loaded_count=loaded_count,
        )

    def finish_stage(
        self,
        _run_config,
        success,
        message,
        loaded_count=None,
    ):
        with self.state_lock:
            if loaded_count in (1, 2, 3):
                self.config = replace(
                    self.config,
                    stage_three_loaded_count=loaded_count,
                )
            self.busy = False
            current_config = self.config
        state = (
            AllStepStatus.SUCCEEDED if success else AllStepStatus.FAILED)
        self.publish_status(
            state,
            message,
            current_config,
            loaded_count=loaded_count,
        )


def main(args=None):
    rclpy.init(args=args)
    node = AllStepControl()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
