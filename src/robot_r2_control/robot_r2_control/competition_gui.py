from collections import deque
from dataclasses import dataclass
from functools import partial
import math
import threading
import tkinter as tk
from tkinter import ttk

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from robot_r2_common import ABORT_TOPIC
from robot_r2_control.joint_control_gui import (
    JointControlGuiMixin,
    JointControlNodeMixin,
)
from robot_r2_control.stage_two_grid_gui import (
    StageTwoGridEditor,
    make_stage_two_route_request,
)
from robot_r2_interfaces.srv import (
    SetBasePose,
    StageOne,
    StageThree,
    StageTwo,
)
from std_msgs.msg import Empty


@dataclass(frozen=True)
class CompetitionGuiEvent:
    message: str
    stage: int
    success: bool
    loaded_count: int | None = None


class CompetitionGuiNode(JointControlNodeMixin, Node):
    SET_BASE_POSE_SERVICE = '/r2/set_base_pose'
    SET_BASE_POSE_ODIN_SERVICE = '/r2/set_base_pose_odin'
    STAGE_ONE_SERVICE = '/r2/stage_one'
    STAGE_TWO_SERVICE = '/r2/stage_two'
    STAGE_THREE_SERVICE = '/r2/stage_three'
    STAGE_ONE_RELOCALIZATION_DEFAULT = (0.0,) * 6
    STAGE_TWO_RELOCALIZATION_DEFAULT = (
        5.568, -2.2, 0.0, 0.0, 0.0, math.pi)

    def __init__(self):
        super().__init__('competition_gui')
        self.state_lock = threading.RLock()
        self.status_events = deque()
        self.stage_request_in_flight = False
        self.active_stage = None

        self.initialize_joint_control()
        self.declare_parameter(
            'stage_one_relocalization_pose',
            list(self.STAGE_ONE_RELOCALIZATION_DEFAULT),
        )
        self.declare_parameter(
            'stage_two_relocalization_pose',
            list(self.STAGE_TWO_RELOCALIZATION_DEFAULT),
        )
        self.stage_one_relocalization_pose = self.validate_pose_parameter(
            self.get_parameter('stage_one_relocalization_pose').value,
            'stage_one_relocalization_pose',
        )
        self.stage_two_relocalization_pose = self.validate_pose_parameter(
            self.get_parameter('stage_two_relocalization_pose').value,
            'stage_two_relocalization_pose',
        )
        self.set_base_pose_client = self.create_client(
            SetBasePose, self.SET_BASE_POSE_SERVICE)
        self.set_base_pose_odin_client = self.create_client(
            SetBasePose, self.SET_BASE_POSE_ODIN_SERVICE)
        self.stage_one_client = self.create_client(
            StageOne, self.STAGE_ONE_SERVICE)
        self.stage_two_client = self.create_client(
            StageTwo, self.STAGE_TWO_SERVICE)
        self.stage_three_client = self.create_client(
            StageThree, self.STAGE_THREE_SERVICE)
        self.abort_publisher = self.create_publisher(
            Empty, ABORT_TOPIC, 10)
        self.add_on_set_parameters_callback(self.on_parameters_changed)

    def abort_current_task(self):
        self.abort_publisher.publish(Empty())
        return '已发送取消当前任务请求'

    @staticmethod
    def validate_team(team):
        if team not in (StageOne.Request.RED, StageOne.Request.BLUE):
            raise ValueError(f'team must be red or blue, got {team!r}')

    @staticmethod
    def validate_pose_parameter(values, name):
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
                stage_one_pose = self.stage_one_relocalization_pose
                stage_two_pose = self.stage_two_relocalization_pose
            if 'stage_one_relocalization_pose' in updates:
                stage_one_pose = self.validate_pose_parameter(
                    updates['stage_one_relocalization_pose'],
                    'stage_one_relocalization_pose',
                )
            if 'stage_two_relocalization_pose' in updates:
                stage_two_pose = self.validate_pose_parameter(
                    updates['stage_two_relocalization_pose'],
                    'stage_two_relocalization_pose',
                )
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        joint_result = self.on_joint_parameters_changed(parameters)
        if not joint_result.successful:
            return joint_result
        with self.state_lock:
            self.stage_one_relocalization_pose = stage_one_pose
            self.stage_two_relocalization_pose = stage_two_pose
        return SetParametersResult(successful=True)

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

    def stage_two_relocalization_for_team(self, team):
        # 基准为蓝方（负 Y）；红方仅将 Y 取反。
        self.validate_team(team)
        with self.state_lock:
            pose = list(self.stage_two_relocalization_pose)
        if team == StageTwo.Request.RED:
            pose[1] = -pose[1]
        return tuple(pose)

    def is_stage_busy(self):
        with self.state_lock:
            return self.stage_request_in_flight

    def pop_status_events(self):
        with self.state_lock:
            events = list(self.status_events)
            self.status_events.clear()
        return events

    def _begin_stage_request(self, stage, client, request, description):
        with self.state_lock:
            if self.stage_request_in_flight:
                return False, '已有阶段任务正在执行'
            if not client.service_is_ready():
                return False, f'{description} 服务不可用'
            self.stage_request_in_flight = True
            self.active_stage = stage

        try:
            future = client.call_async(request)
        except Exception as exc:
            with self.state_lock:
                self.stage_request_in_flight = False
                self.active_stage = None
            return False, f'{description} 请求发送失败：{exc}'
        future.add_done_callback(partial(
            self._on_stage_complete,
            stage=stage,
            description=description,
        ))
        return True, f'{description}：正在执行'

    def _begin_relocalized_stage_request(
        self,
        stage,
        relocalization_client,
        relocalization_pose,
        stage_client,
        stage_request,
        description,
    ):
        with self.state_lock:
            if self.stage_request_in_flight:
                return False, '已有阶段任务正在执行'
            if not relocalization_client.service_is_ready():
                return False, f'{description} 重定位服务不可用'
            if not stage_client.service_is_ready():
                return False, f'{description} 服务不可用'
            self.stage_request_in_flight = True
            self.active_stage = stage

        request = self.base_pose_request(relocalization_pose)
        try:
            future = relocalization_client.call_async(request)
        except Exception as exc:
            with self.state_lock:
                self.stage_request_in_flight = False
                self.active_stage = None
            return False, f'{description} 重定位请求发送失败：{exc}'
        future.add_done_callback(partial(
            self._on_relocalization_complete,
            stage=stage,
            stage_client=stage_client,
            stage_request=stage_request,
            description=description,
        ))
        return True, f'{description}：正在重定位'

    def request_stage_one(self, team):
        try:
            self.validate_team(team)
        except ValueError as exc:
            return False, str(exc)
        request = StageOne.Request()
        request.team = team
        with self.state_lock:
            relocalization_pose = self.stage_one_relocalization_pose
        return self._begin_relocalized_stage_request(
            1,
            self.set_base_pose_client,
            relocalization_pose,
            self.stage_one_client,
            request,
            'Step1',
        )

    def request_stage_two(self, team, move_cells, kfs_cells):
        try:
            self.validate_team(team)
        except ValueError as exc:
            return False, str(exc)
        request = make_stage_two_route_request(
            team, move_cells, kfs_cells)
        relocalization_pose = self.stage_two_relocalization_for_team(team)
        return self._begin_relocalized_stage_request(
            2,
            self.set_base_pose_odin_client,
            relocalization_pose,
            self.stage_two_client,
            request,
            'Step2',
        )

    def request_stage_three(self, team, loaded_count):
        try:
            self.validate_team(team)
        except ValueError as exc:
            return False, str(exc)
        if loaded_count not in (1, 2, 3):
            return False, 'Step3 KFS 数量必须是 1、2 或 3'
        request = StageThree.Request()
        request.team = team
        request.loaded_count = loaded_count
        return self._begin_stage_request(
            3, self.stage_three_client, request, 'Step3')

    def _finish_stage(self, stage, message, success, loaded_count=None):
        with self.state_lock:
            self.stage_request_in_flight = False
            self.active_stage = None
            self.status_events.append(CompetitionGuiEvent(
                message=message,
                stage=stage,
                success=success,
                loaded_count=loaded_count,
            ))

    def _on_relocalization_complete(
        self,
        future,
        stage,
        stage_client,
        stage_request,
        description,
    ):
        try:
            response = future.result()
        except Exception as exc:
            self._finish_stage(
                stage,
                f'{description} 重定位调用异常：{exc}',
                False,
            )
            return
        if response is None:
            self._finish_stage(
                stage, f'{description} 重定位失败：无响应', False)
            return
        if not response.success:
            self._finish_stage(
                stage,
                f'{description} 重定位失败：{response.message}',
                False,
            )
            return

        try:
            stage_future = stage_client.call_async(stage_request)
        except Exception as exc:
            self._finish_stage(
                stage,
                f'{description} 请求发送失败：{exc}',
                False,
            )
            return
        stage_future.add_done_callback(partial(
            self._on_stage_complete,
            stage=stage,
            description=description,
        ))

    def _on_stage_complete(self, future, stage, description):
        loaded_count = None
        try:
            response = future.result()
        except Exception as exc:
            success = False
            message = f'{description} 调用异常：{exc}'
        else:
            if response is None:
                success = False
                message = f'{description} 失败：无响应'
            elif response.success:
                success = True
                message = f'{description} 完成：{response.message}'
                if stage == 2:
                    loaded_count = int(response.loaded_count)
            else:
                success = False
                message = f'{description} 失败：{response.message}'

        self._finish_stage(
            stage,
            message,
            success,
            loaded_count=loaded_count,
        )


class CompetitionGuiApp(JointControlGuiMixin):
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Robot R2 正式比赛控制')
        self.root.resizable(False, False)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.team_value = tk.StringVar(value=StageOne.Request.RED)
        self.stage_three_count = tk.IntVar(value=3)
        self.status_text = tk.StringVar(value='已就绪')
        self.interactive_widgets = []
        self.last_stage_busy = None
        self._closed = False

        self.initialize_joint_ui()
        self._build_ui()
        self.sync_joint_ranges()
        self.root.after(10, self._poll_ros)

    def _build_ui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.grid(row=0, column=0, sticky='nsew')
        task_column = ttk.Frame(main_frame)
        task_column.grid(row=0, column=0, sticky='new', padx=(0, 10))
        joint_column = ttk.Frame(main_frame)
        joint_column.grid(row=0, column=1, sticky='new')

        self._build_team_controls(task_column)
        self._build_stage_one_controls(task_column)
        self._build_stage_two_controls(task_column)
        self._build_stage_three_controls(task_column)
        self.build_joint_controls(joint_column)

        self.abort_button = ttk.Button(
            main_frame,
            text='取消当前任务',
            command=self._abort_current_task,
        )
        self.abort_button.grid(
            row=1,
            column=0,
            columnspan=2,
            sticky='ew',
            pady=(8, 0),
        )

        ttk.Label(
            main_frame,
            textvariable=self.status_text,
            anchor='w',
        ).grid(
            row=2,
            column=0,
            columnspan=2,
            sticky='ew',
            pady=(8, 0),
        )

    def _abort_current_task(self):
        self.status_text.set(self.node.abort_current_task())

    def _build_team_controls(self, parent):
        frame = ttk.LabelFrame(parent, text='比赛队伍', padding=10)
        frame.grid(row=0, column=0, sticky='ew')
        for column, (label, value) in enumerate((
            ('红方（正 Y）', StageOne.Request.RED),
            ('蓝方（负 Y）', StageOne.Request.BLUE),
        )):
            button = ttk.Radiobutton(
                frame,
                text=label,
                variable=self.team_value,
                value=value,
            )
            button.grid(
                row=0,
                column=column,
                sticky='w',
                padx=(0 if column == 0 else 10, 0),
            )
            self.interactive_widgets.append(button)

    def _build_stage_one_controls(self, parent):
        frame = ttk.LabelFrame(parent, text='Step1', padding=10)
        frame.grid(row=1, column=0, sticky='ew', pady=(8, 0))
        self.stage_one_button = ttk.Button(
            frame,
            text='启动 Step1',
            command=self._start_stage_one,
        )
        self.stage_one_button.grid(row=0, column=0, sticky='ew')
        self.interactive_widgets.append(self.stage_one_button)

    def _build_stage_two_controls(self, parent):
        frame = ttk.LabelFrame(parent, text='Step2 路线模式', padding=10)
        frame.grid(row=2, column=0, sticky='ew', pady=(8, 0))
        self.stage_two_grid_editor = StageTwoGridEditor(
            frame,
            status_callback=self.status_text.set,
        )
        self.stage_two_grid_editor.grid(
            row=0, column=0, columnspan=2, sticky='n')
        self.interactive_widgets.extend(
            self.stage_two_grid_editor.interactive_widgets)

        self.stage_two_button = ttk.Button(
            frame,
            text='启动 Step2',
            command=self._start_stage_two,
        )
        self.stage_two_button.grid(
            row=1, column=0, columnspan=2, sticky='ew', pady=(8, 0))
        self.interactive_widgets.append(self.stage_two_button)

    def _build_stage_three_controls(self, parent):
        frame = ttk.LabelFrame(parent, text='Step3 KFS 数量', padding=10)
        frame.grid(row=3, column=0, sticky='ew', pady=(8, 0))
        for column, count in enumerate((1, 2, 3)):
            button = ttk.Radiobutton(
                frame,
                text=str(count),
                variable=self.stage_three_count,
                value=count,
            )
            button.grid(row=0, column=column, sticky='w', padx=6)
            self.interactive_widgets.append(button)
        self.stage_three_button = ttk.Button(
            frame,
            text='启动 Step3',
            command=self._start_stage_three,
        )
        self.stage_three_button.grid(
            row=1, column=0, columnspan=3, sticky='ew', pady=(8, 0))
        self.interactive_widgets.append(self.stage_three_button)

    def _start_stage_one(self):
        self._handle_start_result(
            self.node.request_stage_one(self.team_value.get()))

    def _start_stage_two(self):
        try:
            route = self.stage_two_grid_editor.model.validated_route()
        except ValueError as exc:
            self.status_text.set(str(exc))
            return
        self._handle_start_result(self.node.request_stage_two(
            self.team_value.get(),
            route,
            self.stage_two_grid_editor.model.sorted_kfs_cells(),
        ))

    def _start_stage_three(self):
        self._handle_start_result(self.node.request_stage_three(
            self.team_value.get(),
            self.stage_three_count.get(),
        ))

    def _handle_start_result(self, result):
        accepted, message = result
        self.status_text.set(message)
        if accepted:
            self.last_stage_busy = True
            self._set_busy_state(True)

    def joint_commands_allowed(self):
        return not self.node.is_stage_busy()

    def _set_busy_state(self, busy):
        state = tk.DISABLED if busy else tk.NORMAL
        for widget in self.interactive_widgets:
            widget.configure(state=state)
        self.set_joint_controls_state(state)

    def _handle_event(self, event):
        message = event.message
        if event.stage == 2 and event.success:
            if event.loaded_count in (1, 2, 3):
                self.stage_three_count.set(event.loaded_count)
                message += f'；Step3 KFS 数量已更新为 {event.loaded_count}'
            elif event.loaded_count == 0:
                message += '；未装载 KFS，Step3 数量保持不变'
        self.status_text.set(message)

    def _poll_ros(self):
        if self._closed:
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
            self.sync_joint_ranges()
            self.sync_joint_feedback()
            for event in self.node.pop_status_events():
                self._handle_event(event)
            busy = self.node.is_stage_busy()
            if busy != self.last_stage_busy:
                self.last_stage_busy = busy
                self._set_busy_state(busy)
        except Exception as exc:
            self.node.get_logger().error(f'正式 GUI ROS 回调异常：{exc}')
            self.status_text.set(f'ROS 回调异常：{exc}')
        if not self._closed:
            self.root.after(10, self._poll_ros)

    def run(self):
        self.root.mainloop()

    def close(self):
        if self._closed:
            return
        self._closed = True
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CompetitionGuiNode()
        CompetitionGuiApp(node).run()
    except KeyboardInterrupt:
        pass
    except tk.TclError as exc:
        if node is not None:
            node.get_logger().error(f'无法启动正式图形界面：{exc}')
        else:
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
