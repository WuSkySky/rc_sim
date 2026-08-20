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
from robot_r2_control.joint_control_gui import (
    JointControlGuiMixin,
    JointControlNodeMixin,
)
from robot_r2_interfaces.msg import CellIndex
from robot_r2_interfaces.srv import (
    SetBasePose,
    StageOne,
    StageThree,
    StageTwo,
)


GRID_CELLS = tuple(
    (forward_index, lateral_index)
    for forward_index in range(1, 5)
    for lateral_index in range(1, 4)
)


def grid_display_row(forward_index):
    return forward_index


def gui_cell_to_service_cell(team, cell):
    if team not in (StageTwo.Request.RED, StageTwo.Request.BLUE):
        raise ValueError(f'team must be red or blue, got {team!r}')
    if cell not in GRID_CELLS:
        raise ValueError(
            f'cell {cell} must be inside forward rows 1..4 and '
            'lateral lanes 1..3')
    if team == StageTwo.Request.RED:
        return cell[0], 4 - cell[1]
    return cell


class StageTwoGridModel:
    def __init__(self):
        self.route_cells = []
        self.kfs_cells = set()

    def toggle_route(self, cell):
        self.validate_cell(cell)
        if cell in self.route_cells:
            self.route_cells.remove(cell)
        else:
            self.route_cells.append(cell)
        return tuple(self.route_cells)

    def toggle_kfs(self, cell):
        self.validate_cell(cell)
        if cell in self.kfs_cells:
            self.kfs_cells.remove(cell)
        else:
            self.kfs_cells.add(cell)
        return self.sorted_kfs_cells()

    def clear_route(self):
        self.route_cells.clear()

    def clear_kfs(self):
        self.kfs_cells.clear()

    def sorted_kfs_cells(self):
        return tuple(sorted(self.kfs_cells))

    @staticmethod
    def validate_cell(cell):
        if cell not in GRID_CELLS:
            raise ValueError(
                f'cell {cell} must be inside forward rows 1..4 and '
                'lateral lanes 1..3')

    def validated_route(self):
        route = tuple(self.route_cells)
        if not route:
            raise ValueError('Step2 路线不能为空')
        if route[0] != (4, 2):
            raise ValueError('Step2 路线必须从 (4,2) 开始')
        if route[-1] not in ((1, 1), (1, 3)):
            raise ValueError('Step2 路线必须在 (1,1) 或 (1,3) 结束')
        for source, target in zip(route, route[1:]):
            distance = (
                abs(target[0] - source[0]) +
                abs(target[1] - source[1])
            )
            if distance != 1:
                raise ValueError(
                    f'Step2 路线 {source} → {target} 不是四邻接移动')
        return route


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
        self.add_on_set_parameters_callback(self.on_parameters_changed)

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
    def cell_message(index):
        return CellIndex(
            forward_index=index[0],
            lateral_index=index[1],
        )

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
        self.validate_team(team)
        with self.state_lock:
            pose = list(self.stage_two_relocalization_pose)
        if team == StageTwo.Request.BLUE:
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
        service_move_cells = tuple(
            gui_cell_to_service_cell(team, index)
            for index in move_cells
        )
        service_kfs_cells = tuple(sorted(
            gui_cell_to_service_cell(team, index)
            for index in kfs_cells
        ))
        request = StageTwo.Request()
        request.team = team
        request.fake_kfs_decision = 0
        request.mode = StageTwo.Request.ROUTE
        request.move_cells = [
            self.cell_message(index) for index in service_move_cells]
        request.kfs_cells = [
            self.cell_message(index) for index in service_kfs_cells]
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
    ROUTE_SELECT_COLOR = '#4f83cc'
    KFS_SELECT_COLOR = '#e09f3e'

    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Robot R2 正式比赛控制')
        self.root.resizable(False, False)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.grid_model = StageTwoGridModel()
        self.team_value = tk.StringVar(value=StageOne.Request.RED)
        self.stage_three_count = tk.IntVar(value=3)
        self.status_text = tk.StringVar(value='已就绪')
        self.route_variables = {
            cell: tk.BooleanVar(value=False) for cell in GRID_CELLS
        }
        self.kfs_variables = {
            cell: tk.BooleanVar(value=False) for cell in GRID_CELLS
        }
        self.route_buttons = {}
        self.kfs_buttons = {}
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

        ttk.Label(
            main_frame,
            textvariable=self.status_text,
            anchor='w',
        ).grid(
            row=1,
            column=0,
            columnspan=2,
            sticky='ew',
            pady=(8, 0),
        )

    def _build_team_controls(self, parent):
        frame = ttk.LabelFrame(parent, text='比赛队伍', padding=10)
        frame.grid(row=0, column=0, sticky='ew')
        for column, (label, value) in enumerate((
            ('红方（负 Y）', StageOne.Request.RED),
            ('蓝方（正 Y）', StageOne.Request.BLUE),
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
        route_frame = ttk.LabelFrame(frame, text='移动路线（按点击顺序）')
        route_frame.grid(row=0, column=0, sticky='n', padx=(0, 8))
        kfs_frame = ttk.LabelFrame(frame, text='需要 Load 的 KFS')
        kfs_frame.grid(row=0, column=1, sticky='n')
        self._build_toggle_grid(route_frame, route=True)
        self._build_toggle_grid(kfs_frame, route=False)

        self.stage_two_button = ttk.Button(
            frame,
            text='启动 Step2',
            command=self._start_stage_two,
        )
        self.stage_two_button.grid(
            row=1, column=0, columnspan=2, sticky='ew', pady=(8, 0))
        self.interactive_widgets.append(self.stage_two_button)

    def _build_toggle_grid(self, parent, route):
        variables = self.route_variables if route else self.kfs_variables
        buttons = self.route_buttons if route else self.kfs_buttons
        color = self.ROUTE_SELECT_COLOR if route else self.KFS_SELECT_COLOR
        callback = self._toggle_route if route else self._toggle_kfs
        clear_callback = self._clear_route if route else self._clear_kfs

        ttk.Label(parent, text='出口').grid(
            row=0,
            column=0,
            columnspan=3,
            sticky='ew',
            pady=(2, 5),
        )
        for forward_index in range(1, 5):
            display_row = grid_display_row(forward_index)
            for lateral_index in range(1, 4):
                cell = (forward_index, lateral_index)
                button = tk.Checkbutton(
                    parent,
                    text='',
                    variable=variables[cell],
                    command=partial(callback, cell),
                    indicatoron=False,
                    selectcolor=color,
                    activebackground=color,
                    width=8,
                    height=2,
                    relief=tk.RAISED,
                    offrelief=tk.RAISED,
                )
                button.grid(
                    row=display_row,
                    column=lateral_index - 1,
                    padx=2,
                    pady=2,
                    sticky='nsew',
                )
                buttons[cell] = button
                self.interactive_widgets.append(button)
        ttk.Label(parent, text='入口').grid(
            row=5,
            column=0,
            columnspan=3,
            sticky='ew',
            pady=(5, 2),
        )
        clear_button = ttk.Button(
            parent,
            text='清空',
            command=clear_callback,
        )
        clear_button.grid(
            row=6, column=0, columnspan=3, sticky='ew', pady=(5, 2))
        self.interactive_widgets.append(clear_button)

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

    def _toggle_route(self, cell):
        self.grid_model.toggle_route(cell)
        self._refresh_route_buttons()

    def _toggle_kfs(self, cell):
        self.grid_model.toggle_kfs(cell)
        self._refresh_kfs_buttons()

    def _clear_route(self):
        self.grid_model.clear_route()
        for variable in self.route_variables.values():
            variable.set(False)
        self._refresh_route_buttons()
        self.status_text.set('已清空 Step2 路线')

    def _clear_kfs(self):
        self.grid_model.clear_kfs()
        for variable in self.kfs_variables.values():
            variable.set(False)
        self._refresh_kfs_buttons()
        self.status_text.set('已清空 Step2 KFS 标记')

    def _refresh_route_buttons(self):
        orders = {
            cell: index
            for index, cell in enumerate(self.grid_model.route_cells, start=1)
        }
        for cell, button in self.route_buttons.items():
            text = f'第 {orders[cell]} 步' if cell in orders else ''
            button.configure(text=text)

    def _refresh_kfs_buttons(self):
        for cell, button in self.kfs_buttons.items():
            text = 'KFS' if cell in self.grid_model.kfs_cells else ''
            button.configure(text=text)

    def _start_stage_one(self):
        self._handle_start_result(
            self.node.request_stage_one(self.team_value.get()))

    def _start_stage_two(self):
        try:
            route = self.grid_model.validated_route()
        except ValueError as exc:
            self.status_text.set(str(exc))
            return
        self._handle_start_result(self.node.request_stage_two(
            self.team_value.get(),
            route,
            self.grid_model.sorted_kfs_cells(),
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
