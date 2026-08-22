from collections import deque
from dataclasses import dataclass
import threading
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_r2_common import ABORT_TOPIC
from robot_r2_control.joint_control_gui import (
    JointControlGuiMixin,
    JointControlNodeMixin,
)
from robot_r2_control.stage_two_grid_gui import StageTwoGridEditor
from robot_r2_interfaces.msg import AllStepStatus, CellIndex
from robot_r2_interfaces.srv import ConfigureAllStep, StageOne
from std_msgs.msg import Empty


@dataclass(frozen=True)
class CompetitionGuiConfigResult:
    applied: bool
    ready: bool
    message: str


class CompetitionGuiNode(JointControlNodeMixin, Node):
    CONFIGURE_SERVICE = '/r2/all_step/configure'
    STATUS_TOPIC = '/r2/all_step/status'

    def __init__(self):
        super().__init__('competition_gui')
        self.state_lock = threading.RLock()
        self.status_events = deque()
        self.configuration_results = deque()
        self.stage_busy = False

        self.initialize_joint_control()
        self.configure_client = self.create_client(
            ConfigureAllStep, self.CONFIGURE_SERVICE)
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_subscription = self.create_subscription(
            AllStepStatus,
            self.STATUS_TOPIC,
            self.on_all_step_status,
            status_qos,
        )
        self.abort_publisher = self.create_publisher(
            Empty, ABORT_TOPIC, 10)
        self.add_on_set_parameters_callback(self.on_parameters_changed)

    def on_parameters_changed(self, parameters):
        return self.on_joint_parameters_changed(parameters)

    def abort_current_task(self):
        self.abort_publisher.publish(Empty())
        return '已发送取消当前任务请求'

    def on_all_step_status(self, message):
        with self.state_lock:
            self.stage_busy = message.state == AllStepStatus.RUNNING
            self.status_events.append(message)

    def is_stage_busy(self):
        with self.state_lock:
            return self.stage_busy

    def pop_status_events(self):
        with self.state_lock:
            events = list(self.status_events)
            self.status_events.clear()
        return events

    def pop_configuration_results(self):
        with self.state_lock:
            results = list(self.configuration_results)
            self.configuration_results.clear()
        return results

    def send_configuration(self, request):
        if not self.configure_client.service_is_ready():
            return False
        try:
            future = self.configure_client.call_async(request)
        except Exception as exc:
            with self.state_lock:
                self.configuration_results.append(
                    CompetitionGuiConfigResult(
                        applied=False,
                        ready=False,
                        message=f'总控配置请求发送失败：{exc}',
                    )
                )
            return True
        future.add_done_callback(self.on_configuration_complete)
        return True

    def on_configuration_complete(self, future):
        try:
            response = future.result()
        except Exception as exc:
            result = CompetitionGuiConfigResult(
                applied=False,
                ready=False,
                message=f'总控配置调用异常：{exc}',
            )
        else:
            if response is None:
                result = CompetitionGuiConfigResult(
                    applied=False,
                    ready=False,
                    message='总控配置失败：无响应',
                )
            else:
                result = CompetitionGuiConfigResult(
                    applied=bool(response.applied),
                    ready=bool(response.ready),
                    message=str(response.message),
                )
        with self.state_lock:
            self.configuration_results.append(result)


class CompetitionGuiApp(JointControlGuiMixin):
    CONFIG_RETRY_SEC = 1.0

    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Robot R2 正式比赛控制')
        self.root.resizable(False, False)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.team_value = tk.StringVar(value=StageOne.Request.RED)
        self.selected_stage = tk.IntVar(
            value=ConfigureAllStep.Request.STAGE_ONE)
        self.stage_three_count = tk.IntVar(value=3)
        self.status_text = tk.StringVar(value='正在连接 All Step 总控')
        self.interactive_widgets = []
        self.last_stage_busy = None
        self.configuration_dirty = True
        self.configuration_in_flight = False
        self.next_configuration_retry = 0.0
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
        self._build_stage_selector(task_column)
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
                command=self._configuration_changed,
            )
            button.grid(
                row=0,
                column=column,
                sticky='w',
                padx=(0 if column == 0 else 10, 0),
            )
            self.interactive_widgets.append(button)

    def _build_stage_selector(self, parent):
        frame = ttk.LabelFrame(parent, text='物理按钮启动阶段', padding=10)
        frame.grid(row=1, column=0, sticky='ew', pady=(8, 0))
        for column, stage in enumerate((
            ConfigureAllStep.Request.STAGE_ONE,
            ConfigureAllStep.Request.STAGE_TWO,
            ConfigureAllStep.Request.STAGE_THREE,
        )):
            button = ttk.Radiobutton(
                frame,
                text=f'Step{stage}',
                variable=self.selected_stage,
                value=stage,
                command=self._configuration_changed,
            )
            button.grid(row=0, column=column, sticky='w', padx=6)
            self.interactive_widgets.append(button)

    def _build_stage_two_controls(self, parent):
        frame = ttk.LabelFrame(parent, text='Step2 路线模式', padding=10)
        frame.grid(row=2, column=0, sticky='ew', pady=(8, 0))
        self.stage_two_grid_editor = StageTwoGridEditor(
            frame,
            status_callback=self.status_text.set,
            change_callback=self._configuration_changed,
        )
        self.stage_two_grid_editor.grid(row=0, column=0, sticky='n')
        self.interactive_widgets.extend(
            self.stage_two_grid_editor.interactive_widgets)

    def _build_stage_three_controls(self, parent):
        frame = ttk.LabelFrame(parent, text='Step3 KFS 数量', padding=10)
        frame.grid(row=3, column=0, sticky='ew', pady=(8, 0))
        for column, count in enumerate((1, 2, 3)):
            button = ttk.Radiobutton(
                frame,
                text=str(count),
                variable=self.stage_three_count,
                value=count,
                command=self._configuration_changed,
            )
            button.grid(row=0, column=column, sticky='w', padx=6)
            self.interactive_widgets.append(button)

    def _configuration_changed(self):
        self.configuration_dirty = True
        self.next_configuration_retry = 0.0

    @staticmethod
    def _cell_message(cell):
        return CellIndex(
            forward_index=int(cell[0]), lateral_index=int(cell[1]))

    def _configuration_request(self):
        request = ConfigureAllStep.Request()
        request.selected_stage = int(self.selected_stage.get())
        request.team = self.team_value.get()
        request.stage_two_move_cells = [
            self._cell_message(cell)
            for cell in self.stage_two_grid_editor.model.route_cells
        ]
        request.stage_two_kfs_cells = [
            self._cell_message(cell)
            for cell in self.stage_two_grid_editor.model.sorted_kfs_cells()
        ]
        request.stage_three_loaded_count = int(
            self.stage_three_count.get())
        return request

    def _try_sync_configuration(self):
        if (
            not self.configuration_dirty
            or self.configuration_in_flight
            or self.node.is_stage_busy()
        ):
            return
        now = time.monotonic()
        if now < self.next_configuration_retry:
            return
        if not self.node.send_configuration(self._configuration_request()):
            self.status_text.set('All Step 总控服务不可用，等待重试')
            self.next_configuration_retry = now + self.CONFIG_RETRY_SEC
            return
        self.configuration_dirty = False
        self.configuration_in_flight = True

    def joint_commands_allowed(self):
        return not self.node.is_stage_busy()

    def _set_busy_state(self, busy):
        state = tk.DISABLED if busy else tk.NORMAL
        for widget in self.interactive_widgets:
            widget.configure(state=state)
        self.set_joint_controls_state(state)

    def _handle_status(self, status):
        if (
            status.state == AllStepStatus.SUCCEEDED
            and status.selected_stage == ConfigureAllStep.Request.STAGE_TWO
            and status.loaded_count in (1, 2, 3)
        ):
            self.stage_three_count.set(status.loaded_count)
            self.configuration_dirty = True
            self.status_text.set(
                f'{status.message}；Step3 KFS 数量已更新为 '
                f'{status.loaded_count}'
            )
            return
        self.status_text.set(status.message)

    def _handle_configuration_result(self, result):
        self.configuration_in_flight = False
        self.status_text.set(result.message)
        if not result.applied:
            self.configuration_dirty = True
            self.next_configuration_retry = (
                time.monotonic() + self.CONFIG_RETRY_SEC)

    def _poll_ros(self):
        if self._closed:
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
            self.sync_joint_ranges()
            self.sync_joint_feedback()
            for status in self.node.pop_status_events():
                self._handle_status(status)
            for result in self.node.pop_configuration_results():
                self._handle_configuration_result(result)
            busy = self.node.is_stage_busy()
            if busy != self.last_stage_busy:
                self.last_stage_busy = busy
                self._set_busy_state(busy)
            self._try_sync_configuration()
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
