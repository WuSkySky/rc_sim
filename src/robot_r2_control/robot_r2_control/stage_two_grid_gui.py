import tkinter as tk
from tkinter import ttk

from robot_r2_interfaces.msg import CellIndex
from robot_r2_interfaces.srv import StageTwo


GRID_CELLS = tuple(
    (forward_index, lateral_index)
    for forward_index in range(1, 5)
    for lateral_index in range(1, 4)
)


def grid_display_row(forward_index):
    return forward_index


def gui_cell_to_service_cell(team, cell):
    # GUI 最左列（lateral 1）对应机器人朝向 -x 时的左边，
    # 服务 lane 编号与本队物理 lane 一致，红蓝映射相同。
    if team not in (StageTwo.Request.RED, StageTwo.Request.BLUE):
        raise ValueError(f'team must be red or blue, got {team!r}')
    if cell not in GRID_CELLS:
        raise ValueError('格子必须位于 Step2 的 4×3 矩阵内')
    return cell


def make_stage_two_route_request(team, move_cells, kfs_cells):
    service_move_cells = tuple(
        gui_cell_to_service_cell(team, cell) for cell in move_cells)
    service_kfs_cells = tuple(sorted(
        gui_cell_to_service_cell(team, cell) for cell in kfs_cells))

    request = StageTwo.Request()
    request.team = team
    request.fake_kfs_decision = 0
    request.mode = StageTwo.Request.ROUTE
    request.move_cells = [
        CellIndex(forward_index=cell[0], lateral_index=cell[1])
        for cell in service_move_cells
    ]
    request.kfs_cells = [
        CellIndex(forward_index=cell[0], lateral_index=cell[1])
        for cell in service_kfs_cells
    ]
    return request


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
            raise ValueError('格子必须位于 Step2 的 4×3 矩阵内')

    def validated_route(self):
        route = tuple(self.route_cells)
        if not route:
            raise ValueError('Step2 路线不能为空')
        if route[0] != (4, 2):
            raise ValueError('Step2 路线必须从入口中间格开始')
        if route[-1] not in ((1, 1), (1, 3)):
            raise ValueError('Step2 路线必须在出口左侧或右侧格结束')
        for step, (source, target) in enumerate(
            zip(route, route[1:]),
            start=1,
        ):
            distance = (
                abs(target[0] - source[0]) +
                abs(target[1] - source[1])
            )
            if distance != 1:
                raise ValueError(
                    f'Step2 路线第 {step} 步和第 {step + 1} 步必须共边')
        return route


class StageTwoGridEditor(ttk.Frame):
    ROUTE_SELECT_COLOR = '#4f83cc'
    KFS_SELECT_COLOR = '#e09f3e'

    def __init__(
        self,
        parent,
        compact=False,
        status_callback=None,
        change_callback=None,
    ):
        super().__init__(parent)
        self.model = StageTwoGridModel()
        self.compact = compact
        self.status_callback = status_callback
        self.change_callback = change_callback
        self.route_variables = {
            cell: tk.BooleanVar(value=False) for cell in GRID_CELLS
        }
        self.kfs_variables = {
            cell: tk.BooleanVar(value=False) for cell in GRID_CELLS
        }
        self.route_buttons = {}
        self.kfs_buttons = {}
        self.interactive_widgets = []
        self._build()

    def _build(self):
        route_frame = ttk.LabelFrame(self, text='移动路线（按点击顺序）')
        route_frame.grid(row=0, column=0, sticky='n', padx=(0, 8))
        kfs_frame = ttk.LabelFrame(self, text='需要 Load 的 KFS')
        kfs_frame.grid(row=0, column=1, sticky='n')
        self._build_toggle_grid(route_frame, route=True)
        self._build_toggle_grid(kfs_frame, route=False)

    def _build_toggle_grid(self, parent, route):
        variables = self.route_variables if route else self.kfs_variables
        buttons = self.route_buttons if route else self.kfs_buttons
        color = (
            self.ROUTE_SELECT_COLOR if route else self.KFS_SELECT_COLOR)
        callback = self._toggle_route if route else self._toggle_kfs
        clear_callback = self.clear_route if route else self.clear_kfs
        width = 5 if self.compact else 8
        height = 1 if self.compact else 2

        ttk.Label(parent, text='出口').grid(
            row=0, column=0, columnspan=3, sticky='ew', pady=(2, 5))
        for forward_index in range(1, 5):
            display_row = grid_display_row(forward_index)
            for lateral_index in range(1, 4):
                cell = (forward_index, lateral_index)
                button = tk.Checkbutton(
                    parent,
                    text='',
                    variable=variables[cell],
                    command=lambda selected=cell: callback(selected),
                    indicatoron=False,
                    selectcolor=color,
                    activebackground=color,
                    width=width,
                    height=height,
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
            row=5, column=0, columnspan=3, sticky='ew', pady=(5, 2))
        clear_button = ttk.Button(
            parent,
            text='清空',
            command=clear_callback,
        )
        clear_button.grid(
            row=6, column=0, columnspan=3, sticky='ew', pady=(5, 2))
        self.interactive_widgets.append(clear_button)

    def _toggle_route(self, cell):
        self.model.toggle_route(cell)
        self.refresh_route_buttons()
        self._report_change()

    def _toggle_kfs(self, cell):
        self.model.toggle_kfs(cell)
        self.refresh_kfs_buttons()
        self._report_change()

    def clear_route(self):
        self.model.clear_route()
        for variable in self.route_variables.values():
            variable.set(False)
        self.refresh_route_buttons()
        self._report_status('已清空 Step2 路线')
        self._report_change()

    def clear_kfs(self):
        self.model.clear_kfs()
        for variable in self.kfs_variables.values():
            variable.set(False)
        self.refresh_kfs_buttons()
        self._report_status('已清空 Step2 KFS 标记')
        self._report_change()

    def refresh_route_buttons(self):
        orders = {
            cell: index
            for index, cell in enumerate(self.model.route_cells, start=1)
        }
        for cell, button in self.route_buttons.items():
            text = f'第 {orders[cell]} 步' if cell in orders else ''
            button.configure(text=text)

    def refresh_kfs_buttons(self):
        for cell, button in self.kfs_buttons.items():
            text = 'KFS' if cell in self.model.kfs_cells else ''
            button.configure(text=text)

    def _report_status(self, message):
        if self.status_callback is not None:
            self.status_callback(message)

    def _report_change(self):
        if self.change_callback is not None:
            self.change_callback()

    def set_state(self, state):
        for widget in self.interactive_widgets:
            widget.configure(state=state)
