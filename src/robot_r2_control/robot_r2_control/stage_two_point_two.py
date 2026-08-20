import math
import threading
import time

from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.srv import (
    Align,
    GetKfsType,
    KfsAction,
    MoveRelative,
    MoveToPose,
    SetLift,
    StageTwoPointTwo,
    StageTwoPointTwoExit,
    TraverseStep,
)


STAGE_TWO_POINT_TWO_SERVICE = '/r2/stage_two_point_two'
STAGE_TWO_POINT_TWO_EXIT_SERVICE = '/r2/stage_two_point_two_exit'
POSE_FEEDBACK_TOPIC = '/r2/pose_feedback_odin'
MOVE_TO_POSE_SERVICE = '/r2/move_to_pose'
MOVE_RELATIVE_SERVICE = '/r2/move_relative'
SET_LIFT_SERVICE = '/r2/lift/set'
STEP_TRAVERSE_SERVICE = '/r2/step_traverse'
GET_KFS_TYPE_SERVICE = '/r2/detection/get_type'
ALIGN_TO_KFS_SERVICE = '/r2/align_to_kfs'
KFS_ACTION_SERVICE = '/r2/kfs/action'


class StageTwoPointTwoController(Node):
    FORWARD = (-1, 0)
    LEFT = (0, -1)
    RIGHT = (0, 1)
    POINT_ONE_COVERED_ENTRY = (4, 2)
    KFS_TRUE_CLASS = 'true'

    def __init__(self):
        super().__init__('stage_two_point_two')
        self.callback_group = ReentrantCallbackGroup()
        self.service_lock = threading.Lock()
        self.config_lock = threading.Lock()
        self.pose_condition = threading.Condition()
        self.current_pose = None
        self.loaded_count = 0
        self.team = StageTwoPointTwo.Request.RED
        self.arrival_direction = None
        self.mode = StageTwoPointTwo.Request.STANDARD
        self.move_cells = ()
        self.load_cells = ()
        self.route_load_targets = ()

        self.declare_parameter('dependency_timeout_sec', 2.0)
        self.declare_parameter('pose_timeout_sec', 2.0)
        self.declare_parameter('move_timeout_sec', 35.0)
        self.declare_parameter('traverse_timeout_sec', 150.0)
        self.declare_parameter('detection_timeout_sec', 10.0)
        self.declare_parameter('align_timeout_sec', 15.0)
        self.declare_parameter('load_timeout_sec', 70.0)
        self.declare_parameter('release_timeout_sec', 70.0)

        self.declare_parameter('forward_x', [0.0])
        self.declare_parameter('lateral_y', [0.0])
        self.declare_parameter('cell_heights', [0.0])
        self.declare_parameter('initial_forward_index', 5)
        self.declare_parameter('initial_lateral_index', 2)
        self.declare_parameter('terminal_forward_index', 0)
        self.declare_parameter('chassis_front_offset', 0.35)
        self.declare_parameter('higher_kfs_edge_offset', 0.2)
        self.declare_parameter('lower_kfs_edge_offset', 0.4)
        self.declare_parameter('release_edge_offset', 0.2)
        self.declare_parameter('detection_sample_count', 10)
        self.declare_parameter(
            'exit_cell_0_0_pose', [-2.6, -5.4, math.pi])
        self.declare_parameter('exit_x_offset', 2.9)
        self.declare_parameter('exit_lift_height', 0.03)
        self.declare_parameter('lift_timeout_sec', 15.0)

        self.dependency_timeout_sec = self._positive_parameter(
            'dependency_timeout_sec')
        self.pose_timeout_sec = self._positive_parameter(
            'pose_timeout_sec')
        self.move_timeout_sec = self._positive_parameter(
            'move_timeout_sec')
        self.traverse_timeout_sec = self._positive_parameter(
            'traverse_timeout_sec')
        self.detection_timeout_sec = self._positive_parameter(
            'detection_timeout_sec')
        self.align_timeout_sec = self._positive_parameter(
            'align_timeout_sec')
        self.load_timeout_sec = self._positive_parameter(
            'load_timeout_sec')
        self.release_timeout_sec = self._positive_parameter(
            'release_timeout_sec')

        self.forward_x = self._finite_array_parameter('forward_x')
        self.lateral_y = self._finite_array_parameter('lateral_y')
        self.cell_heights = tuple(
            float(value)
            for value in self.get_parameter('cell_heights').value
        )
        expected_height_count = len(self.forward_x) * len(self.lateral_y)
        if len(self.cell_heights) != expected_height_count:
            raise ValueError(
                'cell_heights must contain '
                f'{expected_height_count} values')
        self.cell_detection_results = [
            [None for _ in self.lateral_y]
            for _ in self.forward_x
        ]

        self.initial_index = (
            int(self.get_parameter('initial_forward_index').value),
            int(self.get_parameter('initial_lateral_index').value),
        )
        self.terminal_forward_index = int(
            self.get_parameter('terminal_forward_index').value)
        self.chassis_front_offset = self._non_negative_parameter(
            'chassis_front_offset')
        self.higher_kfs_edge_offset = self._positive_parameter(
            'higher_kfs_edge_offset')
        self.lower_kfs_edge_offset = self._positive_parameter(
            'lower_kfs_edge_offset')
        self.release_edge_offset = self._positive_parameter(
            'release_edge_offset')
        self.detection_sample_count = int(
            self.get_parameter('detection_sample_count').value)
        if self.detection_sample_count <= 0:
            raise ValueError('detection_sample_count must be positive')
        self.exit_cell_0_0_pose = self._pose_parameter(
            'exit_cell_0_0_pose')
        self.exit_x_offset = self._non_negative_parameter('exit_x_offset')
        self.exit_lift_height = self._non_negative_parameter(
            'exit_lift_height')
        self.lift_timeout_sec = self._positive_parameter(
            'lift_timeout_sec')
        self._validate_exit_config(
            self.exit_cell_0_0_pose, self.exit_x_offset)

        self.get_cell(self.initial_index)
        if not 0 <= self.terminal_forward_index < len(self.forward_x):
            raise ValueError('terminal_forward_index is out of range')

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            POSE_FEEDBACK_TOPIC,
            self.on_pose_feedback,
            10,
            callback_group=self.callback_group,
        )
        self.move_client = self.create_client(
            MoveToPose,
            MOVE_TO_POSE_SERVICE,
            callback_group=self.callback_group,
        )
        self.move_relative_client = self.create_client(
            MoveRelative,
            MOVE_RELATIVE_SERVICE,
            callback_group=self.callback_group,
        )
        self.lift_client = self.create_client(
            SetLift,
            SET_LIFT_SERVICE,
            callback_group=self.callback_group,
        )
        self.traverse_client = self.create_client(
            TraverseStep,
            STEP_TRAVERSE_SERVICE,
            callback_group=self.callback_group,
        )
        self.detection_client = self.create_client(
            GetKfsType,
            GET_KFS_TYPE_SERVICE,
            callback_group=self.callback_group,
        )
        self.align_client = self.create_client(
            Align,
            ALIGN_TO_KFS_SERVICE,
            callback_group=self.callback_group,
        )
        self.kfs_action_client = self.create_client(
            KfsAction,
            KFS_ACTION_SERVICE,
            callback_group=self.callback_group,
        )
        self.task_service = self.create_service(
            StageTwoPointTwo,
            STAGE_TWO_POINT_TWO_SERVICE,
            self.handle_task,
            callback_group=self.callback_group,
        )
        self.exit_service = self.create_service(
            StageTwoPointTwoExit,
            STAGE_TWO_POINT_TWO_EXIT_SERVICE,
            self.handle_exit,
            callback_group=self.callback_group,
        )
        self.add_on_set_parameters_callback(self._on_parameters_changed)

    def _positive_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def _non_negative_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f'{name} must be finite and non-negative')
        return value

    def _finite_array_parameter(self, name):
        values = tuple(
            float(value) for value in self.get_parameter(name).value)
        if not values or not all(math.isfinite(value) for value in values):
            raise ValueError(f'{name} must contain finite values')
        return values

    def _pose_parameter(self, name):
        values = self._finite_array_parameter(name)
        if len(values) != 3:
            raise ValueError(f'{name} must contain exactly 3 values')
        return values

    @staticmethod
    def _validate_exit_config(pose, x_offset):
        if not math.isfinite(pose[0] - x_offset):
            raise ValueError(
                'exit_cell_0_0_pose x minus exit_x_offset must be finite')

    def _on_parameters_changed(self, parameters):
        updates = {parameter.name: parameter.value for parameter in parameters}
        positive_names = {
            'dependency_timeout_sec',
            'pose_timeout_sec',
            'move_timeout_sec',
            'traverse_timeout_sec',
            'detection_timeout_sec',
            'align_timeout_sec',
            'load_timeout_sec',
            'release_timeout_sec',
            'higher_kfs_edge_offset',
            'lower_kfs_edge_offset',
            'release_edge_offset',
            'lift_timeout_sec',
        }
        non_negative_names = {
            'chassis_front_offset',
            'exit_x_offset',
            'exit_lift_height',
        }
        integer_names = {
            'initial_forward_index',
            'initial_lateral_index',
            'terminal_forward_index',
            'detection_sample_count',
        }
        geometry_names = {
            'forward_x',
            'lateral_y',
            'cell_heights',
            'initial_forward_index',
            'initial_lateral_index',
            'terminal_forward_index',
        }
        if geometry_names.intersection(updates) and self.service_lock.locked():
            return SetParametersResult(
                successful=False,
                reason='grid parameters cannot change while a task is active',
            )

        def numeric(name, value):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f'{name} must be numeric')
            converted = float(value)
            if not math.isfinite(converted):
                raise ValueError(f'{name} must be finite')
            return converted

        with self.config_lock:
            candidate = {
                name: getattr(self, name)
                for name in (
                    *positive_names,
                    *non_negative_names,
                    'forward_x',
                    'lateral_y',
                    'cell_heights',
                    'terminal_forward_index',
                    'detection_sample_count',
                    'exit_cell_0_0_pose',
                )
            }
            candidate['initial_forward_index'] = self.initial_index[0]
            candidate['initial_lateral_index'] = self.initial_index[1]

            try:
                for name, value in updates.items():
                    if name in positive_names:
                        converted = numeric(name, value)
                        if converted <= 0.0:
                            raise ValueError(f'{name} must be positive')
                        candidate[name] = converted
                    elif name in non_negative_names:
                        converted = numeric(name, value)
                        if converted < 0.0:
                            raise ValueError(
                                f'{name} must be non-negative')
                        candidate[name] = converted
                    elif name in integer_names:
                        if (
                            isinstance(value, bool) or
                            not isinstance(value, int)
                        ):
                            raise ValueError(f'{name} must be an integer')
                        candidate[name] = int(value)
                    elif name in ('forward_x', 'lateral_y'):
                        if not isinstance(value, (list, tuple)) or not value:
                            raise ValueError(f'{name} must not be empty')
                        candidate[name] = tuple(
                            numeric(name, item) for item in value)
                    elif name == 'cell_heights':
                        if not isinstance(value, (list, tuple)) or not value:
                            raise ValueError('cell_heights must not be empty')
                        heights = []
                        for item in value:
                            if (
                                isinstance(item, bool) or
                                not isinstance(item, (int, float))
                            ):
                                raise ValueError(
                                    'cell_heights must be numeric')
                            converted = float(item)
                            if math.isinf(converted):
                                raise ValueError(
                                    'cell_heights must not contain infinity')
                            heights.append(converted)
                        candidate[name] = tuple(heights)
                    elif name == 'exit_cell_0_0_pose':
                        if (
                            not isinstance(value, (list, tuple)) or
                            len(value) != 3
                        ):
                            raise ValueError(
                                'exit_cell_0_0_pose must contain exactly 3 '
                                'values')
                        candidate[name] = tuple(
                            numeric(name, item) for item in value)

                expected_height_count = (
                    len(candidate['forward_x']) *
                    len(candidate['lateral_y'])
                )
                if len(candidate['cell_heights']) != expected_height_count:
                    raise ValueError(
                        'cell_heights must contain '
                        f'{expected_height_count} values')
                if candidate['detection_sample_count'] <= 0:
                    raise ValueError(
                        'detection_sample_count must be positive')

                initial_index = (
                    candidate['initial_forward_index'],
                    candidate['initial_lateral_index'],
                )
                self.validate_grid_index(
                    initial_index,
                    candidate['forward_x'],
                    candidate['lateral_y'],
                    candidate['cell_heights'],
                )
                if not (
                    0 <= candidate['terminal_forward_index'] <
                    len(candidate['forward_x'])
                ):
                    raise ValueError(
                        'terminal_forward_index is out of range')
                self._validate_exit_config(
                    candidate['exit_cell_0_0_pose'],
                    candidate['exit_x_offset'],
                )
            except ValueError as exc:
                return SetParametersResult(
                    successful=False, reason=str(exc))

            for name in positive_names | non_negative_names:
                setattr(self, name, candidate[name])
            self.forward_x = candidate['forward_x']
            self.lateral_y = candidate['lateral_y']
            self.cell_heights = candidate['cell_heights']
            self.initial_index = initial_index
            self.terminal_forward_index = candidate[
                'terminal_forward_index']
            self.detection_sample_count = candidate[
                'detection_sample_count']
            self.exit_cell_0_0_pose = candidate['exit_cell_0_0_pose']
            if geometry_names.intersection(updates):
                self._reset_detection_results_locked()
        return SetParametersResult(successful=True)

    @staticmethod
    def validate_grid_index(index, forward_x, lateral_y, cell_heights):
        forward_index, lateral_index = index
        if not 0 <= forward_index < len(forward_x):
            raise ValueError(f'forward index {forward_index} is invalid')
        lateral_offset = lateral_index - 1
        if not 0 <= lateral_offset < len(lateral_y):
            raise ValueError(f'lateral index {lateral_index} is invalid')
        height_index = forward_index * len(lateral_y) + lateral_offset
        if not math.isfinite(cell_heights[height_index]):
            raise ValueError(f'cell {index} is not traversable')

    def get_cell(self, index):
        with self.config_lock:
            forward_x = self.forward_x
            lateral_y = self.lateral_y
            cell_heights = self.cell_heights
            team = self.team
        self.validate_grid_index(
            index, forward_x, lateral_y, cell_heights)
        forward_index, lateral_index = index
        lateral_offset = lateral_index - 1
        height_index = forward_index * len(lateral_y) + lateral_offset
        red_y = lateral_y[lateral_offset]
        return (
            forward_x[forward_index],
            red_y if team == StageTwoPointTwo.Request.RED else -red_y,
            cell_heights[height_index],
        )

    def on_pose_feedback(self, msg):
        pose = msg.pose
        current_pose = (
            float(pose.position.x),
            float(pose.position.y),
            self.yaw_from_quaternion(pose.orientation),
        )
        if not all(math.isfinite(value) for value in current_pose):
            return
        with self.pose_condition:
            self.current_pose = current_pose
            self.pose_condition.notify_all()

    def wait_for_pose(self):
        with self.config_lock:
            pose_timeout_sec = self.pose_timeout_sec
        deadline = time.monotonic() + pose_timeout_sec
        with self.pose_condition:
            while self.current_pose is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RuntimeError('Pose feedback unavailable')
                self.pose_condition.wait(timeout=remaining)
            return self.current_pose

    def handle_task(self, request, response):
        with self.service_lock:
            self.loaded_count = int(request.loaded_count)
            self.team = request.team
            self.mode = int(request.mode)
            self.arrival_direction = None
            try:
                self.validate_team(self.team)
                self.validate_loaded_count(self.loaded_count)
                (
                    self.move_cells,
                    self.load_cells,
                    self.route_load_targets,
                ) = self.validate_mode_and_routes(
                    self.mode,
                    request.move_cells,
                    request.load_cells,
                )
                if self.mode != StageTwoPointTwo.Request.ROUTE:
                    self.validate_decision(request.fake_kfs_decision)
                if self.mode == StageTwoPointTwo.Request.STANDARD:
                    self.reset_detection_results()
                self.wait_for_dependencies()
                self.servo_to_initial_cell()
                final_index = self.execute_task(
                    request.fake_kfs_decision)
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                response.loaded_count = self.loaded_count
                return response

            response.success = True
            response.message = (
                f'Stage 2.2 completed for {self.team} team at {final_index}')
            response.loaded_count = self.loaded_count
            return response

    def exit_targets(self, team):
        self.validate_team(team)
        with self.config_lock:
            x, red_y, yaw = self.exit_cell_0_0_pose
            x_offset = self.exit_x_offset
        y = red_y if team == StageTwoPointTwo.Request.RED else -red_y
        return (x, y, yaw), (x - x_offset, y, yaw)

    def exit_config(self, team):
        self.validate_team(team)
        with self.config_lock:
            x, red_y, yaw = self.exit_cell_0_0_pose
            x_offset = self.exit_x_offset
            lift_height = self.exit_lift_height
            lift_timeout_sec = self.lift_timeout_sec
        y = red_y if team == StageTwoPointTwo.Request.RED else -red_y
        targets = ((x, y, yaw), (x - x_offset, y, yaw))
        return lift_height, lift_timeout_sec, targets

    def handle_exit(self, request, response):
        with self.service_lock:
            try:
                lift_height, lift_timeout_sec, targets = self.exit_config(
                    request.team)
                with self.config_lock:
                    dependency_timeout_sec = self.dependency_timeout_sec
                if not self.lift_client.wait_for_service(
                    timeout_sec=dependency_timeout_sec
                ):
                    raise RuntimeError('SetLift service unavailable')
                if not self.move_client.wait_for_service(
                    timeout_sec=dependency_timeout_sec
                ):
                    raise RuntimeError('MoveToPose service unavailable')
                self.set_lift(lift_height, lift_timeout_sec)
                for target in targets:
                    self.move_to_pose(*target)
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                return response

            response.success = True
            response.message = (
                f'Stage 2.2 exit completed for {request.team} team at '
                f'({targets[-1][0]:.3f}, {targets[-1][1]:.3f}, '
                f'{targets[-1][2]:.3f})')
            return response

    def set_lift(self, height, timeout_sec):
        request = SetLift.Request()
        request.front_lift = float(height)
        request.rear_lift = float(height)
        request.tolerance = 0.0
        request.timeout_sec = float(timeout_sec)
        response = self.wait_for_future(
            self.lift_client.call_async(request),
            timeout_sec,
            'SetLift',
        )
        if not response.success:
            raise RuntimeError(f'SetLift failed: {response.message}')

    @staticmethod
    def validate_team(team):
        if team not in (
            StageTwoPointTwo.Request.RED,
            StageTwoPointTwo.Request.BLUE,
        ):
            raise ValueError(f'team must be red or blue, got {team!r}')

    @staticmethod
    def validate_decision(decision):
        if decision not in (
            StageTwoPointTwo.Request.LEFT,
            StageTwoPointTwo.Request.RIGHT,
        ):
            raise ValueError(
                f'fake_kfs_decision must be LEFT(1) or RIGHT(2), got '
                f'{decision}')

    @staticmethod
    def validate_loaded_count(loaded_count):
        if not 0 <= loaded_count <= 3:
            raise ValueError(
                f'loaded_count must be between 0 and 3, got '
                f'{loaded_count}')

    @staticmethod
    def cell_index(message):
        return int(message.forward_index), int(message.lateral_index)

    @staticmethod
    def index_delta(source_index, target_index):
        return (
            target_index[0] - source_index[0],
            target_index[1] - source_index[1],
        )

    def validate_route_step(self, source_index, target_index):
        delta = self.index_delta(source_index, target_index)
        if abs(delta[0]) + abs(delta[1]) != 1:
            raise ValueError(
                f'route cells {source_index} and {target_index} '
                'must be adjacent')
        source = self.get_cell(source_index)
        target = self.get_cell(target_index)
        height_difference = target[2] - source[2]
        if not math.isclose(
            abs(height_difference), 1.0, rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError(
                f'route move {source_index}->{target_index} must cross '
                f'exactly one height level, got {height_difference}')
        return delta

    def validate_mode_and_routes(self, mode, move_messages, load_messages):
        valid_modes = (
            StageTwoPointTwo.Request.STANDARD,
            StageTwoPointTwo.Request.SKIP,
            StageTwoPointTwo.Request.ROUTE,
        )
        if mode not in valid_modes:
            raise ValueError(f'unknown Stage 2.2 mode: {mode}')

        move_cells = tuple(self.cell_index(item) for item in move_messages)
        load_cells = tuple(self.cell_index(item) for item in load_messages)
        if mode != StageTwoPointTwo.Request.ROUTE:
            if move_cells or load_cells:
                raise ValueError(
                    'move_cells and load_cells must be empty unless mode '
                    'is ROUTE')
            return (), (), ()

        if not move_cells:
            raise ValueError('move_cells must not be empty in ROUTE mode')
        if self.initial_index != (5, 2):
            raise ValueError(
                'ROUTE mode requires initial_index to be (5, 2)')
        if move_cells[0] != self.POINT_ONE_COVERED_ENTRY:
            raise ValueError('ROUTE move_cells must start at (4, 2)')
        if move_cells[-1] not in ((1, 1), (1, 3)):
            raise ValueError(
                'ROUTE move_cells must end at (1, 1) or (1, 3)')
        if len(set(move_cells)) != len(move_cells):
            raise ValueError('move_cells must not contain duplicates')
        if len(set(load_cells)) != len(load_cells):
            raise ValueError('load_cells must not contain duplicates')

        terminal = (0, move_cells[-1][1])
        if self.initial_index in move_cells or terminal in move_cells:
            raise ValueError(
                'move_cells must not revisit the initial or automatic '
                'terminal cell')
        full_path = (self.initial_index, *move_cells, terminal)
        for source_index, target_index in zip(full_path, full_path[1:]):
            self.validate_route_step(source_index, target_index)

        for load_index in load_cells:
            self.get_cell(load_index)

        remaining = set(load_cells)
        route_load_targets = []
        for path_offset, current_index in enumerate(move_cells, start=1):
            previous_index = full_path[path_offset - 1]
            next_index = full_path[path_offset + 1]
            arrival_delta = self.index_delta(previous_index, current_index)
            next_delta = self.index_delta(current_index, next_index)
            deltas = [
                self.rotate_left(arrival_delta),
                arrival_delta,
                self.rotate_right(arrival_delta),
            ]
            if next_delta in deltas:
                deltas = [delta for delta in deltas if delta != next_delta]
                deltas.append(next_delta)

            current_targets = []
            for delta in deltas:
                target_index = self.add_index(current_index, delta)
                if target_index not in remaining:
                    continue
                current = self.get_cell(current_index)
                target = self.get_cell(target_index)
                if math.isclose(
                    current[2], target[2], rel_tol=0.0, abs_tol=1e-9
                ):
                    raise ValueError(
                        f'load cell {target_index} has the same height as '
                        f'route cell {current_index}')
                current_targets.append(target_index)
                remaining.remove(target_index)
            route_load_targets.append(tuple(current_targets))

        if remaining:
            unreachable = ', '.join(str(index) for index in sorted(remaining))
            raise ValueError(
                'load_cells are not front/left/right neighbors of the '
                f'route: {unreachable}')
        return move_cells, load_cells, tuple(route_load_targets)

    def reset_detection_results(self):
        with self.config_lock:
            self._reset_detection_results_locked()

    def _reset_detection_results_locked(self):
        self.cell_detection_results = [
            [None for _ in self.lateral_y]
            for _ in self.forward_x
        ]

    def wait_for_dependencies(self):
        with self.config_lock:
            dependency_timeout_sec = self.dependency_timeout_sec
        dependencies = [
            (self.move_client, 'MoveToPose'),
            (self.move_relative_client, 'MoveRelative'),
            (self.traverse_client, 'TraverseStep'),
            (self.kfs_action_client, 'KfsAction'),
        ]
        if self.mode == StageTwoPointTwo.Request.STANDARD:
            dependencies.append((self.detection_client, 'GetKfsType'))
            dependencies.append((self.align_client, 'AlignToKfs'))
        elif self.mode == StageTwoPointTwo.Request.ROUTE:
            dependencies.append((self.align_client, 'AlignToKfs'))
        for client, name in dependencies:
            if not client.wait_for_service(
                timeout_sec=dependency_timeout_sec
            ):
                raise RuntimeError(f'{name} service unavailable')

    @staticmethod
    def wait_for_future(future, timeout_sec, description):
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout_sec + 1.0):
            raise RuntimeError(f'{description} timed out waiting for response')

        response = future.result()
        if response is None:
            raise RuntimeError(f'{description} call failed')
        return response

    def move_to_pose(self, x, y, yaw):
        with self.config_lock:
            move_timeout_sec = self.move_timeout_sec
        request = MoveToPose.Request()
        request.pose_source = MoveToPose.Request.ODIN
        request.x = float(x)
        request.y = float(y)
        request.yaw = float(yaw)
        request.position_tolerance = 0.0
        request.yaw_tolerance = 0.0
        request.timeout_sec = move_timeout_sec
        response = self.wait_for_future(
            self.move_client.call_async(request),
            move_timeout_sec,
            'MoveToPose',
        )
        if not response.success:
            raise RuntimeError(f'MoveToPose failed: {response.message}')

    def move_relative(self, forward, left=0.0, yaw_delta=0.0):
        with self.config_lock:
            move_timeout_sec = self.move_timeout_sec
        request = MoveRelative.Request()
        request.pose_source = MoveRelative.Request.ODIN
        request.forward = float(forward)
        request.left = float(left)
        request.yaw_delta = float(yaw_delta)
        request.position_tolerance = 0.0
        request.yaw_tolerance = 0.0
        request.timeout_sec = move_timeout_sec
        response = self.wait_for_future(
            self.move_relative_client.call_async(request),
            move_timeout_sec,
            'MoveRelative',
        )
        if not response.success:
            raise RuntimeError(f'MoveRelative failed: {response.message}')

    def servo_to_initial_cell(self):
        with self.config_lock:
            initial_index = self.initial_index
        first_index = self.add_index(initial_index, self.FORWARD)
        initial = self.get_cell(initial_index)
        direction_x, direction_y = self.cell_direction(
            initial_index, first_index)
        yaw = math.atan2(direction_y, direction_x)
        self.get_logger().info(
            f'Servoing to Stage 2.2 initial cell {initial_index}')
        self.move_to_pose(initial[0], initial[1], yaw)

    def detect_kfs_type(self):
        """Return (front, left, right) class names."""
        with self.config_lock:
            sample_count = self.detection_sample_count
            detection_timeout_sec = self.detection_timeout_sec
        request = GetKfsType.Request()
        request.sample_count = sample_count
        request.timeout_sec = detection_timeout_sec
        response = self.wait_for_future(
            self.detection_client.call_async(request),
            detection_timeout_sec,
            'GetKfsType',
        )
        # 只看各相机结果；超时/低置信度时 class_name 为空，静默视为非 true。
        return (
            response.front.class_name,
            response.left.class_name,
            response.right.class_name,
        )

    def load_kfs(self, mode):
        with self.config_lock:
            load_timeout_sec = self.load_timeout_sec
        request = KfsAction.Request()
        request.action = KfsAction.Request.LOAD
        request.mode = mode
        response = self.wait_for_future(
            self.kfs_action_client.call_async(request),
            load_timeout_sec,
            'KfsAction load',
        )
        if not response.success:
            raise RuntimeError(f'KfsAction load failed: {response.message}')
        self.loaded_count += 1

    def release_kfs(self):
        with self.config_lock:
            release_timeout_sec = self.release_timeout_sec
        request = KfsAction.Request()
        request.action = KfsAction.Request.RELEASE
        response = self.wait_for_future(
            self.kfs_action_client.call_async(request),
            release_timeout_sec,
            'KfsAction release',
        )
        if not response.success:
            raise RuntimeError(
                f'KfsAction release failed: {response.message}')
        self.loaded_count -= 1

    def traverse_step(self, is_up, distance_to_step):
        with self.config_lock:
            traverse_timeout_sec = self.traverse_timeout_sec
        request = TraverseStep.Request()
        request.direction = (
            TraverseStep.Request.UP
            if is_up
            else TraverseStep.Request.DOWN
        )
        request.distance_to_step = float(distance_to_step)
        response = self.wait_for_future(
            self.traverse_client.call_async(request),
            traverse_timeout_sec,
            'TraverseStep',
        )
        if not response.success:
            raise RuntimeError(f'TraverseStep failed: {response.message}')

    @staticmethod
    def add_index(index, delta):
        return index[0] + delta[0], index[1] + delta[1]

    @staticmethod
    def rotate_left(delta):
        # 机器人左侧摄像头对应的网格方向：逆时针 90°。
        return -delta[1], delta[0]

    @staticmethod
    def rotate_right(delta):
        # 机器人右侧摄像头对应的网格方向：顺时针 90°。
        return delta[1], -delta[0]

    def cell_direction(self, source_index, target_index):
        index_distance = (
            abs(target_index[0] - source_index[0]) +
            abs(target_index[1] - source_index[1])
        )
        if index_distance != 1:
            raise ValueError(
                f'cells {source_index} and {target_index} are not adjacent')

        source = self.get_cell(source_index)
        target = self.get_cell(target_index)
        delta_x = target[0] - source[0]
        delta_y = target[1] - source[1]
        distance = math.hypot(delta_x, delta_y)
        if distance <= 0.0:
            raise ValueError('adjacent cell centers must be distinct')
        return delta_x / distance, delta_y / distance

    def move_one_cell(self, source_index, target_index):
        source = self.get_cell(source_index)
        target = self.get_cell(target_index)
        direction_x, direction_y = self.cell_direction(
            source_index, target_index)
        direction_yaw = math.atan2(direction_y, direction_x)

        height_difference = target[2] - source[2]
        if not math.isclose(
            abs(height_difference), 1.0, rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError(
                f'move {source_index}->{target_index} must cross exactly '
                f'one height level, got {height_difference}')

        actual_pose = self.wait_for_pose()
        self.move_to_pose(actual_pose[0], actual_pose[1], direction_yaw)
        actual_x, actual_y, actual_yaw = self.wait_for_pose()

        boundary_x = (source[0] + target[0]) / 2.0
        boundary_y = (source[1] + target[1]) / 2.0
        with self.config_lock:
            chassis_front_offset = self.chassis_front_offset
        front_x = actual_x + (
            chassis_front_offset * math.cos(actual_yaw))
        front_y = actual_y + (
            chassis_front_offset * math.sin(actual_yaw))
        distance_to_step = (
            (boundary_x - front_x) * direction_x +
            (boundary_y - front_y) * direction_y
        )

        is_up = height_difference > 0.0
        self.get_logger().info(
            f'Moving {source_index}->{target_index}: '
            f'{"up" if is_up else "down"}, '
            f'distance_to_step={distance_to_step:.3f} m')
        self.traverse_step(is_up, distance_to_step)
        self.move_to_pose(target[0], target[1], direction_yaw)
        self.arrival_direction = (direction_x, direction_y)

    def scan_deltas(self, index, arrival_delta):
        if index == self.POINT_ONE_COVERED_ENTRY:
            return (self.FORWARD,)

        _, lateral_index = index
        if lateral_index == 2:
            deltas = [self.LEFT, self.FORWARD, self.RIGHT]
        elif lateral_index == 1:
            deltas = [self.FORWARD, self.RIGHT]
        elif lateral_index == 3:
            deltas = [self.LEFT, self.FORWARD]
        else:
            raise ValueError(
                f'no scan rule for lateral index {lateral_index}')

        if index[0] == 1:
            deltas = [
                delta for delta in deltas
                if delta != self.FORWARD
            ]

        if arrival_delta in (self.LEFT, self.RIGHT):
            came_from_delta = (-arrival_delta[0], -arrival_delta[1])
            deltas = [
                delta for delta in deltas
                if delta != came_from_delta
            ]

        if self.loaded_count == 3:
            deltas = [
                delta for delta in deltas
                if delta == self.FORWARD
            ]

        return tuple(deltas)

    def align_kfs(self):
        with self.config_lock:
            align_timeout_sec = self.align_timeout_sec
        request = Align.Request()
        # 0.0 -> the alignment node uses its own pixel_tolerance / timeout.
        request.pixel_tolerance = 0.0
        request.timeout_sec = 0.0
        response = self.wait_for_future(
            self.align_client.call_async(request),
            align_timeout_sec,
            'AlignToKfs',
        )
        if not response.success:
            self.get_logger().warn(
                f'KFS 对齐未完成，继续装载：{response.message}')

    def pickup_kfs(self, current_index, target_index, return_to_center=True):
        current = self.get_cell(current_index)
        target = self.get_cell(target_index)
        direction_x, direction_y = self.cell_direction(
            current_index, target_index)
        direction_yaw = math.atan2(direction_y, direction_x)

        if target[2] > current[2]:
            with self.config_lock:
                offset = self.higher_kfs_edge_offset
            load_mode_name = 'front'
            load_modes = {
                0: KfsAction.Request.MODE_1,
                1: KfsAction.Request.MODE_1,
                2: KfsAction.Request.MODE_2,
            }
        elif target[2] < current[2]:
            with self.config_lock:
                offset = self.lower_kfs_edge_offset
            load_mode_name = 'top'
            load_modes = {
                0: KfsAction.Request.MODE_3,
                1: KfsAction.Request.MODE_5,
                2: KfsAction.Request.MODE_4,
            }
        else:
            raise ValueError(
                f'KFS at {target_index} has the same height as '
                f'{current_index}')

        self.get_logger().info(
            f'Picking KFS at {target_index} with {load_mode_name} load')
        # 在格子中心转向并视觉对齐，再从对齐后的实际位姿相对前进。
        # 不能伺服到基于标称格心计算的绝对边缘坐标，否则会撤销横向对齐。
        self.move_to_pose(current[0], current[1], direction_yaw)
        self.align_kfs()
        self.move_relative(offset)

        if self.loaded_count == 3:
            if self.arrival_direction is None:
                raise RuntimeError(
                    'arrival direction is unavailable for releasing KFS')
            came_from_x = -self.arrival_direction[0]
            came_from_y = -self.arrival_direction[1]
            with self.config_lock:
                release_edge_offset = self.release_edge_offset
            release_yaw = math.atan2(came_from_y, came_from_x)
            # 先沿装载方向退回对齐后的格心，再转向来路并前往释放边缘。
            # 释放后按相反路径回到同一个对齐后的装载位置。
            self.move_relative(
                -offset,
                yaw_delta=release_yaw - direction_yaw,
            )
            self.move_relative(release_edge_offset)
            self.release_kfs()
            self.move_relative(
                -release_edge_offset,
                yaw_delta=direction_yaw - release_yaw,
            )
            self.move_relative(offset)

        self.load_kfs(load_modes[self.loaded_count])
        if return_to_center:
            self.move_relative(-offset)

    def detect_directions(self, current_index, arrival_delta):
        """One fused detection; return {delta: class_name} for 3 directions."""
        if self.mode == StageTwoPointTwo.Request.SKIP:
            return {}

        front_class, left_class, right_class = self.detect_kfs_type()
        direction_to_class = {
            arrival_delta: front_class,
            self.rotate_left(arrival_delta): left_class,
            self.rotate_right(arrival_delta): right_class,
        }

        direction_results = {}
        for delta, detected_class in direction_to_class.items():
            target_index = self.add_index(current_index, delta)
            target_forward, target_lateral = target_index
            try:
                self.get_cell(target_index)
            except ValueError:
                continue
            cached = self.cell_detection_results[
                target_forward
            ][target_lateral - 1]
            direction_results[delta] = (
                cached if cached is not None else detected_class
            )
        return direction_results

    def load_directions(
        self, current_index, direction_results, load_deltas, next_delta
    ):
        """Load KFS on load_deltas; next_delta last and stays at the edge."""
        if self.mode == StageTwoPointTwo.Request.SKIP:
            return
        ordered = [delta for delta in load_deltas if delta != next_delta]
        if next_delta in load_deltas:
            ordered.append(next_delta)
        for delta in ordered:
            target_index = self.add_index(current_index, delta)
            target_forward, target_lateral = target_index
            if self.cell_detection_results[
                target_forward
            ][target_lateral - 1] is not None:
                continue
            class_name = direction_results.get(delta, '')
            if class_name == self.KFS_TRUE_CLASS:
                self.pickup_kfs(
                    current_index,
                    target_index,
                    return_to_center=(delta != next_delta),
                )
                self.cell_detection_results[target_forward][
                    target_lateral - 1
                ] = class_name
            else:
                self.cell_detection_results[target_forward][
                    target_lateral - 1
                ] = class_name

    def selected_lateral_delta(self, decision, current_index):
        lateral_index = current_index[1]
        if lateral_index == 1:
            return self.RIGHT
        if lateral_index == 3:
            return self.LEFT
        if decision == StageTwoPointTwo.Request.LEFT:
            return self.LEFT
        return self.RIGHT

    def load_route_targets(self, current_index, targets, next_index):
        for target_index in targets:
            self.pickup_kfs(
                current_index,
                target_index,
                return_to_center=(target_index != next_index),
            )

    def execute_route_task(self):
        current_index = self.initial_index
        terminal_index = (0, self.move_cells[-1][1])
        for offset, target_index in enumerate(self.move_cells):
            self.move_one_cell(current_index, target_index)
            current_index = target_index
            next_index = (
                self.move_cells[offset + 1]
                if offset + 1 < len(self.move_cells)
                else terminal_index
            )
            self.load_route_targets(
                current_index,
                self.route_load_targets[offset],
                next_index,
            )

        self.move_one_cell(current_index, terminal_index)
        self.get_logger().info(f'Reached terminal cell {terminal_index}')
        return terminal_index

    def execute_dynamic_task(self, decision):
        current_index = self.initial_index
        next_delta = self.FORWARD

        while True:
            target_index = self.add_index(current_index, next_delta)
            self.move_one_cell(current_index, target_index)
            current_index = target_index
            arrival_delta = next_delta

            if current_index[0] == self.terminal_forward_index:
                self.get_logger().info(
                    f'Reached terminal cell {current_index}')
                return current_index

            direction_results = self.detect_directions(
                current_index, arrival_delta)
            front_result = direction_results.get(self.FORWARD)

            force_lateral = current_index == (1, 2)
            front_is_blocked = (
                front_result is not None and
                front_result != '' and
                front_result != self.KFS_TRUE_CLASS
            )
            if force_lateral or front_is_blocked:
                next_delta = self.selected_lateral_delta(
                    decision, current_index)
                reason = '(1, 2)' if force_lateral else 'front blocked'
                self.get_logger().info(
                    f'Next move is lateral due to {reason}')
            else:
                next_delta = self.FORWARD

            # 观察范围 = scan_deltas + (满 3 个时被迫横向的 next_delta)。
            load_deltas = list(
                self.scan_deltas(current_index, arrival_delta))
            if self.loaded_count == 3 and next_delta not in load_deltas:
                load_deltas.append(next_delta)
            self.load_directions(
                current_index, direction_results, load_deltas, next_delta)

    def execute_task(self, decision):
        if self.mode == StageTwoPointTwo.Request.ROUTE:
            return self.execute_route_task()
        return self.execute_dynamic_task(decision)

    @staticmethod
    def yaw_from_quaternion(quaternion):
        sin_yaw = 2.0 * (
            quaternion.w * quaternion.z +
            quaternion.x * quaternion.y
        )
        cos_yaw = 1.0 - 2.0 * (
            quaternion.y * quaternion.y +
            quaternion.z * quaternion.z
        )
        return math.atan2(sin_yaw, cos_yaw)


def main():
    rclpy.init()
    node = StageTwoPointTwoController()
    executor = MultiThreadedExecutor(num_threads=6)
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
