import math
import threading

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.msg import CellIndex
from robot_r2_interfaces.srv import (
    StageTwo,
    StageTwoPointOne,
    StageTwoPointTwo,
)


STAGE_TWO_SERVICE = '/r2/stage_two'
STAGE_TWO_POINT_ONE_SERVICE = '/r2/stage_two_point_one'
STAGE_TWO_POINT_TWO_SERVICE = '/r2/stage_two_point_two'


class StageTwoController(Node):
    ENTRY_FORWARD_INDEX = 4
    GRID_FORWARD_INDICES = frozenset((1, 2, 3, 4))
    GRID_LATERAL_INDICES = frozenset((1, 2, 3))
    POINT_ONE_ORDER = (3, 1, 2)
    TIMEOUT_PARAMETERS = (
        'dependency_timeout_sec',
        'stage_two_point_one_timeout_sec',
        'stage_two_point_two_timeout_sec',
    )

    def __init__(self):
        super().__init__('stage_two_control')
        self.callback_group = ReentrantCallbackGroup()
        self.service_lock = threading.Lock()
        self.config_lock = threading.Lock()
        self.loaded_count = 0

        self.declare_parameter('dependency_timeout_sec', 2.0)
        self.declare_parameter('stage_two_point_one_timeout_sec', 450.0)
        self.declare_parameter('stage_two_point_two_timeout_sec', 1800.0)

        self.dependency_timeout_sec = self._positive_parameter(
            'dependency_timeout_sec')
        self.point_one_timeout_sec = self._positive_parameter(
            'stage_two_point_one_timeout_sec')
        self.point_two_timeout_sec = self._positive_parameter(
            'stage_two_point_two_timeout_sec')

        self.point_one_client = self.create_client(
            StageTwoPointOne,
            STAGE_TWO_POINT_ONE_SERVICE,
            callback_group=self.callback_group,
        )
        self.point_two_client = self.create_client(
            StageTwoPointTwo,
            STAGE_TWO_POINT_TWO_SERVICE,
            callback_group=self.callback_group,
        )
        self.stage_two_service = self.create_service(
            StageTwo,
            STAGE_TWO_SERVICE,
            self.handle_stage_two,
            callback_group=self.callback_group,
        )
        self.add_on_set_parameters_callback(self._on_parameters_changed)

    def _positive_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    @staticmethod
    def _positive_value(name, value):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'{name} must be numeric')
        converted = float(value)
        if not math.isfinite(converted) or converted <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return converted

    def _on_parameters_changed(self, parameters):
        updates = {parameter.name: parameter.value for parameter in parameters}
        try:
            converted = {
                name: self._positive_value(name, value)
                for name, value in updates.items()
                if name in self.TIMEOUT_PARAMETERS
            }
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        attributes = {
            'dependency_timeout_sec': 'dependency_timeout_sec',
            'stage_two_point_one_timeout_sec': 'point_one_timeout_sec',
            'stage_two_point_two_timeout_sec': 'point_two_timeout_sec',
        }
        with self.config_lock:
            for name, value in converted.items():
                setattr(self, attributes[name], value)
        return SetParametersResult(successful=True)

    def config_snapshot(self):
        with self.config_lock:
            return (
                self.dependency_timeout_sec,
                self.point_one_timeout_sec,
                self.point_two_timeout_sec,
            )

    @staticmethod
    def validate_team(team):
        if team not in (StageTwo.Request.RED, StageTwo.Request.BLUE):
            raise ValueError(f'team must be red or blue, got {team!r}')

    @staticmethod
    def validate_decision(decision):
        if decision not in (
            StageTwo.Request.LEFT,
            StageTwo.Request.RIGHT,
        ):
            raise ValueError(
                f'fake_kfs_decision must be LEFT(1) or RIGHT(2), got '
                f'{decision}')

    @staticmethod
    def cell_index(message):
        return int(message.forward_index), int(message.lateral_index)

    @staticmethod
    def cell_message(index):
        return CellIndex(
            forward_index=index[0],
            lateral_index=index[1],
        )

    @classmethod
    def validate_and_split_request(cls, mode, move_messages, kfs_messages):
        valid_modes = (
            StageTwo.Request.STANDARD,
            StageTwo.Request.SKIP,
            StageTwo.Request.ROUTE,
        )
        if mode not in valid_modes:
            raise ValueError(f'unknown StageTwo mode: {mode}')
        if mode != StageTwo.Request.ROUTE:
            return (), (), ()

        move_cells = tuple(cls.cell_index(item) for item in move_messages)
        kfs_cells = tuple(cls.cell_index(item) for item in kfs_messages)
        if not move_cells:
            raise ValueError('move_cells must not be empty in ROUTE mode')
        if len(set(move_cells)) != len(move_cells):
            raise ValueError('move_cells must not contain duplicates')
        if len(set(kfs_cells)) != len(kfs_cells):
            raise ValueError('kfs_cells must not contain duplicates')

        if move_cells[0] != (4, 2):
            raise ValueError('ROUTE move_cells must start at (4, 2)')
        if move_cells[-1] not in ((1, 1), (1, 3)):
            raise ValueError(
                'ROUTE move_cells must end at (1, 1) or (1, 3)')
        for source, target in zip(move_cells, move_cells[1:]):
            distance = (
                abs(target[0] - source[0]) +
                abs(target[1] - source[1])
            )
            if distance != 1:
                raise ValueError(
                    f'route cells {source} and {target} must be adjacent')

        for name, cells in (
            ('move_cells', move_cells),
            ('kfs_cells', kfs_cells),
        ):
            for index in cells:
                if (
                    index[0] not in cls.GRID_FORWARD_INDICES or
                    index[1] not in cls.GRID_LATERAL_INDICES
                ):
                    raise ValueError(
                        f'{name} cell {index} must be inside forward rows '
                        '1..4 and lateral lanes 1..3')

        entry_lanes = {
            lateral_index
            for forward_index, lateral_index in kfs_cells
            if forward_index == cls.ENTRY_FORWARD_INDEX
        }
        point_one_route = tuple(
            lane for lane in cls.POINT_ONE_ORDER if lane in entry_lanes)
        point_two_loads = tuple(sorted(
            index for index in kfs_cells
            if index[0] != cls.ENTRY_FORWARD_INDEX
        ))
        return move_cells, point_one_route, point_two_loads

    def wait_for_dependencies(self, timeout_sec, include_point_one=True):
        dependencies = [(self.point_two_client, 'StageTwoPointTwo')]
        if include_point_one:
            dependencies.insert(
                0, (self.point_one_client, 'StageTwoPointOne'))
        for client, name in dependencies:
            if not client.wait_for_service(timeout_sec=timeout_sec):
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

    def run_point_one(self, team, mode, route_cells, timeout_sec):
        request = StageTwoPointOne.Request()
        request.team = team
        request.loaded_count = self.loaded_count
        request.mode = int(mode)
        request.route_cells = list(route_cells)
        response = self.wait_for_future(
            self.point_one_client.call_async(request),
            timeout_sec,
            'StageTwoPointOne',
        )
        self.loaded_count = int(response.loaded_count)
        if not response.success:
            raise RuntimeError(
                f'StageTwoPointOne failed: {response.message}')

    def run_point_two(
        self,
        team,
        decision,
        mode,
        move_cells,
        load_cells,
        timeout_sec,
    ):
        request = StageTwoPointTwo.Request()
        request.team = team
        request.fake_kfs_decision = int(decision)
        request.loaded_count = self.loaded_count
        request.mode = int(mode)
        request.move_cells = [
            self.cell_message(index) for index in move_cells]
        request.load_cells = [
            self.cell_message(index) for index in load_cells]
        response = self.wait_for_future(
            self.point_two_client.call_async(request),
            timeout_sec,
            'StageTwoPointTwo',
        )
        self.loaded_count = int(response.loaded_count)
        if not response.success:
            raise RuntimeError(
                f'StageTwoPointTwo failed: {response.message}')

    def handle_stage_two(self, request, response):
        with self.service_lock:
            self.loaded_count = 0
            try:
                self.validate_team(request.team)
                mode = int(request.mode)
                move_cells, point_one_route, point_two_loads = (
                    self.validate_and_split_request(
                        mode, request.move_cells, request.kfs_cells))
                if mode != StageTwo.Request.ROUTE:
                    self.validate_decision(request.fake_kfs_decision)

                (
                    dependency_timeout_sec,
                    point_one_timeout_sec,
                    point_two_timeout_sec,
                ) = self.config_snapshot()
                run_point_one = (
                    mode != StageTwo.Request.ROUTE or bool(point_one_route))
                self.wait_for_dependencies(
                    dependency_timeout_sec,
                    include_point_one=run_point_one,
                )

                if run_point_one:
                    self.run_point_one(
                        request.team,
                        mode,
                        (
                            point_one_route
                            if mode == StageTwo.Request.ROUTE else ()
                        ),
                        point_one_timeout_sec,
                    )
                self.run_point_two(
                    request.team,
                    request.fake_kfs_decision,
                    mode,
                    move_cells if mode == StageTwo.Request.ROUTE else (),
                    point_two_loads if mode == StageTwo.Request.ROUTE else (),
                    point_two_timeout_sec,
                )
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                response.loaded_count = self.loaded_count
                return response

            stages = '2.1 -> 2.2' if run_point_one else '2.2 (2.1 skipped)'
            response.success = True
            response.message = (
                f'Stage two completed for {request.team} team: {stages}')
            response.loaded_count = self.loaded_count
            return response


def main():
    rclpy.init()
    node = StageTwoController()
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
