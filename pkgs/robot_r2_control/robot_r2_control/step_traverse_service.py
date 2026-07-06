import math
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.srv import MoveToPose, SetLift, TraverseAdjacentStep


GRID = [
    [None, (-2.6, -4.2, 0), None, (-2.6, -1.8, 0), None],
    [(-1.4, -5.4, 0), (-1.4, -4.2, 1), (-1.4, -3.0, 2), (-1.4, -1.8, 1), (-1.4, -0.6, 0)],
    [None, (-0.2, -4.2, 2), (-0.2, -3.0, 3), (-0.2, -1.8, 2), None],
    [(1.0, -5.4, 0), (1.0, -4.2, 1), (1.0, -3.0, 2), (1.0, -1.8, 3), None],
    [None, (2.2, -4.2, 2), (2.2, -3.0, 1), (2.2, -1.8, 2), None],
    [None, None, (3.4, -3.0, 0), None, None],
]

MOVE_TO_POSE_SERVICE = '/r2/move_to_pose'
SET_LIFT_SERVICE = '/r2/lift/set'
TRAVERSE_SERVICE = '/r2/traverse_adjacent_step'

DEFAULT_NEAR_EDGE_OFFSET = 0.4
DEFAULT_FAR_EDGE_OFFSET = 0.5
MOVE_TO_POSE_WAIT_TIMEOUT_SEC = 35.0
SET_LIFT_WAIT_TIMEOUT_SEC = 15.0

LIFT_PRESETS = {
    1: (0.2, 0.0),
    2: (-0.2, 0.0),
    3: (0.0, 0.2),
    4: (0.0, -0.2),
    5: (0.0, 0.0),
}


def validate_cell(grid_data, index):
    row, col = index
    if row < 0 or row >= len(grid_data):
        raise IndexError(f'Row {row} out of range')
    if col < 0 or col >= len(grid_data[row]):
        raise IndexError(f'Column {col} out of range')
    cell = grid_data[row][col]
    if cell is None:
        raise ValueError(f'Grid cell {index} is empty')
    return cell


def is_adjacent_4(current_index, target_index):
    row_delta = abs(target_index[0] - current_index[0])
    col_delta = abs(target_index[1] - current_index[1])
    return row_delta + col_delta == 1


def get_direction(current_index, target_index):
    row_delta = target_index[0] - current_index[0]
    col_delta = target_index[1] - current_index[1]

    if row_delta == 1 and col_delta == 0:
        return (1.0, 0.0)
    if row_delta == -1 and col_delta == 0:
        return (-1.0, 0.0)
    if row_delta == 0 and col_delta == 1:
        return (0.0, 1.0)
    if row_delta == 0 and col_delta == -1:
        return (0.0, -1.0)

    raise ValueError(
        f'Grid cells {current_index} and {target_index} are not 4-neighbors')


def get_cell_center(grid_data, index):
    x, y, height_level = validate_cell(grid_data, index)
    return x, y, height_level


def get_edge_point(center, direction, edge_offset):
    return (
        center[0] + direction[0] * edge_offset,
        center[1] + direction[1] * edge_offset,
    )


def get_direction_yaw(direction):
    return math.atan2(direction[1], direction[0])


class StepTraverseService(Node):
    def __init__(self):
        super().__init__('robot_r2_step_traverse_service')
        self.callback_group = ReentrantCallbackGroup()
        self.service_lock = threading.Lock()

        self.declare_parameter('near_edge_offset', DEFAULT_NEAR_EDGE_OFFSET)
        self.declare_parameter('far_edge_offset', DEFAULT_FAR_EDGE_OFFSET)

        self.near_edge_offset = self.get_parameter('near_edge_offset').value
        self.far_edge_offset = self.get_parameter('far_edge_offset').value

        self.move_to_pose_client = self.create_client(
            MoveToPose,
            MOVE_TO_POSE_SERVICE,
            callback_group=self.callback_group,
        )
        self.set_lift_client = self.create_client(
            SetLift,
            SET_LIFT_SERVICE,
            callback_group=self.callback_group,
        )
        self.traverse_service = self.create_service(
            TraverseAdjacentStep,
            TRAVERSE_SERVICE,
            self.handle_traverse_request,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            f'Step traverse service active: service={TRAVERSE_SERVICE}, '
            f'move={MOVE_TO_POSE_SERVICE}, lift={SET_LIFT_SERVICE}')

    def handle_traverse_request(self, request, response):
        with self.service_lock:
            try:
                self.wait_for_dependencies()
                path = self.build_path(request.rows, request.cols)
                self.follow_path(
                    GRID,
                    path,
                    near_edge_offset=self.near_edge_offset,
                    far_edge_offset=self.far_edge_offset,
                )
                response.success = True
                response.message = (
                    f'Traverse succeeded across {len(path) - 1} segment(s)')
            except Exception as exc:
                response.success = False
                response.message = str(exc)
            return response

    @staticmethod
    def build_path(rows, cols):
        if len(rows) != len(cols):
            raise ValueError('rows and cols length mismatch')
        if len(rows) < 2:
            raise ValueError('path must contain at least 2 points')
        return [(int(row), int(col)) for row, col in zip(rows, cols)]

    def wait_for_dependencies(self, timeout_sec=2.0):
        if not self.move_to_pose_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f'{MOVE_TO_POSE_SERVICE} service unavailable')
        if not self.set_lift_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f'{SET_LIFT_SERVICE} service unavailable')

    def wait_for_future(self, future, timeout_sec, description):
        done_event = threading.Event()
        future.add_done_callback(lambda _: done_event.set())

        if not done_event.wait(timeout_sec + 1.0):
            raise RuntimeError(f'{description} timed out waiting for response')

        response = future.result()
        if response is None:
            raise RuntimeError(f'{description} call failed')
        return response

    def call_move_to_pose(
        self,
        x,
        y,
        yaw,
    ):
        request = MoveToPose.Request()
        request.x = float(x)
        request.y = float(y)
        request.yaw = float(yaw)
        request.position_tolerance = 0.0
        request.yaw_tolerance = 0.0
        request.timeout_sec = 0.0

        future = self.move_to_pose_client.call_async(request)
        response = self.wait_for_future(
            future,
            MOVE_TO_POSE_WAIT_TIMEOUT_SEC,
            'MoveToPose',
        )
        if not response.success:
            raise RuntimeError(
                f'MoveToPose failed: {response.message} '
                f'(position_error={response.position_error:.4f}, '
                f'yaw_error={response.yaw_error:.4f})')
        return response

    def call_set_lift(
        self,
        front_lift,
        rear_lift,
    ):
        request = SetLift.Request()
        request.front_lift = float(front_lift)
        request.rear_lift = float(rear_lift)
        request.tolerance = 0.0
        request.timeout_sec = 0.0

        future = self.set_lift_client.call_async(request)
        response = self.wait_for_future(
            future,
            SET_LIFT_WAIT_TIMEOUT_SEC,
            'SetLift',
        )
        if not response.success:
            raise RuntimeError(
                f'SetLift failed: {response.message} '
                f'(front_error={response.front_error:.4f}, '
                f'rear_error={response.rear_error:.4f})')
        return response

    def move_between_adjacent_steps(
        self,
        grid_data,
        current_index,
        target_index,
        near_edge_offset,
        far_edge_offset,
    ):
        current_center = get_cell_center(grid_data, current_index)
        target_center = get_cell_center(grid_data, target_index)

        if not is_adjacent_4(current_index, target_index):
            raise ValueError(
                f'Target {target_index} is not adjacent to {current_index}')

        direction = get_direction(current_index, target_index)
        direction_yaw = get_direction_yaw(direction)
        current_height = current_center[2]
        target_height = target_center[2]

        self.call_move_to_pose(
            current_center[0],
            current_center[1],
            direction_yaw,
        )

        if target_height > current_height:
            current_edge = get_edge_point(
                current_center, direction, near_edge_offset)
            target_edge = get_edge_point(
                target_center,
                (-direction[0], -direction[1]),
                far_edge_offset,
            )
            self.call_set_lift(
                *LIFT_PRESETS[1],
            )
            self.call_move_to_pose(
                current_edge[0],
                current_edge[1],
                direction_yaw,
            )
            self.call_set_lift(
                *LIFT_PRESETS[4],
            )
            self.call_move_to_pose(
                target_edge[0],
                target_edge[1],
                direction_yaw,
            )
            self.call_set_lift(
                *LIFT_PRESETS[5],
            )
            self.call_move_to_pose(
                target_center[0],
                target_center[1],
                direction_yaw,
            )
            return

        if target_height < current_height:
            current_edge = get_edge_point(
                current_center, direction, far_edge_offset)
            target_edge = get_edge_point(
                target_center,
                (-direction[0], -direction[1]),
                near_edge_offset,
            )
            self.call_move_to_pose(
                current_edge[0],
                current_edge[1],
                direction_yaw,
            )
            self.call_set_lift(
                *LIFT_PRESETS[2],
            )
            self.call_move_to_pose(
                target_edge[0],
                target_edge[1],
                direction_yaw,
            )
            self.call_set_lift(
                *LIFT_PRESETS[3],
            )
            self.call_move_to_pose(
                target_center[0],
                target_center[1],
                direction_yaw,
            )
            self.call_set_lift(
                *LIFT_PRESETS[5],
            )
            return

        self.call_move_to_pose(
            target_center[0],
            target_center[1],
            direction_yaw,
        )

    def follow_path(
        self,
        grid_data,
        path,
        near_edge_offset,
        far_edge_offset,
    ):
        for current_index, target_index in zip(path[:-1], path[1:]):
            self.move_between_adjacent_steps(
                grid_data,
                current_index,
                target_index,
                near_edge_offset=near_edge_offset,
                far_edge_offset=far_edge_offset,
            )


def main():
    rclpy.init()
    node = StepTraverseService()
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
