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

DEFAULT_NEAR_EDGE_OFFSET = 0.4
DEFAULT_FAR_EDGE_OFFSET = 0.5
DEFAULT_POSITION_TOLERANCE = 0.03
DEFAULT_YAW_TOLERANCE = 0.05
DEFAULT_MOVE_TIMEOUT_SEC = 20.0
DEFAULT_LIFT_TOLERANCE = 0.01
DEFAULT_LIFT_TIMEOUT_SEC = 10.0

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

        self.declare_parameter('move_to_pose_service', '/r2/move_to_pose')
        self.declare_parameter('set_lift_service', '/r2/lift/set')
        self.declare_parameter('traverse_service', '/r2/traverse_adjacent_step')
        self.declare_parameter('near_edge_offset', DEFAULT_NEAR_EDGE_OFFSET)
        self.declare_parameter('far_edge_offset', DEFAULT_FAR_EDGE_OFFSET)
        self.declare_parameter('position_tolerance', DEFAULT_POSITION_TOLERANCE)
        self.declare_parameter('yaw_tolerance', DEFAULT_YAW_TOLERANCE)
        self.declare_parameter('move_timeout_sec', DEFAULT_MOVE_TIMEOUT_SEC)
        self.declare_parameter('lift_tolerance', DEFAULT_LIFT_TOLERANCE)
        self.declare_parameter('lift_timeout_sec', DEFAULT_LIFT_TIMEOUT_SEC)

        move_to_pose_service = self.get_parameter('move_to_pose_service').value
        set_lift_service = self.get_parameter('set_lift_service').value
        traverse_service = self.get_parameter('traverse_service').value

        self.default_near_edge_offset = self.get_parameter(
            'near_edge_offset').value
        self.default_far_edge_offset = self.get_parameter(
            'far_edge_offset').value
        self.default_position_tolerance = self.get_parameter(
            'position_tolerance').value
        self.default_yaw_tolerance = self.get_parameter('yaw_tolerance').value
        self.default_move_timeout_sec = self.get_parameter(
            'move_timeout_sec').value
        self.default_lift_tolerance = self.get_parameter(
            'lift_tolerance').value
        self.default_lift_timeout_sec = self.get_parameter(
            'lift_timeout_sec').value

        self.move_to_pose_client = self.create_client(
            MoveToPose,
            move_to_pose_service,
            callback_group=self.callback_group,
        )
        self.set_lift_client = self.create_client(
            SetLift,
            set_lift_service,
            callback_group=self.callback_group,
        )
        self.traverse_service = self.create_service(
            TraverseAdjacentStep,
            traverse_service,
            self.handle_traverse_request,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            f'Step traverse service active: service={traverse_service}, '
            f'move={move_to_pose_service}, lift={set_lift_service}')

    def handle_traverse_request(self, request, response):
        with self.service_lock:
            try:
                self.wait_for_dependencies()
                path = self.build_path(request.rows, request.cols)
                near_edge_offset = (
                    request.near_edge_offset
                    if request.near_edge_offset > 0.0
                    else self.default_near_edge_offset
                )
                far_edge_offset = (
                    request.far_edge_offset
                    if request.far_edge_offset > 0.0
                    else self.default_far_edge_offset
                )
                position_tolerance = (
                    request.position_tolerance
                    if request.position_tolerance > 0.0
                    else self.default_position_tolerance
                )
                yaw_tolerance = (
                    request.yaw_tolerance
                    if request.yaw_tolerance > 0.0
                    else self.default_yaw_tolerance
                )
                move_timeout_sec = (
                    request.move_timeout_sec
                    if request.move_timeout_sec > 0.0
                    else self.default_move_timeout_sec
                )
                lift_tolerance = (
                    request.lift_tolerance
                    if request.lift_tolerance > 0.0
                    else self.default_lift_tolerance
                )
                lift_timeout_sec = (
                    request.lift_timeout_sec
                    if request.lift_timeout_sec > 0.0
                    else self.default_lift_timeout_sec
                )

                self.follow_path(
                    GRID,
                    path,
                    near_edge_offset=near_edge_offset,
                    far_edge_offset=far_edge_offset,
                    position_tolerance=position_tolerance,
                    yaw_tolerance=yaw_tolerance,
                    move_timeout_sec=move_timeout_sec,
                    lift_tolerance=lift_tolerance,
                    lift_timeout_sec=lift_timeout_sec,
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
            raise RuntimeError('/r2/move_to_pose service unavailable')
        if not self.set_lift_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError('/r2/lift/set service unavailable')

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
        position_tolerance,
        yaw_tolerance,
        timeout_sec,
    ):
        request = MoveToPose.Request()
        request.x = float(x)
        request.y = float(y)
        request.yaw = float(yaw)
        request.position_tolerance = float(position_tolerance)
        request.yaw_tolerance = float(yaw_tolerance)
        request.timeout_sec = float(timeout_sec)

        future = self.move_to_pose_client.call_async(request)
        response = self.wait_for_future(future, timeout_sec, 'MoveToPose')
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
        tolerance,
        timeout_sec,
    ):
        request = SetLift.Request()
        request.front_lift = float(front_lift)
        request.rear_lift = float(rear_lift)
        request.tolerance = float(tolerance)
        request.timeout_sec = float(timeout_sec)

        future = self.set_lift_client.call_async(request)
        response = self.wait_for_future(future, timeout_sec, 'SetLift')
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
        position_tolerance,
        yaw_tolerance,
        move_timeout_sec,
        lift_tolerance,
        lift_timeout_sec,
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
            position_tolerance,
            yaw_tolerance,
            move_timeout_sec,
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
                tolerance=lift_tolerance,
                timeout_sec=lift_timeout_sec,
            )
            self.call_move_to_pose(
                current_edge[0],
                current_edge[1],
                direction_yaw,
                position_tolerance,
                yaw_tolerance,
                move_timeout_sec,
            )
            self.call_set_lift(
                *LIFT_PRESETS[4],
                tolerance=lift_tolerance,
                timeout_sec=lift_timeout_sec,
            )
            self.call_move_to_pose(
                target_edge[0],
                target_edge[1],
                direction_yaw,
                position_tolerance,
                yaw_tolerance,
                move_timeout_sec,
            )
            self.call_set_lift(
                *LIFT_PRESETS[5],
                tolerance=lift_tolerance,
                timeout_sec=lift_timeout_sec,
            )
            self.call_move_to_pose(
                target_center[0],
                target_center[1],
                direction_yaw,
                position_tolerance,
                yaw_tolerance,
                move_timeout_sec,
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
                position_tolerance,
                yaw_tolerance,
                move_timeout_sec,
            )
            self.call_set_lift(
                *LIFT_PRESETS[2],
                tolerance=lift_tolerance,
                timeout_sec=lift_timeout_sec,
            )
            self.call_move_to_pose(
                target_edge[0],
                target_edge[1],
                direction_yaw,
                position_tolerance,
                yaw_tolerance,
                move_timeout_sec,
            )
            self.call_set_lift(
                *LIFT_PRESETS[3],
                tolerance=lift_tolerance,
                timeout_sec=lift_timeout_sec,
            )
            self.call_move_to_pose(
                target_center[0],
                target_center[1],
                direction_yaw,
                position_tolerance,
                yaw_tolerance,
                move_timeout_sec,
            )
            self.call_set_lift(
                *LIFT_PRESETS[5],
                tolerance=lift_tolerance,
                timeout_sec=lift_timeout_sec,
            )
            return

        current_edge = get_edge_point(
            current_center, direction, near_edge_offset)
        target_edge = get_edge_point(
            target_center,
            (-direction[0], -direction[1]),
            far_edge_offset,
        )
        self.call_move_to_pose(
            target_center[0],
            target_center[1],
            direction_yaw,
            position_tolerance,
            yaw_tolerance,
            move_timeout_sec,
        )

    def follow_path(
        self,
        grid_data,
        path,
        near_edge_offset,
        far_edge_offset,
        position_tolerance,
        yaw_tolerance,
        move_timeout_sec,
        lift_tolerance,
        lift_timeout_sec,
    ):
        for current_index, target_index in zip(path[:-1], path[1:]):
            self.move_between_adjacent_steps(
                grid_data,
                current_index,
                target_index,
                near_edge_offset=near_edge_offset,
                far_edge_offset=far_edge_offset,
                position_tolerance=position_tolerance,
                yaw_tolerance=yaw_tolerance,
                move_timeout_sec=move_timeout_sec,
                lift_tolerance=lift_tolerance,
                lift_timeout_sec=lift_timeout_sec,
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
