import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.srv import LoadKfs, MoveToPose, SetLift
from robot_r2_interfaces.srv import StageTwo


class StageTwoController(Node):
    def __init__(self):
        super().__init__('stage_two_control')
        self.callback_group = ReentrantCallbackGroup()
        self.service_lock = threading.Lock()

        self.declare_parameter('service_name', '/r2/stage_two')
        self.declare_parameter('move_to_pose_service', '/r2/move_to_pose')
        self.declare_parameter('set_lift_service', '/r2/lift/set')
        self.declare_parameter('load_kfs_service', '/r2/kfs/load')
        self.declare_parameter('dependency_timeout_sec', 2.0)
        self.declare_parameter('move_wait_timeout_sec', 35.0)
        self.declare_parameter('lift_wait_timeout_sec', 15.0)
        self.declare_parameter('load_wait_timeout_sec', 70.0)
        self.declare_parameter('align_placeholder_wait_sec', 0.5)

        self.declare_parameter('map_rows', 6)
        self.declare_parameter('map_columns', 5)
        self.declare_parameter('map_x', [0.0])
        self.declare_parameter('map_y', [0.0])
        self.declare_parameter('map_heights', [0.0])
        self.declare_parameter('near_edge_offset', 0.3)
        self.declare_parameter('kfs_align_edge_offset', 0.72)
        self.declare_parameter('kfs_pickup_edge_offset', 0.55)
        self.declare_parameter('target_far_edge_offset', 0.487)
        self.declare_parameter('source_far_edge_offset', 0.487)
        self.declare_parameter('down_lower_edge_offset', 0.2)
        self.declare_parameter('lift_up_front', 0.2)
        self.declare_parameter('lift_up_rear', 0.2)
        self.declare_parameter('lift_at_near_edge_front', 0.0)
        self.declare_parameter('lift_at_near_edge_rear', 0.2)
        self.declare_parameter('lift_front_up_front', 0.2)
        self.declare_parameter('lift_front_up_rear', 0.0)
        self.declare_parameter('lift_final_front', 0.0)
        self.declare_parameter('lift_final_rear', 0.0)

        service_name = self.get_parameter('service_name').value
        move_service = self.get_parameter('move_to_pose_service').value
        lift_service = self.get_parameter('set_lift_service').value
        load_service = self.get_parameter('load_kfs_service').value
        self.dependency_timeout_sec = self.get_parameter(
            'dependency_timeout_sec').value
        self.move_wait_timeout_sec = self.get_parameter(
            'move_wait_timeout_sec').value
        self.lift_wait_timeout_sec = self.get_parameter(
            'lift_wait_timeout_sec').value
        self.load_wait_timeout_sec = self.get_parameter(
            'load_wait_timeout_sec').value
        self.align_placeholder_wait_sec = self.get_parameter(
            'align_placeholder_wait_sec').value

        self.grid_data = self.build_grid(
            self.get_parameter('map_rows').value,
            self.get_parameter('map_columns').value,
            self.get_parameter('map_x').value,
            self.get_parameter('map_y').value,
            self.get_parameter('map_heights').value,
        )
        self.near_edge_offset = self.get_parameter('near_edge_offset').value
        self.kfs_align_edge_offset = self.get_parameter(
            'kfs_align_edge_offset').value
        self.kfs_pickup_edge_offset = self.get_parameter(
            'kfs_pickup_edge_offset').value
        self.target_far_edge_offset = self.get_parameter(
            'target_far_edge_offset').value
        self.source_far_edge_offset = self.get_parameter(
            'source_far_edge_offset').value
        self.down_lower_edge_offset = self.get_parameter(
            'down_lower_edge_offset').value
        self.lift_up_front = self.get_parameter('lift_up_front').value
        self.lift_up_rear = self.get_parameter('lift_up_rear').value
        self.lift_at_near_edge_front = self.get_parameter(
            'lift_at_near_edge_front').value
        self.lift_at_near_edge_rear = self.get_parameter(
            'lift_at_near_edge_rear').value
        self.lift_front_up_front = self.get_parameter(
            'lift_front_up_front').value
        self.lift_front_up_rear = self.get_parameter(
            'lift_front_up_rear').value
        self.lift_final_front = self.get_parameter(
            'lift_final_front').value
        self.lift_final_rear = self.get_parameter(
            'lift_final_rear').value

        self.move_client = self.create_client(
            MoveToPose,
            move_service,
            callback_group=self.callback_group,
        )
        self.lift_client = self.create_client(
            SetLift,
            lift_service,
            callback_group=self.callback_group,
        )
        self.load_client = self.create_client(
            LoadKfs,
            load_service,
            callback_group=self.callback_group,
        )
        self.stage_two_service = self.create_service(
            StageTwo,
            service_name,
            self.handle_stage_two,
            callback_group=self.callback_group,
        )

    @staticmethod
    def build_grid(rows, columns, xs, ys, heights):
        expected_cell_count = rows * columns
        if rows <= 0 or columns <= 0:
            raise ValueError('map_rows and map_columns must be positive')
        if any(len(values) != expected_cell_count
               for values in (xs, ys, heights)):
            raise ValueError(
                'map_x, map_y and map_heights must each contain '
                f'{expected_cell_count} values')

        grid_data = []
        for row in range(rows):
            grid_row = []
            for column in range(columns):
                index = row * columns + column
                x = float(xs[index])
                y = float(ys[index])
                height = float(heights[index])
                if math.isnan(x) and math.isnan(y) and math.isnan(height):
                    grid_row.append(None)
                    continue
                if not all(math.isfinite(value) for value in (x, y, height)):
                    raise ValueError(
                        f'Map cell ({row}, {column}) must be fully defined '
                        'or all NaN')
                grid_row.append((x, y, height))
            grid_data.append(grid_row)
        return grid_data

    @staticmethod
    def build_path(rows, columns):
        if len(rows) != len(columns):
            raise ValueError('rows and cols length mismatch')
        if len(rows) < 2:
            raise ValueError('path must contain at least 2 points')
        return [(int(row), int(column))
                for row, column in zip(rows, columns)]

    def get_cell_center(self, index):
        row, column = index
        if row < 0 or row >= len(self.grid_data):
            raise IndexError(f'Row {row} out of range')
        if column < 0 or column >= len(self.grid_data[row]):
            raise IndexError(f'Column {column} out of range')
        cell = self.grid_data[row][column]
        if cell is None:
            raise ValueError(f'Grid cell {index} is empty')
        return cell

    @staticmethod
    def get_direction(current_index, target_index):
        row_delta = target_index[0] - current_index[0]
        column_delta = target_index[1] - current_index[1]
        if row_delta == 1 and column_delta == 0:
            return 1.0, 0.0
        if row_delta == -1 and column_delta == 0:
            return -1.0, 0.0
        if row_delta == 0 and column_delta == 1:
            return 0.0, 1.0
        if row_delta == 0 and column_delta == -1:
            return 0.0, -1.0
        raise ValueError(
            f'Grid cells {current_index} and {target_index} '
            'are not 4-neighbors')

    def classify_path(self, path):
        step_directions = []
        for current_index, target_index in zip(path[:-1], path[1:]):
            current_center = self.get_cell_center(current_index)
            target_center = self.get_cell_center(target_index)
            direction_x, direction_y = self.get_direction(
                current_index, target_index)
            if target_center[2] > current_center[2]:
                step_type = 'up'
            elif target_center[2] < current_center[2]:
                step_type = 'down'
            else:
                step_type = 'flat'
            step_directions.append((
                current_center,
                target_center,
                direction_x,
                direction_y,
                step_type,
            ))
        return step_directions

    def wait_for_dependencies(self, steps):
        timeout = self.dependency_timeout_sec
        clients = [
            (self.move_client, 'MoveToPose'),
        ]
        step_types = {step[4] for step in steps}
        if step_types & {'up', 'down'}:
            clients.append((self.lift_client, 'SetLift'))
            clients.append((self.load_client, 'LoadKfs'))
        for client, name in clients:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f'{name} service unavailable')

    @staticmethod
    def wait_for_future(future, timeout_sec, description):
        done_event = threading.Event()
        future.add_done_callback(lambda _: done_event.set())
        if not done_event.wait(timeout_sec + 1.0):
            raise RuntimeError(f'{description} timed out waiting for response')

        response = future.result()
        if response is None:
            raise RuntimeError(f'{description} call failed')
        return response

    def move_to_pose(self, x, y, yaw):
        request = MoveToPose.Request()
        request.x = float(x)
        request.y = float(y)
        request.yaw = float(yaw)
        request.position_tolerance = 0.0
        request.yaw_tolerance = 0.0
        request.timeout_sec = 0.0
        response = self.wait_for_future(
            self.move_client.call_async(request),
            self.move_wait_timeout_sec,
            'MoveToPose',
        )
        if not response.success:
            raise RuntimeError(f'MoveToPose failed: {response.message}')

    def set_lift(self, front_lift, rear_lift):
        request = SetLift.Request()
        request.front_lift = float(front_lift)
        request.rear_lift = float(rear_lift)
        request.timeout_sec = 0.0
        response = self.wait_for_future(
            self.lift_client.call_async(request),
            self.lift_wait_timeout_sec,
            'SetLift',
        )
        if not response.success:
            raise RuntimeError(f'SetLift failed: {response.message}')

    def load_kfs(self):
        response = self.wait_for_future(
            self.load_client.call_async(LoadKfs.Request()),
            self.load_wait_timeout_sec,
            'LoadKfs',
        )
        if not response.success:
            raise RuntimeError(f'LoadKfs failed: {response.message}')

    def wait_for_kfs_alignment(self):
        time.sleep(self.align_placeholder_wait_sec)

    @staticmethod
    def edge_point(center_x, center_y, direction_x, direction_y, offset):
        return (
            center_x + direction_x * offset,
            center_y + direction_y * offset,
        )

    @staticmethod
    def opposite_edge_point(center_x, center_y, direction_x, direction_y,
                            offset):
        return (
            center_x - direction_x * offset,
            center_y - direction_y * offset,
        )

    def run_up_step(self, source_center, target_center, direction_x,
                    direction_y):
        source_x, source_y, _ = source_center
        target_x, target_y, _ = target_center
        direction_yaw = math.atan2(direction_y, direction_x)
        source_near_x, source_near_y = self.edge_point(
            source_x,
            source_y,
            direction_x,
            direction_y,
            self.near_edge_offset,
        )
        target_align_x, target_align_y = self.opposite_edge_point(
            target_x,
            target_y,
            direction_x,
            direction_y,
            self.kfs_align_edge_offset,
        )
        target_pickup_x, target_pickup_y = self.opposite_edge_point(
            target_x,
            target_y,
            direction_x,
            direction_y,
            self.kfs_pickup_edge_offset,
        )
        target_far_x, target_far_y = self.opposite_edge_point(
            target_x,
            target_y,
            direction_x,
            direction_y,
            self.target_far_edge_offset,
        )

        self.move_to_pose(source_x, source_y, direction_yaw)
        self.set_lift(self.lift_up_front, self.lift_up_rear)
        self.move_to_pose(source_near_x, source_near_y, direction_yaw)
        self.set_lift(
            self.lift_at_near_edge_front,
            self.lift_at_near_edge_rear,
        )
        self.move_to_pose(target_align_x, target_align_y, direction_yaw)
        self.wait_for_kfs_alignment()
        self.move_to_pose(target_pickup_x, target_pickup_y, direction_yaw)
        self.load_kfs()
        self.move_to_pose(target_far_x, target_far_y, direction_yaw)
        self.set_lift(
            self.lift_final_front,
            self.lift_final_rear,
        )
        self.move_to_pose(target_x, target_y, direction_yaw)

    def run_down_step(self, source_center, target_center, direction_x,
                      direction_y):
        source_x, source_y, _ = source_center
        target_x, target_y, _ = target_center
        direction_yaw = math.atan2(direction_y, direction_x)
        source_far_x, source_far_y = self.edge_point(
            source_x,
            source_y,
            direction_x,
            direction_y,
            self.source_far_edge_offset,
        )
        target_near_x, target_near_y = self.opposite_edge_point(
            target_x,
            target_y,
            direction_x,
            direction_y,
            self.near_edge_offset,
        )
        target_lower_x, target_lower_y = self.opposite_edge_point(
            target_x,
            target_y,
            direction_x,
            direction_y,
            self.down_lower_edge_offset,
        )

        self.move_to_pose(source_x, source_y, direction_yaw)
        self.move_to_pose(source_far_x, source_far_y, direction_yaw)
        self.set_lift(
            self.lift_front_up_front,
            self.lift_front_up_rear,
        )
        self.move_to_pose(target_near_x, target_near_y, direction_yaw)
        self.set_lift(self.lift_up_front, self.lift_up_rear)
        self.move_to_pose(target_lower_x, target_lower_y, direction_yaw)
        self.set_lift(
            self.lift_final_front,
            self.lift_final_rear,
        )
        self.load_kfs()
        self.move_to_pose(target_x, target_y, direction_yaw)

    def run_flat_step(self, source_center, target_center, direction_x,
                      direction_y):
        direction_yaw = math.atan2(direction_y, direction_x)
        self.move_to_pose(source_center[0], source_center[1], direction_yaw)
        self.move_to_pose(target_center[0], target_center[1], direction_yaw)

    def handle_stage_two(self, request, response):
        with self.service_lock:
            try:
                path = self.build_path(request.rows, request.cols)
                steps = self.classify_path(path)
                self.wait_for_dependencies(steps)
                for (source_center, target_center, direction_x, direction_y,
                     step_type) in steps:
                    if step_type == 'up':
                        self.run_up_step(
                            source_center,
                            target_center,
                            direction_x,
                            direction_y,
                        )
                    elif step_type == 'down':
                        self.run_down_step(
                            source_center,
                            target_center,
                            direction_x,
                            direction_y,
                        )
                    else:
                        self.run_flat_step(
                            source_center,
                            target_center,
                            direction_x,
                            direction_y,
                        )
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                return response

            response.success = True
            response.message = f'Stage two completed across {len(steps)} segment(s)'
            return response


def main():
    rclpy.init()
    node = StageTwoController()
    executor = MultiThreadedExecutor(num_threads=4)
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
