import math
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.srv import (
    GetKfsType,
    LoadKfs,
    MoveToPose,
    ReleaseKfs,
    SetLift,
    StageTwoPointOne,
)


class StageTwoPointOneController(Node):
    def __init__(self):
        super().__init__('stage_two_point_one')
        self.callback_group = ReentrantCallbackGroup()
        self.service_lock = threading.Lock()
        self.loaded_count = 0
        self.arrival_direction = None
        self.current_cell_center = None

        self.declare_parameter('service_name', '/r2/stage_two_point_one')
        self.declare_parameter('move_to_pose_service', '/r2/move_to_pose')
        self.declare_parameter('set_lift_service', '/r2/lift/set')
        self.declare_parameter(
            'get_kfs_type_service', '/r2/detection/get_type')
        self.declare_parameter('load_kfs_service', '/r2/kfs/load')
        self.declare_parameter('release_kfs_service', '/r2/kfs/release')
        self.declare_parameter('dependency_timeout_sec', 2.0)
        self.declare_parameter('move_timeout_sec', 35.0)
        self.declare_parameter('lift_timeout_sec', 15.0)
        self.declare_parameter('detection_timeout_sec', 10.0)
        self.declare_parameter('load_timeout_sec', 70.0)
        self.declare_parameter('release_timeout_sec', 70.0)

        self.declare_parameter(
            'cell_5_3_high_kfs_edge_pose', [3.2, -1.8, math.pi])
        self.declare_parameter(
            'cell_5_2_high_kfs_edge_pose', [3.2, -3.0, math.pi])
        self.declare_parameter(
            'cell_5_1_high_kfs_edge_pose', [3.2, -4.2, math.pi])
        self.declare_parameter('high_kfs_edge_offset', 0.2)
        self.declare_parameter('release_edge_offset', 0.2)
        self.declare_parameter('detection_sample_count', 10)
        self.declare_parameter('target_class_name', 'r2')
        self.declare_parameter('lift_up_front', 0.2)
        self.declare_parameter('lift_up_rear', 0.2)
        self.declare_parameter('lift_down_front', 0.0)
        self.declare_parameter('lift_down_rear', 0.0)

        service_name = self.get_parameter('service_name').value
        move_service = self.get_parameter('move_to_pose_service').value
        lift_service = self.get_parameter('set_lift_service').value
        detection_service = self.get_parameter(
            'get_kfs_type_service').value
        load_service = self.get_parameter('load_kfs_service').value
        release_service = self.get_parameter('release_kfs_service').value

        self.dependency_timeout_sec = self._positive_float_parameter(
            'dependency_timeout_sec')
        self.move_timeout_sec = self._positive_float_parameter(
            'move_timeout_sec')
        self.lift_timeout_sec = self._positive_float_parameter(
            'lift_timeout_sec')
        self.detection_timeout_sec = self._positive_float_parameter(
            'detection_timeout_sec')
        self.load_timeout_sec = self._positive_float_parameter(
            'load_timeout_sec')
        self.release_timeout_sec = self._positive_float_parameter(
            'release_timeout_sec')

        self.cell_5_3_edge_pose = self._pose_parameter(
            'cell_5_3_high_kfs_edge_pose')
        self.cell_5_2_edge_pose = self._pose_parameter(
            'cell_5_2_high_kfs_edge_pose')
        self.cell_5_1_edge_pose = self._pose_parameter(
            'cell_5_1_high_kfs_edge_pose')
        self.high_kfs_edge_offset = self._positive_float_parameter(
            'high_kfs_edge_offset')
        self.release_edge_offset = self._positive_float_parameter(
            'release_edge_offset')
        self.detection_sample_count = int(
            self.get_parameter('detection_sample_count').value)
        if self.detection_sample_count <= 0:
            raise ValueError('detection_sample_count must be positive')
        self.target_class_name = str(
            self.get_parameter('target_class_name').value)
        if not self.target_class_name:
            raise ValueError('target_class_name must not be empty')

        self.lift_up = self._lift_pair('lift_up')
        self.lift_down = self._lift_pair('lift_down')

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
        self.detection_client = self.create_client(
            GetKfsType,
            detection_service,
            callback_group=self.callback_group,
        )
        self.load_client = self.create_client(
            LoadKfs,
            load_service,
            callback_group=self.callback_group,
        )
        self.release_client = self.create_client(
            ReleaseKfs,
            release_service,
            callback_group=self.callback_group,
        )
        self.task_service = self.create_service(
            StageTwoPointOne,
            service_name,
            self.handle_task,
            callback_group=self.callback_group,
        )

    def _positive_float_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def _pose_parameter(self, name):
        values = self.get_parameter(name).value
        if len(values) != 3:
            raise ValueError(f'{name} must contain [x, y, yaw]')
        pose = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in pose):
            raise ValueError(f'{name} values must be finite')
        return pose

    def _lift_pair(self, prefix):
        front = float(self.get_parameter(f'{prefix}_front').value)
        rear = float(self.get_parameter(f'{prefix}_rear').value)
        if not math.isfinite(front) or not math.isfinite(rear):
            raise ValueError(f'{prefix} values must be finite')
        return front, rear

    def handle_task(self, request, response):
        with self.service_lock:
            self.loaded_count = int(request.loaded_count)
            self.arrival_direction = None
            self.current_cell_center = None
            try:
                self.validate_loaded_count(self.loaded_count)
                self.wait_for_dependencies()
                self.execute_task()
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                response.loaded_count = self.loaded_count
                return response

            response.success = True
            response.message = 'Stage 2.1 completed'
            response.loaded_count = self.loaded_count
            return response

    @staticmethod
    def validate_loaded_count(loaded_count):
        if not 0 <= loaded_count <= 3:
            raise ValueError(
                f'loaded_count must be between 0 and 3, got '
                f'{loaded_count}')

    def wait_for_dependencies(self):
        dependencies = (
            (self.move_client, 'MoveToPose'),
            (self.lift_client, 'SetLift'),
            (self.detection_client, 'GetKfsType'),
            (self.load_client, 'LoadKfs'),
            (self.release_client, 'ReleaseKfs'),
        )
        for client, name in dependencies:
            if not client.wait_for_service(
                timeout_sec=self.dependency_timeout_sec
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

    def move_to_pose(self, pose):
        request = MoveToPose.Request()
        request.x = pose[0]
        request.y = pose[1]
        request.yaw = pose[2]
        request.position_tolerance = 0.0
        request.yaw_tolerance = 0.0
        request.timeout_sec = self.move_timeout_sec
        response = self.wait_for_future(
            self.move_client.call_async(request),
            self.move_timeout_sec,
            'MoveToPose',
        )
        if not response.success:
            raise RuntimeError(f'MoveToPose failed: {response.message}')

    def set_lift(self, positions):
        request = SetLift.Request()
        request.front_lift = positions[0]
        request.rear_lift = positions[1]
        request.tolerance = 0.0
        request.timeout_sec = self.lift_timeout_sec
        response = self.wait_for_future(
            self.lift_client.call_async(request),
            self.lift_timeout_sec,
            'SetLift',
        )
        if not response.success:
            raise RuntimeError(f'SetLift failed: {response.message}')

    def detect_kfs_type(self):
        request = GetKfsType.Request()
        request.sample_count = self.detection_sample_count
        request.timeout_sec = self.detection_timeout_sec
        response = self.wait_for_future(
            self.detection_client.call_async(request),
            self.detection_timeout_sec,
            'GetKfsType',
        )
        if not response.success:
            raise RuntimeError(f'GetKfsType failed: {response.message}')
        return response.class_name

    def load_front_kfs(self, load_method):
        request = LoadKfs.Request()
        request.mode = LoadKfs.Request.FRONT
        request.load_method = load_method
        response = self.wait_for_future(
            self.load_client.call_async(request),
            self.load_timeout_sec,
            'LoadKfs',
        )
        if not response.success:
            raise RuntimeError(f'LoadKfs failed: {response.message}')
        self.loaded_count += 1

    def release_kfs(self):
        request = ReleaseKfs.Request()
        response = self.wait_for_future(
            self.release_client.call_async(request),
            self.release_timeout_sec,
            'ReleaseKfs',
        )
        if not response.success:
            raise RuntimeError(f'ReleaseKfs failed: {response.message}')
        self.loaded_count -= 1

    def cell_center_from_edge_pose(self, edge_pose):
        return (
            edge_pose[0] - (
                self.high_kfs_edge_offset * math.cos(edge_pose[2])),
            edge_pose[1] - (
                self.high_kfs_edge_offset * math.sin(edge_pose[2])),
        )

    def move_to_loading_edge(self, edge_pose):
        target_center = self.cell_center_from_edge_pose(edge_pose)
        if self.current_cell_center is not None:
            delta_x = target_center[0] - self.current_cell_center[0]
            delta_y = target_center[1] - self.current_cell_center[1]
            distance = math.hypot(delta_x, delta_y)
            if distance <= 0.0:
                raise ValueError('consecutive cell centers must be distinct')
            self.arrival_direction = (
                delta_x / distance,
                delta_y / distance,
            )

        self.move_to_pose(edge_pose)
        self.current_cell_center = target_center

    def release_at_arrival_edge(self, loading_edge_pose):
        if self.arrival_direction is None:
            raise RuntimeError(
                'arrival direction is unavailable for releasing KFS')

        came_from_x = -self.arrival_direction[0]
        came_from_y = -self.arrival_direction[1]
        release_pose = (
            self.current_cell_center[0] + (
                came_from_x * self.release_edge_offset),
            self.current_cell_center[1] + (
                came_from_y * self.release_edge_offset),
            math.atan2(came_from_y, came_from_x),
        )
        self.move_to_pose(release_pose)
        self.release_kfs()
        self.move_to_pose(loading_edge_pose)

    def detect_and_maybe_load(self, loading_edge_pose):
        class_name = self.detect_kfs_type()
        if class_name != self.target_class_name:
            return

        if self.loaded_count == 3:
            self.release_at_arrival_edge(loading_edge_pose)

        load_method = (
            LoadKfs.Request.TRANSFER
            if self.loaded_count == 2
            else LoadKfs.Request.STANDARD
        )
        self.load_front_kfs(load_method)

    def execute_task(self):
        self.move_to_loading_edge(self.cell_5_3_edge_pose)
        self.set_lift(self.lift_up)
        self.detect_and_maybe_load(self.cell_5_3_edge_pose)

        self.move_to_loading_edge(self.cell_5_1_edge_pose)
        self.detect_and_maybe_load(self.cell_5_1_edge_pose)

        self.set_lift(self.lift_down)
        self.move_to_loading_edge(self.cell_5_2_edge_pose)
        self.detect_and_maybe_load(self.cell_5_2_edge_pose)


def main():
    rclpy.init()
    node = StageTwoPointOneController()
    executor = MultiThreadedExecutor(num_threads=5)
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
