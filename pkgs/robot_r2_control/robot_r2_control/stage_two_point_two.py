import math
import threading
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.srv import (
    GetKfsType,
    LoadKfs,
    MoveToPose,
    ReleaseKfs,
    StageTwoPointTwo,
    TraverseStep,
)


class StageTwoPointTwoController(Node):
    FORWARD = (-1, 0)
    LEFT = (0, -1)
    RIGHT = (0, 1)
    POINT_ONE_COVERED_ENTRY = (4, 2)

    def __init__(self):
        super().__init__('stage_two_point_two')
        self.callback_group = ReentrantCallbackGroup()
        self.service_lock = threading.Lock()
        self.pose_condition = threading.Condition()
        self.current_pose = None
        self.loaded_count = 0
        self.arrival_direction = None

        self.declare_parameter('service_name', '/r2/stage_two_point_two')
        self.declare_parameter('current_pose_topic', '/r2/pose_feedback')
        self.declare_parameter('move_to_pose_service', '/r2/move_to_pose')
        self.declare_parameter(
            'traverse_step_service', '/r2/step_traverse')
        self.declare_parameter(
            'get_kfs_type_service', '/r2/detection/get_type')
        self.declare_parameter('load_kfs_service', '/r2/kfs/load')
        self.declare_parameter('release_kfs_service', '/r2/kfs/release')

        self.declare_parameter('dependency_timeout_sec', 2.0)
        self.declare_parameter('pose_timeout_sec', 2.0)
        self.declare_parameter('move_timeout_sec', 35.0)
        self.declare_parameter('traverse_timeout_sec', 150.0)
        self.declare_parameter('detection_timeout_sec', 10.0)
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
        self.declare_parameter('target_class_name', 'r2')

        service_name = self.get_parameter('service_name').value
        pose_topic = self.get_parameter('current_pose_topic').value
        move_service = self.get_parameter('move_to_pose_service').value
        traverse_service = self.get_parameter(
            'traverse_step_service').value
        detection_service = self.get_parameter(
            'get_kfs_type_service').value
        load_service = self.get_parameter('load_kfs_service').value
        release_service = self.get_parameter('release_kfs_service').value

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
        self.target_class_name = str(
            self.get_parameter('target_class_name').value)
        if not self.target_class_name:
            raise ValueError('target_class_name must not be empty')

        self.get_cell(self.initial_index)
        if not 0 <= self.terminal_forward_index < len(self.forward_x):
            raise ValueError('terminal_forward_index is out of range')

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            pose_topic,
            self.on_pose_feedback,
            10,
            callback_group=self.callback_group,
        )
        self.move_client = self.create_client(
            MoveToPose,
            move_service,
            callback_group=self.callback_group,
        )
        self.traverse_client = self.create_client(
            TraverseStep,
            traverse_service,
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
            StageTwoPointTwo,
            service_name,
            self.handle_task,
            callback_group=self.callback_group,
        )

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

    def get_cell(self, index):
        forward_index, lateral_index = index
        if not 0 <= forward_index < len(self.forward_x):
            raise ValueError(f'forward index {forward_index} is invalid')

        lateral_offset = lateral_index - 1
        if not 0 <= lateral_offset < len(self.lateral_y):
            raise ValueError(f'lateral index {lateral_index} is invalid')

        height_index = (
            forward_index * len(self.lateral_y) + lateral_offset)
        height = self.cell_heights[height_index]
        if not math.isfinite(height):
            raise ValueError(f'cell {index} is not traversable')
        return (
            self.forward_x[forward_index],
            self.lateral_y[lateral_offset],
            height,
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
        deadline = time.monotonic() + self.pose_timeout_sec
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
            self.arrival_direction = None
            try:
                self.validate_decision(request.fake_kfs_decision)
                self.validate_loaded_count(self.loaded_count)
                self.wait_for_dependencies()
                final_index = self.execute_task(
                    request.fake_kfs_decision)
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                response.loaded_count = self.loaded_count
                return response

            response.success = True
            response.message = f'Stage 2.2 completed at {final_index}'
            response.loaded_count = self.loaded_count
            return response

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

    def wait_for_dependencies(self):
        dependencies = (
            (self.move_client, 'MoveToPose'),
            (self.traverse_client, 'TraverseStep'),
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

    def move_to_pose(self, x, y, yaw):
        request = MoveToPose.Request()
        request.x = float(x)
        request.y = float(y)
        request.yaw = float(yaw)
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

    def load_kfs(self, mode, load_method):
        request = LoadKfs.Request()
        request.mode = mode
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

    def traverse_step(self, is_up, distance_to_step):
        request = TraverseStep.Request()
        request.direction = (
            TraverseStep.Request.UP
            if is_up
            else TraverseStep.Request.DOWN
        )
        request.distance_to_step = float(distance_to_step)
        response = self.wait_for_future(
            self.traverse_client.call_async(request),
            self.traverse_timeout_sec,
            'TraverseStep',
        )
        if not response.success:
            raise RuntimeError(f'TraverseStep failed: {response.message}')

    @staticmethod
    def add_index(index, delta):
        return index[0] + delta[0], index[1] + delta[1]

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
        front_x = actual_x + (
            self.chassis_front_offset * math.cos(actual_yaw))
        front_y = actual_y + (
            self.chassis_front_offset * math.sin(actual_yaw))
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

    def pickup_kfs(self, current_index, target_index):
        current = self.get_cell(current_index)
        target = self.get_cell(target_index)
        direction_x, direction_y = self.cell_direction(
            current_index, target_index)
        direction_yaw = math.atan2(direction_y, direction_x)

        if target[2] > current[2]:
            offset = self.higher_kfs_edge_offset
            load_mode = LoadKfs.Request.FRONT
            load_mode_name = 'front'
        elif target[2] < current[2]:
            offset = self.lower_kfs_edge_offset
            load_mode = LoadKfs.Request.TOP
            load_mode_name = 'top'
        else:
            raise ValueError(
                f'KFS at {target_index} has the same height as '
                f'{current_index}')

        edge_x = current[0] + direction_x * offset
        edge_y = current[1] + direction_y * offset
        self.get_logger().info(
            f'Picking KFS at {target_index} with {load_mode_name} load')
        self.move_to_pose(edge_x, edge_y, direction_yaw)

        if self.loaded_count == 3:
            if self.arrival_direction is None:
                raise RuntimeError(
                    'arrival direction is unavailable for releasing KFS')
            came_from_x = -self.arrival_direction[0]
            came_from_y = -self.arrival_direction[1]
            release_x = current[0] + (
                came_from_x * self.release_edge_offset)
            release_y = current[1] + (
                came_from_y * self.release_edge_offset)
            release_yaw = math.atan2(came_from_y, came_from_x)
            self.move_to_pose(release_x, release_y, release_yaw)
            self.release_kfs()
            self.move_to_pose(edge_x, edge_y, direction_yaw)

        load_method = (
            LoadKfs.Request.TRANSFER
            if self.loaded_count == 2
            else LoadKfs.Request.STANDARD
        )
        self.load_kfs(load_mode, load_method)
        self.move_to_pose(current[0], current[1], direction_yaw)

    def detect_direction(self, current_index, delta):
        current = self.get_cell(current_index)
        target_index = self.add_index(current_index, delta)
        target_forward_index, target_lateral_index = target_index
        self.get_cell(target_index)
        cached_result = self.cell_detection_results[
            target_forward_index
        ][target_lateral_index - 1]
        if cached_result is not None:
            displayed_class = (
                cached_result if cached_result else '<empty>')
            self.get_logger().info(
                f'Cached detection {current_index}->{target_index}: '
                f'{displayed_class}')
            return cached_result

        direction_x, direction_y = self.cell_direction(
            current_index, target_index)
        direction_yaw = math.atan2(direction_y, direction_x)
        self.move_to_pose(current[0], current[1], direction_yaw)

        class_name = self.detect_kfs_type()
        displayed_class = class_name if class_name else '<empty>'
        self.get_logger().info(
            f'Detection {current_index}->{target_index}: '
            f'{displayed_class}')

        if class_name == self.target_class_name:
            self.pickup_kfs(current_index, target_index)

        self.cell_detection_results[target_forward_index][
            target_lateral_index - 1
        ] = class_name
        return class_name

    def scan_current_cell(self, current_index, arrival_delta):
        scan_results = {}

        for delta in self.scan_deltas(current_index, arrival_delta):
            if self.loaded_count == 3 and delta != self.FORWARD:
                continue
            scan_results[delta] = self.detect_direction(
                current_index, delta)

        return scan_results

    def selected_lateral_delta(self, decision, current_index):
        lateral_index = current_index[1]
        if lateral_index == 1:
            return self.RIGHT
        if lateral_index == 3:
            return self.LEFT
        if decision == StageTwoPointTwo.Request.LEFT:
            return self.LEFT
        return self.RIGHT

    def execute_task(self, decision):
        current_index = self.initial_index
        next_delta = self.FORWARD

        while True:
            target_index = self.add_index(current_index, next_delta)
            self.move_one_cell(current_index, target_index)
            current_index = target_index

            if current_index[0] == self.terminal_forward_index:
                self.get_logger().info(
                    f'Reached terminal cell {current_index}')
                return current_index

            scan_results = self.scan_current_cell(
                current_index, next_delta)
            front_result = scan_results.get(self.FORWARD)

            force_lateral = current_index == (1, 2)
            front_is_blocked = (
                front_result is not None and
                front_result != '' and
                front_result != self.target_class_name
            )
            if force_lateral or front_is_blocked:
                next_delta = self.selected_lateral_delta(
                    decision, current_index)
                if (
                    self.loaded_count == 3 and
                    next_delta not in scan_results
                ):
                    self.detect_direction(current_index, next_delta)
                reason = '(1, 2)' if force_lateral else 'front blocked'
                self.get_logger().info(
                    f'Next move is lateral due to {reason}')
            else:
                next_delta = self.FORWARD

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
