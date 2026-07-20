import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.srv import MoveToPose, SetLift, TraverseStep


class StepTraverseController(Node):
    def __init__(self):
        super().__init__('step_traverse')
        self.callback_group = ReentrantCallbackGroup()
        self.service_lock = threading.Lock()
        self.pose_condition = threading.Condition()
        self.current_pose = None

        self.declare_parameter('service_name', '/r2/step_traverse')
        self.declare_parameter('current_pose_topic', '/r2/pose_feedback')
        self.declare_parameter('move_to_pose_service', '/r2/move_to_pose')
        self.declare_parameter('set_lift_service', '/r2/lift/set')
        self.declare_parameter('dependency_timeout_sec', 2.0)
        self.declare_parameter('pose_wait_timeout_sec', 2.0)
        self.declare_parameter('move_timeout_sec', 35.0)
        self.declare_parameter('lift_timeout_sec', 15.0)

        self.declare_parameter('a1', 0.2)
        self.declare_parameter('a2', 0.2)
        self.declare_parameter('a3', 0.2)
        self.declare_parameter('b1', 0.2)
        self.declare_parameter('b2', 0.2)
        self.declare_parameter('b3', 0.2)

        self.declare_parameter('lift_all_front', 0.2)
        self.declare_parameter('lift_all_rear', 0.2)
        self.declare_parameter('lift_front_only_front', 0.2)
        self.declare_parameter('lift_front_only_rear', 0.0)
        self.declare_parameter('lift_rear_only_front', 0.0)
        self.declare_parameter('lift_rear_only_rear', 0.2)
        self.declare_parameter('lift_down_front', 0.0)
        self.declare_parameter('lift_down_rear', 0.0)

        service_name = self.get_parameter('service_name').value
        current_pose_topic = self.get_parameter('current_pose_topic').value
        move_service = self.get_parameter('move_to_pose_service').value
        lift_service = self.get_parameter('set_lift_service').value

        self.dependency_timeout_sec = self._positive_parameter(
            'dependency_timeout_sec')
        self.pose_wait_timeout_sec = self._positive_parameter(
            'pose_wait_timeout_sec')
        self.move_timeout_sec = self._positive_parameter(
            'move_timeout_sec')
        self.lift_timeout_sec = self._positive_parameter(
            'lift_timeout_sec')

        self.a1 = self._distance_parameter('a1')
        self.a2 = self._distance_parameter('a2')
        self.a3 = self._distance_parameter('a3')
        self.b1 = self._distance_parameter('b1')
        self.b2 = self._distance_parameter('b2')
        self.b3 = self._distance_parameter('b3')

        self.lift_all = self._lift_pair('lift_all')
        self.lift_front_only = self._lift_pair('lift_front_only')
        self.lift_rear_only = self._lift_pair('lift_rear_only')
        self.lift_down = self._lift_pair('lift_down')

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            current_pose_topic,
            self.on_pose_feedback,
            10,
            callback_group=self.callback_group,
        )
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
        self.traverse_service = self.create_service(
            TraverseStep,
            service_name,
            self.handle_traverse_step,
            callback_group=self.callback_group,
        )

    def _positive_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def _distance_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f'{name} must be finite and non-negative')
        return value

    def _lift_pair(self, prefix):
        front = float(self.get_parameter(f'{prefix}_front').value)
        rear = float(self.get_parameter(f'{prefix}_rear').value)
        if not math.isfinite(front) or not math.isfinite(rear):
            raise ValueError(f'{prefix} lift values must be finite')
        return front, rear

    def on_pose_feedback(self, msg):
        pose = msg.pose
        current = (
            float(pose.position.x),
            float(pose.position.y),
            self.yaw_from_quaternion(pose.orientation),
        )
        if not all(math.isfinite(value) for value in current):
            return
        with self.pose_condition:
            self.current_pose = current
            self.pose_condition.notify_all()

    def handle_traverse_step(self, request, response):
        with self.service_lock:
            try:
                distance_to_step = self.validate_request(request)
                self.wait_for_dependencies()
                start_pose = self.wait_for_pose()

                if request.direction == TraverseStep.Request.UP:
                    self.run_up_step(start_pose, distance_to_step)
                    direction_name = 'up'
                else:
                    self.run_down_step(start_pose, distance_to_step)
                    direction_name = 'down'
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                return response

            response.success = True
            response.message = f'{direction_name} step traversal completed'
            return response

    @staticmethod
    def validate_request(request):
        if request.direction not in (
            TraverseStep.Request.UP,
            TraverseStep.Request.DOWN,
        ):
            raise ValueError(
                f'unsupported step direction: {request.direction}')

        distance = float(request.distance_to_step)
        if not math.isfinite(distance):
            raise ValueError('distance_to_step must be finite')
        if request.direction == TraverseStep.Request.UP and distance < 0.0:
            raise ValueError(
                'distance_to_step must be non-negative for an up step')
        return distance

    def wait_for_dependencies(self):
        timeout = self.dependency_timeout_sec
        if not self.move_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('MoveToPose service unavailable')
        if not self.lift_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('SetLift service unavailable')

    def wait_for_pose(self):
        deadline = time.monotonic() + self.pose_wait_timeout_sec
        with self.pose_condition:
            while self.current_pose is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RuntimeError('Pose feedback unavailable')
                self.pose_condition.wait(timeout=remaining)
            return self.current_pose

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

    def move_from_start(self, start_pose, cumulative_distance):
        start_x, start_y, initial_yaw = start_pose
        target_x = start_x + cumulative_distance * math.cos(initial_yaw)
        target_y = start_y + cumulative_distance * math.sin(initial_yaw)

        request = MoveToPose.Request()
        request.x = target_x
        request.y = target_y
        request.yaw = initial_yaw
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

    def set_lift(self, lift_positions):
        request = SetLift.Request()
        request.front_lift = lift_positions[0]
        request.rear_lift = lift_positions[1]
        request.tolerance = 0.0
        request.timeout_sec = self.lift_timeout_sec
        response = self.wait_for_future(
            self.lift_client.call_async(request),
            self.lift_timeout_sec,
            'SetLift',
        )
        if not response.success:
            raise RuntimeError(f'SetLift failed: {response.message}')

    def run_up_step(self, start_pose, distance_to_step):
        self.set_lift(self.lift_all)

        cumulative_distance = distance_to_step + self.a1
        self.move_from_start(start_pose, cumulative_distance)

        self.set_lift(self.lift_rear_only)
        cumulative_distance += self.a2
        self.move_from_start(start_pose, cumulative_distance)

        self.set_lift(self.lift_down)
        cumulative_distance += self.a3
        self.move_from_start(start_pose, cumulative_distance)

    def run_down_step(self, start_pose, distance_to_step):
        cumulative_distance = distance_to_step + self.b1
        self.move_from_start(start_pose, cumulative_distance)

        self.set_lift(self.lift_front_only)
        cumulative_distance += self.b2
        self.move_from_start(start_pose, cumulative_distance)

        self.set_lift(self.lift_all)
        cumulative_distance += self.b3
        self.move_from_start(start_pose, cumulative_distance)
        self.set_lift(self.lift_down)

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
    node = StepTraverseController()
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
