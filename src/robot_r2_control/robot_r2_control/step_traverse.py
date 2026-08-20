import math
import threading

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.srv import MoveRelative, SetLift, TraverseStep


STEP_TRAVERSE_SERVICE = '/r2/step_traverse'
MOVE_RELATIVE_SERVICE = '/r2/move_relative'
SET_LIFT_SERVICE = '/r2/lift/set'


class StepTraverseController(Node):
    # 支持运行时动态修改（ros2 param set）的参数：
    # 移动距离（非负）与抬升位置（有限）。
    DISTANCE_PARAMETER_NAMES = (
        'a1', 'a1_backoff',
        'a2', 'a2_backoff',
        'a3',
        'b1', 'b2', 'b3',
        'up_pre_lift_clearance',
    )
    LIFT_PAIR_NAMES = (
        'lift_all',
        'lift_front_only',
        'lift_rear_only',
        'lift_down',
    )
    LIFT_PARAMETER_NAMES = tuple(
        f'{prefix}_{side}'
        for prefix in LIFT_PAIR_NAMES
        for side in ('front', 'rear')
    )

    def __init__(self):
        super().__init__('step_traverse')
        self.callback_group = ReentrantCallbackGroup()
        self.service_lock = threading.Lock()
        self.config_lock = threading.Lock()

        self.declare_parameter('dependency_timeout_sec', 2.0)
        self.declare_parameter('move_timeout_sec', 35.0)
        self.declare_parameter('lift_timeout_sec', 15.0)

        self.declare_parameter('a1', 0.2)
        self.declare_parameter('a1_backoff', 0.015)
        self.declare_parameter('a2', 0.2)
        self.declare_parameter('a2_backoff', 0.015)
        self.declare_parameter('a3', 0.2)
        self.declare_parameter('up_pre_lift_clearance', 0.05)
        self.declare_parameter('b1', 0.2)
        self.declare_parameter('b2', 0.2)
        self.declare_parameter('b3', 0.2)

        self.declare_parameter('lift_all_front', 0.2)
        self.declare_parameter('lift_all_rear', 0.2)
        self.declare_parameter('lift_front_only_front', 0.2)
        # 不抬升侧保持 0.01 m 离地间隙，避免机构贴地。
        self.declare_parameter('lift_front_only_rear', 0.01)
        self.declare_parameter('lift_rear_only_front', 0.01)
        self.declare_parameter('lift_rear_only_rear', 0.2)
        self.declare_parameter('lift_down_front', 0.01)
        self.declare_parameter('lift_down_rear', 0.01)

        self.dependency_timeout_sec = self._positive_parameter(
            'dependency_timeout_sec')
        self.move_timeout_sec = self._positive_parameter(
            'move_timeout_sec')
        self.lift_timeout_sec = self._positive_parameter(
            'lift_timeout_sec')

        self.a1 = self._distance_parameter('a1')
        self.a1_backoff = self._distance_parameter('a1_backoff')
        self.a2 = self._distance_parameter('a2')
        self.a2_backoff = self._distance_parameter('a2_backoff')
        self.a3 = self._distance_parameter('a3')
        self.up_pre_lift_clearance = self._distance_parameter(
            'up_pre_lift_clearance')
        self.b1 = self._distance_parameter('b1')
        self.b2 = self._distance_parameter('b2')
        self.b3 = self._distance_parameter('b3')

        self.lift_all = self._lift_pair('lift_all')
        self.lift_front_only = self._lift_pair('lift_front_only')
        self.lift_rear_only = self._lift_pair('lift_rear_only')
        self.lift_down = self._lift_pair('lift_down')

        self.move_client = self.create_client(
            MoveRelative,
            MOVE_RELATIVE_SERVICE,
            callback_group=self.callback_group,
        )
        self.lift_client = self.create_client(
            SetLift,
            SET_LIFT_SERVICE,
            callback_group=self.callback_group,
        )
        self.traverse_service = self.create_service(
            TraverseStep,
            STEP_TRAVERSE_SERVICE,
            self.handle_traverse_step,
            callback_group=self.callback_group,
        )
        self.add_on_set_parameters_callback(self.on_parameters_changed)

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

    def on_parameters_changed(self, parameters):
        # 距离参数：有限且非负（移动距离，允许 0；向上预靠近可为负后退
        # 由 distance_to_step - clearance 在调用时产生，参数本身非负）。
        distance_updates = {}
        lift_updates = {}
        for parameter in parameters:
            if parameter.name in self.DISTANCE_PARAMETER_NAMES:
                value = parameter.value
                if (
                    isinstance(value, bool) or
                    not isinstance(value, (int, float))
                ):
                    return SetParametersResult(
                        successful=False,
                        reason=f'{parameter.name} must be numeric',
                    )
                distance = float(value)
                if not math.isfinite(distance) or distance < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            f'{parameter.name} must be finite and '
                            'non-negative'
                        ),
                    )
                distance_updates[parameter.name] = distance
            elif parameter.name in self.LIFT_PARAMETER_NAMES:
                value = parameter.value
                if (
                    isinstance(value, bool) or
                    not isinstance(value, (int, float))
                ):
                    return SetParametersResult(
                        successful=False,
                        reason=f'{parameter.name} must be numeric',
                    )
                lift_value = float(value)
                if not math.isfinite(lift_value):
                    return SetParametersResult(
                        successful=False,
                        reason=f'{parameter.name} must be finite',
                    )
                lift_updates[parameter.name] = lift_value

        if not distance_updates and not lift_updates:
            return SetParametersResult(successful=True)

        # 全部校验通过后一次性原子更新；运行中的台阶跨越使用锁内快照，
        # 不会观察到部分更新。
        with self.config_lock:
            for name, value in distance_updates.items():
                setattr(self, name, value)
            for prefix in self.LIFT_PAIR_NAMES:
                current = getattr(self, prefix)
                front = lift_updates.get(f'{prefix}_front', current[0])
                rear = lift_updates.get(f'{prefix}_rear', current[1])
                setattr(self, prefix, (front, rear))
        return SetParametersResult(successful=True)

    def handle_traverse_step(self, request, response):
        with self.service_lock:
            try:
                distance_to_step = self.validate_request(request)
                self.wait_for_dependencies()

                if request.direction == TraverseStep.Request.UP:
                    self.run_up_step(distance_to_step)
                    direction_name = 'up'
                else:
                    self.run_down_step(distance_to_step)
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
            raise RuntimeError('MoveRelative service unavailable')
        if not self.lift_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('SetLift service unavailable')

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

    def move_relative(self, forward):
        request = MoveRelative.Request()
        request.pose_source = MoveRelative.Request.SERIAL
        request.forward = float(forward)
        request.left = 0.0
        request.yaw_delta = 0.0
        request.position_tolerance = 0.0
        request.yaw_tolerance = 0.0
        request.timeout_sec = self.move_timeout_sec
        response = self.wait_for_future(
            self.move_client.call_async(request),
            self.move_timeout_sec,
            'MoveRelative',
        )
        if not response.success:
            raise RuntimeError(f'MoveRelative failed: {response.message}')

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

    def run_up_step(self, distance_to_step):
        with self.config_lock:
            a1 = self.a1
            a1_backoff = self.a1_backoff
            a2 = self.a2
            a2_backoff = self.a2_backoff
            a3 = self.a3
            pre_lift_clearance = self.up_pre_lift_clearance
            lift_all = self.lift_all
            lift_rear_only = self.lift_rear_only
            lift_down = self.lift_down

        # 预靠近到台阶前 clearance 处；若车头已经太近则相对后退。
        self.move_relative(distance_to_step - pre_lift_clearance)
        self.set_lift(lift_all)

        self.move_relative(a1)
        self.move_relative(-a1_backoff)
        self.set_lift(lift_rear_only)

        self.move_relative(a2)
        self.move_relative(-a2_backoff)
        self.set_lift(lift_down)

        self.move_relative(a3)

    def run_down_step(self, distance_to_step):
        with self.config_lock:
            b1 = self.b1
            b2 = self.b2
            b3 = self.b3
            lift_front_only = self.lift_front_only
            lift_all = self.lift_all
            lift_down = self.lift_down

        self.move_relative(distance_to_step + b1)

        self.set_lift(lift_front_only)
        self.move_relative(b2)

        self.set_lift(lift_all)
        self.move_relative(b3)
        self.set_lift(lift_down)


def main():
    rclpy.init()
    node = StepTraverseController()
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
