import math
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.srv import (
    LoadKfs,
    SetGripperGrip,
    SetGripperRotate,
    SetGripperTipRotate,
)


class KfsLoaderController(Node):
    def __init__(self):
        super().__init__('kfs_loader_control')
        self.callback_group = ReentrantCallbackGroup()
        self.operation_lock = threading.Lock()

        self.declare_parameter('service_name', '/r2/kfs/load')
        self.declare_parameter('grip_service', '/r2/gripper/set_grip')
        self.declare_parameter(
            'root_rotate_service', '/r2/gripper/set_rotate')
        self.declare_parameter(
            'tip_rotate_service', '/r2/gripper/set_tip_rotate')
        self.declare_parameter('service_timeout_sec', 10.0)
        self.declare_parameter('kfs_grip_position', 0.064)
        self.declare_parameter('grip_open_position', 0.0)
        self.declare_parameter(
            'front_pickup_root_position', -math.pi / 6.0)
        self.declare_parameter(
            'front_pickup_tip_position', math.pi / 6.0)
        self.declare_parameter(
            'top_pickup_root_position', 0.1687239216732498)
        self.declare_parameter(
            'top_pickup_tip_position', 1.4020724051216468)
        self.declare_parameter('tip_rear_limit_position', -math.pi / 2.0)
        self.declare_parameter('root_initial_position', -math.pi / 2.0)
        self.declare_parameter('max_loaded_kfs', 2)
        self.declare_parameter('initial_loaded_kfs', 0)

        service_name = self.get_parameter('service_name').value
        grip_service = self.get_parameter('grip_service').value
        root_rotate_service = self.get_parameter(
            'root_rotate_service').value
        tip_rotate_service = self.get_parameter('tip_rotate_service').value
        self.service_timeout_sec = self.get_parameter(
            'service_timeout_sec').value
        self.kfs_grip_position = self.get_parameter(
            'kfs_grip_position').value
        self.grip_open_position = self.get_parameter(
            'grip_open_position').value
        self.front_pickup_root_position = self.get_parameter(
            'front_pickup_root_position').value
        self.front_pickup_tip_position = self.get_parameter(
            'front_pickup_tip_position').value
        self.top_pickup_root_position = self.get_parameter(
            'top_pickup_root_position').value
        self.top_pickup_tip_position = self.get_parameter(
            'top_pickup_tip_position').value
        self.tip_rear_limit_position = self.get_parameter(
            'tip_rear_limit_position').value
        self.root_initial_position = self.get_parameter(
            'root_initial_position').value
        self.max_loaded_kfs = int(self.get_parameter('max_loaded_kfs').value)
        self.loaded_kfs = int(self.get_parameter('initial_loaded_kfs').value)

        if self.max_loaded_kfs < 1:
            raise ValueError('max_loaded_kfs must be positive')
        if self.loaded_kfs < 0 or self.loaded_kfs > self.max_loaded_kfs:
            raise ValueError(
                'initial_loaded_kfs must be within the loaded KFS limit')

        self.grip_client = self.create_client(
            SetGripperGrip,
            grip_service,
            callback_group=self.callback_group,
        )
        self.root_rotate_client = self.create_client(
            SetGripperRotate,
            root_rotate_service,
            callback_group=self.callback_group,
        )
        self.tip_rotate_client = self.create_client(
            SetGripperTipRotate,
            tip_rotate_service,
            callback_group=self.callback_group,
        )
        self.load_service = self.create_service(
            LoadKfs,
            service_name,
            self.handle_load_kfs,
            callback_group=self.callback_group,
        )

    def wait_for_dependencies(self):
        timeout = self.service_timeout_sec
        if not self.grip_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('gripper grip service unavailable')
        if not self.root_rotate_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('gripper root rotate service unavailable')
        if not self.tip_rotate_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('gripper tip rotate service unavailable')

    @staticmethod
    def wait_for_future(future, timeout_sec, description):
        done_event = threading.Event()
        future.add_done_callback(lambda _: done_event.set())
        if not done_event.wait(timeout_sec + 1.0):
            raise RuntimeError(f'{description} timed out')

        response = future.result()
        if response is None:
            raise RuntimeError(f'{description} call failed')
        return response

    def set_grip(self, position):
        request = SetGripperGrip.Request()
        request.position = float(position)
        request.tolerance = 0.0
        request.timeout_sec = float(self.service_timeout_sec)
        response = self.wait_for_future(
            self.grip_client.call_async(request),
            self.service_timeout_sec,
            'SetGripperGrip',
        )
        if not response.success:
            raise RuntimeError(f'SetGripperGrip failed: {response.message}')

    def make_rotation_request(self, service_type, position):
        request = service_type.Request()
        request.position = float(position)
        request.tolerance = 0.0
        request.timeout_sec = float(self.service_timeout_sec)
        return request

    def set_root_rotate(self, position):
        response = self.wait_for_future(
            self.root_rotate_client.call_async(self.make_rotation_request(
                SetGripperRotate, position)),
            self.service_timeout_sec,
            'SetGripperRotate',
        )
        if not response.success:
            raise RuntimeError(f'SetGripperRotate failed: {response.message}')

    def set_tip_rotate(self, position):
        response = self.wait_for_future(
            self.tip_rotate_client.call_async(self.make_rotation_request(
                SetGripperTipRotate, position)),
            self.service_timeout_sec,
            'SetGripperTipRotate',
        )
        if not response.success:
            raise RuntimeError(
                f'SetGripperTipRotate failed: {response.message}')

    def set_pickup_pose(self, root_position, tip_position):
        # Send both requests before waiting so both axes move concurrently.
        root_future = self.root_rotate_client.call_async(
            self.make_rotation_request(
                SetGripperRotate, root_position))
        tip_future = self.tip_rotate_client.call_async(
            self.make_rotation_request(
                SetGripperTipRotate, tip_position))

        root_response = self.wait_for_future(
            root_future,
            self.service_timeout_sec,
            'SetGripperRotate',
        )
        tip_response = self.wait_for_future(
            tip_future,
            self.service_timeout_sec,
            'SetGripperTipRotate',
        )

        failures = []
        if not root_response.success:
            failures.append(f'SetGripperRotate: {root_response.message}')
        if not tip_response.success:
            failures.append(f'SetGripperTipRotate: {tip_response.message}')
        if failures:
            raise RuntimeError('; '.join(failures))

    def execute_load_sequence(self, pickup_root_position,
                              pickup_tip_position, root_returns_first):
        self.set_pickup_pose(
            pickup_root_position,
            pickup_tip_position,
        )
        self.set_grip(self.kfs_grip_position)
        if root_returns_first:
            self.set_root_rotate(self.root_initial_position)
            self.set_tip_rotate(self.tip_rear_limit_position)
        else:
            self.set_tip_rotate(self.tip_rear_limit_position)
            self.set_root_rotate(self.root_initial_position)
        self.set_grip(self.grip_open_position)

    def handle_load_kfs(self, request, response):
        with self.operation_lock:
            try:
                if request.mode == LoadKfs.Request.FRONT:
                    pickup_root_position = self.front_pickup_root_position
                    pickup_tip_position = self.front_pickup_tip_position
                    root_returns_first = False
                    mode_name = 'front'
                elif request.mode == LoadKfs.Request.TOP:
                    pickup_root_position = self.top_pickup_root_position
                    pickup_tip_position = self.top_pickup_tip_position
                    root_returns_first = True
                    mode_name = 'top'
                else:
                    raise ValueError(
                        f'unsupported KFS load mode: {request.mode}')

                self.wait_for_dependencies()
                self.execute_load_sequence(
                    pickup_root_position,
                    pickup_tip_position,
                    root_returns_first,
                )
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                response.loaded_count = self.loaded_kfs
                return response

            if self.loaded_kfs < self.max_loaded_kfs:
                self.loaded_kfs += 1
            response.success = True
            response.message = f'{mode_name} KFS load sequence completed'
            response.loaded_count = self.loaded_kfs
            return response


def main():
    rclpy.init()
    node = KfsLoaderController()
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
