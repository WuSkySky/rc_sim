import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.srv import LoadKfs, SetGripperGrip, SetGripperLift
from robot_r2_interfaces.srv import SetGripperRotate


class KfsLoaderController(Node):
    def __init__(self):
        super().__init__('kfs_loader_controller')
        self.callback_group = ReentrantCallbackGroup()
        self.operation_lock = threading.Lock()

        self.declare_parameter('service_name', '/r2/kfs/load')
        self.declare_parameter('grip_service', '/r2/gripper/set_grip')
        self.declare_parameter('lift_service', '/r2/gripper/set_lift')
        self.declare_parameter('rotate_service', '/r2/gripper/set_rotate')
        self.declare_parameter('service_timeout_sec', 10.0)
        self.declare_parameter('kfs_grip_position', 0.064)
        self.declare_parameter('grip_open_position', 0.0)
        self.declare_parameter('rotate_forward_position', -math.pi)
        self.declare_parameter('rotate_home_position', 0.0)
        self.declare_parameter('lift_base_position', -0.180)
        self.declare_parameter('lift_stack_increment', 0.35)
        self.declare_parameter('max_loaded_kfs', 2)
        self.declare_parameter('initial_loaded_kfs', 0)

        service_name = self.get_parameter('service_name').value
        grip_service = self.get_parameter('grip_service').value
        lift_service = self.get_parameter('lift_service').value
        rotate_service = self.get_parameter('rotate_service').value
        self.service_timeout_sec = self.get_parameter(
            'service_timeout_sec').value
        self.kfs_grip_position = self.get_parameter(
            'kfs_grip_position').value
        self.grip_open_position = self.get_parameter(
            'grip_open_position').value
        self.rotate_forward_position = self.get_parameter(
            'rotate_forward_position').value
        self.rotate_home_position = self.get_parameter(
            'rotate_home_position').value
        self.lift_base_position = self.get_parameter(
            'lift_base_position').value
        self.lift_stack_increment = self.get_parameter(
            'lift_stack_increment').value
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
        self.lift_client = self.create_client(
            SetGripperLift,
            lift_service,
            callback_group=self.callback_group,
        )
        self.rotate_client = self.create_client(
            SetGripperRotate,
            rotate_service,
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
        if not self.lift_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('gripper lift service unavailable')
        if not self.rotate_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('gripper rotate service unavailable')

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

    def set_lift(self, position):
        request = SetGripperLift.Request()
        request.position = float(position)
        request.tolerance = 0.0
        request.timeout_sec = float(self.service_timeout_sec)
        response = self.wait_for_future(
            self.lift_client.call_async(request),
            self.service_timeout_sec,
            'SetGripperLift',
        )
        if not response.success:
            raise RuntimeError(f'SetGripperLift failed: {response.message}')

    def set_rotate(self, position):
        request = SetGripperRotate.Request()
        request.position = float(position)
        request.tolerance = 0.0
        request.timeout_sec = float(self.service_timeout_sec)
        response = self.wait_for_future(
            self.rotate_client.call_async(request),
            self.service_timeout_sec,
            'SetGripperRotate',
        )
        if not response.success:
            raise RuntimeError(f'SetGripperRotate failed: {response.message}')

    def handle_load_kfs(self, request, response):
        del request
        with self.operation_lock:
            if self.loaded_kfs >= self.max_loaded_kfs:
                response.success = False
                response.message = 'KFS storage is full'
                response.loaded_count = self.loaded_kfs
                return response

            target_lift = (
                self.lift_base_position +
                self.loaded_kfs * self.lift_stack_increment
            )

            try:
                self.wait_for_dependencies()
                self.set_grip(self.kfs_grip_position)
                self.set_lift(target_lift)
                self.set_rotate(self.rotate_forward_position)
                time.sleep(0.3)
                self.set_grip(self.grip_open_position)
                self.set_rotate(self.rotate_home_position)
                self.set_lift(self.lift_base_position)
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                response.loaded_count = self.loaded_kfs
                return response

            self.loaded_kfs += 1
            response.success = True
            response.message = 'KFS loaded successfully'
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
