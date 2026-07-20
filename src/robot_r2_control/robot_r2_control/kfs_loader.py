import math
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.srv import (
    LoadKfs,
    ReleaseKfs,
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
        self.declare_parameter(
            'release_service_name', '/r2/kfs/release')
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
        self.declare_parameter('tip_front_limit_position', math.pi / 2.0)
        self.declare_parameter('tip_rear_limit_position', -math.pi / 2.0)
        self.declare_parameter('root_initial_position', -math.pi / 2.0)
        self.declare_parameter('release_root_position', -math.pi / 4.0)
        self.declare_parameter('release_tip_position', math.pi / 4.0)

        service_name = self.get_parameter('service_name').value
        release_service_name = self.get_parameter(
            'release_service_name').value
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
        self.tip_front_limit_position = self.get_parameter(
            'tip_front_limit_position').value
        self.tip_rear_limit_position = self.get_parameter(
            'tip_rear_limit_position').value
        self.root_initial_position = self.get_parameter(
            'root_initial_position').value
        self.release_root_position = self.get_parameter(
            'release_root_position').value
        self.release_tip_position = self.get_parameter(
            'release_tip_position').value

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
        self.release_service = self.create_service(
            ReleaseKfs,
            release_service_name,
            self.handle_release_kfs,
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

    def set_rotation_pose(self, root_position, tip_position):
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
                              pickup_tip_position, root_returns_first,
                              load_method):
        self.set_rotation_pose(
            pickup_root_position,
            pickup_tip_position,
        )
        self.set_grip(self.kfs_grip_position)

        if load_method == LoadKfs.Request.STANDARD:
            if root_returns_first:
                self.set_root_rotate(self.root_initial_position)
                self.set_tip_rotate(self.tip_rear_limit_position)
            else:
                self.set_tip_rotate(self.tip_rear_limit_position)
                self.set_root_rotate(self.root_initial_position)
            self.set_grip(self.grip_open_position)
        else:
            self.set_rotation_pose(
                self.root_initial_position,
                self.tip_front_limit_position,
            )

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

                if request.load_method == LoadKfs.Request.STANDARD:
                    load_method_name = 'standard'
                elif request.load_method == LoadKfs.Request.TRANSFER:
                    load_method_name = 'transfer'
                else:
                    raise ValueError(
                        'unsupported KFS load method: '
                        f'{request.load_method}')

                self.wait_for_dependencies()
                self.execute_load_sequence(
                    pickup_root_position,
                    pickup_tip_position,
                    root_returns_first,
                    request.load_method,
                )
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                return response

            response.success = True
            response.message = (
                f'{mode_name} KFS load sequence completed with '
                f'{load_method_name} method')
            return response

    def handle_release_kfs(self, request, response):
        del request
        with self.operation_lock:
            try:
                self.wait_for_dependencies()
                self.set_rotation_pose(
                    self.release_root_position,
                    self.release_tip_position,
                )
                self.set_grip(self.grip_open_position)
                self.set_rotation_pose(
                    self.root_initial_position,
                    self.tip_front_limit_position,
                )
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                return response

            response.success = True
            response.message = (
                'KFS released; gripper returned to initial pose')
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
