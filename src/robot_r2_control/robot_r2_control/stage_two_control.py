import math
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.srv import (
    StageTwo,
    StageTwoPointOne,
    StageTwoPointTwo,
)


class StageTwoController(Node):
    def __init__(self):
        super().__init__('stage_two_control')
        self.callback_group = ReentrantCallbackGroup()
        self.service_lock = threading.Lock()
        self.loaded_count = 0

        self.declare_parameter('service_name', '/r2/stage_two')
        self.declare_parameter(
            'stage_two_point_one_service', '/r2/stage_two_point_one')
        self.declare_parameter(
            'stage_two_point_two_service', '/r2/stage_two_point_two')
        self.declare_parameter('dependency_timeout_sec', 2.0)
        self.declare_parameter('stage_two_point_one_timeout_sec', 450.0)
        self.declare_parameter('stage_two_point_two_timeout_sec', 1800.0)

        service_name = str(self.get_parameter('service_name').value)
        point_one_service = str(
            self.get_parameter('stage_two_point_one_service').value)
        point_two_service = str(
            self.get_parameter('stage_two_point_two_service').value)
        self.dependency_timeout_sec = self._positive_parameter(
            'dependency_timeout_sec')
        self.point_one_timeout_sec = self._positive_parameter(
            'stage_two_point_one_timeout_sec')
        self.point_two_timeout_sec = self._positive_parameter(
            'stage_two_point_two_timeout_sec')

        self.point_one_client = self.create_client(
            StageTwoPointOne,
            point_one_service,
            callback_group=self.callback_group,
        )
        self.point_two_client = self.create_client(
            StageTwoPointTwo,
            point_two_service,
            callback_group=self.callback_group,
        )
        self.stage_two_service = self.create_service(
            StageTwo,
            service_name,
            self.handle_stage_two,
            callback_group=self.callback_group,
        )

    def _positive_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    @staticmethod
    def validate_decision(decision):
        if decision not in (
            StageTwo.Request.LEFT,
            StageTwo.Request.RIGHT,
        ):
            raise ValueError(
                f'fake_kfs_decision must be LEFT(1) or RIGHT(2), got '
                f'{decision}')

    def wait_for_dependencies(self):
        dependencies = (
            (self.point_one_client, 'StageTwoPointOne'),
            (self.point_two_client, 'StageTwoPointTwo'),
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

    def run_point_one(self):
        request = StageTwoPointOne.Request()
        request.loaded_count = 0
        response = self.wait_for_future(
            self.point_one_client.call_async(request),
            self.point_one_timeout_sec,
            'StageTwoPointOne',
        )
        self.loaded_count = int(response.loaded_count)
        if not response.success:
            raise RuntimeError(
                f'StageTwoPointOne failed: {response.message}')

    def run_point_two(self, decision):
        request = StageTwoPointTwo.Request()
        request.fake_kfs_decision = int(decision)
        request.loaded_count = self.loaded_count
        response = self.wait_for_future(
            self.point_two_client.call_async(request),
            self.point_two_timeout_sec,
            'StageTwoPointTwo',
        )
        self.loaded_count = int(response.loaded_count)
        if not response.success:
            raise RuntimeError(
                f'StageTwoPointTwo failed: {response.message}')

    def handle_stage_two(self, request, response):
        with self.service_lock:
            self.loaded_count = 0
            try:
                self.validate_decision(request.fake_kfs_decision)
                self.wait_for_dependencies()
                self.run_point_one()
                self.run_point_two(request.fake_kfs_decision)
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                response.loaded_count = self.loaded_count
                return response

            response.success = True
            response.message = 'Stage two completed: 2.1 -> 2.2'
            response.loaded_count = self.loaded_count
            return response


def main():
    rclpy.init()
    node = StageTwoController()
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
