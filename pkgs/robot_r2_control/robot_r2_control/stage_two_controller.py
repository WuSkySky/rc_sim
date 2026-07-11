import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.srv import AlignToKFS, LoadKfs, MoveToPose, SetLift
from robot_r2_interfaces.srv import StageTwo


class StageTwoController(Node):
    def __init__(self):
        super().__init__('stage_two_controller')
        self.callback_group = ReentrantCallbackGroup()
        self.service_lock = threading.Lock()

        self.declare_parameter('service_name', '/r2/stage_two')
        self.declare_parameter('move_to_pose_service', '/r2/move_to_pose')
        self.declare_parameter('set_lift_service', '/r2/lift/set')
        self.declare_parameter('align_kfs_service', '/r2/align_to_kfs')
        self.declare_parameter('load_kfs_service', '/r2/kfs/load')
        self.declare_parameter('dependency_timeout_sec', 2.0)
        self.declare_parameter('move_wait_timeout_sec', 35.0)
        self.declare_parameter('lift_wait_timeout_sec', 15.0)
        self.declare_parameter('align_wait_timeout_sec', 15.0)
        self.declare_parameter('load_wait_timeout_sec', 70.0)

        self.declare_parameter('source_x', 3.4)
        self.declare_parameter('source_y', -3.0)
        self.declare_parameter('target_x', 2.2)
        self.declare_parameter('target_y', -3.0)
        self.declare_parameter('travel_yaw', math.pi)
        self.declare_parameter('near_edge_offset', 0.3)
        self.declare_parameter('kfs_align_edge_offset', 0.72)
        self.declare_parameter('kfs_pickup_edge_offset', 0.55)
        self.declare_parameter('target_far_edge_offset', 0.487)
        self.declare_parameter('lift_up_front', 0.2)
        self.declare_parameter('lift_up_rear', 0.2)
        self.declare_parameter('lift_at_near_edge_front', 0.0)
        self.declare_parameter('lift_at_near_edge_rear', 0.2)
        self.declare_parameter('lift_final_front', 0.0)
        self.declare_parameter('lift_final_rear', 0.0)

        service_name = self.get_parameter('service_name').value
        move_service = self.get_parameter('move_to_pose_service').value
        lift_service = self.get_parameter('set_lift_service').value
        align_service = self.get_parameter('align_kfs_service').value
        load_service = self.get_parameter('load_kfs_service').value
        self.dependency_timeout_sec = self.get_parameter(
            'dependency_timeout_sec').value
        self.move_wait_timeout_sec = self.get_parameter(
            'move_wait_timeout_sec').value
        self.lift_wait_timeout_sec = self.get_parameter(
            'lift_wait_timeout_sec').value
        self.align_wait_timeout_sec = self.get_parameter(
            'align_wait_timeout_sec').value
        self.load_wait_timeout_sec = self.get_parameter(
            'load_wait_timeout_sec').value

        self.source_x = self.get_parameter('source_x').value
        self.source_y = self.get_parameter('source_y').value
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.travel_yaw = self.get_parameter('travel_yaw').value
        self.near_edge_offset = self.get_parameter('near_edge_offset').value
        self.kfs_align_edge_offset = self.get_parameter(
            'kfs_align_edge_offset').value
        self.kfs_pickup_edge_offset = self.get_parameter(
            'kfs_pickup_edge_offset').value
        self.target_far_edge_offset = self.get_parameter(
            'target_far_edge_offset').value
        self.lift_up_front = self.get_parameter('lift_up_front').value
        self.lift_up_rear = self.get_parameter('lift_up_rear').value
        self.lift_at_near_edge_front = self.get_parameter(
            'lift_at_near_edge_front').value
        self.lift_at_near_edge_rear = self.get_parameter(
            'lift_at_near_edge_rear').value
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
        self.align_client = self.create_client(
            AlignToKFS,
            align_service,
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

    def wait_for_dependencies(self):
        timeout = self.dependency_timeout_sec
        clients = (
            (self.move_client, 'MoveToPose'),
            (self.lift_client, 'SetLift'),
            (self.align_client, 'AlignToKFS'),
            (self.load_client, 'LoadKfs'),
        )
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

    def move_to_pose(self, x, y):
        request = MoveToPose.Request()
        request.x = float(x)
        request.y = float(y)
        request.yaw = float(self.travel_yaw)
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

    def align_kfs(self):
        request = AlignToKFS.Request()
        request.pixel_tolerance = 0.0
        request.timeout_sec = 0.0
        response = self.wait_for_future(
            self.align_client.call_async(request),
            self.align_wait_timeout_sec,
            'AlignToKFS',
        )
        if not response.success:
            raise RuntimeError(f'AlignToKFS failed: {response.message}')

    def load_kfs(self):
        response = self.wait_for_future(
            self.load_client.call_async(LoadKfs.Request()),
            self.load_wait_timeout_sec,
            'LoadKfs',
        )
        if not response.success:
            raise RuntimeError(f'LoadKfs failed: {response.message}')

    def handle_stage_two(self, request, response):
        del request
        with self.service_lock:
            try:
                self.wait_for_dependencies()
                direction_x = math.cos(self.travel_yaw)
                direction_y = math.sin(self.travel_yaw)
                source_near_x = (
                    self.source_x + direction_x * self.near_edge_offset)
                source_near_y = (
                    self.source_y + direction_y * self.near_edge_offset)
                target_align_x = (
                    self.target_x -
                    direction_x * self.kfs_align_edge_offset)
                target_align_y = (
                    self.target_y -
                    direction_y * self.kfs_align_edge_offset)
                target_pickup_x = (
                    self.target_x -
                    direction_x * self.kfs_pickup_edge_offset)
                target_pickup_y = (
                    self.target_y -
                    direction_y * self.kfs_pickup_edge_offset)
                target_far_x = (
                    self.target_x -
                    direction_x * self.target_far_edge_offset)
                target_far_y = (
                    self.target_y -
                    direction_y * self.target_far_edge_offset)

                self.move_to_pose(self.source_x, self.source_y)
                self.set_lift(self.lift_up_front, self.lift_up_rear)
                self.move_to_pose(source_near_x, source_near_y)
                self.set_lift(
                    self.lift_at_near_edge_front,
                    self.lift_at_near_edge_rear,
                )
                self.move_to_pose(target_align_x, target_align_y)
                self.get_logger().info('KFS alignment placeholder: sleeping 0.5s')
                time.sleep(0.5)
                self.move_to_pose(target_pickup_x, target_pickup_y)
                self.load_kfs()
                self.move_to_pose(target_far_x, target_far_y)
                self.set_lift(
                    self.lift_final_front,
                    self.lift_final_rear,
                )
                self.move_to_pose(self.target_x, self.target_y)
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                return response

            response.success = True
            response.message = 'Stage two completed'
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
