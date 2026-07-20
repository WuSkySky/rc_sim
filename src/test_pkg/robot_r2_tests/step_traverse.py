#!/usr/bin/env python3
"""
Standalone step traverse — direct move_to_pose + set_lift calls.
独立于 /r2/traverse_adjacent_step 服务，单元测试用。
"""
import math
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.srv import MoveToPose, SetLift


GRID = [
    [None, (-2.6, -4.2, 0), None, (-2.6, -1.8, 0), None],
    [(-1.4, -5.4, 0), (-1.4, -4.2, 1), (-1.4, -3.0, 2), (-1.4, -1.8, 1), (-1.4, -0.6, 0)],
    [None, (-0.2, -4.2, 2), (-0.2, -3.0, 3), (-0.2, -1.8, 2), None],
    [(1.0, -5.4, 0), (1.0, -4.2, 1), (1.0, -3.0, 2), (1.0, -1.8, 3), None],
    [None, (2.2, -4.2, 2), (2.2, -3.0, 1), (2.2, -1.8, 2), None],
    [None, None, (3.4, -3.0, 0), None, None],
]

MOVE_TO_POSE_SERVICE = '/r2/move_to_pose'
SET_LIFT_SERVICE = '/r2/lift/set'
MOVE_TIMEOUT = 35.0
LIFT_TIMEOUT = 15.0

LIFT_FRONT_UP = (0.2, 0.0)
LIFT_REAR_UP = (0.0, 0.2)
LIFT_BOTH_UP = (0.2, 0.2)
LIFT_ALL_DOWN = (0.0, 0.0)


def validate_cell(grid_data, index):
    row, col = index
    if row < 0 or row >= len(grid_data):
        raise IndexError(f'Row {row} out of range')
    if col < 0 or col >= len(grid_data[row]):
        raise IndexError(f'Column {col} out of range')
    cell = grid_data[row][col]
    if cell is None:
        raise ValueError(f'Grid cell {index} is empty')
    return cell


def is_adjacent_4(current_index, target_index):
    return abs(target_index[0] - current_index[0]) + abs(target_index[1] - current_index[1]) == 1


def get_direction(current_index, target_index):
    dr = target_index[0] - current_index[0]
    dc = target_index[1] - current_index[1]
    if dr == 1 and dc == 0:  return (1.0, 0.0)
    if dr == -1 and dc == 0: return (-1.0, 0.0)
    if dr == 0 and dc == 1:  return (0.0, 1.0)
    if dr == 0 and dc == -1: return (0.0, -1.0)
    raise ValueError(f'{current_index} -> {target_index} not 4-neighbors')


def get_cell_center(grid_data, index):
    x, y, h = validate_cell(grid_data, index)
    return x, y, h


def get_edge_point(center, direction, offset):
    return (center[0] + direction[0] * offset,
            center[1] + direction[1] * offset)


def get_direction_yaw(direction):
    return math.atan2(direction[1], direction[0])


class StandaloneStepTraverse(Node):
    def __init__(self, near_offset=0.3, far_offset=0.487):
        super().__init__('standalone_step_traverse')
        self.cbg = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.near = near_offset
        self.far = far_offset

        self.move_cli = self.create_client(MoveToPose, MOVE_TO_POSE_SERVICE, callback_group=self.cbg)
        self.lift_cli = self.create_client(SetLift, SET_LIFT_SERVICE, callback_group=self.cbg)

    def _wait_svcs(self, timeout=2.0):
        if not self.move_cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('move_to_pose unavailable')
        if not self.lift_cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('set_lift unavailable')

    def _wait(self, fut, timeout, label):
        ev = threading.Event()
        fut.add_done_callback(lambda _: ev.set())
        if not ev.wait(timeout + 1.0):
            raise RuntimeError(f'{label} timed out')
        r = fut.result()
        if r is None:
            raise RuntimeError(f'{label} call failed')
        return r

    def move(self, x, y, yaw):
        req = MoveToPose.Request()
        req.x = float(x); req.y = float(y); req.yaw = float(yaw)
        req.position_tolerance = 0.0; req.yaw_tolerance = 0.0; req.timeout_sec = 0.0
        r = self._wait(self.move_cli.call_async(req), MOVE_TIMEOUT, 'Move')
        if not r.success:
            raise RuntimeError(f'Move failed: {r.message}')

    def lift(self, front, rear):
        req = SetLift.Request()
        req.front_lift = float(front); req.rear_lift = float(rear); req.timeout_sec = 0.0
        r = self._wait(self.lift_cli.call_async(req), LIFT_TIMEOUT, 'Lift')
        if not r.success:
            raise RuntimeError(f'Lift failed: {r.message}')

    def up_step(self, cur, tgt, d, yaw):
        ce = get_edge_point(cur, d, self.near)
        te = get_edge_point(tgt, (-d[0], -d[1]), self.far)
        self.move(cur[0], cur[1], yaw)
        self.lift(*LIFT_BOTH_UP)
        self.move(ce[0], ce[1], yaw)
        self.lift(*LIFT_REAR_UP)
        self.move(te[0], te[1], yaw)
        self.lift(*LIFT_ALL_DOWN)
        self.move(tgt[0], tgt[1], yaw)

    def down_step(self, cur, tgt, d, yaw):
        ce = get_edge_point(cur, d, self.far)
        te = get_edge_point(tgt, (-d[0], -d[1]), self.near)
        self.move(cur[0], cur[1], yaw)
        self.move(ce[0], ce[1], yaw)
        self.lift(*LIFT_FRONT_UP)
        self.move(te[0], te[1], yaw)
        self.lift(*LIFT_BOTH_UP)
        self.move(tgt[0], tgt[1], yaw)
        self.lift(*LIFT_ALL_DOWN)

    def one_step(self, grid, cur_idx, tgt_idx):
        cur = get_cell_center(grid, cur_idx)
        tgt = get_cell_center(grid, tgt_idx)
        if not is_adjacent_4(cur_idx, tgt_idx):
            raise ValueError(f'{tgt_idx} not adjacent to {cur_idx}')
        d = get_direction(cur_idx, tgt_idx)
        yaw = get_direction_yaw(d)
        if tgt[2] > cur[2]:
            self.up_step(cur, tgt, d, yaw)
        elif tgt[2] < cur[2]:
            self.down_step(cur, tgt, d, yaw)
        else:
            self.move(cur[0], cur[1], yaw)
            self.move(tgt[0], tgt[1], yaw)

    def traverse(self, rows, cols):
        with self.lock:
            self._wait_svcs()
            path = [(int(r), int(c)) for r, c in zip(rows, cols)]
            if len(path) < 2:
                raise ValueError('need >= 2 points')
            for a, b in zip(path[:-1], path[1:]):
                self.one_step(GRID, a, b)
            self.get_logger().info(f'Traverse done: {len(path)-1} segment(s)')


def main():
    rclpy.init()
    node = StandaloneStepTraverse()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    import time

    fwd_rows = [5, 4, 3, 2, 1, 1, 0]
    fwd_cols = [2, 2, 2, 2, 2, 1, 1]
    rev_rows = list(reversed(fwd_rows))
    rev_cols = list(reversed(fwd_cols))

    print(f'Forward:  rows={fwd_rows} cols={fwd_cols}')
    print(f'Reverse:  rows={rev_rows} cols={rev_cols}')
    time.sleep(3)

    try:
        while True:
            print('>>> Forward traverse')
            node.traverse(fwd_rows, fwd_cols)
            print('<<< Forward done')

            time.sleep(2)

            print('>>> Reverse traverse')
            node.traverse(rev_rows, rev_cols)
            print('<<< Reverse done')
    except Exception as e:
        print(f'FAILED: {e}')
    finally:
        executor.shutdown()
        executor_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
