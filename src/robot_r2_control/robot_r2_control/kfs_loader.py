import math
import threading
import time
from dataclasses import dataclass

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from robot_r2_interfaces.srv import (
    LoadKfs,
    PopKfs,
    ReleaseKfs,
    SetGripperGrip,
    SetGripperRotate,
    SetGripperTipRotate,
)


STEP_WIDTH = 6
ROOT_POSITION_RANGE = (0.0, 2.356194490192345)
TIP_POSITION_RANGE = (-math.pi, 0.0)
GRIP_POSITION_RANGE = (0.0, 0.209)
TRAJECTORY_PARAMETER_NAMES = (
    'front_standard_sequence',
    'front_transfer_sequence',
    'top_standard_sequence',
    'top_transfer_sequence',
    'release_sequence',
    'pop_sequence',
)


@dataclass(frozen=True)
class MotionStep:
    root_position: float
    tip_position: float
    grip_position: float
    root_tolerance: float
    tip_tolerance: float
    grip_tolerance: float


def _finite_number(value, description):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{description} must be a number')
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'{description} must be finite')
    return result


def _validate_range(value, limits, description):
    lower, upper = limits
    if not lower <= value <= upper:
        raise ValueError(
            f'{description} must be between {lower} and {upper}')


def parse_sequence(parameter_name, values):
    if not isinstance(values, (list, tuple)):
        raise ValueError(f'{parameter_name} must be a double array')
    if not values:
        raise ValueError(f'{parameter_name} must not be empty')
    if len(values) % STEP_WIDTH != 0:
        raise ValueError(
            f'{parameter_name} length must be a multiple of {STEP_WIDTH}')

    steps = []
    for offset in range(0, len(values), STEP_WIDTH):
        step_number = offset // STEP_WIDTH + 1
        raw_step = values[offset:offset + STEP_WIDTH]
        parsed = [
            _finite_number(
                value,
                f'{parameter_name} step {step_number} field {field + 1}',
            )
            for field, value in enumerate(raw_step)
        ]
        step = MotionStep(*parsed)

        _validate_range(
            step.root_position,
            ROOT_POSITION_RANGE,
            f'{parameter_name} step {step_number} root position',
        )
        _validate_range(
            step.tip_position,
            TIP_POSITION_RANGE,
            f'{parameter_name} step {step_number} tip position',
        )
        _validate_range(
            step.grip_position,
            GRIP_POSITION_RANGE,
            f'{parameter_name} step {step_number} grip position',
        )

        tolerances = (
            ('root', step.root_tolerance),
            ('tip', step.tip_tolerance),
            ('grip', step.grip_tolerance),
        )
        for motor_name, tolerance in tolerances:
            if tolerance <= 0.0:
                raise ValueError(
                    f'{parameter_name} step {step_number} '
                    f'{motor_name} tolerance must be greater than zero')
        steps.append(step)

    return tuple(steps)


def validate_timeout(value):
    timeout = _finite_number(value, 'service_timeout_sec')
    if timeout <= 0.0:
        raise ValueError('service_timeout_sec must be greater than zero')
    return timeout


class KfsLoaderController(Node):
    LOAD_SERVICE = '/r2/kfs/load'
    RELEASE_SERVICE = '/r2/kfs/release'
    POP_SERVICE = '/r2/kfs/pop'
    GRIP_SERVICE = '/r2/gripper/set_grip'
    ROOT_ROTATE_SERVICE = '/r2/gripper/set_rotate'
    TIP_ROTATE_SERVICE = '/r2/gripper/set_tip_rotate'

    def __init__(self):
        super().__init__('kfs_loader_control')
        self.callback_group = ReentrantCallbackGroup()
        self.operation_lock = threading.Lock()
        self.config_lock = threading.Lock()

        self.declare_parameter('service_timeout_sec', 10.0)
        for parameter_name in TRAJECTORY_PARAMETER_NAMES:
            self.declare_parameter(
                parameter_name,
                Parameter.Type.DOUBLE_ARRAY,
            )

        self.service_timeout_sec = validate_timeout(
            self.get_parameter('service_timeout_sec').value)
        self.sequences = {
            parameter_name: parse_sequence(
                parameter_name,
                self.get_parameter(parameter_name).value,
            )
            for parameter_name in TRAJECTORY_PARAMETER_NAMES
        }
        self._parameter_callback = self.add_on_set_parameters_callback(
            self.on_parameters_changed)

        self.grip_client = self.create_client(
            SetGripperGrip,
            self.GRIP_SERVICE,
            callback_group=self.callback_group,
        )
        self.root_rotate_client = self.create_client(
            SetGripperRotate,
            self.ROOT_ROTATE_SERVICE,
            callback_group=self.callback_group,
        )
        self.tip_rotate_client = self.create_client(
            SetGripperTipRotate,
            self.TIP_ROTATE_SERVICE,
            callback_group=self.callback_group,
        )
        self.load_service = self.create_service(
            LoadKfs,
            self.LOAD_SERVICE,
            self.handle_load_kfs,
            callback_group=self.callback_group,
        )
        self.release_service = self.create_service(
            ReleaseKfs,
            self.RELEASE_SERVICE,
            self.handle_release_kfs,
            callback_group=self.callback_group,
        )
        self.pop_service = self.create_service(
            PopKfs,
            self.POP_SERVICE,
            self.handle_pop_kfs,
            callback_group=self.callback_group,
        )

    def on_parameters_changed(self, parameters):
        new_sequences = {}
        new_timeout = None
        try:
            for parameter in parameters:
                if parameter.name in TRAJECTORY_PARAMETER_NAMES:
                    new_sequences[parameter.name] = parse_sequence(
                        parameter.name,
                        parameter.value,
                    )
                elif parameter.name == 'service_timeout_sec':
                    new_timeout = validate_timeout(parameter.value)
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        with self.config_lock:
            self.sequences.update(new_sequences)
            if new_timeout is not None:
                self.service_timeout_sec = new_timeout
        return SetParametersResult(successful=True)

    def snapshot_config(self, sequence_name):
        with self.config_lock:
            return (
                self.sequences[sequence_name],
                self.service_timeout_sec,
            )

    def wait_for_dependencies(self, timeout_sec):
        clients = (
            (self.grip_client, 'gripper grip'),
            (self.root_rotate_client, 'gripper root rotate'),
            (self.tip_rotate_client, 'gripper tip rotate'),
        )
        for client, description in clients:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                raise RuntimeError(f'{description} service unavailable')

    @staticmethod
    def make_request(service_type, position, tolerance, timeout_sec):
        request = service_type.Request()
        request.position = float(position)
        request.tolerance = float(tolerance)
        request.timeout_sec = float(timeout_sec)
        return request

    @staticmethod
    def wait_for_step_futures(futures, timeout_sec, step_description):
        done_events = {}
        for motor_name, future in futures.items():
            done_event = threading.Event()
            future.add_done_callback(lambda _, event=done_event: event.set())
            done_events[motor_name] = done_event

        deadline = time.monotonic() + timeout_sec + 1.0
        responses = {}
        failures = []
        for motor_name, future in futures.items():
            remaining = max(0.0, deadline - time.monotonic())
            if not done_events[motor_name].wait(remaining):
                failures.append(f'{motor_name}: call timed out')
                continue
            try:
                response = future.result()
            except Exception as exc:
                failures.append(f'{motor_name}: call failed: {exc}')
                continue
            if response is None:
                failures.append(f'{motor_name}: call returned no response')
            elif not response.success:
                failures.append(f'{motor_name}: {response.message}')
            else:
                responses[motor_name] = response

        if failures:
            raise RuntimeError(
                f'{step_description} failed: ' + '; '.join(failures))
        return responses

    def execute_sequence(self, sequence_name, sequence, timeout_sec):
        self.wait_for_dependencies(timeout_sec)
        for step_index, step in enumerate(sequence, start=1):
            requests = {
                'root': self.make_request(
                    SetGripperRotate,
                    step.root_position,
                    step.root_tolerance,
                    timeout_sec,
                ),
                'tip': self.make_request(
                    SetGripperTipRotate,
                    step.tip_position,
                    step.tip_tolerance,
                    timeout_sec,
                ),
                'grip': self.make_request(
                    SetGripperGrip,
                    step.grip_position,
                    step.grip_tolerance,
                    timeout_sec,
                ),
            }
            futures = {
                'root': self.root_rotate_client.call_async(requests['root']),
                'tip': self.tip_rotate_client.call_async(requests['tip']),
                'grip': self.grip_client.call_async(requests['grip']),
            }
            self.wait_for_step_futures(
                futures,
                timeout_sec,
                f'{sequence_name} step {step_index}',
            )

    @staticmethod
    def load_sequence_name(request):
        mode_names = {
            LoadKfs.Request.FRONT: 'front',
            LoadKfs.Request.TOP: 'top',
        }
        method_names = {
            LoadKfs.Request.STANDARD: 'standard',
            LoadKfs.Request.TRANSFER: 'transfer',
        }
        if request.mode not in mode_names:
            raise ValueError(f'unsupported KFS load mode: {request.mode}')
        if request.load_method not in method_names:
            raise ValueError(
                f'unsupported KFS load method: {request.load_method}')
        return (
            f'{mode_names[request.mode]}_'
            f'{method_names[request.load_method]}_sequence'
        )

    def handle_load_kfs(self, request, response):
        with self.operation_lock:
            try:
                sequence_name = self.load_sequence_name(request)
                sequence, timeout_sec = self.snapshot_config(sequence_name)
                self.execute_sequence(
                    sequence_name,
                    sequence,
                    timeout_sec,
                )
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                return response

            response.success = True
            response.message = f'{sequence_name} completed'
            return response

    def handle_release_kfs(self, request, response):
        del request
        with self.operation_lock:
            try:
                sequence_name = 'release_sequence'
                sequence, timeout_sec = self.snapshot_config(sequence_name)
                self.execute_sequence(
                    sequence_name,
                    sequence,
                    timeout_sec,
                )
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                return response

            response.success = True
            response.message = f'{sequence_name} completed'
            return response

    def handle_pop_kfs(self, request, response):
        del request
        with self.operation_lock:
            try:
                sequence_name = 'pop_sequence'
                sequence, timeout_sec = self.snapshot_config(sequence_name)
                self.execute_sequence(
                    sequence_name,
                    sequence,
                    timeout_sec,
                )
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                return response

            response.success = True
            response.message = f'{sequence_name} completed'
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
