import math
import threading
import time
from dataclasses import dataclass

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from robot_r2_common import AbortMonitor, AbortableMixin
from robot_r2_interfaces.srv import (
    Align,
    DetectLed,
    MoveRelative,
    SetJointPosition,
    SetLift,
    StageOne,
)


STAGE_ONE_SERVICE = '/r2/stage_one'
MOVE_RELATIVE_SERVICE = '/r2/move_relative'
SET_LIFT_SERVICE = '/r2/lift/set'
ALIGN_TO_TIP_SERVICE = '/r2/align_to_tip'
WEAPON_ROTATE_SERVICE = '/r2/weapon/set_rotate'
WEAPON_GRIP_SERVICE = '/r2/weapon/set_grip'
LED_DETECT_SERVICE = '/r2/led_detection/detect'


@dataclass(frozen=True)
class StageOneConfig:
    action_1_lift_height_m: float
    action_2_left_m: float
    action_2_backward_m: float
    action_3_lift_height_m: float
    action_4_pixel_tolerance_px: float
    action_5_weapon_rotate_rad: float
    action_5_weapon_grip_m: float
    action_6_backward_m: float
    action_7_weapon_grip_m: float
    action_8_lift_increment_m: float
    action_8_weapon_rotate_rad: float
    action_9_pre_lower_forward_m: float
    action_9_lift_height_m: float
    action_10_forward_m: float
    action_11_yaw_delta_rad: float
    led_target_states: tuple[bool, ...]
    final_weapon_grip_m: float
    lift_tolerance_m: float
    position_tolerance_m: float
    yaw_tolerance_rad: float
    weapon_rotate_tolerance_rad: float
    weapon_grip_tolerance_m: float
    dependency_timeout_sec: float
    move_timeout_sec: float
    lift_timeout_sec: float
    alignment_timeout_sec: float
    weapon_timeout_sec: float
    led_detection_timeout_sec: float


PARAMETER_DEFAULTS = {
    'action_1_lift_height_m': 0.01,
    'action_2_left_m': 0.781,
    'action_2_backward_m': 0.887,
    'action_3_lift_height_m': 0.14,
    'action_4_pixel_tolerance_px': 5.0,
    'action_5_weapon_rotate_rad': math.pi / 2.0,
    'action_5_weapon_grip_m': 0.028,
    'action_6_backward_m': 0.10,
    'action_7_weapon_grip_m': 0.0,
    'action_8_lift_increment_m': 0.03,
    'action_8_weapon_rotate_rad': math.radians(142.0),
    'action_9_pre_lower_forward_m': 0.05,
    'action_9_lift_height_m': 0.01,
    'action_10_forward_m': 0.20,
    'action_11_yaw_delta_rad': math.pi,
    'led_target_states': [True],
    'final_weapon_grip_m': 0.028,
    'lift_tolerance_m': 0.002,
    'position_tolerance_m': 0.005,
    'yaw_tolerance_rad': 0.01,
    'weapon_rotate_tolerance_rad': 0.01,
    'weapon_grip_tolerance_m': 0.001,
    'dependency_timeout_sec': 2.0,
    'move_timeout_sec': 35.0,
    'lift_timeout_sec': 15.0,
    'alignment_timeout_sec': 15.0,
    'weapon_timeout_sec': 10.0,
    'led_detection_timeout_sec': 32.0,
}


class StageOneController(AbortableMixin, Node):
    def __init__(self):
        super().__init__('stage_one')
        self.callback_group = ReentrantCallbackGroup()
        self.service_lock = threading.Lock()
        self.config_lock = threading.RLock()
        self.abort_monitor = AbortMonitor(
            self, callback_group=self.callback_group)

        for name, default in PARAMETER_DEFAULTS.items():
            self.declare_parameter(name, default)
        self._config = self._read_config()

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
        self.align_client = self.create_client(
            Align,
            ALIGN_TO_TIP_SERVICE,
            callback_group=self.callback_group,
        )
        self.weapon_rotate_client = self.create_client(
            SetJointPosition,
            WEAPON_ROTATE_SERVICE,
            callback_group=self.callback_group,
        )
        self.weapon_grip_client = self.create_client(
            SetJointPosition,
            WEAPON_GRIP_SERVICE,
            callback_group=self.callback_group,
        )
        self.led_detect_client = self.create_client(
            DetectLed,
            LED_DETECT_SERVICE,
            callback_group=self.callback_group,
        )
        self.task_service = self.create_service(
            StageOne,
            STAGE_ONE_SERVICE,
            self.handle_task,
            callback_group=self.callback_group,
        )
        self.parameter_callback = self.add_on_set_parameters_callback(
            self.on_parameters_changed)

    @staticmethod
    def validate_config(config):
        for name in StageOneConfig.__dataclass_fields__:
            if name == 'led_target_states':
                continue
            value = getattr(config, name)
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite')

        if not config.led_target_states:
            raise ValueError('led_target_states must not be empty')
        if not all(type(state) is bool for state in config.led_target_states):
            raise ValueError('led_target_states must contain only booleans')

        non_negative = (
            'action_1_lift_height_m',
            'action_2_left_m',
            'action_2_backward_m',
            'action_3_lift_height_m',
            'action_5_weapon_rotate_rad',
            'action_5_weapon_grip_m',
            'action_6_backward_m',
            'action_7_weapon_grip_m',
            'action_8_lift_increment_m',
            'action_8_weapon_rotate_rad',
            'action_9_pre_lower_forward_m',
            'action_9_lift_height_m',
            'action_10_forward_m',
            'final_weapon_grip_m',
        )
        for name in non_negative:
            if getattr(config, name) < 0.0:
                raise ValueError(f'{name} must be non-negative')

        positive = (
            'action_4_pixel_tolerance_px',
            'lift_tolerance_m',
            'position_tolerance_m',
            'yaw_tolerance_rad',
            'weapon_rotate_tolerance_rad',
            'weapon_grip_tolerance_m',
            'dependency_timeout_sec',
            'move_timeout_sec',
            'lift_timeout_sec',
            'alignment_timeout_sec',
            'weapon_timeout_sec',
            'led_detection_timeout_sec',
        )
        for name in positive:
            if getattr(config, name) <= 0.0:
                raise ValueError(f'{name} must be positive')

    @classmethod
    def config_from_values(cls, values):
        normalized = dict(values)
        normalized['led_target_states'] = tuple(
            normalized['led_target_states'])
        config = StageOneConfig(**normalized)
        cls.validate_config(config)
        return config

    def _read_config(self):
        values = {}
        for name in StageOneConfig.__dataclass_fields__:
            value = self.get_parameter(name).value
            values[name] = (
                tuple(bool(state) for state in value)
                if name == 'led_target_states'
                else float(value)
            )
        return self.config_from_values(values)

    def config_snapshot(self):
        with self.config_lock:
            return self._config

    def on_parameters_changed(self, parameters):
        with self.config_lock:
            values = {
                name: getattr(self._config, name)
                for name in StageOneConfig.__dataclass_fields__
            }
            for parameter in parameters:
                if parameter.name not in values:
                    continue
                if parameter.name == 'led_target_states':
                    if parameter.type_ != Parameter.Type.BOOL_ARRAY:
                        return SetParametersResult(
                            successful=False,
                            reason='led_target_states must be a bool array',
                        )
                    values[parameter.name] = tuple(parameter.value)
                    continue
                if parameter.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{parameter.name} must be a double',
                    )
                values[parameter.name] = float(parameter.value)
            try:
                candidate = self.config_from_values(values)
            except ValueError as exc:
                return SetParametersResult(
                    successful=False, reason=str(exc))
            self._config = candidate
        return SetParametersResult(successful=True)

    def wait_for_dependencies(self, timeout_sec):
        dependencies = (
            (self.move_client, 'MoveRelative'),
            (self.lift_client, 'SetLift'),
            (self.align_client, 'AlignToTip'),
            (self.weapon_rotate_client, 'WeaponRotate'),
            (self.weapon_grip_client, 'WeaponGrip'),
            (self.led_detect_client, 'LedDetect'),
        )
        for client, name in dependencies:
            if not self.wait_for_service_or_abort(client, timeout_sec):
                raise RuntimeError(f'{name} service unavailable')

    @staticmethod
    def validate_team(team):
        if team not in (StageOne.Request.RED, StageOne.Request.BLUE):
            raise ValueError(
                f'team must be red or blue, got {team!r}')

    @staticmethod
    def lateral_sign(team):
        StageOneController.validate_team(team)
        return 1.0 if team == StageOne.Request.RED else -1.0

    def wait_for_future(self, future, timeout_sec, description):
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not self.wait_for_event_or_abort(completed, timeout_sec + 1.0):
            raise RuntimeError(f'{description} timed out waiting for response')
        response = future.result()
        if response is None:
            raise RuntimeError(f'{description} call failed')
        return response

    def wait_for_parallel_futures(self, futures, timeout_sec):
        events = []
        for _, future in futures:
            event = threading.Event()
            future.add_done_callback(lambda _, done=event: done.set())
            events.append(event)

        deadline = time.monotonic() + timeout_sec + 1.0
        errors = []
        for (description, future), event in zip(futures, events):
            remaining = deadline - time.monotonic()
            if (
                remaining <= 0.0 or
                not self.wait_for_event_or_abort(event, remaining)
            ):
                errors.append(
                    f'{description} timed out waiting for response')
                continue
            try:
                response = future.result()
            except Exception as exc:
                errors.append(f'{description} call failed: {exc}')
                continue
            if response is None:
                errors.append(f'{description} call failed')
            elif not response.success:
                errors.append(f'{description} failed: {response.message}')
        if errors:
            raise RuntimeError('; '.join(errors))

    def set_lift(self, height, config):
        request = SetLift.Request()
        request.front_lift = height
        request.rear_lift = height
        request.tolerance = config.lift_tolerance_m
        request.timeout_sec = config.lift_timeout_sec
        response = self.wait_for_future(
            self.call_async_or_abort(self.lift_client, request),
            config.lift_timeout_sec,
            'SetLift',
        )
        if not response.success:
            raise RuntimeError(f'SetLift failed: {response.message}')

    def move_relative(self, forward, left, yaw_delta, config):
        request = self.move_relative_request(
            forward, left, yaw_delta, config)
        response = self.wait_for_future(
            self.call_async_or_abort(self.move_client, request),
            config.move_timeout_sec,
            'MoveRelative',
        )
        if not response.success:
            raise RuntimeError(f'MoveRelative failed: {response.message}')

    @staticmethod
    def move_relative_request(forward, left, yaw_delta, config):
        request = MoveRelative.Request()
        request.pose_source = MoveRelative.Request.SERIAL
        request.forward = float(forward)
        request.left = float(left)
        request.yaw_delta = float(yaw_delta)
        request.position_tolerance = config.position_tolerance_m
        request.yaw_tolerance = config.yaw_tolerance_rad
        request.timeout_sec = config.move_timeout_sec
        return request

    def align_tip(self, config):
        request = Align.Request()
        request.pixel_tolerance = config.action_4_pixel_tolerance_px
        request.timeout_sec = config.alignment_timeout_sec
        response = self.wait_for_future(
            self.call_async_or_abort(self.align_client, request),
            config.alignment_timeout_sec,
            'AlignToTip',
        )
        if not response.success:
            raise RuntimeError(f'AlignToTip failed: {response.message}')

    def weapon_request(self, position, tolerance, timeout_sec):
        request = SetJointPosition.Request()
        request.position = position
        request.tolerance = tolerance
        request.timeout_sec = timeout_sec
        return request

    def set_weapon_joint(
        self, client, description, position, tolerance, config
    ):
        request = self.weapon_request(
            position, tolerance, config.weapon_timeout_sec)
        response = self.wait_for_future(
            self.call_async_or_abort(client, request),
            config.weapon_timeout_sec,
            description,
        )
        if not response.success:
            raise RuntimeError(f'{description} failed: {response.message}')

    def set_weapon_pair(self, rotate_position, grip_position, config):
        rotate_request = self.weapon_request(
            rotate_position,
            config.weapon_rotate_tolerance_rad,
            config.weapon_timeout_sec,
        )
        grip_request = self.weapon_request(
            grip_position,
            config.weapon_grip_tolerance_m,
            config.weapon_timeout_sec,
        )
        futures = (
            (
                'WeaponRotate',
                self.call_async_or_abort(
                    self.weapon_rotate_client, rotate_request),
            ),
            (
                'WeaponGrip',
                self.call_async_or_abort(
                    self.weapon_grip_client, grip_request),
            ),
        )
        self.wait_for_parallel_futures(
            futures, config.weapon_timeout_sec)

    def move_and_set_weapon_pair(
        self, forward, left, yaw_delta, rotate_position, grip_position, config
    ):
        move_request = self.move_relative_request(
            forward, left, yaw_delta, config)
        rotate_request = self.weapon_request(
            rotate_position,
            config.weapon_rotate_tolerance_rad,
            config.weapon_timeout_sec,
        )
        grip_request = self.weapon_request(
            grip_position,
            config.weapon_grip_tolerance_m,
            config.weapon_timeout_sec,
        )
        futures = (
            (
                'MoveRelative',
                self.call_async_or_abort(self.move_client, move_request),
            ),
            (
                'WeaponRotate',
                self.call_async_or_abort(
                    self.weapon_rotate_client, rotate_request),
            ),
            (
                'WeaponGrip',
                self.call_async_or_abort(
                    self.weapon_grip_client, grip_request),
            ),
        )
        self.wait_for_parallel_futures(
            futures, max(config.move_timeout_sec, config.weapon_timeout_sec))

    def detect_led(self, config):
        request = DetectLed.Request()
        request.target_states = list(config.led_target_states)
        response = self.wait_for_future(
            self.call_async_or_abort(self.led_detect_client, request),
            config.led_detection_timeout_sec,
            'LedDetect',
        )
        if not response.success:
            raise RuntimeError(f'LedDetect failed: {response.message}')

    def run_action(self, number, description, operation):
        self.get_logger().info(
            f'Step1 action {number}/14 started: {description}')
        try:
            operation()
        except Exception as exc:
            raise RuntimeError(
                f'Action {number} ({description}) failed: {exc}') from exc
        self.get_logger().info(
            f'Step1 action {number}/14 completed: {description}')

    def execute_task(self, config, team):
        lateral_sign = self.lateral_sign(team)
        lateral_direction = 'left' if lateral_sign > 0.0 else 'right'
        self.run_action(
            1,
            'lift chassis to initial height',
            lambda: self.set_lift(config.action_1_lift_height_m, config),
        )
        self.run_action(
            2,
            f'move {lateral_direction} and backward while preparing weapon',
            lambda: self.move_and_set_weapon_pair(
                -config.action_2_backward_m,
                lateral_sign * config.action_2_left_m,
                0.0,
                config.action_5_weapon_rotate_rad,
                config.action_5_weapon_grip_m,
                config,
            ),
        )
        self.run_action(
            3,
            'lift chassis to working height',
            lambda: self.set_lift(config.action_3_lift_height_m, config),
        )
        self.run_action(4, 'align weapon tip', lambda: self.align_tip(config))
        self.run_action(
            5,
            'move backward',
            lambda: self.move_relative(
                -config.action_6_backward_m, 0.0, 0.0, config),
        )
        self.run_action(
            6,
            'close weapon gripper',
            lambda: self.set_weapon_joint(
                self.weapon_grip_client,
                'WeaponGrip',
                config.action_7_weapon_grip_m,
                config.weapon_grip_tolerance_m,
                config,
            ),
        )
        self.run_action(
            7,
            'lift chassis before final weapon rotate',
            lambda: self.set_lift(
                config.action_3_lift_height_m
                + config.action_8_lift_increment_m,
                config,
            ),
        )
        self.run_action(
            8,
            'rotate weapon to final angle',
            lambda: self.set_weapon_joint(
                self.weapon_rotate_client,
                'WeaponRotate',
                config.action_8_weapon_rotate_rad,
                config.weapon_rotate_tolerance_rad,
                config,
            ),
        )
        self.run_action(
            9,
            'move forward before lowering chassis',
            lambda: self.move_relative(
                config.action_9_pre_lower_forward_m, 0.0, 0.0, config),
        )
        self.run_action(
            10,
            'lower chassis to travel height',
            lambda: self.set_lift(config.action_9_lift_height_m, config),
        )
        self.run_action(
            11,
            'move forward',
            lambda: self.move_relative(
                config.action_10_forward_m, 0.0, 0.0, config),
        )
        self.run_action(
            12,
            'rotate chassis by pi radians',
            lambda: self.move_relative(
                0.0, 0.0, config.action_11_yaw_delta_rad, config),
        )
        self.run_action(
            13,
            'wait for target LED state',
            lambda: self.detect_led(config),
        )
        self.run_action(
            14,
            'release weapon gripper after LED confirmation',
            lambda: self.set_weapon_joint(
                self.weapon_grip_client,
                'WeaponGrip',
                config.final_weapon_grip_m,
                config.weapon_grip_tolerance_m,
                config,
            ),
        )

    def handle_task(self, request, response):
        abort_scope = self.abort_scope()
        with self.service_lock, abort_scope:
            config = self.config_snapshot()
            try:
                self.raise_if_abort_requested()
                self.validate_team(request.team)
                self.wait_for_dependencies(config.dependency_timeout_sec)
                self.execute_task(config, request.team)
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                return response
            response.success = True
            response.message = f'Stage 1 completed for {request.team} team'
            return response


def main(args=None):
    rclpy.init(args=args)
    node = StageOneController()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
