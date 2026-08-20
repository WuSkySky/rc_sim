import math
import threading

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.srv import (
    KfsAction,
    MoveRelative,
    MoveToPose,
    SetBasePose,
    SetJointPosition,
    StageThree,
)


STAGE_THREE_SERVICE = '/r2/stage_three'
SET_BASE_POSE_ODIN_SERVICE = '/r2/set_base_pose_odin'
MOVE_TO_POSE_SERVICE = '/r2/move_to_pose'
MOVE_RELATIVE_SERVICE = '/r2/move_relative'
KFS_ACTION_SERVICE = '/r2/kfs/action'
KFS_LIFT_SERVICE = '/r2/kfs_lift'


class StageThreeController(Node):
    POSITIVE_PARAMETERS = (
        'dependency_timeout_sec',
        'relocalization_timeout_sec',
        'move_timeout_sec',
        'pop_timeout_sec',
        'kfs_lift_timeout_sec',
        'first_target_x_offset',
        'target_x_spacing',
        'target_y_offset',
        'intermediate_backoff_distance',
        'standard_final_backoff_distance',
        'single_final_backoff_distance',
        'kfs_lift_tolerance',
    )
    FINITE_PARAMETERS = ('stage_two_exit_endpoint_x',)
    NON_NEGATIVE_PARAMETERS = ('kfs_lift_height',)

    def __init__(self):
        super().__init__('stage_three')
        self.callback_group = ReentrantCallbackGroup()
        self.service_lock = threading.Lock()
        self.config_lock = threading.Lock()

        self.declare_parameter('dependency_timeout_sec', 2.0)
        self.declare_parameter('relocalization_timeout_sec', 5.0)
        self.declare_parameter('move_timeout_sec', 35.0)
        self.declare_parameter('pop_timeout_sec', 70.0)
        self.declare_parameter('kfs_lift_timeout_sec', 15.0)
        self.declare_parameter(
            'blue_relocalization_pose',
            [-5.69, 5.53, 0.0, 0.0, 0.0, math.pi / 2.0],
        )
        self.declare_parameter('stage_two_exit_endpoint_x', -5.5)
        self.declare_parameter('first_target_x_offset', 0.21)
        self.declare_parameter('target_x_spacing', 0.54)
        self.declare_parameter('target_y_offset', 4.63)
        self.declare_parameter('intermediate_backoff_distance', 0.25)
        self.declare_parameter('standard_final_backoff_distance', 3.0)
        self.declare_parameter('single_final_backoff_distance', 2.0)
        self.declare_parameter('kfs_lift_height', 0.35)
        self.declare_parameter('kfs_lift_tolerance', 0.005)

        self.config = {
            name: self.validate_positive(
                self.get_parameter(name).value, name)
            for name in self.POSITIVE_PARAMETERS
        }
        self.config.update({
            name: self.validate_finite(
                self.get_parameter(name).value, name)
            for name in self.FINITE_PARAMETERS
        })
        self.config.update({
            name: self.validate_non_negative(
                self.get_parameter(name).value, name)
            for name in self.NON_NEGATIVE_PARAMETERS
        })
        self.blue_relocalization_pose = self.validate_pose(
            self.get_parameter('blue_relocalization_pose').value)
        self.validate_derived_config(
            self.config, self.blue_relocalization_pose)

        self.set_base_pose_client = self.create_client(
            SetBasePose,
            SET_BASE_POSE_ODIN_SERVICE,
            callback_group=self.callback_group,
        )
        self.move_to_pose_client = self.create_client(
            MoveToPose,
            MOVE_TO_POSE_SERVICE,
            callback_group=self.callback_group,
        )
        self.move_relative_client = self.create_client(
            MoveRelative,
            MOVE_RELATIVE_SERVICE,
            callback_group=self.callback_group,
        )
        self.kfs_action_client = self.create_client(
            KfsAction,
            KFS_ACTION_SERVICE,
            callback_group=self.callback_group,
        )
        self.kfs_lift_client = self.create_client(
            SetJointPosition,
            KFS_LIFT_SERVICE,
            callback_group=self.callback_group,
        )
        self.task_service = self.create_service(
            StageThree,
            STAGE_THREE_SERVICE,
            self.handle_task,
            callback_group=self.callback_group,
        )
        self.add_on_set_parameters_callback(self._on_parameters_changed)

    @staticmethod
    def validate_finite(raw_value, name):
        if isinstance(raw_value, bool) or not isinstance(
            raw_value, (int, float)
        ):
            raise ValueError(f'{name} must be numeric')
        value = float(raw_value)
        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
        return value

    @classmethod
    def validate_positive(cls, raw_value, name):
        value = cls.validate_finite(raw_value, name)
        if value <= 0.0:
            raise ValueError(f'{name} must be positive')
        return value

    @classmethod
    def validate_non_negative(cls, raw_value, name):
        value = cls.validate_finite(raw_value, name)
        if value < 0.0:
            raise ValueError(f'{name} must be non-negative')
        return value

    @staticmethod
    def validate_pose(values, name='blue_relocalization_pose'):
        if isinstance(values, (str, bytes)):
            raise ValueError(f'{name} must contain exactly 6 values')
        try:
            pose = tuple(values)
        except TypeError as exc:
            raise ValueError(
                f'{name} must contain exactly 6 values') from exc
        if len(pose) != 6:
            raise ValueError(f'{name} must contain exactly 6 values')
        if any(
            isinstance(value, bool) or not isinstance(value, (int, float))
            for value in pose
        ):
            raise ValueError(f'{name} must contain numeric values')
        pose = tuple(float(value) for value in pose)
        if not all(math.isfinite(value) for value in pose):
            raise ValueError(f'{name} must contain finite values')
        return pose

    @staticmethod
    def validate_team(team):
        if team not in (
            StageThree.Request.RED,
            StageThree.Request.BLUE,
        ):
            raise ValueError(f'team must be red or blue, got {team!r}')

    @staticmethod
    def validate_loaded_count(loaded_count):
        if loaded_count not in (1, 2, 3):
            raise ValueError(
                f'loaded_count must be 1, 2 or 3, got {loaded_count}')

    @staticmethod
    def validate_derived_config(config, blue_relocalization_pose):
        derived_values = (
            config['stage_two_exit_endpoint_x'] +
            config['first_target_x_offset'] +
            2.0 * config['target_x_spacing'],
            blue_relocalization_pose[1] + config['target_y_offset'],
        )
        if not all(math.isfinite(value) for value in derived_values):
            raise ValueError('derived Stage 3 target pose must be finite')

    def _on_parameters_changed(self, parameters):
        updates = {parameter.name: parameter.value for parameter in parameters}
        with self.config_lock:
            new_config = dict(self.config)
            new_pose = self.blue_relocalization_pose
            try:
                for name in self.POSITIVE_PARAMETERS:
                    if name in updates:
                        new_config[name] = self.validate_positive(
                            updates[name], name)
                for name in self.FINITE_PARAMETERS:
                    if name in updates:
                        new_config[name] = self.validate_finite(
                            updates[name], name)
                for name in self.NON_NEGATIVE_PARAMETERS:
                    if name in updates:
                        new_config[name] = self.validate_non_negative(
                            updates[name], name)
                if 'blue_relocalization_pose' in updates:
                    new_pose = self.validate_pose(
                        updates['blue_relocalization_pose'])
                self.validate_derived_config(new_config, new_pose)
            except (TypeError, ValueError) as exc:
                return SetParametersResult(
                    successful=False,
                    reason=str(exc),
                )
            self.config = new_config
            self.blue_relocalization_pose = new_pose
        return SetParametersResult(successful=True)

    def task_config(self, team, loaded_count):
        self.validate_team(team)
        self.validate_loaded_count(loaded_count)
        with self.config_lock:
            config = dict(self.config)
            blue_relocalization_pose = self.blue_relocalization_pose

        relocalization_pose = list(blue_relocalization_pose)
        blue_target_y = (
            blue_relocalization_pose[1] + config['target_y_offset'])
        target_y = blue_target_y
        if team == StageThree.Request.RED:
            relocalization_pose[1] = -relocalization_pose[1]
            relocalization_pose[5] = -relocalization_pose[5]
            target_y = -blue_target_y

        target_yaw = relocalization_pose[5]

        first_target_x = (
            config['stage_two_exit_endpoint_x'] +
            config['first_target_x_offset']
        )
        targets = tuple(
            (
                first_target_x + index * config['target_x_spacing'],
                target_y,
                target_yaw,
            )
            for index in range(loaded_count)
        )
        full_pop_plan = (
            (KfsAction.Request.MODE_1, False),
            (KfsAction.Request.MODE_2, False),
            (KfsAction.Request.MODE_2, True),
        )
        pop_plan = full_pop_plan[3 - loaded_count:]
        if loaded_count == 1:
            backoff_distances = (
                config['single_final_backoff_distance'],)
        else:
            backoff_distances = (
                (config['intermediate_backoff_distance'],) *
                (loaded_count - 1) +
                (config['standard_final_backoff_distance'],)
            )
        return {
            **config,
            'relocalization_pose': tuple(relocalization_pose),
            'targets': targets,
            'pop_plan': pop_plan,
            'backoff_distances': backoff_distances,
        }

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

    @staticmethod
    def wait_for_service(client, timeout_sec, description):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f'{description} service unavailable')

    def relocalize(self, pose, timeout_sec):
        request = SetBasePose.Request()
        (
            request.x,
            request.y,
            request.z,
            request.roll,
            request.pitch,
            request.yaw,
        ) = pose
        response = self.wait_for_future(
            self.set_base_pose_client.call_async(request),
            timeout_sec,
            'SetBasePose',
        )
        if not response.success:
            raise RuntimeError(f'SetBasePose failed: {response.message}')

    def move_to_pose(self, target, timeout_sec):
        request = MoveToPose.Request()
        request.pose_source = MoveToPose.Request.ODIN
        request.x, request.y, request.yaw = target
        request.position_tolerance = 0.0
        request.yaw_tolerance = 0.0
        request.timeout_sec = timeout_sec
        response = self.wait_for_future(
            self.move_to_pose_client.call_async(request),
            timeout_sec,
            'MoveToPose',
        )
        if not response.success:
            raise RuntimeError(f'MoveToPose failed: {response.message}')

    def move_backward(self, distance, timeout_sec):
        request = MoveRelative.Request()
        request.pose_source = MoveRelative.Request.ODIN
        request.forward = -float(distance)
        request.left = 0.0
        request.yaw_delta = 0.0
        request.position_tolerance = 0.0
        request.yaw_tolerance = 0.0
        request.timeout_sec = timeout_sec
        response = self.wait_for_future(
            self.move_relative_client.call_async(request),
            timeout_sec,
            'MoveRelative',
        )
        if not response.success:
            raise RuntimeError(f'MoveRelative failed: {response.message}')

    def pop_kfs(self, mode, timeout_sec):
        request = KfsAction.Request()
        request.action = KfsAction.Request.POP
        request.mode = mode
        response = self.wait_for_future(
            self.kfs_action_client.call_async(request),
            timeout_sec,
            'KfsAction pop',
        )
        if not response.success:
            raise RuntimeError(f'KfsAction pop failed: {response.message}')

    def lift_kfs(self, height, tolerance, timeout_sec):
        request = SetJointPosition.Request()
        request.position = height
        request.tolerance = tolerance
        request.timeout_sec = timeout_sec
        response = self.wait_for_future(
            self.kfs_lift_client.call_async(request),
            timeout_sec,
            'KfsLift',
        )
        if not response.success:
            raise RuntimeError(f'KfsLift failed: {response.message}')

    def wait_for_action_dependencies(self, timeout_sec):
        dependencies = (
            (self.move_to_pose_client, 'MoveToPose'),
            (self.move_relative_client, 'MoveRelative'),
            (self.kfs_action_client, 'KfsAction'),
            (self.kfs_lift_client, 'KfsLift'),
        )
        for client, description in dependencies:
            self.wait_for_service(client, timeout_sec, description)

    def handle_task(self, request, response):
        with self.service_lock:
            try:
                config = self.task_config(
                    request.team, int(request.loaded_count))
                dependency_timeout = config['dependency_timeout_sec']
                self.wait_for_service(
                    self.set_base_pose_client,
                    dependency_timeout,
                    'SetBasePose',
                )
                self.relocalize(
                    config['relocalization_pose'],
                    config['relocalization_timeout_sec'],
                )
                self.wait_for_action_dependencies(dependency_timeout)

                for target, pop_step, backoff_distance in zip(
                    config['targets'],
                    config['pop_plan'],
                    config['backoff_distances'],
                ):
                    pop_mode, lift_before_pop = pop_step
                    self.move_to_pose(target, config['move_timeout_sec'])
                    if lift_before_pop:
                        self.lift_kfs(
                            config['kfs_lift_height'],
                            config['kfs_lift_tolerance'],
                            config['kfs_lift_timeout_sec'],
                        )
                    self.pop_kfs(pop_mode, config['pop_timeout_sec'])
                    self.move_backward(
                        backoff_distance, config['move_timeout_sec'])
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                return response

            final_target = config['targets'][-1]
            response.success = True
            response.message = (
                f'Stage 3 completed for {request.team} team with '
                f'{request.loaded_count} KFS at '
                f'({final_target[0]:.3f}, {final_target[1]:.3f}, '
                f'{final_target[2]:.3f})')
            return response


def main(args=None):
    rclpy.init(args=args)
    node = StageThreeController()
    executor = MultiThreadedExecutor(num_threads=2)
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
