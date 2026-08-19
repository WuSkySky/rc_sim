"""TF post-processing for the lower-machine chassis odometry.

``serial_bridge`` publishes the lower-machine odometry pose as a
``nav_msgs/Odometry`` message carrying ``odom_serial -> base_link_serial``.
This node keeps the ``map -> odom_serial`` transform so
``map -> base_link_serial`` equals the default or service-reset base pose,
broadcasts both transforms, and publishes ``/r2/pose_feedback``.

The pose topic and reset service remain the canonical interfaces used by the
control stack. Its TF frames are source-specific so the Odin branch can run at
the same time without creating multiple parents for one frame.
"""

import math
import threading
from collections.abc import Sequence
from numbers import Real

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_r2_interfaces.srv import SetBasePose
from tf2_ros import TransformBroadcaster

from serial_pkg.frame_ids import (
    BASE_FRAME as SERIAL_BASE_FRAME,
    MAP_FRAME as SERIAL_MAP_FRAME,
    ODOM_FRAME as SERIAL_ODOM_FRAME,
)
from serial_pkg.transform_utils import (
    compose_transforms,
    invert_transform,
    is_finite,
    normalize_quaternion,
    quaternion_from_rpy,
)


DEFAULT_PUBLISH_RATE = 50.0


def validate_vector(name, values):
    if (
        not isinstance(values, Sequence)
        or isinstance(values, (str, bytes))
        or len(values) != 3
    ):
        raise ValueError(f'{name} must contain exactly three values')
    if any(
        isinstance(value, bool) or not isinstance(value, Real)
        for value in values
    ):
        raise ValueError(f'{name} values must be numbers')

    vector = tuple(float(value) for value in values)
    if not is_finite(vector):
        raise ValueError(f'{name} values must be finite')
    return vector


def validate_publish_rate(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError('publish_rate must be a number')
    publish_rate = float(value)
    if not math.isfinite(publish_rate) or publish_rate <= 0.0:
        raise ValueError('publish_rate must be finite and positive')
    return publish_rate


class OdometryTf(Node):
    INPUT_ODOMETRY_TOPIC = '/r2/odometry'
    OUTPUT_POSE_TOPIC = '/r2/pose_feedback'
    SET_BASE_POSE_SERVICE = '/r2/set_base_pose'

    MAP_FRAME = SERIAL_MAP_FRAME
    ODOM_FRAME = SERIAL_ODOM_FRAME
    BASE_FRAME = SERIAL_BASE_FRAME

    def __init__(self):
        super().__init__('odometry_tf')

        self.declare_parameter('default_base_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('default_base_rpy', [0.0, 0.0, 0.0])
        self.declare_parameter('publish_rate', DEFAULT_PUBLISH_RATE)

        default_base_translation = self._read_vector_parameter(
            'default_base_xyz')
        default_base_rpy = self._read_vector_parameter('default_base_rpy')
        publish_rate = validate_publish_rate(
            self.get_parameter('publish_rate').value)

        self._lock = threading.RLock()
        self._default_base_translation = default_base_translation
        self._default_base_rotation = normalize_quaternion(
            quaternion_from_rpy(*default_base_rpy)
        )
        self._latest_odom_to_base = None
        self._map_to_odom = None
        self._waiting_for_odometry_logged = False

        self._transform_broadcaster = TransformBroadcaster(self)
        self._pose_publisher = self.create_publisher(
            PoseStamped,
            self.OUTPUT_POSE_TOPIC,
            10,
        )
        odometry_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._odometry_subscription = self.create_subscription(
            Odometry,
            self.INPUT_ODOMETRY_TOPIC,
            self._on_odometry,
            odometry_qos,
        )
        self._set_base_pose_service = self.create_service(
            SetBasePose,
            self.SET_BASE_POSE_SERVICE,
            self._on_set_base_pose,
        )
        self._publish_timer = self.create_timer(
            1.0 / publish_rate,
            self._publish,
        )
        self.add_on_set_parameters_callback(self._on_parameters_changed)

    def _read_vector_parameter(self, name):
        return validate_vector(name, self.get_parameter(name).value)

    @staticmethod
    def _calculate_map_to_odom(target_translation, target_rotation,
                               odom_to_base):
        base_to_odom_translation, base_to_odom_rotation = invert_transform(
            odom_to_base[0],
            odom_to_base[1],
        )
        translation, rotation = compose_transforms(
            target_translation,
            target_rotation,
            base_to_odom_translation,
            base_to_odom_rotation,
        )
        if (
            not is_finite(translation)
            or rotation is None
            or not is_finite(rotation)
        ):
            return None
        return translation, rotation

    def _on_odometry(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        translation = (position.x, position.y, position.z)
        rotation = normalize_quaternion((
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ))

        with self._lock:
            if not is_finite(translation) or rotation is None:
                self._latest_odom_to_base = None
                return
            self._latest_odom_to_base = (
                translation,
                rotation,
                message.header.stamp,
            )
            self._waiting_for_odometry_logged = False

    def _publish(self):
        initialized_default_pose = False
        with self._lock:
            odom_to_base = self._latest_odom_to_base
            map_to_odom = self._map_to_odom

            if odom_to_base is None:
                if not self._waiting_for_odometry_logged:
                    self.get_logger().info(
                        f'Waiting for odometry on '
                        f'{self.INPUT_ODOMETRY_TOPIC}'
                    )
                    self._waiting_for_odometry_logged = True
            elif map_to_odom is None:
                map_to_odom = self._calculate_map_to_odom(
                    self._default_base_translation,
                    self._default_base_rotation,
                    odom_to_base,
                )
                self._map_to_odom = map_to_odom
                initialized_default_pose = map_to_odom is not None

        if initialized_default_pose:
            self.get_logger().info(
                f'Initialized {self.BASE_FRAME} pose from default parameters'
            )
        if map_to_odom is not None:
            self._broadcast_map_to_odom(map_to_odom)
        if odom_to_base is None or map_to_odom is None:
            return

        self._broadcast_odom_to_base(odom_to_base)
        self._publish_base_pose(map_to_odom, odom_to_base)

    def _on_set_base_pose(self, request, response):
        target_translation = (request.x, request.y, request.z)
        target_rpy = (request.roll, request.pitch, request.yaw)
        if not is_finite(target_translation + target_rpy):
            response.success = False
            response.message = (
                f'target {self.BASE_FRAME} pose must contain finite values'
            )
            return response

        target_rotation = normalize_quaternion(
            quaternion_from_rpy(*target_rpy)
        )
        with self._lock:
            odom_to_base = self._latest_odom_to_base
            if odom_to_base is None:
                response.success = False
                response.message = (
                    f'failed to read current {self.BASE_FRAME} pose'
                )
                return response
            map_to_odom = self._calculate_map_to_odom(
                target_translation,
                target_rotation,
                odom_to_base,
            )
            if map_to_odom is None:
                response.success = False
                response.message = (
                    'failed to calculate map to odom transform')
                return response
            self._map_to_odom = map_to_odom

        self._broadcast_map_to_odom(map_to_odom)
        response.success = True
        response.message = f'{self.BASE_FRAME} pose updated'
        return response

    def _on_parameters_changed(self, parameters):
        values = {parameter.name: parameter.value for parameter in parameters}
        with self._lock:
            default_base_translation = self._default_base_translation
            default_base_rotation = self._default_base_rotation
            current_rate = 1.0 / (
                self._publish_timer.timer_period_ns / 1.0e9)

        try:
            if 'default_base_xyz' in values:
                default_base_translation = validate_vector(
                    'default_base_xyz', values['default_base_xyz'])
            if 'default_base_rpy' in values:
                default_base_rpy = validate_vector(
                    'default_base_rpy', values['default_base_rpy'])
                default_base_rotation = normalize_quaternion(
                    quaternion_from_rpy(*default_base_rpy)
                )
            publish_rate = validate_publish_rate(
                values.get('publish_rate', current_rate))
        except ValueError as error:
            return SetParametersResult(
                successful=False,
                reason=str(error),
            )

        default_base_changed = (
            'default_base_xyz' in values or 'default_base_rpy' in values
        )
        map_to_odom = None
        with self._lock:
            self._default_base_translation = default_base_translation
            self._default_base_rotation = default_base_rotation
            if default_base_changed:
                self._map_to_odom = None
                if self._latest_odom_to_base is not None:
                    map_to_odom = self._calculate_map_to_odom(
                        default_base_translation,
                        default_base_rotation,
                        self._latest_odom_to_base,
                    )
                    self._map_to_odom = map_to_odom

            new_period = 1.0 / publish_rate
            current_period = self._publish_timer.timer_period_ns / 1.0e9
            if abs(new_period - current_period) > 1.0e-9:
                old_timer = self._publish_timer
                old_timer.cancel()
                self._publish_timer = self.create_timer(
                    new_period,
                    self._publish,
                )
                self.destroy_timer(old_timer)

        if map_to_odom is not None:
            self._broadcast_map_to_odom(map_to_odom)
        return SetParametersResult(successful=True)

    def _broadcast_odom_to_base(self, odom_to_base):
        transform = self._make_transform(
            self.ODOM_FRAME,
            self.BASE_FRAME,
            odom_to_base[0],
            odom_to_base[1],
            odom_to_base[2],
        )
        self._transform_broadcaster.sendTransform(transform)

    def _broadcast_map_to_odom(self, map_to_odom):
        transform = self._make_transform(
            self.MAP_FRAME,
            self.ODOM_FRAME,
            map_to_odom[0],
            map_to_odom[1],
            self.get_clock().now().to_msg(),
        )
        self._transform_broadcaster.sendTransform(transform)

    def _publish_base_pose(self, map_to_odom, odom_to_base):
        translation, rotation = compose_transforms(
            map_to_odom[0],
            map_to_odom[1],
            odom_to_base[0],
            odom_to_base[1],
        )
        pose = PoseStamped()
        pose.header.stamp = odom_to_base[2]
        pose.header.frame_id = self.MAP_FRAME
        pose.pose.position.x = translation[0]
        pose.pose.position.y = translation[1]
        pose.pose.position.z = translation[2]
        pose.pose.orientation.x = rotation[0]
        pose.pose.orientation.y = rotation[1]
        pose.pose.orientation.z = rotation[2]
        pose.pose.orientation.w = rotation[3]
        self._pose_publisher.publish(pose)

    @staticmethod
    def _make_transform(parent_frame, child_frame, translation, rotation,
                        stamp):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]
        return transform


def main():
    rclpy.init()
    node = OdometryTf()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
