import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from odin_data_postprocess.transform_utils import (
    compose_transforms,
    invert_transform,
    is_finite,
    normalize_quaternion,
    quaternion_from_rpy,
)


class OdometryTfPublisher(Node):
    def __init__(self):
        super().__init__('odometry_tf_publisher')

        self.declare_parameter(
            'input_odometry_topic', '/odin1/odometry_highfreq')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odin_frame', 'odin_link')
        self.declare_parameter('base_to_odin_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('base_to_odin_rpy', [0.0, 0.0, 0.0])
        self.declare_parameter('publish_rate', 100.0)

        input_topic = str(
            self.get_parameter('input_odometry_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.odin_frame = str(self.get_parameter('odin_frame').value)
        base_to_odin_translation = self._read_vector_parameter(
            'base_to_odin_xyz')
        base_to_odin_rpy = self._read_vector_parameter('base_to_odin_rpy')
        publish_rate = float(self.get_parameter('publish_rate').value)

        if not input_topic:
            raise ValueError('input_odometry_topic must not be empty')
        if not all((self.odom_frame, self.base_frame, self.odin_frame)):
            raise ValueError('TF frame names must not be empty')
        if len({self.odom_frame, self.base_frame, self.odin_frame}) != 3:
            raise ValueError('TF frame names must be unique')
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError('publish_rate must be finite and positive')

        base_to_odin_rotation = normalize_quaternion(
            quaternion_from_rpy(*base_to_odin_rpy)
        )
        (
            self.odin_to_base_translation,
            self.odin_to_base_rotation,
        ) = invert_transform(
            base_to_odin_translation,
            base_to_odin_rotation,
        )

        self.latest_odometry = None
        self.transform_broadcaster = TransformBroadcaster(self)
        self.static_transform_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_transform(
            base_to_odin_translation,
            base_to_odin_rotation,
        )

        odometry_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.odometry_subscription = self.create_subscription(
            Odometry,
            input_topic,
            self.on_odometry,
            odometry_qos,
        )
        self.publish_timer = self.create_timer(
            1.0 / publish_rate, self.publish_latest_transform)

    def _read_vector_parameter(self, name):
        values = self.get_parameter(name).value
        if len(values) != 3:
            raise ValueError(f'{name} must contain exactly three values')

        vector = tuple(float(value) for value in values)
        if not is_finite(vector):
            raise ValueError(f'{name} values must be finite')
        return vector

    def _publish_static_transform(self, translation, rotation):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.odin_frame
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]
        self.static_transform_broadcaster.sendTransform(transform)

    def on_odometry(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        translation = (position.x, position.y, position.z)
        rotation = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        if not is_finite(translation) or not is_finite(rotation):
            self.latest_odometry = None
            return
        if normalize_quaternion(rotation) is None:
            self.latest_odometry = None
            return

        self.latest_odometry = message

    def publish_latest_transform(self):
        odometry = self.latest_odometry
        if odometry is None:
            return

        position = odometry.pose.pose.position
        orientation = odometry.pose.pose.orientation
        odom_to_odin_translation = (position.x, position.y, position.z)
        odom_to_odin_rotation = normalize_quaternion((
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ))
        if odom_to_odin_rotation is None:
            self.latest_odometry = None
            return

        # T_odom_base = T_odom_odin * inverse(T_base_odin). Odin's
        # child_frame_id is intentionally ignored.
        odom_to_base_translation, odom_to_base_rotation = compose_transforms(
            odom_to_odin_translation,
            odom_to_odin_rotation,
            self.odin_to_base_translation,
            self.odin_to_base_rotation,
        )
        if (
            not is_finite(odom_to_base_translation)
            or odom_to_base_rotation is None
            or not is_finite(odom_to_base_rotation)
        ):
            self.latest_odometry = None
            return

        transform = TransformStamped()
        transform.header.stamp = odometry.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = odom_to_base_translation[0]
        transform.transform.translation.y = odom_to_base_translation[1]
        transform.transform.translation.z = odom_to_base_translation[2]
        transform.transform.rotation.x = odom_to_base_rotation[0]
        transform.transform.rotation.y = odom_to_base_rotation[1]
        transform.transform.rotation.z = odom_to_base_rotation[2]
        transform.transform.rotation.w = odom_to_base_rotation[3]
        self.transform_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = OdometryTfPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
