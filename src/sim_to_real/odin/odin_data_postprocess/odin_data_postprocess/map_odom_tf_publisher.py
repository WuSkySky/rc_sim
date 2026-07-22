import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from robot_r2_interfaces.srv import SetBasePose
from tf2_ros import Buffer, TransformBroadcaster, TransformException
from tf2_ros import TransformListener

from odin_data_postprocess.transform_utils import (
    compose_transforms,
    invert_transform,
    is_finite,
    normalize_quaternion,
    quaternion_from_rpy,
)


class MapOdomTfPublisher(Node):
    def __init__(self):
        super().__init__('map_odom_tf_publisher')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('default_base_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('default_base_rpy', [0.0, 0.0, 0.0])
        self.declare_parameter('service_name', '/r2/set_base_pose')
        self.declare_parameter('publish_rate', 100.0)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.default_base_translation = self._read_vector_parameter(
            'default_base_xyz')
        self.default_base_rpy = self._read_vector_parameter(
            'default_base_rpy')
        service_name = str(self.get_parameter('service_name').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        if not all((self.map_frame, self.odom_frame, self.base_frame)):
            raise ValueError('TF frame names must not be empty')
        if len({self.map_frame, self.odom_frame, self.base_frame}) != 3:
            raise ValueError('TF frame names must be unique')
        if not service_name:
            raise ValueError('service_name must not be empty')
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError('publish_rate must be finite and positive')

        self.default_base_rotation = normalize_quaternion(
            quaternion_from_rpy(*self.default_base_rpy)
        )
        self.map_to_odom = None
        self.waiting_for_tf_logged = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.transform_broadcaster = TransformBroadcaster(self)
        self.service = self.create_service(
            SetBasePose,
            service_name,
            self.on_set_base_pose,
        )
        self.publish_timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_transform,
        )

    def _read_vector_parameter(self, name):
        values = self.get_parameter(name).value
        if len(values) != 3:
            raise ValueError(f'{name} must contain exactly three values')

        vector = tuple(float(value) for value in values)
        if not is_finite(vector):
            raise ValueError(f'{name} values must be finite')
        return vector

    def _calculate_map_to_odom(self, target_translation, target_rotation):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
            )
        except TransformException as error:
            return False, str(error)

        current_translation = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )
        current_rotation = normalize_quaternion((
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ))
        if not is_finite(current_translation) or current_rotation is None:
            return False, (
                f'{self.odom_frame} to {self.base_frame} TF contains an '
                'invalid pose'
            )

        base_to_odom_translation, base_to_odom_rotation = invert_transform(
            current_translation,
            current_rotation,
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
            return False, 'calculated map to odom TF is invalid'

        self.map_to_odom = (translation, rotation)
        return True, ''

    def on_set_base_pose(self, request, response):
        target_translation = (request.x, request.y, request.z)
        target_rpy = (request.roll, request.pitch, request.yaw)
        if not is_finite(target_translation + target_rpy):
            response.success = False
            response.message = (
                f'target {self.base_frame} pose must contain finite values'
            )
            return response

        target_rotation = normalize_quaternion(
            quaternion_from_rpy(*target_rpy)
        )
        success, message = self._calculate_map_to_odom(
            target_translation,
            target_rotation,
        )
        response.success = success
        if success:
            response.message = f'{self.base_frame} pose updated'
            self.waiting_for_tf_logged = False
            self._broadcast_transform()
        else:
            response.message = (
                f'failed to read current {self.base_frame} pose: {message}'
            )
        return response

    def publish_transform(self):
        if self.map_to_odom is None:
            success, message = self._calculate_map_to_odom(
                self.default_base_translation,
                self.default_base_rotation,
            )
            if not success:
                if not self.waiting_for_tf_logged:
                    self.get_logger().info(
                        f'Waiting for {self.odom_frame} to '
                        f'{self.base_frame} TF: {message}'
                    )
                    self.waiting_for_tf_logged = True
                return
            self.waiting_for_tf_logged = False
            self.get_logger().info(
                f'Initialized {self.base_frame} pose from default parameters'
            )

        self._broadcast_transform()

    def _broadcast_transform(self):
        translation, rotation = self.map_to_odom
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]
        self.transform_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = MapOdomTfPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
