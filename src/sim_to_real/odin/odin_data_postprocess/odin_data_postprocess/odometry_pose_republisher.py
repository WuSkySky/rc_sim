import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class OdometryPoseRepublisher(Node):
    def __init__(self):
        super().__init__('odometry_pose_republisher')

        self.declare_parameter('output_pose_topic', '/r2/pose_feedback')
        self.declare_parameter('fixed_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate', 100.0)

        output_topic = str(
            self.get_parameter('output_pose_topic').value)
        self.fixed_frame = str(self.get_parameter('fixed_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        if not output_topic:
            raise ValueError('output_pose_topic must not be empty')
        if not self.fixed_frame or not self.base_frame:
            raise ValueError('TF frame names must not be empty')
        if self.fixed_frame == self.base_frame:
            raise ValueError('fixed_frame and base_frame must differ')
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError('publish_rate must be finite and positive')

        self.pose_publisher = self.create_publisher(
            PoseStamped, output_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publish_timer = self.create_timer(
            1.0 / publish_rate, self.publish_base_pose)

    def publish_base_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                self.base_frame,
                Time(),
            )
        except TransformException:
            return

        pose = PoseStamped()
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = self.fixed_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self.pose_publisher.publish(pose)


def main():
    rclpy.init()
    node = OdometryPoseRepublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
