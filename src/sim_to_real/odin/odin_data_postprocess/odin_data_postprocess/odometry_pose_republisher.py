from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class OdometryPoseRepublisher(Node):
    def __init__(self):
        super().__init__('odometry_pose_republisher')

        self.declare_parameter(
            'input_odometry_topic', '/odin1/odometry_highfreq')
        self.declare_parameter('output_pose_topic', '/r2/pose_feedback')
        self.declare_parameter('publish_rate', 100.0)

        input_topic = str(
            self.get_parameter('input_odometry_topic').value)
        output_topic = str(
            self.get_parameter('output_pose_topic').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be positive')

        odometry_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pose_publisher = self.create_publisher(
            PoseStamped, output_topic, 10)
        self.odometry_subscription = self.create_subscription(
            Odometry,
            input_topic,
            self.on_odometry,
            odometry_qos,
        )
        self.publish_timer = self.create_timer(
            1.0 / publish_rate, self.publish_latest_pose)

        self.latest_odometry = None

    def on_odometry(self, message):
        self.latest_odometry = message

    def publish_latest_pose(self):
        odometry = self.latest_odometry
        if odometry is None:
            return

        pose = PoseStamped()
        pose.header = odometry.header
        pose.pose = odometry.pose.pose
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
