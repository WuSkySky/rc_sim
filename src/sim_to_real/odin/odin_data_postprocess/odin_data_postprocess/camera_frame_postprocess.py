"""Convert the Odin undistorted Image stream to bounded CameraFrame."""

from __future__ import annotations

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from robot_r2_interfaces.msg import CameraFrame
from sensor_msgs.msg import Image


INPUT_IMAGE_TOPIC = '/odin1/image/undistorted'
OUTPUT_IMAGE_TOPIC = '/r2/rear_camera/image_raw'
DEBUG_IMAGE_TOPIC = '/r2/rear_camera/image_raw/debug'


def image_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class CameraFramePostprocess(Node):
    """Own the standard-to-bounded boundary after the Odin driver."""

    def __init__(self) -> None:
        super().__init__('camera_frame_postprocess')
        self.declare_parameter('visualization_enabled', False)

        self._visualization_enabled = bool(
            self.get_parameter('visualization_enabled').value)

        qos = image_qos()
        self._frame = CameraFrame()
        self._sequence = 0
        self._publisher = self.create_publisher(
            CameraFrame,
            OUTPUT_IMAGE_TOPIC,
            qos,
        )
        self._standard_publisher = self.create_publisher(
            Image,
            DEBUG_IMAGE_TOPIC,
            qos,
        )
        self._subscription = self.create_subscription(
            Image,
            INPUT_IMAGE_TOPIC,
            self._on_image,
            qos,
        )
        self.add_on_set_parameters_callback(
            self._on_parameters_changed)

        self.get_logger().info(
            f'Converting Odin images from {INPUT_IMAGE_TOPIC} '
            f'to {OUTPUT_IMAGE_TOPIC}')

    def _on_parameters_changed(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name != 'visualization_enabled':
                continue
            if not isinstance(parameter.value, bool):
                return SetParametersResult(
                    successful=False,
                    reason='visualization_enabled must be a boolean',
                )
            self._visualization_enabled = parameter.value
            state = 'enabled' if parameter.value else 'disabled'
            self.get_logger().info(
                f'Odin postprocess debug image publication {state}')
        return SetParametersResult(successful=True)

    def _on_image(self, source: Image) -> None:
        try:
            encoding, channels = self._encoding(source.encoding)
            width = int(source.width)
            height = int(source.height)
            step = int(source.step)
            if width <= 0 or height <= 0:
                raise ValueError('image height and width must be positive')
            minimum_step = width * channels
            if step < minimum_step:
                raise ValueError(
                    f'image step {step} is smaller than {minimum_step}')
            data_size = height * step
            if data_size != len(source.data):
                raise ValueError(
                    f'image data has {len(source.data)} bytes, '
                    f'expected {data_size}')
            if data_size > CameraFrame.DATA_CAPACITY:
                raise ValueError(
                    f'image data has {data_size} bytes, capacity is '
                    f'{CameraFrame.DATA_CAPACITY}')

            frame_id = source.header.frame_id.encode('utf-8')
            if len(frame_id) > CameraFrame.FRAME_ID_CAPACITY:
                raise ValueError(
                    f'frame_id has {len(frame_id)} bytes, capacity is '
                    f'{CameraFrame.FRAME_ID_CAPACITY}')

            frame = self._frame
            frame.sequence = self._sequence
            frame.stamp_sec = int(source.header.stamp.sec)
            frame.stamp_nanosec = int(source.header.stamp.nanosec)
            frame.width = width
            frame.height = height
            frame.step = step
            frame.data_size = data_size
            frame.encoding = encoding
            frame.is_bigendian = int(source.is_bigendian)
            frame.layout_version = CameraFrame.LAYOUT_VERSION
            frame.frame_id_size = len(frame_id)
            frame.frame_id.fill(0)
            for index, value in enumerate(frame_id):
                frame.frame_id[index] = value
            del frame.data[:]
            frame.data.frombytes(source.data)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(
                f'Dropping invalid Odin image: {exc}')
            return

        self._publisher.publish(frame)
        if self._visualization_enabled:
            self._standard_publisher.publish(source)
        self._sequence += 1

    @staticmethod
    def _encoding(name: str) -> tuple[int, int]:
        normalized = name.lower()
        if normalized == 'bgr8':
            return CameraFrame.ENCODING_BGR8, 3
        if normalized == 'rgb8':
            return CameraFrame.ENCODING_RGB8, 3
        if normalized == 'mono8':
            return CameraFrame.ENCODING_MONO8, 1
        raise ValueError(f'unsupported image encoding: {name}')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraFramePostprocess()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
