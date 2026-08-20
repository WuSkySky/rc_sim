"""Convert the Odin undistorted Image stream to bounded CameraFrame."""

from __future__ import annotations

import math
import threading
import time

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
MAX_OUTPUT_RATE_HZ = 15.0


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
        self._config_lock = threading.Lock()
        self.declare_parameter('visualization_enabled', False)
        self.declare_parameter('max_publish_rate', 15.0)

        self._visualization_enabled = bool(
            self.get_parameter('visualization_enabled').value)
        max_publish_rate = float(
            self.get_parameter('max_publish_rate').value)
        if (
            not math.isfinite(max_publish_rate)
            or not 0.0 < max_publish_rate <= MAX_OUTPUT_RATE_HZ
        ):
            raise ValueError(
                'max_publish_rate must be finite and in (0, 15]')
        self._max_publish_rate = max_publish_rate
        self._publish_period_sec = 1.0 / max_publish_rate
        self._next_publish_at = 0.0

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
        visualization_enabled = None
        max_publish_rate = None
        for parameter in parameters:
            if parameter.name == 'max_publish_rate':
                if isinstance(parameter.value, bool) or not isinstance(
                    parameter.value, (int, float)
                ):
                    return SetParametersResult(
                        successful=False,
                        reason='max_publish_rate must be a number',
                    )
                max_publish_rate = float(parameter.value)
                if (
                    not math.isfinite(max_publish_rate)
                    or not 0.0 < max_publish_rate <= MAX_OUTPUT_RATE_HZ
                ):
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            'max_publish_rate must be finite and in (0, 15]'
                        ),
                    )
                continue
            if parameter.name != 'visualization_enabled':
                continue
            if not isinstance(parameter.value, bool):
                return SetParametersResult(
                    successful=False,
                    reason='visualization_enabled must be a boolean',
                )
            visualization_enabled = parameter.value

        if visualization_enabled is not None:
            with self._config_lock:
                self._visualization_enabled = visualization_enabled
            state = 'enabled' if visualization_enabled else 'disabled'
            self.get_logger().info(
                f'Odin postprocess debug image publication {state}')
        if max_publish_rate is not None:
            with self._config_lock:
                self._max_publish_rate = max_publish_rate
                self._publish_period_sec = 1.0 / max_publish_rate
                self._next_publish_at = 0.0
            self.get_logger().info(
                'Odin rear CameraFrame rate limit changed to '
                f'{max_publish_rate:g} Hz')
        return SetParametersResult(successful=True)

    def _on_image(self, source: Image) -> None:
        now = time.monotonic()
        with self._config_lock:
            if now < self._next_publish_at:
                return
            visualization_enabled = self._visualization_enabled
            self._next_publish_at = now + self._publish_period_sec

        # Avoid packing the large bounded sample while neither LED detection
        # nor another CameraFrame reader is active.
        if (
            self._publisher.get_subscription_count() == 0
            and not visualization_enabled
        ):
            return

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
        if visualization_enabled:
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
