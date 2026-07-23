#!/usr/bin/env python3
"""Align the chassis with red or blue KFS regions in the front image."""

from __future__ import annotations

import math
import threading
import time

import cv2
from geometry_msgs.msg import Twist
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_r2_interfaces.srv import AlignToKFS
from sensor_msgs.msg import Image


def image_message_to_bgr(msg: Image) -> np.ndarray:
    """Convert a packed rgb8/bgr8 ROS image into owned contiguous BGR data."""
    encoding = msg.encoding.lower()
    if encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'unsupported image encoding: {msg.encoding}')
    if msg.height <= 0 or msg.width <= 0:
        raise ValueError('image height and width must be positive')

    row_size = int(msg.width) * 3
    if msg.step < row_size:
        raise ValueError(
            f'image step {msg.step} is smaller than row size {row_size}')
    expected_size = int(msg.height) * int(msg.step)
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if data.size != expected_size:
        raise ValueError(
            f'image data has {data.size} bytes, expected {expected_size}')

    rows = data.reshape(int(msg.height), int(msg.step))
    image = rows[:, :row_size].reshape(int(msg.height), int(msg.width), 3)
    if encoding == 'rgb8':
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return np.ascontiguousarray(image)


class KfsAlignmentController(Node):
    def __init__(self):
        super().__init__('kfs_alignment')

        self.service_lock = threading.Lock()
        self.state_condition = threading.Condition()
        self._image_callback_group = MutuallyExclusiveCallbackGroup()
        self._service_callback_group = ReentrantCallbackGroup()

        self._declare_parameters()
        self._load_parameters()

        self._frame_sequence = 0
        self._latest_offset_x: int | None = None
        self._latest_frame_started_at = 0.0
        self._latest_frame_time = 0.0
        self._alignment_active = False

        # Depth one intentionally discards queued stale images while image
        # processing is slower than the camera publisher.
        self._sub = self.create_subscription(
            Image,
            self._color_topic,
            self._on_image,
            1,
            callback_group=self._image_callback_group,
        )
        self._pub_cmd = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._pub_visualization = self.create_publisher(
            Image, '/r2/alignment/viz', 1)
        self._srv = self.create_service(
            AlignToKFS,
            self._align_service,
            self._handle_align,
            callback_group=self._service_callback_group,
        )
        self.add_on_set_parameters_callback(self._on_parameters_changed)

    # ---- parameters ------------------------------------------------

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            'color_topic', '/r2/front_camera/image_raw')
        self.declare_parameter('cmd_vel_topic', '/r2/cmd_vel')
        self.declare_parameter('align_service', '/r2/align_to_kfs')
        self.declare_parameter('pixel_tolerance', 5)
        self.declare_parameter('stable_cycles', 10)
        self.declare_parameter('default_timeout_sec', 10.0)
        self.declare_parameter('target_processing_rate', 30.0)
        self.declare_parameter('visualization_enabled', False)

        self.declare_parameter('blue_hsv_lower', [90, 80, 60])
        self.declare_parameter('blue_hsv_upper', [130, 255, 255])
        self.declare_parameter('red_low_hsv_lower', [0, 80, 60])
        self.declare_parameter('red_low_hsv_upper', [10, 255, 255])
        self.declare_parameter('red_high_hsv_lower', [170, 80, 60])
        self.declare_parameter('red_high_hsv_upper', [179, 255, 255])
        self.declare_parameter('column_threshold_ratio', 0.8)

        self.declare_parameter('kp', 0.008)
        self.declare_parameter('ki', 0.0003)
        self.declare_parameter('kd', 0.001)
        self.declare_parameter('integral_limit', 0.5)
        self.declare_parameter('output_limit', 1.0)

    def _load_parameters(self) -> None:
        self._color_topic = str(
            self.get_parameter('color_topic').value)
        self._cmd_vel_topic = str(
            self.get_parameter('cmd_vel_topic').value)
        self._align_service = str(
            self.get_parameter('align_service').value)
        self._pixel_tolerance = int(
            self.get_parameter('pixel_tolerance').value)
        self._stable_cycles = int(
            self.get_parameter('stable_cycles').value)
        self._default_timeout_sec = float(
            self.get_parameter('default_timeout_sec').value)
        target_processing_rate = float(
            self.get_parameter('target_processing_rate').value)
        self._visualization_enabled = bool(
            self.get_parameter('visualization_enabled').value)

        if self._pixel_tolerance < 0:
            raise ValueError('pixel_tolerance must be non-negative')
        if self._stable_cycles <= 0:
            raise ValueError('stable_cycles must be greater than zero')
        if (
            not math.isfinite(self._default_timeout_sec) or
            self._default_timeout_sec <= 0.0
        ):
            raise ValueError(
                'default_timeout_sec must be finite and positive')
        if (
            not math.isfinite(target_processing_rate) or
            target_processing_rate <= 0.0
        ):
            raise ValueError(
                'target_processing_rate must be finite and positive')
        self._target_processing_rate = target_processing_rate
        self._processing_deadline_sec = 1.0 / target_processing_rate

        blue_lower = self._hsv_parameter('blue_hsv_lower')
        blue_upper = self._hsv_parameter('blue_hsv_upper')
        red_low_lower = self._hsv_parameter('red_low_hsv_lower')
        red_low_upper = self._hsv_parameter('red_low_hsv_upper')
        red_high_lower = self._hsv_parameter('red_high_hsv_lower')
        red_high_upper = self._hsv_parameter('red_high_hsv_upper')
        self._validate_hsv_range(
            'blue_hsv', blue_lower, blue_upper)
        self._validate_hsv_range(
            'red_low_hsv', red_low_lower, red_low_upper)
        self._validate_hsv_range(
            'red_high_hsv', red_high_lower, red_high_upper)
        self._blue_hsv_lower = np.asarray(blue_lower, dtype=np.uint8)
        self._blue_hsv_upper = np.asarray(blue_upper, dtype=np.uint8)
        self._red_low_hsv_lower = np.asarray(
            red_low_lower, dtype=np.uint8)
        self._red_low_hsv_upper = np.asarray(
            red_low_upper, dtype=np.uint8)
        self._red_high_hsv_lower = np.asarray(
            red_high_lower, dtype=np.uint8)
        self._red_high_hsv_upper = np.asarray(
            red_high_upper, dtype=np.uint8)

        self._column_threshold_ratio = float(
            self.get_parameter('column_threshold_ratio').value)
        if (
            not math.isfinite(self._column_threshold_ratio) or
            not 0.0 < self._column_threshold_ratio <= 1.0
        ):
            raise ValueError(
                'column_threshold_ratio must be in the range (0, 1]')

        self._kp = float(self.get_parameter('kp').value)
        self._ki = float(self.get_parameter('ki').value)
        self._kd = float(self.get_parameter('kd').value)
        self._integral_limit = abs(
            float(self.get_parameter('integral_limit').value))
        self._output_limit = abs(
            float(self.get_parameter('output_limit').value))

    def _on_parameters_changed(self, parameters) -> SetParametersResult:
        visualization_enabled = None
        for parameter in parameters:
            if parameter.name != 'visualization_enabled':
                continue
            if not isinstance(parameter.value, bool):
                return SetParametersResult(
                    successful=False,
                    reason='visualization_enabled must be a boolean',
                )
            visualization_enabled = parameter.value

        if visualization_enabled is not None:
            with self.state_condition:
                self._visualization_enabled = visualization_enabled
            state = 'enabled' if visualization_enabled else 'disabled'
            self.get_logger().info(f'KFS alignment visualization {state}')

        return SetParametersResult(successful=True)

    def _hsv_parameter(self, name: str) -> tuple[int, int, int]:
        values = tuple(int(value) for value in self.get_parameter(name).value)
        if len(values) != 3:
            raise ValueError(f'{name} must contain [hue, saturation, value]')
        hue, saturation, value = values
        if not 0 <= hue <= 179:
            raise ValueError(f'{name} hue must be in the range [0, 179]')
        if not 0 <= saturation <= 255 or not 0 <= value <= 255:
            raise ValueError(
                f'{name} saturation and value must be in the range [0, 255]')
        return values

    @staticmethod
    def _validate_hsv_range(
        name: str,
        lower: tuple[int, int, int],
        upper: tuple[int, int, int],
    ) -> None:
        if any(low > high for low, high in zip(lower, upper)):
            raise ValueError(
                f'{name} lower bounds must not exceed upper bounds')

    # ---- image processing -----------------------------------------

    def _on_image(self, msg: Image) -> None:
        with self.state_condition:
            should_process = (
                self._visualization_enabled or self._alignment_active)
        if not should_process:
            return

        started_at = time.monotonic()
        offset_x: int | None = None
        image: np.ndarray | None = None
        combined_mask: np.ndarray | None = None
        left_column: int | None = None
        right_column: int | None = None
        center_x: int | None = None
        error_message = ''

        try:
            image = image_message_to_bgr(msg)
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            blue_mask = cv2.inRange(
                hsv, self._blue_hsv_lower, self._blue_hsv_upper)
            red_low_mask = cv2.inRange(
                hsv, self._red_low_hsv_lower, self._red_low_hsv_upper)
            red_high_mask = cv2.inRange(
                hsv, self._red_high_hsv_lower, self._red_high_hsv_upper)
            combined_mask = cv2.bitwise_or(
                blue_mask, cv2.bitwise_or(red_low_mask, red_high_mask))

            column_lengths = np.count_nonzero(combined_mask, axis=0)
            max_column_length = int(column_lengths.max())
            if max_column_length > 0:
                length_threshold = (
                    max_column_length * self._column_threshold_ratio)
                valid_columns = np.flatnonzero(
                    column_lengths >= length_threshold)
                left_column = int(valid_columns[0])
                right_column = int(valid_columns[-1])
                center_x = (left_column + right_column) // 2
                offset_x = center_x - image.shape[1] // 2
        except Exception as exc:
            error_message = f'Failed to process alignment image: {exc}'

        detection_completed_at = time.monotonic()

        with self.state_condition:
            self._frame_sequence += 1
            self._latest_offset_x = offset_x
            self._latest_frame_started_at = started_at
            self._latest_frame_time = detection_completed_at
            visualization_enabled = self._visualization_enabled
            self.state_condition.notify_all()

        if error_message:
            self.get_logger().error(error_message)

        if (
            visualization_enabled and
            image is not None and
            combined_mask is not None
        ):
            visualization = self._make_visualization(
                image,
                combined_mask,
                left_column,
                right_column,
                center_x,
                offset_x,
            )
            visualization_message = self._bgr_to_image_message(
                visualization, msg)
            # Recheck the dynamic switch so disabling it while this frame is
            # processed prevents a late visualization publication.
            with self.state_condition:
                visualization_enabled = self._visualization_enabled
            if visualization_enabled:
                self._pub_visualization.publish(visualization_message)

        processing_time = time.monotonic() - started_at
        if processing_time > self._processing_deadline_sec:
            self.get_logger().warn(
                'KFS alignment image processing overrun: '
                f'{processing_time * 1000.0:.2f} ms > '
                f'{self._processing_deadline_sec * 1000.0:.2f} ms '
                f'(target {self._target_processing_rate:g} Hz)'
            )

    @staticmethod
    def _make_visualization(
        image: np.ndarray,
        combined_mask: np.ndarray,
        left_column: int | None,
        right_column: int | None,
        center_x: int | None,
        offset_x: int | None,
    ) -> np.ndarray:
        result = image.copy()
        mask_visual = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)

        if (
            left_column is not None and
            right_column is not None and
            center_x is not None
        ):
            # The green boundaries cover only mask pixels, instead of drawing
            # full-height lines through unrelated parts of the image.
            for column in (left_column, right_column):
                start = max(0, column - 1)
                end = min(combined_mask.shape[1], column + 2)
                white_pixels = combined_mask[:, start:end] != 0
                for view in (result, mask_visual):
                    boundary_region = view[:, start:end]
                    boundary_region[white_pixels] = (0, 255, 0)

            for view in (result, mask_visual):
                cv2.line(
                    view,
                    (center_x, 0),
                    (center_x, view.shape[0] - 1),
                    (0, 0, 255),
                    2,
                )

        offset_text = (
            f'offset_x: {offset_x:+d} px'
            if offset_x is not None
            else 'offset_x: N/A'
        )
        cv2.putText(
            result,
            offset_text,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            result,
            offset_text,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return np.hstack((result, mask_visual))

    @staticmethod
    def _bgr_to_image_message(image: np.ndarray, source: Image) -> Image:
        image = np.ascontiguousarray(image, dtype=np.uint8)
        message = Image()
        message.header = source.header
        message.height = image.shape[0]
        message.width = image.shape[1]
        message.encoding = 'bgr8'
        message.is_bigendian = 0
        message.step = image.shape[1] * 3
        message.data = image.tobytes()
        return message

    # ---- PID ------------------------------------------------------

    def _pid_reset(self) -> None:
        self._integral = 0.0
        self._last_error = 0.0
        self._has_last_error = False

    def _pid_update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        self._integral += error * dt
        if self._integral_limit > 0.0:
            self._integral = max(
                -self._integral_limit,
                min(self._integral, self._integral_limit),
            )

        derivative = 0.0
        if self._has_last_error:
            derivative = (error - self._last_error) / dt

        output = (
            self._kp * error
            + self._ki * self._integral
            + self._kd * derivative
        )

        self._last_error = error
        self._has_last_error = True

        if self._output_limit > 0.0:
            output = max(-self._output_limit, min(output, self._output_limit))

        return output

    # ---- service handler ------------------------------------------

    def _handle_align(self, request, response):
        with self.service_lock:
            requested_tolerance = float(request.pixel_tolerance)
            requested_timeout = float(request.timeout_sec)
            if not math.isfinite(requested_tolerance):
                return self._failure_response(
                    response, 'pixel_tolerance must be finite', None)
            if not math.isfinite(requested_timeout):
                return self._failure_response(
                    response, 'timeout_sec must be finite', None)

            tolerance = (
                requested_tolerance
                if requested_tolerance > 0.0
                else float(self._pixel_tolerance)
            )
            timeout_sec = (
                requested_timeout
                if requested_timeout > 0.0
                else self._default_timeout_sec
            )

            with self.state_condition:
                self._alignment_active = True
            try:
                return self._execute_alignment(
                    response, tolerance, timeout_sec)
            finally:
                with self.state_condition:
                    self._alignment_active = False

    def _execute_alignment(self, response, tolerance, timeout_sec):
        self._pid_reset()
        stable_cycle_count = 0
        last_frame_time: float | None = None
        last_offset: int | None = None
        saw_detection = False
        service_started_at = time.monotonic()
        deadline = service_started_at + timeout_sec

        # A frame processed before this service request is never accepted.
        with self.state_condition:
            handled_sequence = self._frame_sequence

        while rclpy.ok():
            with self.state_condition:
                while (
                    self._frame_sequence <= handled_sequence and
                    rclpy.ok()
                ):
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        break
                    self.state_condition.wait(timeout=min(remaining, 0.1))

                if self._frame_sequence > handled_sequence:
                    handled_sequence = self._frame_sequence
                    offset = self._latest_offset_x
                    frame_started_at = self._latest_frame_started_at
                    frame_time = self._latest_frame_time
                else:
                    offset = None
                    frame_started_at = 0.0
                    frame_time = 0.0

            if time.monotonic() >= deadline:
                if not saw_detection:
                    message = (
                        'Alignment timeout: no red or blue region detected')
                elif last_offset is None:
                    message = 'Alignment timeout: target lost'
                else:
                    message = (
                        f'Alignment timeout: final offset={last_offset}px, '
                        f'stable={stable_cycle_count}/{self._stable_cycles}')
                return self._failure_response(response, message, last_offset)

            # A callback already processing when the service was called still
            # contains an older camera frame. Wait for the next one.
            if frame_started_at < service_started_at:
                continue

            if offset is None:
                stable_cycle_count = 0
                last_offset = None
                last_frame_time = None
                self._pid_reset()
                self._stop()
                continue

            saw_detection = True
            last_offset = offset

            if abs(offset) <= tolerance:
                stable_cycle_count += 1
                last_frame_time = frame_time
                self._pid_reset()
                self._stop()
                if stable_cycle_count >= self._stable_cycles:
                    response.success = True
                    response.message = (
                        f'Alignment complete: offset={offset}px, '
                        f'stable={stable_cycle_count}/{self._stable_cycles}, '
                        f'tolerance={tolerance:g}px')
                    response.final_offset_x = offset
                    return response
                continue

            stable_cycle_count = 0
            if last_frame_time is None:
                dt = self._processing_deadline_sec
            else:
                dt = frame_time - last_frame_time
                if dt <= 0.0:
                    dt = self._processing_deadline_sec
            last_frame_time = frame_time

            # Positive image offset means the target is to the right, so the
            # chassis follows it with negative body-frame linear.y.
            output = self._pid_update(-float(offset), dt)
            cmd = Twist()
            cmd.linear.y = output
            self._pub_cmd.publish(cmd)

        return self._failure_response(
            response, 'Alignment aborted: ROS shutdown', last_offset)

    def _failure_response(self, response, message, offset):
        self._stop()
        response.success = False
        response.message = message
        response.final_offset_x = offset if offset is not None else 0
        return response

    # ---- helpers --------------------------------------------------

    def _stop(self) -> None:
        self._pub_cmd.publish(Twist())


def main():
    rclpy.init()
    node = KfsAlignmentController()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
