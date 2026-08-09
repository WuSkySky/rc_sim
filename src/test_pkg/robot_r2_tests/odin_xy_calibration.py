#!/usr/bin/env python3

import math
import os
import queue
import select
import sys
import termios
import threading
import tty
from dataclasses import dataclass

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


RAW_ODOMETRY_TOPIC = '/odin1/odometry_highfreq'
ODIN_MOUNT_YAW = math.pi
MINIMUM_SAMPLES = 100
MINIMUM_ROTATION_RAD = math.radians(300.0)
SAMPLE_YAW_STEP_RAD = math.radians(0.25)


@dataclass(frozen=True)
class CalibrationSample:
    x: float
    y: float
    yaw: float
    unwrapped_yaw: float


@dataclass(frozen=True)
class CalibrationResult:
    base_to_odin_x: float
    base_to_odin_y: float
    center_x: float
    center_y: float
    radius: float
    rmse: float
    maximum_error: float
    condition_number: float


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(quaternion):
    components = np.asarray(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(components)):
        return None
    norm = float(np.linalg.norm(components))
    if norm <= 1e-12:
        return None
    x, y, z, w = components / norm
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return yaw if math.isfinite(yaw) else None


def fit_installation_xy(samples):
    sample_count = len(samples)
    if sample_count < 2:
        raise ValueError('at least two samples are required')

    design = np.empty((sample_count * 2, 4), dtype=np.float64)
    observations = np.empty(sample_count * 2, dtype=np.float64)
    for index, sample in enumerate(samples):
        cosine = math.cos(sample.yaw)
        sine = math.sin(sample.yaw)
        row = index * 2
        design[row] = (1.0, 0.0, cosine, -sine)
        design[row + 1] = (0.0, 1.0, sine, cosine)
        observations[row] = sample.x
        observations[row + 1] = sample.y

    solution, _, rank, singular_values = np.linalg.lstsq(
        design,
        observations,
        rcond=None,
    )
    if rank != 4 or singular_values[-1] <= np.finfo(np.float64).eps:
        raise ValueError('recorded orientations do not provide a valid fit')

    center_x, center_y, odin_frame_x, odin_frame_y = solution
    mount_cosine = math.cos(ODIN_MOUNT_YAW)
    mount_sine = math.sin(ODIN_MOUNT_YAW)
    base_to_odin_x = (
        mount_cosine * odin_frame_x - mount_sine * odin_frame_y
    )
    base_to_odin_y = (
        mount_sine * odin_frame_x + mount_cosine * odin_frame_y
    )

    fitted = design @ solution
    residual = observations - fitted
    point_errors = np.hypot(residual[0::2], residual[1::2])
    condition_number = singular_values[0] / singular_values[-1]

    return CalibrationResult(
        base_to_odin_x=float(base_to_odin_x),
        base_to_odin_y=float(base_to_odin_y),
        center_x=float(center_x),
        center_y=float(center_y),
        radius=math.hypot(base_to_odin_x, base_to_odin_y),
        rmse=float(np.sqrt(np.mean(point_errors * point_errors))),
        maximum_error=float(np.max(point_errors)),
        condition_number=float(condition_number),
    )


class TerminalKeyboard:
    def __init__(self, callback):
        self._callback = callback
        self._stop_event = threading.Event()
        self._thread = None
        self._file_descriptor = None
        self._saved_settings = None

    def start(self):
        if not sys.stdin.isatty():
            raise RuntimeError('stdin is not a terminal; keyboard unavailable')

        self._file_descriptor = sys.stdin.fileno()
        self._saved_settings = termios.tcgetattr(self._file_descriptor)
        tty.setcbreak(self._file_descriptor)
        self._thread = threading.Thread(
            target=self._read_loop,
            name='odin-calibration-keyboard',
            daemon=True,
        )
        self._thread.start()

    def _read_loop(self):
        while not self._stop_event.is_set():
            readable, _, _ = select.select(
                [self._file_descriptor], [], [], 0.1)
            if not readable:
                continue
            data = os.read(self._file_descriptor, 1)
            if not data:
                continue
            self._callback(data.decode(errors='ignore'))

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if (
            self._file_descriptor is not None
            and self._saved_settings is not None
        ):
            termios.tcsetattr(
                self._file_descriptor,
                termios.TCSADRAIN,
                self._saved_settings,
            )
            self._saved_settings = None


class OdinXyCalibration(Node):
    def __init__(self):
        super().__init__('odin_xy_calibration')
        self._lock = threading.Lock()
        self._key_queue = queue.SimpleQueue()
        self._recording = False
        self._samples = []
        self._previous_raw_yaw = None
        self._current_unwrapped_yaw = None
        self._last_accepted_unwrapped_yaw = None
        self._last_timestamp_ns = None

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscription = self.create_subscription(
            Odometry,
            RAW_ODOMETRY_TOPIC,
            self._on_odometry,
            qos,
        )
        self._key_timer = self.create_timer(0.05, self._process_keys)

        self.get_logger().info(
            f'Waiting for raw Odin odometry on {RAW_ODOMETRY_TOPIC}')
        self.get_logger().info(
            'Press SPACE to start/stop recording; press Q to quit')

    def enqueue_key(self, key):
        self._key_queue.put(key)

    def _process_keys(self):
        while True:
            try:
                key = self._key_queue.get_nowait()
            except queue.Empty:
                return

            if key == ' ':
                self._toggle_recording()
            elif key.lower() == 'q':
                self.get_logger().info('Exiting calibration tool')
                rclpy.shutdown()
                return

    def _toggle_recording(self):
        with self._lock:
            if not self._recording:
                self._samples.clear()
                self._previous_raw_yaw = None
                self._current_unwrapped_yaw = None
                self._last_accepted_unwrapped_yaw = None
                self._last_timestamp_ns = None
                self._recording = True
                samples = None
            else:
                self._recording = False
                samples = tuple(self._samples)

        if samples is None:
            self.get_logger().info(
                'Recording started; rotate the robot in place')
        else:
            self.get_logger().info(
                f'Recording stopped with {len(samples)} samples')
            self._calculate(samples)

    def _on_odometry(self, message):
        position = message.pose.pose.position
        yaw = quaternion_to_yaw(message.pose.pose.orientation)
        values = (position.x, position.y)
        if yaw is None or not all(math.isfinite(value) for value in values):
            return

        stamp = message.header.stamp
        timestamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec

        with self._lock:
            if not self._recording:
                return
            if (
                self._last_timestamp_ns is not None
                and timestamp_ns <= self._last_timestamp_ns
            ):
                return
            self._last_timestamp_ns = timestamp_ns

            if self._previous_raw_yaw is None:
                unwrapped_yaw = yaw
            else:
                delta = normalize_angle(yaw - self._previous_raw_yaw)
                unwrapped_yaw = self._current_unwrapped_yaw + delta
            self._previous_raw_yaw = yaw
            self._current_unwrapped_yaw = unwrapped_yaw

            if (
                self._last_accepted_unwrapped_yaw is not None
                and abs(
                    unwrapped_yaw - self._last_accepted_unwrapped_yaw
                ) < SAMPLE_YAW_STEP_RAD
            ):
                return

            self._samples.append(CalibrationSample(
                x=float(position.x),
                y=float(position.y),
                yaw=yaw,
                unwrapped_yaw=unwrapped_yaw,
            ))
            self._last_accepted_unwrapped_yaw = unwrapped_yaw

    def _calculate(self, samples):
        if samples:
            unwrapped_yaws = [sample.unwrapped_yaw for sample in samples]
            rotation_coverage = max(unwrapped_yaws) - min(unwrapped_yaws)
        else:
            rotation_coverage = 0.0

        if (
            len(samples) < MINIMUM_SAMPLES
            or rotation_coverage < MINIMUM_ROTATION_RAD
        ):
            self.get_logger().error(
                'Insufficient calibration data: '
                f'samples={len(samples)}/{MINIMUM_SAMPLES}, '
                f'rotation={math.degrees(rotation_coverage):.1f}/'
                f'{math.degrees(MINIMUM_ROTATION_RAD):.1f} deg')
            return

        try:
            result = fit_installation_xy(samples)
        except (ValueError, np.linalg.LinAlgError) as exc:
            self.get_logger().error(f'Calibration failed: {exc}')
            return

        self.get_logger().info(
            '\nCalibration completed'
            f'\n  base_to_odin x: {result.base_to_odin_x:+.6f} m'
            f'\n  base_to_odin y: {result.base_to_odin_y:+.6f} m'
            f'\n  rotation center: '
            f'({result.center_x:+.6f}, {result.center_y:+.6f}) m'
            f'\n  installation radius: {result.radius:.6f} m'
            f'\n  samples: {len(samples)}'
            f'\n  rotation coverage: '
            f'{math.degrees(rotation_coverage):.1f} deg'
            f'\n  position RMSE: {result.rmse:.6f} m'
            f'\n  maximum error: {result.maximum_error:.6f} m'
            f'\n  fit condition number: {result.condition_number:.3f}'
            '\nConfig value:'
            f'\n  base_to_odin_xyz: '
            f'[{result.base_to_odin_x:.6f}, '
            f'{result.base_to_odin_y:.6f}, <keep current z>]')


def main():
    rclpy.init()
    node = OdinXyCalibration()
    keyboard = TerminalKeyboard(node.enqueue_key)

    try:
        keyboard.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
    finally:
        keyboard.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
