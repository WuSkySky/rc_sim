#!/usr/bin/env python3
"""Translate the ArUco marker TF into the R2 chassis pose in the marker frame.

``aruco_detect`` broadcasts ``camera -> marker_<id>`` and the launch file adds
a static ``base_link -> camera`` transform. Together they form the TF tree
``base_link -> camera -> marker_<id>``, so this node looks up
``marker_<id> -> base_link`` — the chassis pose expressed in the marker's
frame — and publishes it as a ``geometry_msgs/PoseStamped``.

Because the marker is fixed to the stationary R1 robot, the marker frame acts
as a fixed "world" frame for the one-shot docking. The downstream consumer is
the existing ``chassis_pose_servo`` node, remapped to subscribe to
``/r2/aruco/pose_feedback`` and to serve ``/r2/aruco/move_to_pose``.
"""

from __future__ import annotations

import math
import threading
import time

import numpy as np
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformException,
    TransformListener,
)

# The chassis frame published by ``odometry_tf`` / ``odometry_postprocess``.
_BASE_FRAME = "base_link"
# TF child frame name convention used by ``aruco_detect``.
_MARKER_FRAME_PREFIX = "marker_"

# Rotation that "levels" a vertically-mounted ArUco marker into a ground frame.
# Marker frame (OpenCV): Z = marker normal (horizontal, points at the camera,
# i.e. the distance-to-marker axis), X = left/right, Y = downward (vertical).
# Ground frame consumed by chassis_pose_servo: x = forward, y = lateral,
# z = upward. The planar servo drops z, reads x/y, and takes yaw around z, so
# feeding the raw marker-frame pose would lose the forward axis and misread
# "up/down" as "left/right". This fixed rotation (verified pure rotation,
# det = +1) maps marker -> ground.
_MARKER_TO_GROUND = np.array(
    [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
)


def _marker_frame(marker_id: int) -> str:
    return f"{_MARKER_FRAME_PREFIX}{marker_id}"


def _quaternion_to_rotation_matrix(
    qx: float, qy: float, qz: float, qw: float,
) -> np.ndarray:
    """Return the 3x3 rotation matrix of a normalized quaternion (x, y, z, w)."""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        return np.eye(3)
    x, y, z, w = qx / norm, qy / norm, qz / norm, qw / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def _rotation_matrix_to_quaternion(
    R: np.ndarray,
) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a normalized quaternion (x, y, z, w)."""
    trace = float(np.trace(R))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (float(R[2, 1]) - float(R[1, 2])) / s
        y = (float(R[0, 2]) - float(R[2, 0])) / s
        z = (float(R[1, 0]) - float(R[0, 1])) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + float(R[0, 0]) - float(R[1, 1]) - float(R[2, 2])) * 2.0
        w = (float(R[2, 1]) - float(R[1, 2])) / s
        x = 0.25 * s
        y = (float(R[0, 1]) + float(R[1, 0])) / s
        z = (float(R[0, 2]) + float(R[2, 0])) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + float(R[1, 1]) - float(R[0, 0]) - float(R[2, 2])) * 2.0
        w = (float(R[0, 2]) - float(R[2, 0])) / s
        x = (float(R[0, 1]) + float(R[1, 0])) / s
        y = 0.25 * s
        z = (float(R[1, 2]) + float(R[2, 1])) / s
    else:
        s = math.sqrt(1.0 + float(R[2, 2]) - float(R[0, 0]) - float(R[1, 1])) * 2.0
        w = (float(R[1, 0]) - float(R[0, 1])) / s
        x = (float(R[0, 2]) + float(R[2, 0])) / s
        y = (float(R[1, 2]) + float(R[2, 1])) / s
        z = 0.25 * s
    return x, y, z, w


def _yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """Return the yaw angle (radians, around Z) of a quaternion.

    Matches ``chassis_pose_servo.yaw_from_quaternion`` so that logged yaw
    values line up with the ``/r2/move_to_pose`` service convention.
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class ArucoPoseBridge(Node):
    """Publish the chassis pose in the marker frame from the ArUco TF tree."""

    OUTPUT_POSE_TOPIC = "/r2/aruco/pose_feedback"

    def __init__(self) -> None:
        super().__init__("aruco_pose_bridge")
        self._state_lock = threading.Lock()

        self._declare_parameters()
        self._load_parameters()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pose_publisher = self.create_publisher(
            PoseStamped, self.OUTPUT_POSE_TOPIC, 10
        )

        self._timer = self.create_timer(1.0 / self._publish_rate, self._publish)

        # Rate-limit the "waiting for TF" warning and the convenience log.
        self._next_missing_warn_at = 0.0
        self._next_pose_log_at = 0.0

        self.add_on_set_parameters_callback(self._on_parameters_changed)

    # ------------------------------------------------------------------
    # Parameter lifecycle
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        self.declare_parameter("target_marker_id", 1)
        self.declare_parameter("publish_rate", 30.0)

    def _load_parameters(self) -> None:
        target_marker_id = int(self.get_parameter("target_marker_id").value)
        publish_rate = float(self.get_parameter("publish_rate").value)

        if target_marker_id < 0:
            raise ValueError("target_marker_id must be non-negative")
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError("publish_rate must be finite and positive")

        self._target_marker_id = target_marker_id
        self._publish_rate = publish_rate

    def _on_parameters_changed(self, parameters) -> SetParametersResult:
        with self._state_lock:
            target_marker_id = self._target_marker_id
            publish_rate = self._publish_rate
        for parameter in parameters:
            if parameter.name == "target_marker_id":
                value = parameter.value
                if isinstance(value, bool) or not isinstance(value, int):
                    return SetParametersResult(
                        successful=False,
                        reason="target_marker_id must be an integer",
                    )
                if int(value) < 0:
                    return SetParametersResult(
                        successful=False,
                        reason="target_marker_id must be non-negative",
                    )
                target_marker_id = int(value)

            elif parameter.name == "publish_rate":
                value = parameter.value
                if isinstance(value, bool) or not isinstance(
                    value, (int, float)
                ):
                    return SetParametersResult(
                        successful=False,
                        reason="publish_rate must be a number",
                    )
                value = float(value)
                if not math.isfinite(value) or value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="publish_rate must be finite and positive",
                    )
                publish_rate = value

            else:
                return SetParametersResult(
                    successful=False,
                    reason=(
                        f"parameter '{parameter.name}' cannot be changed "
                        f"at runtime; restart the node to apply it"
                    ),
                )

        with self._state_lock:
            previous_rate = self._publish_rate
            self._target_marker_id = target_marker_id
            self._publish_rate = publish_rate
        if publish_rate != previous_rate:
            new_timer = self.create_timer(1.0 / publish_rate, self._publish)
            with self._state_lock:
                old_timer = self._timer
                self._timer = new_timer
            old_timer.cancel()
            self.destroy_timer(old_timer)
        self.get_logger().info(
            f"ArUco pose bridge updated: marker={target_marker_id}, "
            f"rate={publish_rate:g} Hz"
        )
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish(self) -> None:
        with self._state_lock:
            marker_frame = _marker_frame(self._target_marker_id)
        try:
            transform = self._tf_buffer.lookup_transform(
                marker_frame, _BASE_FRAME, rclpy.time.Time()
            )
        except (
            LookupException,
            ConnectivityException,
            ExtrapolationException,
            TransformException,
        ):
            self._warn_missing_tf(marker_frame)
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Level the marker-frame pose into a ground frame before publishing.
        # The raw pose is base_link in the marker frame; because the marker is
        # mounted vertically, its Z axis is horizontal (distance-to-marker) and
        # its Y axis is vertical. A planar servo reads only x/y/yaw and drops
        # z, so without this rotation the forward axis would be lost and the
        # yaw would be misread. Left-multiply the fixed marker->ground rotation.
        R_marker_base = _quaternion_to_rotation_matrix(
            rotation.x, rotation.y, rotation.z, rotation.w
        )
        t_marker_base = np.array(
            [translation.x, translation.y, translation.z], dtype=np.float64
        )
        R_ground_base = _MARKER_TO_GROUND @ R_marker_base
        t_ground_base = _MARKER_TO_GROUND @ t_marker_base
        qx, qy, qz, qw = _rotation_matrix_to_quaternion(R_ground_base)

        ground_frame = f"{marker_frame}_ground"

        pose = PoseStamped()
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = ground_frame
        pose.pose.position.x = float(t_ground_base[0])
        pose.pose.position.y = float(t_ground_base[1])
        pose.pose.position.z = float(t_ground_base[2])
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self._pose_publisher.publish(pose)

        self._log_pose(ground_frame, pose.pose.position, pose.pose.orientation)

    def _warn_missing_tf(self, marker_frame: str) -> None:
        now = time.monotonic()
        if now < self._next_missing_warn_at:
            return
        self._next_missing_warn_at = now + 2.0
        self.get_logger().warn(
            f"waiting for TF '{marker_frame} -> {_BASE_FRAME}'; "
            f"is the marker in view and is the static base_link -> camera "
            f"transform published?"
        )

    def _log_pose(self, marker_frame, translation, rotation) -> None:
        now = time.monotonic()
        if now < self._next_pose_log_at:
            return
        self._next_pose_log_at = now + 2.0
        yaw = _yaw_from_quaternion(
            rotation.x, rotation.y, rotation.z, rotation.w
        )
        self.get_logger().info(
            f"base_link in '{marker_frame}': "
            f"x={translation.x:.4f} m, y={translation.y:.4f} m, "
            f"yaw={yaw:.4f} rad"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ArucoPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
