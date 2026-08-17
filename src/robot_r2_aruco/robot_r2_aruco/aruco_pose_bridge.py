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


def _marker_frame(marker_id: int) -> str:
    return f"{_MARKER_FRAME_PREFIX}{marker_id}"


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

        pose = PoseStamped()
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = marker_frame
        pose.pose.position.x = translation.x
        pose.pose.position.y = translation.y
        pose.pose.position.z = translation.z
        pose.pose.orientation.x = rotation.x
        pose.pose.orientation.y = rotation.y
        pose.pose.orientation.z = rotation.z
        pose.pose.orientation.w = rotation.w
        self._pose_publisher.publish(pose)

        self._log_pose(marker_frame, translation, rotation)

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
