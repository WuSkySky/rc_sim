#!/usr/bin/env python3
"""Publish the static ``base_link -> camera`` transform from YAML parameters.

The camera frame id is taken from ``camera_info.header.frame_id`` (the same
source ``aruco_detect`` uses), so the child frame always matches the camera
node and the TF tree ``base_link -> camera -> marker_<id>`` stays connected.

The orientation parameters are expressed in an intuitive convention instead
of the raw optical-frame RPY (which carries a -90 degree roll):

- ``yaw``   : horizontal heading, 0 = facing forward (+x), pi = facing rear
- ``pitch`` : tilt down, positive = look down
- ``roll``  : rotation about the optical axis, 0 = upright

Only the orientation needs to be roughly right (a reversed camera cannot be
recovered by teach-in). The translation may be approximate: the one-shot
teach-in absorbs the mount-offset error.
"""

from __future__ import annotations

import math

import numpy as np
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CameraInfo
from tf2_ros import StaticTransformBroadcaster

# Chassis frame published by odometry_tf / odometry_postprocess.
_BASE_FRAME = "base_link"
# body -> optical rotation for a forward-facing, upright camera
# (optical x = right, y = down, z = forward).
_FORWARD_ROTATION = np.array(
    [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
)


def _rotation_matrix_to_quaternion(R: np.ndarray) -> tuple[float, float, float, float]:
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


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return the ZYX rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _read_vector(name: str, value) -> tuple[float, float, float]:
    """Validate a 3-element numeric parameter and return it as floats."""
    if (
        isinstance(value, (str, bytes))
        or not hasattr(value, "__len__")
        or len(value) != 3
    ):
        raise ValueError(f"{name} must contain exactly three numbers")
    numbers = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ValueError(f"{name} values must be numbers")
        numbers.append(float(item))
    if not all(math.isfinite(item) for item in numbers):
        raise ValueError(f"{name} values must be finite")
    return tuple(numbers)


class ArucoCameraTf(Node):
    """Broadcast the base_link -> camera static transform from parameters."""

    def __init__(self) -> None:
        super().__init__("aruco_camera_tf")

        self.declare_parameter("camera_xyz", [0.15, 0.0, 0.15])
        self.declare_parameter("camera_rpy", [0.0, 0.0, 0.0])

        self._camera_xyz = _read_vector(
            "camera_xyz", self.get_parameter("camera_xyz").value
        )
        self._camera_rpy = _read_vector(
            "camera_rpy", self.get_parameter("camera_rpy").value
        )

        self._camera_frame_id = ""
        self._broadcaster = StaticTransformBroadcaster(self)
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            "camera_info",
            self._on_camera_info,
            QoSProfile(depth=10),
        )

        self.add_on_set_parameters_callback(self._on_parameters_changed)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_camera_info(self, message: CameraInfo) -> None:
        frame_id = message.header.frame_id
        if not frame_id:
            return
        if frame_id != self._camera_frame_id:
            self._camera_frame_id = frame_id
            self.get_logger().info(f"camera frame detected: '{frame_id}'")
            self._broadcast()

    def _on_parameters_changed(self, parameters) -> SetParametersResult:
        xyz = self._camera_xyz
        rpy = self._camera_rpy
        for parameter in parameters:
            try:
                if parameter.name == "camera_xyz":
                    xyz = _read_vector("camera_xyz", parameter.value)
                elif parameter.name == "camera_rpy":
                    rpy = _read_vector("camera_rpy", parameter.value)
                else:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            f"parameter '{parameter.name}' cannot be changed "
                            f"at runtime; restart the node to apply it"
                        ),
                    )
            except ValueError as error:
                return SetParametersResult(
                    successful=False, reason=str(error)
                )

        self._camera_xyz = xyz
        self._camera_rpy = rpy
        if self._camera_frame_id:
            self._broadcast()
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _broadcast(self) -> None:
        roll, pitch, yaw = self._camera_rpy
        rotation = _rpy_to_matrix(roll, pitch, yaw) @ _FORWARD_ROTATION
        qx, qy, qz, qw = _rotation_matrix_to_quaternion(rotation)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = _BASE_FRAME
        transform.child_frame_id = self._camera_frame_id
        transform.transform.translation.x = self._camera_xyz[0]
        transform.transform.translation.y = self._camera_xyz[1]
        transform.transform.translation.z = self._camera_xyz[2]
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._broadcaster.sendTransform(transform)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ArucoCameraTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
