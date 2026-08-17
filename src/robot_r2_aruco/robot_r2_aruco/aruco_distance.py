#!/usr/bin/env python3
"""Print marker distance and pose from /r2/aruco/detections."""

from __future__ import annotations

import math

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from robot_r2_interfaces.msg import ArucoPoseDetection


class DistancePrinter(Node):
    """Subscribe to ArucoPoseDetection and print distance every 0.5 s."""

    def __init__(self) -> None:
        super().__init__("aruco_distance")
        self._cb_group = MutuallyExclusiveCallbackGroup()
        self._sub = self.create_subscription(
            ArucoPoseDetection,
            "/r2/aruco/detections",
            self._on_detection,
            10,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            "watching /r2/aruco/detections — point the marker at the camera"
        )

    def _on_detection(self, msg: ArucoPoseDetection) -> None:
        for marker in msg.markers:
            p = marker.pose.position
            distance = math.sqrt(p.x ** 2 + p.y ** 2 + p.z ** 2)
            print(
                f"\rid={marker.marker_id}  "
                f"x={p.x:+.3f} m  y={p.y:+.3f} m  z={p.z:+.3f} m  "
                f"distance={distance:.3f} m",
                end="",
                flush=True,
            )
        if msg.markers:
            print()


def main() -> None:
    rclpy.init()
    node = DistancePrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
