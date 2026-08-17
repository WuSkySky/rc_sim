#!/usr/bin/env python3
"""Save the latest /r2/aruco/debug frame for visual inspection."""

import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image


OUTDIR = "/tmp/aruco_debug"


class DebugSaver(Node):
    def __init__(self):
        super().__init__("debug_saver")
        os.makedirs(OUTDIR, exist_ok=True)
        self._last_save = 0.0
        self._cb_group = MutuallyExclusiveCallbackGroup()
        self._sub = self.create_subscription(
            Image,
            "/r2/aruco/debug",
            self._on_image,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            ),
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            f"Saving frames to {OUTDIR}/latest.png every second"
        )

    def _on_image(self, msg):
        now = time.monotonic()
        if now - self._last_save < 1.0:
            return
        self._last_save = now
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1
        )
        path = os.path.join(OUTDIR, "latest.png")
        cv2.imwrite(path, arr)


def main():
    rclpy.init()
    node = DebugSaver()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
