#!/usr/bin/env python3
"""Real-time tkinter window showing the /aruco/debug image stream."""

from __future__ import annotations

import threading

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


def debug_qos() -> QoSProfile:
    """Match the debug publisher's QoS exactly."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class DebugViewerNode(Node):
    """Subscribe to /aruco/debug and update a tkinter window in real time."""

    def __init__(self) -> None:
        super().__init__("debug_viewer")
        self._condition = threading.Condition()
        self._latest: np.ndarray | None = None

        self._cb_group = MutuallyExclusiveCallbackGroup()
        self._sub = self.create_subscription(
            Image,
            "/aruco/debug",
            self._on_image,
            debug_qos(),
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            "viewer subscribed to /aruco/debug — point the marker "
            "at the camera"
        )

    def _on_image(self, msg: Image) -> None:
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )
        with self._condition:
            self._latest = arr
            self._condition.notify_all()

    def wait_next(self, timeout: float = 0.2) -> np.ndarray | None:
        """Block until a new frame arrives, or return the latest frame."""
        with self._condition:
            if self._latest is None:
                self._condition.wait(timeout=timeout)
            return self._latest


def main() -> None:
    import tkinter as tk
    from PIL import Image as PILImage
    from PIL import ImageTk

    rclpy.init()
    node = DebugViewerNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    root = tk.Tk()
    root.title("ArUco Detect — Debug View")
    label = tk.Label(root)
    label.pack()

    photo_ref = None

    def poll() -> None:
        """Called periodically by tkinter's main loop."""
        nonlocal photo_ref
        executor.spin_once(timeout_sec=0.0)
        frame = node.wait_next(timeout=0.0)
        if frame is not None:
            pil = PILImage.fromarray(frame[:, :, ::-1])  # BGR → RGB
            photo_ref = ImageTk.PhotoImage(pil)
            label.configure(image=photo_ref)
        root.after(30, poll)

    root.after(30, poll)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
