#!/usr/bin/env python3
"""kfs_detect: YOLO KFS detection ROS2 node.

Subscribes image → YOLO → publishes:
  /r2/detection/raw       — all raw detections (KfsRawDetections)
  /r2/detection/processed — best processed detection (KfsProcessedDetection)
Provides:
  /r2/detection/get_type  — majority vote over the next n processed results
"""

from __future__ import annotations

from collections import Counter
from pathlib import Path


def _select_most_frequent_class(samples: list[str]) -> str:
    """Return the most frequent class, preferring the most recent on ties."""
    if not samples:
        raise ValueError("at least one class sample is required")

    counts = Counter(samples)
    highest_count = max(counts.values())
    tied_classes = {
        class_name
        for class_name, count in counts.items()
        if count == highest_count
    }
    return next(
        class_name
        for class_name in reversed(samples)
        if class_name in tied_classes
    )


def main(args: list[str] | None = None) -> None:
    import math
    import threading
    import time

    import rclpy
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    from robot_r2_interfaces.msg import (
        KfsRawBox,
        KfsRawDetections,
        KfsProcessedDetection,
    )
    from robot_r2_interfaces.srv import GetKfsType

    class KfsDetectNode(Node):
        """Subscribe image → YOLO → publish raw + processed detection."""

        def __init__(self) -> None:
            super().__init__("kfs_detect")
            self._image_callback_group = MutuallyExclusiveCallbackGroup()
            self._service_callback_group = MutuallyExclusiveCallbackGroup()
            self._vote_condition = threading.Condition()
            self._vote_active = False
            self._vote_target_count = 0
            self._vote_samples: list[str] = []

            self._declare_parameters()
            self._load_parameters()
            self._load_model()

            self._bridge = CvBridge()

            self._sub = self.create_subscription(
                Image,
                self._color_topic,
                self._image_cb,
                10,
                callback_group=self._image_callback_group,
            )
            self._pub_raw = self.create_publisher(
                KfsRawDetections, "/r2/detection/raw", 10
            )
            self._pub_processed = self.create_publisher(
                KfsProcessedDetection, "/r2/detection/processed", 10
            )
            self._pub_viz = self.create_publisher(
                Image, "/r2/detection/viz", 10
            )
            self._get_type_service = self.create_service(
                GetKfsType,
                self._vote_service_name,
                self._handle_get_kfs_type,
                callback_group=self._service_callback_group,
            )

        # ---- parameters ----

        def _declare_parameters(self) -> None:
            self.declare_parameter("model_path", "")
            self.declare_parameter("color_topic", "/r2/front_camera/image_raw")
            self.declare_parameter("conf", 0.75)
            self.declare_parameter("viz_topic", "/r2/detection/viz")
            self.declare_parameter(
                "vote_service_name", "/r2/detection/get_type"
            )
            self.declare_parameter("default_vote_timeout_sec", 10.0)

        def _load_parameters(self) -> None:
            from ament_index_python.packages import get_package_share_directory

            raw = str(self.get_parameter("model_path").value)
            model_path = Path(raw).expanduser()

            if not model_path.exists() or raw == "":
                # try package-relative: share/robot_r2_detect/model/best.pt
                pkg_model_dir = (
                    Path(get_package_share_directory("robot_r2_detect"))
                    / "model"
                )
                if raw == "":
                    model_path = pkg_model_dir / "best.pt"
                else:
                    # user gave a bare name like "best.pt" — check pkg model dir
                    candidate = pkg_model_dir / model_path.name
                    if candidate.exists():
                        model_path = candidate

            if not model_path.exists():
                raise FileNotFoundError(
                    f"YOLO model not found: {raw}.  "
                    f"Checked: CWD={Path.cwd()}, {model_path}"
                )
            self._model_path = model_path
            self._color_topic = str(self.get_parameter("color_topic").value)
            self._conf = float(self.get_parameter("conf").value)
            self._vote_service_name = str(
                self.get_parameter("vote_service_name").value
            )
            self._default_vote_timeout_sec = float(
                self.get_parameter("default_vote_timeout_sec").value
            )
            if (
                not math.isfinite(self._default_vote_timeout_sec)
                or self._default_vote_timeout_sec <= 0.0
            ):
                raise ValueError(
                    "default_vote_timeout_sec must be finite and positive"
                )

        def _load_model(self) -> None:
            try:
                from ultralytics import YOLO
            except ImportError as exc:
                raise RuntimeError(
                    "Missing ultralytics. Run: pip install ultralytics"
                ) from exc
            self._model = YOLO(str(self._model_path))

        # ---- callback ----

        def _image_cb(self, msg: Image) -> None:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            h, w = image.shape[:2]

            result = self._model.predict(image, conf=self._conf, verbose=False)[0]

            # ------- Topic 1: raw -------
            raw_boxes: list[KfsRawBox] = []
            for box in result.boxes:
                cls_id = int(box.cls[0])
                cls_name = result.names.get(cls_id, "unknown")
                conf = float(box.conf[0])
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                raw_boxes.append(
                    KfsRawBox(
                        class_name=cls_name,
                        class_id=cls_id,
                        confidence=conf,
                        x1=x1, y1=y1, x2=x2, y2=y2,
                    )
                )

            raw_msg = KfsRawDetections()
            raw_msg.header = msg.header
            raw_msg.boxes = raw_boxes
            self._pub_raw.publish(raw_msg)

            # ------- Topic 2: processed -------
            processed_conf = -1.0
            processed_box: tuple | None = None
            processed_cls_name = ""

            for box in result.boxes:
                conf = float(box.conf[0])
                if conf <= self._conf:
                    continue
                if conf > processed_conf:
                    processed_conf = conf
                    cls_id = int(box.cls[0])
                    processed_cls_name = result.names.get(cls_id, "unknown")
                    x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                    processed_box = (x1, y1, x2, y2)

            processed_msg = KfsProcessedDetection()
            processed_msg.header = msg.header
            processed_msg.image_width = w
            processed_msg.image_height = h

            if processed_box is not None:
                x1, y1, x2, y2 = processed_box
                center_u = (x1 + x2) // 2
                center_v = (y1 + y2) // 2

                processed_msg.class_name = processed_cls_name
                processed_msg.confidence = processed_conf
                processed_msg.x1 = x1
                processed_msg.y1 = y1
                processed_msg.x2 = x2
                processed_msg.y2 = y2
                processed_msg.center_u = center_u
                processed_msg.center_v = center_v
                processed_msg.center_offset_x = center_u - w // 2
                processed_msg.center_offset_y = center_v - h // 2
            else:
                processed_msg.class_name = ""
                processed_msg.confidence = 0.0
                processed_msg.x1 = 0
                processed_msg.y1 = 0
                processed_msg.x2 = 0
                processed_msg.y2 = 0
                processed_msg.center_u = 0
                processed_msg.center_v = 0
                processed_msg.center_offset_x = 0
                processed_msg.center_offset_y = 0

            self._pub_processed.publish(processed_msg)
            self._record_vote_sample(processed_msg.class_name)

            # ------- Topic 3: visualization -------
            import cv2
            viz = image.copy()

            proc_x1 = processed_msg.x1 if processed_msg.class_name else -1

            for box in result.boxes:
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                cls_id = int(box.cls[0])
                cls_name = result.names.get(cls_id, "unknown")
                conf = float(box.conf[0])

                is_processed = (x1 == proc_x1)
                color = (0, 255, 0) if is_processed else (0, 0, 0)
                thickness = 2 if is_processed else 1

                cv2.rectangle(viz, (x1, y1), (x2, y2), color, thickness)
                label = f"{cls_name} {conf:.2f}"
                cv2.putText(viz, label, (x1, max(y1 - 5, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            viz_msg = Image()
            viz_msg.header = msg.header
            viz_msg.height = viz.shape[0]
            viz_msg.width = viz.shape[1]
            viz_msg.encoding = "bgr8"
            viz_msg.is_bigendian = 0
            viz_msg.step = viz.shape[1] * 3
            viz_msg.data = viz.tobytes()
            self._pub_viz.publish(viz_msg)

        # ---- majority-vote service ----

        def _record_vote_sample(self, class_name: str) -> None:
            with self._vote_condition:
                if not self._vote_active:
                    return
                if len(self._vote_samples) >= self._vote_target_count:
                    return

                self._vote_samples.append(class_name)
                if len(self._vote_samples) >= self._vote_target_count:
                    self._vote_condition.notify_all()

        def _handle_get_kfs_type(self, request, response):
            sample_count = int(request.sample_count)
            if sample_count <= 0:
                response.success = False
                response.message = "sample_count must be positive"
                response.class_name = ""
                return response

            requested_timeout = float(request.timeout_sec)
            if not math.isfinite(requested_timeout):
                response.success = False
                response.message = "timeout_sec must be finite"
                response.class_name = ""
                return response
            timeout_sec = (
                requested_timeout
                if requested_timeout > 0.0
                else self._default_vote_timeout_sec
            )

            deadline = time.monotonic() + timeout_sec
            failure_message = ""
            samples: list[str] = []
            with self._vote_condition:
                self._vote_samples = []
                self._vote_target_count = sample_count
                self._vote_active = True
                try:
                    while len(self._vote_samples) < sample_count:
                        if not rclpy.ok():
                            failure_message = (
                                "ROS shutdown while collecting detections"
                            )
                            break

                        remaining = deadline - time.monotonic()
                        if remaining <= 0.0:
                            failure_message = (
                                "Detection vote timed out after collecting "
                                f"{len(self._vote_samples)}/{sample_count} "
                                "samples"
                            )
                            break
                        self._vote_condition.wait(
                            timeout=min(remaining, 0.1)
                        )

                    if not failure_message:
                        samples = list(self._vote_samples)
                finally:
                    self._vote_active = False
                    self._vote_target_count = 0

            if failure_message:
                response.success = False
                response.message = failure_message
                response.class_name = ""
                return response

            response.success = True
            response.message = (
                f"Selected most frequent class from {sample_count} samples"
            )
            response.class_name = _select_most_frequent_class(samples)
            return response

    # ----------------------------------------------------------------
    rclpy.init(args=args)
    node = KfsDetectNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
