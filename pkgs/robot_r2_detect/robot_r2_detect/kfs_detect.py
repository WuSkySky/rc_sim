#!/usr/bin/env python3
"""kfs_detect: YOLO KFS detection ROS2 node.

Subscribes image → YOLO → publishes:
  /r2/detection/raw       — all raw detections (KfsRawDetections)
  /r2/detection/processed — best processed detection (KfsProcessedDetection)
"""

from __future__ import annotations

from pathlib import Path


# ---------------------------------------------------------------------------
# class-name → KFS-type mapping  (blue = true KFS, red = fake KFS)
# ---------------------------------------------------------------------------

_CLASS_MAP = {
    "blue": "true_kfs",
    "red": "fake_kfs",
}


def _map_kfs_type(class_name: str) -> str:
    return _CLASS_MAP.get(class_name.strip().lower(), "unknown")


def main(args: list[str] | None = None) -> None:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    from robot_r2_interfaces.msg import (
        KfsRawBox,
        KfsRawDetections,
        KfsProcessedDetection,
    )

    class KfsDetectNode(Node):
        """Subscribe image → YOLO → publish raw + processed detection."""

        def __init__(self) -> None:
            super().__init__("kfs_detect")
            self._declare_parameters()
            self._load_parameters()
            self._load_model()

            self._bridge = CvBridge()

            self._sub = self.create_subscription(
                Image, self._color_topic, self._image_cb, 10
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

        # ---- parameters ----

        def _declare_parameters(self) -> None:
            self.declare_parameter("model_path", "")
            self.declare_parameter("color_topic", "/r2/front_camera/image_raw")
            self.declare_parameter("conf", 0.75)
            self.declare_parameter("viz_topic", "/r2/detection/viz")

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
                processed_msg.kfs_type = _map_kfs_type(processed_cls_name)
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
                processed_msg.kfs_type = "none"
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

    # ----------------------------------------------------------------
    rclpy.init(args=args)
    node = KfsDetectNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
