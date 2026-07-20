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


def _classify_kfs_model(model_name: str) -> str:
    if "FakeKFS" in model_name:
        return "fake"
    if "TrueKFS" in model_name:
        return "r2"
    if "R1KFS" in model_name:
        return "r1"
    return ""


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
    import json
    import math
    import threading
    import time

    import rclpy
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    from robot_r2_interfaces.msg import (
        KfsRawBox,
        KfsRawDetections,
        KfsProcessedDetection,
    )
    from robot_r2_interfaces.srv import GetKfsType
    from std_msgs.msg import String

    class KfsDetectNode(Node):
        """Subscribe image → YOLO → publish raw + processed detection."""

        def __init__(self) -> None:
            super().__init__("kfs_detect")
            self._image_callback_group = MutuallyExclusiveCallbackGroup()
            self._service_callback_group = MutuallyExclusiveCallbackGroup()
            self._state_callback_group = MutuallyExclusiveCallbackGroup()
            self._vote_condition = threading.Condition()
            self._vote_active = False
            self._vote_target_count = 0
            self._vote_samples: list[str] = []
            self._state_condition = threading.Condition()
            self._latest_placements: dict[str, str] | None = None
            self._latest_seed: int | None = None
            self._cached_placements: dict[str, str] | None = None
            self._cached_seed: int | None = None
            self._robot_pose: tuple[float, float, float] | None = None

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
            self._status_sub = self.create_subscription(
                String,
                self._simulation_status_topic,
                self._simulation_status_cb,
                10,
                callback_group=self._state_callback_group,
            )
            self._pose_sub = self.create_subscription(
                PoseStamped,
                self._robot_pose_topic,
                self._robot_pose_cb,
                10,
                callback_group=self._state_callback_group,
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
            if self._simulation_state_detection:
                self.get_logger().warn(
                    "KFS detection service simulation-state mode is "
                    "enabled; the latest placement state will be frozen on "
                    "the first service request"
                )

        # ---- parameters ----

        def _declare_parameters(self) -> None:
            self.declare_parameter("model_path", "")
            self.declare_parameter("color_topic", "/r2/front_camera/image_raw")
            self.declare_parameter("conf", 0.65)
            self.declare_parameter("viz_topic", "/r2/detection/viz")
            self.declare_parameter(
                "vote_service_name", "/r2/detection/get_type"
            )
            self.declare_parameter("default_vote_timeout_sec", 10.0)
            self.declare_parameter("simulation_state_detection", False)
            self.declare_parameter(
                "simulation_status_topic", "/simulation/status"
            )
            self.declare_parameter("robot_pose_topic", "/r2/pose_feedback")
            self.declare_parameter("simulation_team", "blue")
            self.declare_parameter(
                "grid_x", [-2.6, -1.4, -0.2, 1.0, 2.2, 3.4]
            )
            self.declare_parameter("grid_y", [-4.2, -3.0, -1.8])
            self.declare_parameter("meilin_x", [2.2, 1.0, -0.2, -1.4])
            self.declare_parameter("grid_pitch", 1.2)
            self.declare_parameter("cell_snap_tolerance", 0.55)

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
            self._simulation_state_detection = bool(
                self.get_parameter("simulation_state_detection").value
            )
            self._simulation_status_topic = str(
                self.get_parameter("simulation_status_topic").value
            )
            self._robot_pose_topic = str(
                self.get_parameter("robot_pose_topic").value
            )
            self._simulation_team = str(
                self.get_parameter("simulation_team").value
            )
            if self._simulation_team not in ("red", "blue"):
                raise ValueError("simulation_team must be 'red' or 'blue'")
            self._grid_x = self._finite_array_parameter("grid_x")
            self._grid_y = self._finite_array_parameter("grid_y")
            self._meilin_x = self._finite_array_parameter("meilin_x")
            self._grid_pitch = self._positive_parameter("grid_pitch")
            self._cell_snap_tolerance = self._positive_parameter(
                "cell_snap_tolerance"
            )
            if (
                not math.isfinite(self._default_vote_timeout_sec)
                or self._default_vote_timeout_sec <= 0.0
            ):
                raise ValueError(
                    "default_vote_timeout_sec must be finite and positive"
                )

        def _positive_parameter(self, name: str) -> float:
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
            return value

        def _finite_array_parameter(self, name: str) -> tuple[float, ...]:
            values = tuple(
                float(value) for value in self.get_parameter(name).value
            )
            if not values or not all(math.isfinite(value) for value in values):
                raise ValueError(f"{name} must contain finite values")
            return values

        def _load_model(self) -> None:
            try:
                from ultralytics import YOLO
            except ImportError as exc:
                raise RuntimeError(
                    "Missing ultralytics. Run: pip install ultralytics"
                ) from exc
            self._model = YOLO(str(self._model_path))

        # ---- cached simulation state ----

        def _simulation_status_cb(self, msg: String) -> None:
            try:
                status = json.loads(msg.data)
                placements = status.get("placements", {})
                current_seed = int(status.get("current_seed", -1))
            except (TypeError, ValueError) as exc:
                self.get_logger().warn(
                    f"Ignored invalid simulation status: {exc}"
                )
                return

            if not isinstance(placements, dict) or not placements:
                return

            cached = {
                str(location): str(model)
                for location, model in placements.items()
            }
            with self._state_condition:
                # Once the first detection request freezes a layout, later
                # status publications are intentionally ignored.
                if self._cached_placements is not None:
                    return
                self._latest_placements = cached
                self._latest_seed = current_seed
                self._state_condition.notify_all()

        def _robot_pose_cb(self, msg: PoseStamped) -> None:
            pose = msg.pose
            yaw = self._yaw_from_quaternion(pose.orientation)
            values = (
                float(pose.position.x),
                float(pose.position.y),
                yaw,
            )
            if not all(math.isfinite(value) for value in values):
                return
            with self._state_condition:
                self._robot_pose = values
                self._state_condition.notify_all()

        @staticmethod
        def _yaw_from_quaternion(quaternion) -> float:
            sin_yaw = 2.0 * (
                quaternion.w * quaternion.z +
                quaternion.x * quaternion.y
            )
            cos_yaw = 1.0 - 2.0 * (
                quaternion.y * quaternion.y +
                quaternion.z * quaternion.z
            )
            return math.atan2(sin_yaw, cos_yaw)

        @staticmethod
        def _nearest_value(value: float, candidates: tuple[float, ...]):
            nearest = min(
                candidates,
                key=lambda candidate: abs(candidate - value),
            )
            return nearest, abs(nearest - value)

        def _infer_from_cached_state(self):
            with self._state_condition:
                placements = dict(self._cached_placements or {})
                robot_pose = self._robot_pose

            if not placements:
                raise RuntimeError("Cached KFS placements are unavailable")
            if robot_pose is None:
                raise RuntimeError("Robot pose is unavailable")

            robot_x, robot_y, robot_yaw = robot_pose
            cell_x, error_x = self._nearest_value(robot_x, self._grid_x)
            cell_y, error_y = self._nearest_value(robot_y, self._grid_y)
            if math.hypot(error_x, error_y) > self._cell_snap_tolerance:
                raise RuntimeError(
                    f"Robot at ({robot_x:.3f}, {robot_y:.3f}) is not near "
                    "a configured grid cell"
                )

            heading_x = math.cos(robot_yaw)
            heading_y = math.sin(robot_yaw)
            if abs(heading_x) >= abs(heading_y):
                step_x = (
                    self._grid_pitch
                    if heading_x >= 0.0
                    else -self._grid_pitch
                )
                step_y = 0.0
            else:
                step_x = 0.0
                step_y = (
                    self._grid_pitch
                    if heading_y >= 0.0
                    else -self._grid_pitch
                )

            target_x = cell_x + step_x
            target_y = cell_y + step_y
            meilin_x, meilin_x_error = self._nearest_value(
                target_x, self._meilin_x
            )
            meilin_y, meilin_y_error = self._nearest_value(
                target_y, self._grid_y
            )
            if (
                meilin_x_error > 1e-6 or
                meilin_y_error > 1e-6
            ):
                return "", "", ""

            row = self._meilin_x.index(meilin_x)
            column = self._grid_y.index(meilin_y)
            location = (
                f"{self._simulation_team}_meilin_{row * 3 + column + 1}"
            )
            model_name = placements.get(location, "")
            return _classify_kfs_model(model_name), location, model_name

        def _handle_simulation_state_detection(self, request, response):
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
            with self._state_condition:
                while (
                    self._latest_placements is None or
                    self._robot_pose is None
                ):
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        response.success = False
                        response.message = (
                            "Simulation state detection timed out waiting for "
                            "the first non-empty KFS status and robot pose"
                        )
                        response.class_name = ""
                        return response
                    self._state_condition.wait(timeout=min(remaining, 0.1))

                if self._cached_placements is None:
                    self._cached_placements = dict(
                        self._latest_placements
                    )
                    self._cached_seed = self._latest_seed
                    self.get_logger().info(
                        "Froze simulation KFS layout on first detection: "
                        f"seed={self._cached_seed}, "
                        f"placements={len(self._cached_placements)}"
                    )

            try:
                class_name, location, model_name = (
                    self._infer_from_cached_state()
                )
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                response.class_name = ""
                return response

            response.success = True
            response.class_name = class_name
            if model_name:
                response.message = (
                    f"Cached simulation state: {location} contains "
                    f"{model_name} ({class_name})"
                )
            elif location:
                response.message = (
                    f"Cached simulation state: {location} is empty"
                )
            else:
                response.message = (
                    "Robot is facing outside the configured Meilin cells"
                )
            self.get_logger().info(
                f"KFS state detection result: {response.message}"
            )
            return response

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
            if self._simulation_state_detection:
                return self._handle_simulation_state_detection(
                    request, response
                )

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
    executor = MultiThreadedExecutor(num_threads=3)
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
