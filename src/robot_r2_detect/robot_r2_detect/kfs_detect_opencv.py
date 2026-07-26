#!/usr/bin/env python3
"""KFS classification using offline OpenCV Chamfer features."""

from __future__ import annotations

from pathlib import Path

from robot_r2_detect.kfs_detect import (
    _classify_kfs_model,
    _select_most_frequent_class,
)


def main(args: list[str] | None = None) -> None:
    import cv2
    import json
    import math
    import threading
    import time

    import numpy as np
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from robot_r2_interfaces.msg import (
        KfsProcessedDetection,
        KfsRawBox,
        KfsRawDetections,
        KfsRoiDetection,
    )
    from robot_r2_interfaces.srv import GetKfsType
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    from robot_r2_detect.kfs_chamfer_matcher import (
        KfsChamferMatcher,
        KfsMatchResult,
    )

    def image_message_to_bgr(msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        if encoding not in ("rgb8", "bgr8"):
            raise ValueError(
                f"unsupported image encoding: {msg.encoding}"
            )
        if msg.height <= 0 or msg.width <= 0:
            raise ValueError("ROI image dimensions must be positive")
        row_size = int(msg.width) * 3
        if msg.step < row_size:
            raise ValueError(
                f"ROI step {msg.step} is smaller than {row_size}"
            )
        expected_size = int(msg.height) * int(msg.step)
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if data.size != expected_size:
            raise ValueError(
                f"ROI data has {data.size} bytes, expected "
                f"{expected_size}"
            )
        rows = data.reshape(int(msg.height), int(msg.step))
        image = rows[:, :row_size].reshape(
            int(msg.height),
            int(msg.width),
            3,
        )
        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(image)

    class KfsDetectOpenCvNode(Node):
        """Classify KFS ROI messages while preserving the public API."""

        def __init__(self) -> None:
            super().__init__("kfs_detect")
            self._roi_callback_group = MutuallyExclusiveCallbackGroup()
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
            self._matcher = KfsChamferMatcher(
                self._feature_path,
                self._max_chamfer_distance,
                self._min_class_margin,
                self._conf,
            )

            self._roi_subscription = self.create_subscription(
                KfsRoiDetection,
                self._roi_topic,
                self._on_roi,
                1,
                callback_group=self._roi_callback_group,
            )
            self._status_subscription = self.create_subscription(
                String,
                self._simulation_status_topic,
                self._simulation_status_cb,
                10,
                callback_group=self._state_callback_group,
            )
            self._pose_subscription = self.create_subscription(
                PoseStamped,
                self._robot_pose_topic,
                self._robot_pose_cb,
                10,
                callback_group=self._state_callback_group,
            )
            self._raw_publisher = self.create_publisher(
                KfsRawDetections,
                "/r2/detection/raw",
                10,
            )
            self._processed_publisher = self.create_publisher(
                KfsProcessedDetection,
                "/r2/detection/processed",
                10,
            )
            self._visualization_publisher = self.create_publisher(
                Image,
                self._visualization_topic,
                10,
            )
            self._service = self.create_service(
                GetKfsType,
                self._vote_service_name,
                self._handle_get_kfs_type,
                callback_group=self._service_callback_group,
            )
            self.get_logger().info(
                "Loaded OpenCV KFS Chamfer features from "
                f"{self._feature_path}"
            )
            if self._simulation_state_detection:
                self.get_logger().warn(
                    "KFS detection service simulation-state mode is "
                    "enabled; ROI processing remains active"
                )

        def _declare_parameters(self) -> None:
            self.declare_parameter("feature_path", "")
            self.declare_parameter("roi_topic", "/r2/kfs/roi")
            self.declare_parameter("conf", 0.5)
            self.declare_parameter("max_chamfer_distance", 0.015)
            self.declare_parameter("min_class_margin", 0.003)
            self.declare_parameter(
                "visualization_topic",
                "/r2/detection/viz",
            )
            self.declare_parameter("visualization_enabled", False)
            self.declare_parameter(
                "vote_service_name",
                "/r2/detection/get_type",
            )
            self.declare_parameter("default_vote_timeout_sec", 10.0)
            self.declare_parameter("simulation_state_detection", False)
            self.declare_parameter(
                "simulation_status_topic",
                "/simulation/status",
            )
            self.declare_parameter(
                "robot_pose_topic",
                "/r2/pose_feedback",
            )
            self.declare_parameter("simulation_team", "blue")
            self.declare_parameter(
                "grid_x",
                [-2.6, -1.4, -0.2, 1.0, 2.2, 3.4],
            )
            self.declare_parameter(
                "grid_y",
                [-4.2, -3.0, -1.8],
            )
            self.declare_parameter(
                "meilin_x",
                [2.2, 1.0, -0.2, -1.4],
            )
            self.declare_parameter("grid_pitch", 1.2)
            self.declare_parameter("cell_snap_tolerance", 0.55)

        def _load_parameters(self) -> None:
            from ament_index_python.packages import (
                get_package_share_directory,
            )

            raw_path = str(self.get_parameter("feature_path").value)
            feature_path = Path(raw_path).expanduser()
            package_feature_dir = (
                Path(get_package_share_directory("robot_r2_detect"))
                / "features"
            )
            if raw_path == "":
                feature_path = (
                    package_feature_dir
                    / "kfs_shape_templates.npz"
                )
            elif not feature_path.exists():
                candidate = package_feature_dir / feature_path.name
                if candidate.exists():
                    feature_path = candidate
            if not feature_path.exists():
                raise FileNotFoundError(
                    f"KFS feature archive not found: {raw_path!r}; "
                    f"checked {feature_path}"
                )
            self._feature_path = feature_path
            self._roi_topic = str(
                self.get_parameter("roi_topic").value
            )
            self._conf = float(self.get_parameter("conf").value)
            if not math.isfinite(self._conf) or not 0.0 <= self._conf <= 1.0:
                raise ValueError("conf must be finite and in [0, 1]")
            self._max_chamfer_distance = self._positive_parameter(
                "max_chamfer_distance"
            )
            self._min_class_margin = self._positive_parameter(
                "min_class_margin"
            )
            self._visualization_topic = str(
                self.get_parameter("visualization_topic").value
            )
            self._visualization_enabled = bool(
                self.get_parameter("visualization_enabled").value
            )
            self._vote_service_name = str(
                self.get_parameter("vote_service_name").value
            )
            self._default_vote_timeout_sec = self._positive_parameter(
                "default_vote_timeout_sec"
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

        def _positive_parameter(self, name: str) -> float:
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
            return value

        def _finite_array_parameter(
            self,
            name: str,
        ) -> tuple[float, ...]:
            values = tuple(
                float(value)
                for value in self.get_parameter(name).value
            )
            if not values or not all(
                math.isfinite(value) for value in values
            ):
                raise ValueError(f"{name} must contain finite values")
            return values

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
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y
            )
            cos_yaw = 1.0 - 2.0 * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z
            )
            return math.atan2(sin_yaw, cos_yaw)

        @staticmethod
        def _nearest_value(
            value: float,
            candidates: tuple[float, ...],
        ):
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
                raise RuntimeError(
                    "Cached KFS placements are unavailable"
                )
            if robot_pose is None:
                raise RuntimeError("Robot pose is unavailable")

            robot_x, robot_y, robot_yaw = robot_pose
            cell_x, error_x = self._nearest_value(
                robot_x,
                self._grid_x,
            )
            cell_y, error_y = self._nearest_value(
                robot_y,
                self._grid_y,
            )
            if math.hypot(error_x, error_y) > self._cell_snap_tolerance:
                raise RuntimeError(
                    f"Robot at ({robot_x:.3f}, {robot_y:.3f}) is not "
                    "near a configured grid cell"
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
                target_x,
                self._meilin_x,
            )
            meilin_y, meilin_y_error = self._nearest_value(
                target_y,
                self._grid_y,
            )
            if meilin_x_error > 1e-6 or meilin_y_error > 1e-6:
                return "", "", ""

            row = self._meilin_x.index(meilin_x)
            column = self._grid_y.index(meilin_y)
            location = (
                f"{self._simulation_team}_meilin_"
                f"{row * 3 + column + 1}"
            )
            model_name = placements.get(location, "")
            return (
                _classify_kfs_model(model_name),
                location,
                model_name,
            )

        def _handle_simulation_state_detection(
            self,
            request,
            response,
        ):
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
                    self._latest_placements is None
                    or self._robot_pose is None
                ):
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        response.success = False
                        response.message = (
                            "Simulation state detection timed out waiting "
                            "for KFS status and robot pose"
                        )
                        response.class_name = ""
                        return response
                    self._state_condition.wait(
                        timeout=min(remaining, 0.1)
                    )

                if self._cached_placements is None:
                    self._cached_placements = dict(
                        self._latest_placements
                    )
                    self._cached_seed = self._latest_seed
                    self.get_logger().info(
                        "Froze simulation KFS layout on first "
                        f"detection: seed={self._cached_seed}, "
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

        def _on_roi(self, msg: KfsRoiDetection) -> None:
            if not msg.valid:
                self._publish_empty(msg)
                return
            try:
                self._validate_roi_geometry(msg)
                roi = image_message_to_bgr(msg.roi)
                result = self._matcher.match(roi)
            except (cv2.error, KeyError, TypeError, ValueError) as exc:
                self.get_logger().error(
                    f"Failed to classify KFS ROI: {exc}"
                )
                self._publish_empty(msg)
                return

            raw_message = KfsRawDetections()
            raw_message.header = msg.header
            raw_message.boxes = [
                KfsRawBox(
                    class_name=result.class_name,
                    class_id=result.class_id,
                    confidence=result.confidence,
                    x1=msg.x1,
                    y1=msg.y1,
                    x2=msg.x2,
                    y2=msg.y2,
                )
            ]
            self._raw_publisher.publish(raw_message)

            processed = self._make_processed_message(msg)
            if result.accepted:
                processed.class_name = result.class_name
                processed.confidence = result.confidence
                processed.x1 = msg.x1
                processed.y1 = msg.y1
                processed.x2 = msg.x2
                processed.y2 = msg.y2
                processed.center_u = msg.center_u
                processed.center_v = msg.center_v
                processed.center_offset_x = msg.center_offset_x
                processed.center_offset_y = msg.center_offset_y
                self._record_vote_sample(result.class_name)
            self._processed_publisher.publish(processed)
            if self._visualization_enabled:
                self._publish_visualization(msg.header, roi, result)

        @staticmethod
        def _validate_roi_geometry(msg: KfsRoiDetection) -> None:
            if msg.image_width <= 0 or msg.image_height <= 0:
                raise ValueError("source image dimensions are invalid")
            if (
                msg.x1 < 0
                or msg.y1 < 0
                or msg.x2 < msg.x1
                or msg.y2 < msg.y1
                or msg.x2 >= msg.image_width
                or msg.y2 >= msg.image_height
            ):
                raise ValueError("ROI coordinates are outside source image")
            if (
                int(msg.roi.width) != msg.x2 - msg.x1 + 1
                or int(msg.roi.height) != msg.y2 - msg.y1 + 1
            ):
                raise ValueError(
                    "ROI image dimensions do not match its coordinates"
                )

        @staticmethod
        def _make_processed_message(
            msg: KfsRoiDetection,
        ) -> KfsProcessedDetection:
            processed = KfsProcessedDetection()
            processed.header = msg.header
            processed.image_width = msg.image_width
            processed.image_height = msg.image_height
            return processed

        def _publish_empty(self, msg: KfsRoiDetection) -> None:
            raw = KfsRawDetections()
            raw.header = msg.header
            raw.boxes = []
            self._raw_publisher.publish(raw)
            self._processed_publisher.publish(
                self._make_processed_message(msg)
            )

        def _publish_visualization(
            self,
            header,
            roi: np.ndarray,
            result: KfsMatchResult,
        ) -> None:
            size = result.query_mask.shape[0]
            panel_size = size * 2
            roi_panel = cv2.resize(
                roi,
                (panel_size, panel_size),
                interpolation=(
                    cv2.INTER_AREA
                    if max(roi.shape[:2]) > panel_size
                    else cv2.INTER_LINEAR
                ),
            )
            query_panel = cv2.cvtColor(
                cv2.resize(
                    result.query_mask,
                    (panel_size, panel_size),
                    interpolation=cv2.INTER_NEAREST,
                ),
                cv2.COLOR_GRAY2BGR,
            )
            template_panel = cv2.cvtColor(
                cv2.resize(
                    result.template_mask,
                    (panel_size, panel_size),
                    interpolation=cv2.INTER_NEAREST,
                ),
                cv2.COLOR_GRAY2BGR,
            )
            difference = cv2.absdiff(
                result.query_mask,
                result.template_mask,
            )
            difference_panel = cv2.applyColorMap(
                cv2.resize(
                    difference,
                    (panel_size, panel_size),
                    interpolation=cv2.INTER_NEAREST,
                ),
                cv2.COLORMAP_TURBO,
            )
            panels = [
                ("ROI", roi_panel),
                ("QUERY", query_panel),
                (result.template_name, template_panel),
                ("DIFF", difference_panel),
            ]
            for label, panel in panels:
                cv2.putText(
                    panel,
                    label,
                    (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 0, 0),
                    3,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    panel,
                    label,
                    (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
            visualization = np.hstack(
                [panel for _, panel in panels]
            )
            status = (
                f"{result.class_name} conf={result.confidence:.3f} "
                f"d={result.best_distance:.5f} "
                f"margin={result.class_margin:.5f} "
                f"{'ACCEPT' if result.accepted else 'REJECT'}"
            )
            cv2.putText(
                visualization,
                status,
                (8, visualization.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 0),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                visualization,
                status,
                (8, visualization.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0) if result.accepted else (0, 0, 255),
                1,
                cv2.LINE_AA,
            )

            message = Image()
            message.header = header
            message.height = visualization.shape[0]
            message.width = visualization.shape[1]
            message.encoding = "bgr8"
            message.is_bigendian = 0
            message.step = visualization.shape[1] * 3
            message.data = visualization.tobytes()
            self._visualization_publisher.publish(message)

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
                    request,
                    response,
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
                                "ROS shutdown while collecting "
                                "accepted detections"
                            )
                            break
                        remaining = deadline - time.monotonic()
                        if remaining <= 0.0:
                            failure_message = (
                                "Detection vote timed out after "
                                f"collecting {len(self._vote_samples)}/"
                                f"{sample_count} accepted samples"
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
                "Selected most frequent class from "
                f"{sample_count} accepted samples"
            )
            response.class_name = _select_most_frequent_class(
                samples
            )
            return response

    rclpy.init(args=args)
    node = KfsDetectOpenCvNode()
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
