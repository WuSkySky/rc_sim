#!/usr/bin/env python3
"""kfs_detect: ResNet KFS classification ROS 2 node.

Subscribes image -> ResNet -> publishes:
  /r2/detection/raw       - raw top-1 classification
  /r2/detection/processed - confidence-filtered classification
  /r2/detection/debug     - optional image with classification text
Provides:
  /r2/detection/get_type  - majority vote over the next n processed results
"""

from __future__ import annotations

from pathlib import Path, PosixPath

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
    import torch
    from geometry_msgs.msg import PoseStamped
    from rcl_interfaces.msg import SetParametersResult
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from robot_r2_interfaces.msg import (
        CameraFrame,
        KfsProcessedDetection,
        KfsRawBox,
        KfsRawDetections,
    )
    from robot_r2_interfaces.srv import GetKfsType
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from torchvision.models import resnet18

    from robot_r2_detect.camera_frame import (
        bgr_to_image_message,
        camera_frame_header,
        camera_frame_to_bgr,
        camera_qos,
    )

    class KfsDetectResnetNode(Node):
        """Classify camera frames while preserving the KFS detection API."""

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
            image_qos = camera_qos()

            self._sub = self.create_subscription(
                CameraFrame,
                self._color_topic,
                self._image_cb,
                image_qos,
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
                Image, self._viz_topic, image_qos
            )
            self._get_type_service = self.create_service(
                GetKfsType,
                self._vote_service_name,
                self._handle_get_kfs_type,
                callback_group=self._service_callback_group,
            )
            self.add_on_set_parameters_callback(
                self._on_parameters_changed
            )
            self.get_logger().info(
                f"Loaded ResNet KFS classifier from {self._model_path} "
                f"on {self._device}"
            )
            if self._simulation_state_detection:
                self.get_logger().warn(
                    "KFS detection service simulation-state mode is "
                    "enabled; the latest placement state will be frozen on "
                    "the first service request"
                )

        # ---- parameters and model ----

        def _declare_parameters(self) -> None:
            self.declare_parameter("model_path", "")
            self.declare_parameter("color_topic", "/r2/front_camera/image_raw")
            self.declare_parameter("conf", 0.65)
            self.declare_parameter(
                "visualization_topic", "/r2/detection/debug"
            )
            self.declare_parameter("visualization_enabled", False)
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
            from ament_index_python.packages import (
                get_package_share_directory,
            )

            raw = str(self.get_parameter("model_path").value)
            model_path = Path(raw).expanduser()
            if not model_path.exists() or raw == "":
                package_model_dir = (
                    Path(get_package_share_directory("robot_r2_detect"))
                    / "model"
                )
                if raw == "":
                    model_path = package_model_dir / "best_resnet.pt"
                else:
                    candidate = package_model_dir / model_path.name
                    if candidate.exists():
                        model_path = candidate

            if not model_path.exists():
                raise FileNotFoundError(
                    f"ResNet model not found: {raw}. "
                    f"Checked: CWD={Path.cwd()}, {model_path}"
                )

            self._model_path = model_path
            self._color_topic = str(self.get_parameter("color_topic").value)
            self._viz_topic = str(
                self.get_parameter("visualization_topic").value
            )
            self._visualization_enabled = bool(
                self.get_parameter("visualization_enabled").value
            )
            self._conf = float(self.get_parameter("conf").value)
            if not math.isfinite(self._conf) or not 0.0 <= self._conf <= 1.0:
                raise ValueError("conf must be finite and in [0, 1]")
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

        def _on_parameters_changed(
            self, parameters
        ) -> SetParametersResult:
            for parameter in parameters:
                if parameter.name != "visualization_enabled":
                    continue
                if not isinstance(parameter.value, bool):
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "visualization_enabled must be a boolean"
                        ),
                    )
                self._visualization_enabled = parameter.value
                state = "enabled" if parameter.value else "disabled"
                self.get_logger().info(
                    f"KFS detection visualization {state}"
                )
            return SetParametersResult(successful=True)

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
            safe_globals = getattr(
                torch.serialization, "safe_globals", None
            )
            if safe_globals is None:
                # Jetson PyTorch releases may only provide the persistent
                # allowlist API, not the newer safe_globals context manager.
                torch.serialization.add_safe_globals([PosixPath])
                checkpoint = torch.load(
                    self._model_path,
                    map_location="cpu",
                    weights_only=True,
                )
            else:
                with safe_globals([PosixPath]):
                    checkpoint = torch.load(
                        self._model_path,
                        map_location="cpu",
                        weights_only=True,
                    )
            if not isinstance(checkpoint, dict):
                raise ValueError("ResNet checkpoint must contain a dictionary")
            if checkpoint.get("arch") != "resnet18":
                raise ValueError(
                    "Unsupported ResNet architecture: "
                    f"{checkpoint.get('arch')!r}"
                )

            num_classes = int(checkpoint.get("num_classes", 0))
            class_to_idx = checkpoint.get("class_to_idx")
            if (
                num_classes <= 0
                or not isinstance(class_to_idx, dict)
                or len(class_to_idx) != num_classes
            ):
                raise ValueError("Invalid class metadata in ResNet checkpoint")

            idx_to_class: dict[int, str] = {}
            for class_name, class_id in class_to_idx.items():
                class_id = int(class_id)
                if (
                    class_id < 0
                    or class_id >= num_classes
                    or class_id in idx_to_class
                ):
                    raise ValueError(
                        "class_to_idx must contain unique contiguous IDs"
                    )
                idx_to_class[class_id] = str(class_name)
            if set(idx_to_class) != set(range(num_classes)):
                raise ValueError(
                    "class_to_idx must contain unique contiguous IDs"
                )

            image_size = int(checkpoint.get("imgsz", 0))
            mean = tuple(float(value) for value in checkpoint.get("mean", ()))
            std = tuple(float(value) for value in checkpoint.get("std", ()))
            if image_size <= 0:
                raise ValueError("imgsz must be positive")
            if (
                len(mean) != 3
                or len(std) != 3
                or not all(math.isfinite(value) for value in mean + std)
                or not all(value > 0.0 for value in std)
            ):
                raise ValueError("Invalid normalization metadata")

            model_state = checkpoint.get("model_state")
            if not isinstance(model_state, dict):
                raise ValueError("ResNet checkpoint has no model_state")

            model = resnet18(weights=None)
            model.fc = torch.nn.Linear(model.fc.in_features, num_classes)
            model.load_state_dict(model_state, strict=True)
            self._device = torch.device(
                "cuda" if torch.cuda.is_available() else "cpu"
            )
            self._model = model.to(self._device).eval()
            self._idx_to_class = idx_to_class
            self._image_size = image_size
            self._mean = torch.tensor(
                mean, dtype=torch.float32, device=self._device
            ).view(1, 3, 1, 1)
            self._std = torch.tensor(
                std, dtype=torch.float32, device=self._device
            ).view(1, 3, 1, 1)

        def _classify_image(self, image: np.ndarray) -> tuple[int, str, float]:
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            interpolation = (
                cv2.INTER_AREA
                if max(rgb.shape[:2]) > self._image_size
                else cv2.INTER_LINEAR
            )
            resized = cv2.resize(
                rgb,
                (self._image_size, self._image_size),
                interpolation=interpolation,
            )
            tensor = (
                torch.from_numpy(np.ascontiguousarray(resized))
                .permute(2, 0, 1)
                .unsqueeze(0)
                .to(device=self._device, dtype=torch.float32)
                .div_(255.0)
            )
            tensor = (tensor - self._mean) / self._std
            with torch.inference_mode():
                probabilities = torch.softmax(self._model(tensor), dim=1)
                confidence, class_id = probabilities.max(dim=1)
            class_id_value = int(class_id.item())
            return (
                class_id_value,
                self._idx_to_class[class_id_value],
                float(confidence.item()),
            )

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
            if meilin_x_error > 1e-6 or meilin_y_error > 1e-6:
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
                    self._latest_placements is None
                    or self._robot_pose is None
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
                    self._cached_placements = dict(self._latest_placements)
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

        # ---- image callback ----

        def _image_cb(self, msg: CameraFrame) -> None:
            try:
                image = camera_frame_to_bgr(msg)
                class_id, class_name, confidence = self._classify_image(image)
            except (RuntimeError, TypeError, ValueError) as exc:
                self.get_logger().error(f"KFS classification failed: {exc}")
                return

            height, width = image.shape[:2]

            raw_msg = KfsRawDetections()
            raw_msg.header = camera_frame_header(msg)
            raw_msg.boxes = [
                KfsRawBox(
                    class_name=class_name,
                    class_id=class_id,
                    confidence=confidence,
                    x1=0,
                    y1=0,
                    x2=0,
                    y2=0,
                )
            ]
            self._pub_raw.publish(raw_msg)

            processed_msg = KfsProcessedDetection()
            processed_msg.header = camera_frame_header(msg)
            processed_msg.image_width = width
            processed_msg.image_height = height
            processed_msg.class_name = (
                class_name if confidence >= self._conf else ""
            )
            processed_msg.confidence = (
                confidence if confidence >= self._conf else 0.0
            )
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

            if self._visualization_enabled:
                viz = image.copy()
                passed_threshold = confidence >= self._conf
                color = (0, 255, 0) if passed_threshold else (0, 0, 255)
                cv2.putText(
                    viz,
                    f"{class_name} {confidence:.2f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    color,
                    2,
                )
                self._pub_viz.publish(
                    bgr_to_image_message(viz, camera_frame_header(msg))
                )

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

    rclpy.init(args=args)
    node = KfsDetectResnetNode()
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
