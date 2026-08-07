#!/usr/bin/env python3
"""Classify KFS ROI messages with the YOLO classification model."""

from __future__ import annotations

from collections import Counter
import math
from pathlib import Path
import threading
import time

import cv2
import numpy as np


PUBLIC_CLASS_IDS = {"r2": 0, "fake": 1, "r1": 2}


def _processing_overrun_message(
    processing_time_sec: float,
    processing_config: tuple[float, float],
) -> str | None:
    target_rate, deadline_sec = processing_config
    if processing_time_sec <= deadline_sec:
        return None
    return (
        "KFS detection ROI processing overrun: "
        f"{processing_time_sec * 1000.0:.2f} ms > "
        f"{deadline_sec * 1000.0:.2f} ms "
        f"(target {target_rate:g} Hz)"
    )


def _public_class_name(model_class_name: str) -> str:
    if model_class_name == "r1":
        return "r1"
    if model_class_name.startswith("r2_"):
        return "r2"
    if model_class_name.startswith("fake_"):
        return "fake"
    raise ValueError(f"Unsupported YOLO KFS class name: {model_class_name!r}")


def _validate_model_class_names(class_names: tuple[str, ...]) -> None:
    if len(class_names) != 31:
        raise ValueError(
            f"YOLO KFS model must contain 31 classes, got {len(class_names)}"
        )
    public_names = tuple(_public_class_name(name) for name in class_names)
    if public_names.count("r1") != 1:
        raise ValueError("YOLO KFS model must contain exactly one r1 class")
    if public_names.count("r2") != 15:
        raise ValueError("YOLO KFS model must contain exactly 15 r2 classes")
    if public_names.count("fake") != 15:
        raise ValueError("YOLO KFS model must contain exactly 15 fake classes")


def _aggregate_probabilities(
    probabilities: np.ndarray,
    class_names: tuple[str, ...],
) -> tuple[int, str, float, str]:
    values = np.asarray(probabilities, dtype=np.float32)
    if values.shape != (len(class_names),):
        raise ValueError(
            f"Expected {len(class_names)} probabilities, got {values.shape}"
        )
    if not np.all(np.isfinite(values)) or np.any(values < 0.0):
        raise ValueError("YOLO model returned invalid class probabilities")
    total = float(values.sum())
    if not math.isfinite(total) or total <= 0.0:
        raise ValueError(
            "YOLO model returned an empty probability distribution"
        )
    values = values / total

    grouped = {name: 0.0 for name in PUBLIC_CLASS_IDS}
    for probability, model_name in zip(values, class_names):
        grouped[_public_class_name(model_name)] += float(probability)

    public_name = max(grouped, key=grouped.get)
    detailed_name = class_names[int(np.argmax(values))]
    return (
        PUBLIC_CLASS_IDS[public_name],
        public_name,
        grouped[public_name],
        detailed_name,
    )


def _select_most_frequent_class(samples: list[str]) -> str:
    if not samples:
        raise ValueError("at least one class sample is required")
    counts = Counter(samples)
    highest_count = max(counts.values())
    tied = {name for name, count in counts.items() if count == highest_count}
    return next(name for name in reversed(samples) if name in tied)


class YoloKfsClassifier:
    """Own one Ultralytics classifier and classify BGR ROI images."""

    def __init__(
        self,
        model_path: Path,
        input_size: int,
        device: str,
        half: bool,
    ) -> None:
        if not model_path.is_file():
            raise FileNotFoundError(f"YOLO model not found: {model_path}")
        if model_path.suffix != ".pt":
            raise ValueError(
                f"model_path must point to a .pt file: {model_path}"
            )
        if isinstance(input_size, bool) or input_size <= 0:
            raise ValueError("model_input_size must be a positive integer")
        if not device:
            raise ValueError("device must not be empty")

        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "Missing ultralytics; install robot_r2_detect requirements"
            ) from exc

        self._model = YOLO(str(model_path), task="classify")
        raw_names = self._model.names
        self.class_names = tuple(
            str(raw_names[index]) for index in range(len(raw_names))
        )
        _validate_model_class_names(self.class_names)
        self.model_path = model_path
        self.input_size = input_size
        self.device = device
        self.half = half
        self.backend = f"Ultralytics/{device}{'/FP16' if half else '/FP32'}"

    def classify(self, image: np.ndarray) -> tuple[int, str, float, str]:
        if not isinstance(image, np.ndarray) or image.size == 0:
            raise ValueError("KFS ROI must be a non-empty NumPy array")
        if image.ndim == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        elif image.ndim != 3 or image.shape[2] not in (3, 4):
            raise ValueError(f"Unsupported KFS ROI shape: {image.shape}")
        elif image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        height, width = image.shape[:2]
        side = min(height, width)
        left = (width - side) // 2
        top = (height - side) // 2
        square = image[top:top + side, left:left + side]
        interpolation = (
            cv2.INTER_LINEAR if side < self.input_size else cv2.INTER_AREA
        )
        prepared = cv2.resize(
            square,
            (self.input_size, self.input_size),
            interpolation=interpolation,
        )
        results = self._model.predict(
            source=[prepared],
            imgsz=self.input_size,
            batch=1,
            device=self.device,
            half=self.half,
            verbose=False,
        )
        if len(results) != 1 or results[0].probs is None:
            raise RuntimeError("YOLO model returned no classification result")
        probabilities = results[0].probs.data.detach().float().cpu().numpy()
        return _aggregate_probabilities(probabilities, self.class_names)


def main(args: list[str] | None = None) -> None:
    import rclpy
    from rcl_interfaces.msg import SetParametersResult
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

    from robot_r2_detect.camera_frame import (
        bgr_to_image_message,
        camera_frame_header,
        camera_frame_to_bgr,
        camera_qos,
    )

    class KfsDetectYoloNode(Node):
        def __init__(self) -> None:
            super().__init__("kfs_detect")
            self._roi_callback_group = MutuallyExclusiveCallbackGroup()
            self._service_callback_group = MutuallyExclusiveCallbackGroup()
            self._runtime_lock = threading.Lock()
            self._vote_condition = threading.Condition()
            self._vote_active = False
            self._vote_target_count = 0
            self._vote_samples: list[str] = []

            self._declare_parameters()
            self._load_parameters()
            self._classifier = self._make_classifier(
                self._model_path,
                self._model_input_size,
                self._device,
                self._half,
            )

            image_qos = camera_qos()
            self._subscription = self.create_subscription(
                KfsRoiDetection,
                "/r2/kfs/roi",
                self._on_roi,
                image_qos,
                callback_group=self._roi_callback_group,
            )
            self._raw_publisher = self.create_publisher(
                KfsRawDetections, "/r2/detection/raw", 10
            )
            self._processed_publisher = self.create_publisher(
                KfsProcessedDetection, "/r2/detection/processed", 10
            )
            self._debug_publisher = self.create_publisher(
                Image, "/r2/detection/debug", image_qos
            )
            self._service = self.create_service(
                GetKfsType,
                "/r2/detection/get_type",
                self._handle_get_kfs_type,
                callback_group=self._service_callback_group,
            )
            self.add_on_set_parameters_callback(self._on_parameters_changed)
            self.get_logger().info(
                f"Loaded YOLO KFS classifier from {self._model_path} "
                f"with {self._classifier.backend}"
            )

        def _declare_parameters(self) -> None:
            self.declare_parameter("model_path", "")
            self.declare_parameter("model_input_size", 128)
            self.declare_parameter("device", "0")
            self.declare_parameter("half", True)
            self.declare_parameter("conf", 0.5)
            self.declare_parameter("visualization_enabled", False)
            self.declare_parameter("target_processing_rate", 30.0)
            self.declare_parameter("default_vote_timeout_sec", 10.0)

        def _load_parameters(self) -> None:
            self._model_path = self._resolve_model_path(
                str(self.get_parameter("model_path").value)
            )
            self._model_input_size = int(
                self.get_parameter("model_input_size").value
            )
            self._device = str(self.get_parameter("device").value)
            self._half = bool(self.get_parameter("half").value)
            self._conf = self._confidence(
                self.get_parameter("conf").value
            )
            self._visualization_enabled = bool(
                self.get_parameter("visualization_enabled").value
            )
            target_rate = self._positive(
                "target_processing_rate",
                self.get_parameter("target_processing_rate").value,
            )
            self._processing_config = (target_rate, 1.0 / target_rate)
            self._default_vote_timeout_sec = self._positive(
                "default_vote_timeout_sec",
                self.get_parameter("default_vote_timeout_sec").value,
            )

        @staticmethod
        def _positive(name: str, value) -> float:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be a number")
            result = float(value)
            if not math.isfinite(result) or result <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
            return result

        @staticmethod
        def _confidence(value) -> float:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError("conf must be a number")
            result = float(value)
            if not math.isfinite(result) or not 0.0 <= result <= 1.0:
                raise ValueError("conf must be finite and in [0, 1]")
            return result

        @staticmethod
        def _make_classifier(path, input_size, device, half):
            return YoloKfsClassifier(path, input_size, device, half)

        def _resolve_model_path(self, raw: str) -> Path:
            from ament_index_python.packages import get_package_share_directory

            model_dir = (
                Path(get_package_share_directory("robot_r2_detect")) / "model"
            )
            path = Path(raw).expanduser()
            if not raw:
                path = model_dir / "r2_yolo26n_cls_31.pt"
            elif not path.exists():
                candidate = model_dir / path.name
                if candidate.exists():
                    path = candidate
            if not path.is_file():
                raise FileNotFoundError(f"YOLO model not found: {path}")
            return path

        def _on_parameters_changed(self, parameters) -> SetParametersResult:
            with self._runtime_lock:
                model_path = self._model_path
                input_size = self._model_input_size
                device = self._device
                half = self._half
                conf = self._conf
                visualization_enabled = self._visualization_enabled
                processing_config = self._processing_config
                default_timeout = self._default_vote_timeout_sec

            reload_model = False
            try:
                for parameter in parameters:
                    if parameter.name == "model_path":
                        if not isinstance(parameter.value, str):
                            raise ValueError("model_path must be a string")
                        model_path = self._resolve_model_path(parameter.value)
                        reload_model = True
                    elif parameter.name == "model_input_size":
                        if isinstance(parameter.value, bool) or not isinstance(
                            parameter.value, int
                        ) or parameter.value <= 0:
                            raise ValueError(
                                "model_input_size must be a positive integer"
                            )
                        input_size = parameter.value
                        reload_model = True
                    elif parameter.name == "device":
                        if (
                            not isinstance(parameter.value, str)
                            or not parameter.value
                        ):
                            raise ValueError(
                                "device must be a non-empty string"
                            )
                        device = parameter.value
                        reload_model = True
                    elif parameter.name == "half":
                        if not isinstance(parameter.value, bool):
                            raise ValueError("half must be a boolean")
                        half = parameter.value
                        reload_model = True
                    elif parameter.name == "conf":
                        conf = self._confidence(parameter.value)
                    elif parameter.name == "visualization_enabled":
                        if not isinstance(parameter.value, bool):
                            raise ValueError(
                                "visualization_enabled must be a boolean"
                            )
                        visualization_enabled = parameter.value
                    elif parameter.name == "target_processing_rate":
                        rate = self._positive(
                            parameter.name, parameter.value
                        )
                        processing_config = (rate, 1.0 / rate)
                    elif parameter.name == "default_vote_timeout_sec":
                        default_timeout = self._positive(
                            parameter.name, parameter.value
                        )

                replacement = None
                if reload_model:
                    replacement = self._make_classifier(
                        model_path, input_size, device, half
                    )
            except (OSError, RuntimeError, TypeError, ValueError) as exc:
                return SetParametersResult(successful=False, reason=str(exc))

            with self._runtime_lock:
                if replacement is not None:
                    self._classifier = replacement
                    self._model_path = model_path
                    self._model_input_size = input_size
                    self._device = device
                    self._half = half
                self._conf = conf
                self._visualization_enabled = visualization_enabled
                self._processing_config = processing_config
                self._default_vote_timeout_sec = default_timeout
            return SetParametersResult(successful=True)

        @staticmethod
        def _validate_roi_geometry(msg) -> None:
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
                raise ValueError("ROI dimensions do not match its coordinates")

        @staticmethod
        def _processed_message(msg):
            processed = KfsProcessedDetection()
            processed.header = camera_frame_header(msg.roi)
            processed.image_width = msg.image_width
            processed.image_height = msg.image_height
            return processed

        def _publish_empty(self, msg) -> None:
            raw = KfsRawDetections()
            raw.header = camera_frame_header(msg.roi)
            self._raw_publisher.publish(raw)
            self._processed_publisher.publish(self._processed_message(msg))

        def _on_roi(self, msg: KfsRoiDetection) -> None:
            started_at = time.monotonic()
            if not msg.valid:
                self._publish_empty(msg)
                return
            try:
                self._validate_roi_geometry(msg)
                roi = camera_frame_to_bgr(msg.roi)
                with self._runtime_lock:
                    result = self._classifier.classify(roi)
                    conf_threshold = self._conf
                    visualization_enabled = self._visualization_enabled
                    processing_config = self._processing_config
            except (cv2.error, RuntimeError, TypeError, ValueError) as exc:
                self.get_logger().error(f"Failed to classify KFS ROI: {exc}")
                self._publish_empty(msg)
                return

            class_id, class_name, confidence, detailed_name = result
            raw = KfsRawDetections()
            raw.header = camera_frame_header(msg.roi)
            raw.boxes = [
                KfsRawBox(
                    class_name=class_name,
                    class_id=class_id,
                    confidence=confidence,
                    x1=msg.x1,
                    y1=msg.y1,
                    x2=msg.x2,
                    y2=msg.y2,
                )
            ]
            self._raw_publisher.publish(raw)

            processed = self._processed_message(msg)
            if confidence >= conf_threshold:
                processed.class_name = class_name
                processed.confidence = confidence
                processed.x1 = msg.x1
                processed.y1 = msg.y1
                processed.x2 = msg.x2
                processed.y2 = msg.y2
                processed.center_u = msg.center_u
                processed.center_v = msg.center_v
                processed.center_offset_x = msg.center_offset_x
                processed.center_offset_y = msg.center_offset_y
                self._record_vote_sample(class_name)
            self._processed_publisher.publish(processed)

            if visualization_enabled:
                debug = roi.copy()
                accepted = confidence >= conf_threshold
                label = (
                    f"{detailed_name} -> {class_name} {confidence:.2f}"
                )
                cv2.putText(
                    debug,
                    label,
                    (8, 26),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0) if accepted else (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                self._debug_publisher.publish(
                    bgr_to_image_message(debug, camera_frame_header(msg.roi))
                )

            warning = _processing_overrun_message(
                time.monotonic() - started_at, processing_config
            )
            if warning is not None:
                self.get_logger().warn(
                    warning,
                    throttle_duration_sec=2.0,
                )

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
                return response
            timeout = float(request.timeout_sec)
            if not math.isfinite(timeout):
                response.success = False
                response.message = "timeout_sec must be finite"
                return response
            with self._runtime_lock:
                default_timeout = self._default_vote_timeout_sec
            timeout = timeout if timeout > 0.0 else default_timeout

            deadline = time.monotonic() + timeout
            failure = ""
            samples: list[str] = []
            with self._vote_condition:
                if self._vote_active:
                    response.success = False
                    response.message = (
                        "another detection vote is already active"
                    )
                    return response
                self._vote_samples = []
                self._vote_target_count = sample_count
                self._vote_active = True
                try:
                    while len(self._vote_samples) < sample_count:
                        if not rclpy.ok():
                            failure = (
                                "ROS shutdown while collecting detections"
                            )
                            break
                        remaining = deadline - time.monotonic()
                        if remaining <= 0.0:
                            failure = (
                                "Detection vote timed out after collecting "
                                f"{len(self._vote_samples)}/"
                                f"{sample_count} samples"
                            )
                            break
                        self._vote_condition.wait(
                            timeout=min(remaining, 0.1)
                        )
                    if not failure:
                        samples = list(self._vote_samples)
                finally:
                    self._vote_active = False
                    self._vote_target_count = 0

            if failure:
                response.success = False
                response.message = failure
                return response
            response.success = True
            response.message = (
                f"Selected most frequent class from {sample_count} samples"
            )
            response.class_name = _select_most_frequent_class(samples)
            return response

    rclpy.init(args=args)
    node = KfsDetectYoloNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
