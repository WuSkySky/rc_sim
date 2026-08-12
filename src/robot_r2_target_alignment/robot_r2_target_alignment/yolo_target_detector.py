#!/usr/bin/env python3
"""Detect and select one target from bounded camera frames with YOLO11."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import (
    FloatingPointRange,
    IntegerRange,
    ParameterDescriptor,
    SetParametersResult,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from robot_r2_interfaces.msg import CameraFrame, TargetDetection
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from robot_r2_target_alignment.camera_frame import (
    bgr_to_image,
    message_header,
    message_to_bgr,
)
from robot_r2_target_alignment.detector_core import (
    DetectionCandidate,
    inference_wait_seconds,
    parse_id_filter,
    parse_name_filter,
    select_target,
)


@dataclass(frozen=True)
class DetectorConfig:
    """One atomically replaceable detector configuration."""

    input_video_topic: str
    model_path: Path
    input_size: int
    device: str
    quantize: int
    confidence: float
    iou: float
    max_detections: int
    rate_hz: float
    class_names: tuple[str, ...]
    class_ids: tuple[int, ...]
    max_track_distance_ratio: float
    switch_confidence_margin: float
    visualization_enabled: bool

    @property
    def model_signature(self) -> tuple[Path, int, str, int]:
        return self.model_path, self.input_size, self.device, self.quantize


def _float_descriptor(
    description: str,
    minimum: float,
    maximum: float,
) -> ParameterDescriptor:
    return ParameterDescriptor(
        description=description,
        floating_point_range=[
            FloatingPointRange(
                from_value=minimum,
                to_value=maximum,
                step=0.0,
            )
        ],
    )


def _integer_descriptor(
    description: str,
    minimum: int,
    maximum: int,
) -> ParameterDescriptor:
    return ParameterDescriptor(
        description=description,
        integer_range=[
            IntegerRange(
                from_value=minimum,
                to_value=maximum,
                step=1,
            )
        ],
    )


class YoloTargetDetector(Node):
    """Run YOLO inference off the executor and publish the selected target."""

    PARAMETER_NAMES = (
        "input_video_topic",
        "model.path",
        "model.input_size",
        "model.device",
        "model.quantize",
        "inference.confidence",
        "inference.iou",
        "inference.max_detections",
        "inference.rate_hz",
        "target.class_names",
        "target.class_ids",
        "target.max_track_distance_ratio",
        "target.switch_confidence_margin",
        "visualization.enabled",
    )

    def __init__(self) -> None:
        super().__init__("yolo_target_detector")
        self._runtime_lock = threading.RLock()
        self._inference_lock = threading.Lock()
        self._frame_condition = threading.Condition()
        self._stop_event = threading.Event()
        self._latest_frame: CameraFrame | None = None
        self._latest_frame_sequence = 0
        self._handled_frame_sequence = 0
        self._previous_target: DetectionCandidate | None = None
        self._last_inference_started_at: float | None = None
        self._last_error_log_at = 0.0

        self._declare_parameters()
        self._parameter_values = {
            name: self.get_parameter(name).value
            for name in self.PARAMETER_NAMES
        }
        self._config = self._make_config(self._parameter_values)
        self._model = self._load_model(self._config)
        self._validate_target_filters(self._config, self._model)

        self._image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        detection_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscription = self.create_subscription(
            CameraFrame,
            self._config.input_video_topic,
            self._on_image,
            self._image_qos,
        )
        self._detection_publisher = self.create_publisher(
            TargetDetection,
            "detections",
            detection_qos,
        )
        self._debug_publisher = self.create_publisher(
            Image,
            "debug_image",
            self._image_qos,
        )
        self.add_on_set_parameters_callback(self._on_parameters_changed)

        self._worker = threading.Thread(
            target=self._inference_loop,
            name="yolo_target_inference",
            daemon=False,
        )
        self._worker.start()
        self.get_logger().info(
            f"Loaded YOLO detector from {self._config.model_path} "
            f"on device {self._config.device}; input topic "
            f"{self._config.input_video_topic}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "input_video_topic",
            "/r2/front_camera/image_raw",
            ParameterDescriptor(
                description="CameraFrame input video topic."
            ),
        )
        self.declare_parameter(
            "model.path",
            "",
            ParameterDescriptor(
                description=(
                    "YOLO11 detection weights as an absolute path, package URI, "
                    "or package-name-relative path."
                )
            ),
        )
        self.declare_parameter(
            "model.input_size",
            640,
            _integer_descriptor("YOLO inference image size.", 32, 4096),
        )
        self.declare_parameter(
            "model.device",
            "0",
            ParameterDescriptor(description="Ultralytics inference device."),
        )
        self.declare_parameter(
            "model.quantize",
            32,
            _integer_descriptor(
                "Ultralytics inference precision: 16 for FP16, 32 for FP32.",
                16,
                32,
            ),
        )
        self.declare_parameter(
            "inference.confidence",
            0.5,
            _float_descriptor("Minimum YOLO confidence.", 0.0, 1.0),
        )
        self.declare_parameter(
            "inference.iou",
            0.7,
            _float_descriptor("YOLO non-maximum suppression IoU.", 0.0, 1.0),
        )
        self.declare_parameter(
            "inference.max_detections",
            20,
            _integer_descriptor("Maximum detections per frame.", 1, 1000),
        )
        self.declare_parameter(
            "inference.rate_hz",
            15.0,
            _float_descriptor("Maximum inference rate.", 0.1, 120.0),
        )
        self.declare_parameter(
            "target.class_names",
            "",
            ParameterDescriptor(
                description="Comma-separated allowed class names; empty allows all."
            ),
        )
        self.declare_parameter(
            "target.class_ids",
            "",
            ParameterDescriptor(
                description="Comma-separated allowed class IDs; empty allows all."
            ),
        )
        self.declare_parameter(
            "target.max_track_distance_ratio",
            0.2,
            _float_descriptor(
                "Maximum previous-target displacement divided by image diagonal.",
                0.0,
                1.0,
            ),
        )
        self.declare_parameter(
            "target.switch_confidence_margin",
            0.15,
            _float_descriptor(
                "Confidence advantage required to switch tracked targets.",
                0.0,
                1.0,
            ),
        )
        self.declare_parameter(
            "visualization.enabled",
            False,
            ParameterDescriptor(description="Publish an annotated debug image."),
        )

    @staticmethod
    def _resolve_model_path(raw_path: str) -> Path:
        value = raw_path.strip()
        if not value:
            path = Path(
                get_package_share_directory("robot_r2_detect")
            ) / "model" / "duantou.pt"
        elif value.startswith("package://"):
            resource = value.removeprefix("package://")
            package_name, separator, relative_path = resource.partition("/")
            if not separator or not package_name or not relative_path:
                raise ValueError(
                    "model.path package URI must include a package and file path"
                )
            path = Path(get_package_share_directory(package_name)) / relative_path
        else:
            requested = Path(value).expanduser()
            if requested.is_absolute():
                path = requested
            elif len(requested.parts) >= 2:
                package_name = requested.parts[0]
                relative_path = Path(*requested.parts[1:])
                try:
                    package_share = Path(
                        get_package_share_directory(package_name)
                    )
                except LookupError:
                    package_share = Path(
                        get_package_share_directory(
                            "robot_r2_target_alignment"
                        )
                    ) / "model"
                    relative_path = requested
                path = package_share / relative_path
            else:
                path = Path(
                    get_package_share_directory(
                        "robot_r2_target_alignment"
                    )
                ) / "model" / requested
        path = path.resolve()
        if not path.is_file():
            raise FileNotFoundError(
                f"YOLO model not found: {path}. Add duantou.pt to "
                "robot_r2_detect/model before building, or set model.path to "
                "an existing absolute path."
            )
        if path.suffix.lower() not in (".pt", ".onnx", ".engine"):
            raise ValueError("model.path must use .pt, .onnx, or .engine weights")
        return path

    @classmethod
    def _make_config(cls, values: dict[str, object]) -> DetectorConfig:
        config = DetectorConfig(
            input_video_topic=str(values["input_video_topic"]).strip(),
            model_path=cls._resolve_model_path(str(values["model.path"])),
            input_size=int(values["model.input_size"]),
            device=str(values["model.device"]).strip(),
            quantize=int(values["model.quantize"]),
            confidence=float(values["inference.confidence"]),
            iou=float(values["inference.iou"]),
            max_detections=int(values["inference.max_detections"]),
            rate_hz=float(values["inference.rate_hz"]),
            class_names=parse_name_filter(str(values["target.class_names"])),
            class_ids=parse_id_filter(str(values["target.class_ids"])),
            max_track_distance_ratio=float(
                values["target.max_track_distance_ratio"]
            ),
            switch_confidence_margin=float(
                values["target.switch_confidence_margin"]
            ),
            visualization_enabled=bool(values["visualization.enabled"]),
        )
        numeric = (
            config.confidence,
            config.iou,
            config.rate_hz,
            config.max_track_distance_ratio,
            config.switch_confidence_margin,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("detector numeric parameters must be finite")
        if not config.device:
            raise ValueError("model.device must not be empty")
        if not config.input_video_topic:
            raise ValueError("input_video_topic must not be empty")
        if config.input_size <= 0 or config.max_detections <= 0:
            raise ValueError("model size and maximum detections must be positive")
        if config.input_size % 32 != 0:
            raise ValueError("model.input_size must be a multiple of 32")
        if config.quantize not in (16, 32):
            raise ValueError("model.quantize must be 16 or 32")
        if config.device.lower() == "cpu" and config.quantize == 16:
            raise ValueError("model.quantize must be 32 on the CPU")
        if not 0.0 <= config.confidence <= 1.0:
            raise ValueError("inference.confidence must be in [0, 1]")
        if not 0.0 <= config.iou <= 1.0:
            raise ValueError("inference.iou must be in [0, 1]")
        if config.rate_hz <= 0.0:
            raise ValueError("inference.rate_hz must be positive")
        if not 0.0 <= config.max_track_distance_ratio <= 1.0:
            raise ValueError(
                "target.max_track_distance_ratio must be in [0, 1]"
            )
        if not 0.0 <= config.switch_confidence_margin <= 1.0:
            raise ValueError(
                "target.switch_confidence_margin must be in [0, 1]"
            )
        return config

    @staticmethod
    def _load_model(config: DetectorConfig):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "Missing ultralytics; install this package's requirements.txt"
            ) from exc
        model = YOLO(str(config.model_path), task="detect")
        warmup = np.zeros(
            (config.input_size, config.input_size, 3),
            dtype=np.uint8,
        )
        model.predict(
            source=[warmup],
            imgsz=config.input_size,
            device=config.device,
            quantize=config.quantize,
            verbose=False,
        )
        return model

    @staticmethod
    def _validate_target_filters(config: DetectorConfig, model) -> None:
        names = model.names
        if isinstance(names, dict):
            model_names = {
                int(class_id): str(name)
                for class_id, name in names.items()
            }
        else:
            model_names = {index: str(name) for index, name in enumerate(names)}
        missing_names = set(config.class_names) - set(model_names.values())
        missing_ids = set(config.class_ids) - set(model_names)
        if missing_names:
            raise ValueError(
                "Unknown target class names: " + ", ".join(sorted(missing_names))
            )
        if missing_ids:
            text = ", ".join(str(class_id) for class_id in sorted(missing_ids))
            raise ValueError("Unknown target class IDs: " + text)

    def _on_parameters_changed(self, parameters) -> SetParametersResult:
        proposed = dict(self._parameter_values)
        for parameter in parameters:
            if parameter.name in proposed:
                proposed[parameter.name] = parameter.value
        try:
            new_config = self._make_config(proposed)
            with self._runtime_lock:
                current_config = self._config
                current_signature = self._config.model_signature
                current_model = self._model
            new_model = None
            if new_config.model_signature != current_signature:
                with self._inference_lock:
                    new_model = self._load_model(new_config)
            self._validate_target_filters(
                new_config,
                new_model if new_model is not None else current_model,
            )
            replacement_subscription = None
            if (
                new_config.input_video_topic
                != current_config.input_video_topic
            ):
                try:
                    replacement_subscription = self.create_subscription(
                        CameraFrame,
                        new_config.input_video_topic,
                        self._on_image,
                        self._image_qos,
                    )
                except Exception as exc:
                    raise ValueError(
                        "Failed to subscribe to input_video_topic: "
                        f"{exc}"
                    ) from exc
        except (
            LookupError,
            OSError,
            RuntimeError,
            TypeError,
            ValueError,
        ) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        if replacement_subscription is not None:
            with self._frame_condition:
                self._latest_frame = None
                self._handled_frame_sequence = self._latest_frame_sequence

        previous_subscription = None
        with self._runtime_lock:
            self._config = new_config
            if new_model is not None:
                self._model = new_model
            self._parameter_values = proposed
            self._previous_target = None
            if replacement_subscription is not None:
                previous_subscription = self._subscription
                self._subscription = replacement_subscription
        if previous_subscription is not None:
            if not self.destroy_subscription(previous_subscription):
                self.get_logger().warning(
                    "Failed to destroy the previous YOLO image subscription"
                )
            self.get_logger().info(
                "YOLO input video topic changed to "
                f"{new_config.input_video_topic}"
            )
        with self._frame_condition:
            self._frame_condition.notify_all()
        return SetParametersResult(successful=True)

    def _on_image(self, message: CameraFrame) -> None:
        with self._frame_condition:
            self._latest_frame = message
            self._latest_frame_sequence += 1
            self._frame_condition.notify()

    def _inference_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._frame_condition:
                self._frame_condition.wait_for(
                    lambda: (
                        self._stop_event.is_set()
                        or self._latest_frame_sequence
                        > self._handled_frame_sequence
                    )
                )
                if self._stop_event.is_set():
                    return
                frame = self._latest_frame
                self._handled_frame_sequence = self._latest_frame_sequence
            if frame is None:
                continue

            with self._runtime_lock:
                config = self._config
                model = self._model
            wait_sec = inference_wait_seconds(
                config.rate_hz,
                self._last_inference_started_at,
                time.monotonic(),
            )
            if wait_sec > 0.0 and self._stop_event.wait(wait_sec):
                return
            with self._frame_condition:
                if (
                    self._latest_frame_sequence
                    > self._handled_frame_sequence
                ):
                    frame = self._latest_frame
                    self._handled_frame_sequence = (
                        self._latest_frame_sequence
                    )
            if frame is None:
                continue
            self._last_inference_started_at = time.monotonic()
            try:
                self._process_frame(frame, config, model)
            except Exception as exc:  # Keep the safety stream alive on bad frames.
                now = time.monotonic()
                if now - self._last_error_log_at >= 1.0:
                    self.get_logger().error(f"YOLO inference failed: {exc}")
                    self._last_error_log_at = now
                with self._runtime_lock:
                    self._previous_target = None
                self._publish_invalid(frame)

    def _process_frame(self, frame, config: DetectorConfig, model) -> None:
        image = message_to_bgr(frame)
        height, width = image.shape[:2]
        with self._inference_lock:
            results = model.predict(
                source=[image],
                imgsz=config.input_size,
                conf=config.confidence,
                iou=config.iou,
                max_det=config.max_detections,
                device=config.device,
                quantize=config.quantize,
                verbose=False,
            )
        candidates = self._candidates_from_results(results, model)
        with self._runtime_lock:
            if (
                self._stop_event.is_set()
                or self._config is not config
                or self._model is not model
            ):
                return
            previous_target = self._previous_target
        selected = select_target(
            candidates,
            config.class_names,
            config.class_ids,
            previous_target,
            width,
            height,
            config.max_track_distance_ratio,
            config.switch_confidence_margin,
        )
        with self._runtime_lock:
            if self._config is not config or self._model is not model:
                return
            self._previous_target = selected
        header = message_header(frame)
        self._publish_selected(header, width, height, selected)
        if config.visualization_enabled:
            debug = self._draw_debug(image, candidates, selected)
            self._debug_publisher.publish(bgr_to_image(debug, header))

    @staticmethod
    def _candidates_from_results(results, model) -> list[DetectionCandidate]:
        if len(results) != 1 or results[0].boxes is None:
            return []
        boxes = results[0].boxes
        coordinates = boxes.xyxy.detach().float().cpu().numpy()
        confidences = boxes.conf.detach().float().cpu().numpy()
        class_ids = boxes.cls.detach().int().cpu().numpy()
        names = model.names
        candidates = []
        for coordinates_row, confidence, class_id_value in zip(
            coordinates,
            confidences,
            class_ids,
        ):
            class_id = int(class_id_value)
            if isinstance(names, dict):
                class_name = str(names.get(class_id, class_id))
            else:
                class_name = str(names[class_id])
            candidates.append(
                DetectionCandidate(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=float(confidence),
                    x1=float(coordinates_row[0]),
                    y1=float(coordinates_row[1]),
                    x2=float(coordinates_row[2]),
                    y2=float(coordinates_row[3]),
                )
            )
        return candidates

    def _publish_selected(
        self,
        header: Header,
        width: int,
        height: int,
        selected: DetectionCandidate | None,
    ) -> None:
        message = TargetDetection()
        message.header = header
        message.image_width = width
        message.image_height = height
        message.valid = selected is not None
        if selected is not None:
            x1 = max(0, min(width - 1, int(round(selected.x1))))
            y1 = max(0, min(height - 1, int(round(selected.y1))))
            x2 = max(0, min(width - 1, int(round(selected.x2))))
            y2 = max(0, min(height - 1, int(round(selected.y2))))
            center_u = max(
                0,
                min(width - 1, int(round(selected.center_x))),
            )
            center_v = max(
                0,
                min(height - 1, int(round(selected.center_y))),
            )
            message.class_name = selected.class_name
            message.class_id = selected.class_id
            message.confidence = selected.confidence
            message.x1 = x1
            message.y1 = y1
            message.x2 = x2
            message.y2 = y2
            message.center_u = center_u
            message.center_v = center_v
            message.center_offset_x = center_u - width // 2
            message.center_offset_y = center_v - height // 2
        self._detection_publisher.publish(message)

    def _publish_invalid(self, frame: CameraFrame) -> None:
        try:
            header = message_header(frame)
        except ValueError:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
        self._publish_selected(
            header,
            max(0, int(frame.width)),
            max(0, int(frame.height)),
            None,
        )

    @staticmethod
    def _draw_debug(
        image: np.ndarray,
        candidates: list[DetectionCandidate],
        selected: DetectionCandidate | None,
    ) -> np.ndarray:
        debug = image.copy()
        height, width = debug.shape[:2]
        cv2.line(
            debug,
            (width // 2, 0),
            (width // 2, height - 1),
            (0, 255, 255),
            1,
        )
        for candidate in candidates:
            is_selected = candidate is selected
            color = (0, 255, 0) if is_selected else (128, 128, 128)
            p1 = (int(round(candidate.x1)), int(round(candidate.y1)))
            p2 = (int(round(candidate.x2)), int(round(candidate.y2)))
            cv2.rectangle(debug, p1, p2, color, 2 if is_selected else 1)
            label = f"{candidate.class_name} {candidate.confidence:.2f}"
            cv2.putText(
                debug,
                label,
                (p1[0], max(15, p1[1] - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )
        return debug

    def stop_worker(self) -> None:
        """Stop and join the owned inference thread."""
        self._stop_event.set()
        with self._frame_condition:
            self._frame_condition.notify_all()
        if self._worker.is_alive():
            self._worker.join()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = YoloTargetDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop_worker()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
