"""ArUco-guided LED state detection helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class ArucoDetection:
    """One ArUco marker detection normalized for the LED pipeline."""

    marker_id: int
    corners: tuple[tuple[float, float], ...]
    center: tuple[float, float]
    area_px: float


@dataclass(frozen=True)
class LedRoi:
    """A projected LED sampling region."""

    index: int
    x_px: int
    y_px: int
    radius_px: int


@dataclass(frozen=True)
class LedDetectionResult:
    """Result of processing one source image."""

    valid: bool
    states: tuple[bool, ...] = ()
    brightness: tuple[float, ...] = ()
    marker: ArucoDetection | None = None
    rois: tuple[LedRoi, ...] = ()
    reason: str = ""


class ArucoDetector:
    """Detect markers from one predefined OpenCV ArUco dictionary."""

    def __init__(self, dictionary_name: str = "DICT_4X4_50") -> None:
        import cv2

        if not hasattr(cv2, "aruco"):
            raise RuntimeError(
                "OpenCV was built without the aruco module"
            )
        dictionary_id = getattr(cv2.aruco, dictionary_name, None)
        if dictionary_id is None:
            raise ValueError(
                f"unsupported ArUco dictionary: {dictionary_name}"
            )
        self._dictionary = cv2.aruco.getPredefinedDictionary(
            dictionary_id
        )
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self._parameters = cv2.aruco.DetectorParameters_create()
        else:
            self._parameters = cv2.aruco.DetectorParameters()
        self._detector = (
            cv2.aruco.ArucoDetector(
                self._dictionary,
                self._parameters,
            )
            if hasattr(cv2.aruco, "ArucoDetector")
            else None
        )

    def detect(self, image) -> list[ArucoDetection]:
        """Return all ArUco markers detected in a grayscale or BGR image."""
        import cv2
        import numpy as np

        if image.ndim == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        if self._detector is not None:
            marker_corners, marker_ids, _rejected = (
                self._detector.detectMarkers(gray)
            )
        else:
            marker_corners, marker_ids, _rejected = (
                cv2.aruco.detectMarkers(
                    gray,
                    self._dictionary,
                    parameters=self._parameters,
                )
            )
        if marker_ids is None:
            return []

        detections: list[ArucoDetection] = []
        for raw_corners, raw_id in zip(
            marker_corners, marker_ids.reshape(-1)
        ):
            points = np.asarray(
                raw_corners, dtype=np.float32
            ).reshape(4, 2)
            detections.append(
                ArucoDetection(
                    marker_id=int(raw_id),
                    corners=tuple(
                        (float(point[0]), float(point[1]))
                        for point in points
                    ),
                    center=(
                        float(np.mean(points[:, 0])),
                        float(np.mean(points[:, 1])),
                    ),
                    area_px=float(abs(cv2.contourArea(points))),
                )
            )
        return detections


class ArucoLedMapper:
    """Project marker-relative LED positions into the source image.

    LED coordinates are expressed in millimetres relative to the ArUco
    marker's top-left outer corner. The detected marker corners directly
    represent a square whose side length is ``marker_size_mm``.
    """

    def __init__(
        self,
        marker_size_mm: float,
        led_positions_mm: Sequence[tuple[float, float]],
        led_radius_mm: float,
    ) -> None:
        if not math.isfinite(marker_size_mm) or marker_size_mm <= 0.0:
            raise ValueError("marker_size_mm must be finite and positive")
        if not math.isfinite(led_radius_mm) or led_radius_mm <= 0.0:
            raise ValueError("led_radius_mm must be finite and positive")
        if not led_positions_mm:
            raise ValueError("at least one LED position is required")

        normalized_positions: list[tuple[float, float]] = []
        for position in led_positions_mm:
            if len(position) != 2:
                raise ValueError("each LED position must contain x and y")
            x_mm, y_mm = float(position[0]), float(position[1])
            if not math.isfinite(x_mm) or not math.isfinite(y_mm):
                raise ValueError("LED positions must be finite")
            normalized_positions.append((x_mm, y_mm))

        self._marker_size_mm = float(marker_size_mm)
        self._led_positions_mm = tuple(normalized_positions)
        self._led_radius_mm = float(led_radius_mm)

    def map_rois(
        self,
        marker_corners: Sequence[tuple[float, float]],
        image_shape: tuple[int, int],
    ) -> tuple[tuple[LedRoi, ...], object | None, str]:
        """Map all configured LEDs, rejecting incomplete or invalid ROIs."""
        import cv2
        import numpy as np

        if len(marker_corners) != 4:
            return (), None, "ArUco detection did not contain four corners"

        height, width = int(image_shape[0]), int(image_shape[1])
        if height <= 0 or width <= 0:
            return (), None, "source image dimensions are invalid"

        source_points = np.asarray(
            [
                [0.0, 0.0],
                [self._marker_size_mm, 0.0],
                [self._marker_size_mm, self._marker_size_mm],
                [0.0, self._marker_size_mm],
            ],
            dtype=np.float32,
        )
        destination_points = np.asarray(
            marker_corners, dtype=np.float32
        )
        homography, mask = cv2.findHomography(
            source_points, destination_points, method=0
        )
        if (
            homography is None
            or mask is None
            or not np.all(np.isfinite(homography))
        ):
            return (), None, "failed to estimate ArUco homography"

        pixels_per_mm = self._pixels_per_mm(homography)
        if not math.isfinite(pixels_per_mm) or pixels_per_mm <= 0.0:
            return (), homography, "invalid ArUco image scale"
        radius_px = max(
            1, int(round(self._led_radius_mm * pixels_per_mm))
        )

        rois: list[LedRoi] = []
        for index, (x_mm, y_mm) in enumerate(self._led_positions_mm):
            point = self._project_point(
                homography,
                x_mm,
                y_mm,
            )
            if point is None:
                return (), homography, (
                    f"failed to project LED {index} into the image"
                )
            x_px, y_px = point
            if (
                x_px - radius_px < 0
                or y_px - radius_px < 0
                or x_px + radius_px >= width
                or y_px + radius_px >= height
            ):
                return (), homography, (
                    f"LED {index} ROI is not fully inside the image"
                )
            rois.append(
                LedRoi(
                    index=index,
                    x_px=x_px,
                    y_px=y_px,
                    radius_px=radius_px,
                )
            )

        return tuple(rois), homography, ""

    def _pixels_per_mm(self, homography) -> float:
        import numpy as np

        half = self._marker_size_mm / 2.0
        center = self._project_point_float(homography, half, half)
        horizontal = self._project_point_float(
            homography, half + 1.0, half
        )
        vertical = self._project_point_float(
            homography, half, half + 1.0
        )
        if center is None or horizontal is None or vertical is None:
            return 0.0
        horizontal_scale = np.linalg.norm(
            np.subtract(horizontal, center)
        )
        vertical_scale = np.linalg.norm(np.subtract(vertical, center))
        return float((horizontal_scale + vertical_scale) / 2.0)

    @staticmethod
    def _project_point(
        homography,
        x_mm: float,
        y_mm: float,
    ) -> tuple[int, int] | None:
        projected = ArucoLedMapper._project_point_float(
            homography, x_mm, y_mm
        )
        if projected is None:
            return None
        return int(round(projected[0])), int(round(projected[1]))

    @staticmethod
    def _project_point_float(
        homography,
        x_mm: float,
        y_mm: float,
    ) -> tuple[float, float] | None:
        import numpy as np

        matrix = np.asarray(homography, dtype=np.float64)
        projected = matrix @ np.asarray(
            [x_mm, y_mm, 1.0], dtype=np.float64
        )
        denominator = float(projected[2])
        if not math.isfinite(denominator) or abs(denominator) < 1e-12:
            return None
        x_px = float(projected[0] / denominator)
        y_px = float(projected[1] / denominator)
        if not math.isfinite(x_px) or not math.isfinite(y_px):
            return None
        return x_px, y_px


def roi_mean(image, roi: LedRoi) -> float:
    """Return mean grayscale brightness inside one square LED ROI."""
    import cv2
    import numpy as np

    region = image[
        roi.y_px - roi.radius_px:roi.y_px + roi.radius_px + 1,
        roi.x_px - roi.radius_px:roi.x_px + roi.radius_px + 1,
    ]
    if region.size == 0:
        raise ValueError(f"LED {roi.index} ROI is empty")
    if region.ndim == 3:
        region = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
    return float(np.mean(region))


class LedStateDetector:
    """Detect all configured LED states from one image."""

    def __init__(
        self,
        detector,
        mapper: ArucoLedMapper,
        target_marker_id: int | None,
        brightness_threshold: float,
    ) -> None:
        if (
            not math.isfinite(brightness_threshold)
            or not 0.0 <= brightness_threshold <= 255.0
        ):
            raise ValueError(
                "brightness_threshold must be in the range [0, 255]"
            )
        self._detector = detector
        self._mapper = mapper
        self._target_marker_id = target_marker_id
        self._brightness_threshold = float(brightness_threshold)

    def detect(self, image) -> LedDetectionResult:
        """Return LED states for the largest matching ArUco marker."""
        try:
            detections = self._detector.detect(image)
        except Exception as exc:
            return LedDetectionResult(
                valid=False,
                reason=f"ArUco detection failed: {exc}",
            )

        matching_detections = detections
        if self._target_marker_id is not None:
            matching_detections = [
                detection
                for detection in detections
                if detection.marker_id == self._target_marker_id
            ]
        if not matching_detections:
            if detections and self._target_marker_id is not None:
                detected_ids = sorted(
                    {detection.marker_id for detection in detections}
                )
                reason = (
                    f"target ArUco marker {self._target_marker_id} "
                    "not found; "
                    f"detected IDs: {detected_ids}"
                )
            else:
                reason = "no matching ArUco marker detected"
            return LedDetectionResult(valid=False, reason=reason)

        marker = max(
            matching_detections,
            key=lambda detection: detection.area_px,
        )
        rois, _homography, reason = self._mapper.map_rois(
            marker.corners, image.shape[:2]
        )
        if not rois:
            return LedDetectionResult(
                valid=False,
                marker=marker,
                reason=reason,
            )

        try:
            brightness = tuple(roi_mean(image, roi) for roi in rois)
        except Exception as exc:
            return LedDetectionResult(
                valid=False,
                marker=marker,
                rois=rois,
                reason=f"LED brightness sampling failed: {exc}",
            )
        states = tuple(
            value > self._brightness_threshold for value in brightness
        )
        return LedDetectionResult(
            valid=True,
            states=states,
            brightness=brightness,
            marker=marker,
            rois=rois,
        )


class TargetMatchTracker:
    """Count consecutive valid frames matching one LED target."""

    def __init__(
        self,
        target_states: Sequence[bool],
        required_frames: int = 5,
    ) -> None:
        if required_frames <= 0:
            raise ValueError("required_frames must be positive")
        self.target_states = tuple(bool(state) for state in target_states)
        self.required_frames = int(required_frames)
        self.count = 0

    def update(self, states: Sequence[bool] | None) -> bool:
        """Update the count and return whether the target is now stable."""
        normalized = (
            None if states is None else tuple(bool(state) for state in states)
        )
        if normalized == self.target_states:
            self.count += 1
        else:
            self.count = 0
        return self.count >= self.required_frames

"""QoS profile shared by high-bandwidth, latest-frame-only vision topics."""

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


def vision_qos() -> QoSProfile:
    """Return BEST_EFFORT, VOLATILE, KEEP_LAST(1) for image streams."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

