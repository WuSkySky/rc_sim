"""AprilTag-guided LED state detection helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class TagDetection:
    """One AprilTag detection normalized for the LED pipeline."""

    tag_id: int
    corners: tuple[tuple[float, float], ...]
    center: tuple[float, float]
    decision_margin: float


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
    tag: TagDetection | None = None
    rois: tuple[LedRoi, ...] = ()
    reason: str = ""


class ApriltagDetector:
    """Lazy wrapper around pupil-apriltags."""

    def __init__(self, family: str = "tag36h11") -> None:
        self._family = family
        self._detector = None

    def detect(self, image) -> list[TagDetection]:
        """Return all tags detected in a grayscale or BGR image."""
        detector = self._get_detector()

        if image.ndim == 3:
            import cv2

            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        detections: list[TagDetection] = []
        for raw_detection in detector.detect(gray):
            corners = tuple(
                (float(point[0]), float(point[1]))
                for point in raw_detection.corners
            )
            detections.append(
                TagDetection(
                    tag_id=int(raw_detection.tag_id),
                    corners=corners,
                    center=(
                        float(raw_detection.center[0]),
                        float(raw_detection.center[1]),
                    ),
                    decision_margin=float(
                        getattr(raw_detection, "decision_margin", 0.0)
                    ),
                )
            )
        return detections

    def _get_detector(self):
        if self._detector is not None:
            return self._detector

        try:
            from pupil_apriltags import Detector
        except ImportError as exc:
            raise RuntimeError(
                "pupil-apriltags is not installed; install the dependencies "
                "from robot_r2_detect/requirements.txt"
            ) from exc

        self._detector = Detector(
            families=self._family,
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25,
        )
        return self._detector


class AprilTagLedMapper:
    """Project LED positions from the AprilTag physical plane into an image.

    LED coordinates are expressed in millimetres relative to the top-left
    corner of the tag's black border. The tag geometry follows the migration
    implementation: tag36h11 has eight black units and a one-unit white border
    on every side.
    """

    _TAG_TOTAL_UNITS = 10.0
    _TAG_BLACK_UNITS = 8.0
    _TAG_WHITE_UNITS = 1.0

    def __init__(
        self,
        tag_size_mm: float,
        led_positions_mm: Sequence[tuple[float, float]],
        led_radius_mm: float,
    ) -> None:
        if not math.isfinite(tag_size_mm) or tag_size_mm <= 0.0:
            raise ValueError("tag_size_mm must be finite and positive")
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

        self._tag_size_mm = float(tag_size_mm)
        self._led_positions_mm = tuple(normalized_positions)
        self._led_radius_mm = float(led_radius_mm)
        self._white_border_mm = (
            self._tag_size_mm
            * self._TAG_WHITE_UNITS
            / self._TAG_BLACK_UNITS
        )
        self._full_tag_mm = (
            self._tag_size_mm
            * self._TAG_TOTAL_UNITS
            / self._TAG_BLACK_UNITS
        )

    def map_rois(
        self,
        tag_corners: Sequence[tuple[float, float]],
        image_shape: tuple[int, int],
    ) -> tuple[tuple[LedRoi, ...], object | None, str]:
        """Map all configured LEDs, rejecting incomplete or invalid ROIs."""
        import cv2
        import numpy as np

        if len(tag_corners) != 4:
            return (), None, "AprilTag detection did not contain four corners"

        height, width = int(image_shape[0]), int(image_shape[1])
        if height <= 0 or width <= 0:
            return (), None, "source image dimensions are invalid"

        source_points = np.asarray(
            [
                [0.0, 0.0],
                [self._full_tag_mm, 0.0],
                [self._full_tag_mm, self._full_tag_mm],
                [0.0, self._full_tag_mm],
            ],
            dtype=np.float32,
        )
        destination_points = np.asarray(tag_corners, dtype=np.float32)
        homography, mask = cv2.findHomography(
            source_points, destination_points, method=0
        )
        if (
            homography is None
            or mask is None
            or not np.all(np.isfinite(homography))
        ):
            return (), None, "failed to estimate AprilTag homography"

        pixels_per_mm = self._pixels_per_mm(homography)
        if not math.isfinite(pixels_per_mm) or pixels_per_mm <= 0.0:
            return (), homography, "invalid AprilTag image scale"
        radius_px = max(
            1, int(round(self._led_radius_mm * pixels_per_mm))
        )

        rois: list[LedRoi] = []
        for index, (x_mm, y_mm) in enumerate(self._led_positions_mm):
            point = self._project_point(
                homography,
                self._white_border_mm + x_mm,
                self._white_border_mm + y_mm,
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

        half = self._full_tag_mm / 2.0
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
        projected = AprilTagLedMapper._project_point_float(
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
        mapper: AprilTagLedMapper,
        target_tag_id: int | None,
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
        self._target_tag_id = target_tag_id
        self._brightness_threshold = float(brightness_threshold)

    def detect(self, image) -> LedDetectionResult:
        """Return LED states for the best matching AprilTag."""
        try:
            detections = self._detector.detect(image)
        except Exception as exc:
            return LedDetectionResult(
                valid=False,
                reason=f"AprilTag detection failed: {exc}",
            )

        matching_detections = detections
        if self._target_tag_id is not None:
            matching_detections = [
                detection
                for detection in detections
                if detection.tag_id == self._target_tag_id
            ]
        if not matching_detections:
            if detections and self._target_tag_id is not None:
                detected_ids = sorted(
                    {detection.tag_id for detection in detections}
                )
                reason = (
                    f"target AprilTag {self._target_tag_id} not found; "
                    f"detected IDs: {detected_ids}"
                )
            else:
                reason = "no matching AprilTag detected"
            return LedDetectionResult(valid=False, reason=reason)

        tag = max(
            matching_detections,
            key=lambda detection: detection.decision_margin,
        )
        rois, _homography, reason = self._mapper.map_rois(
            tag.corners, image.shape[:2]
        )
        if not rois:
            return LedDetectionResult(
                valid=False,
                tag=tag,
                reason=reason,
            )

        try:
            brightness = tuple(roi_mean(image, roi) for roi in rois)
        except Exception as exc:
            return LedDetectionResult(
                valid=False,
                tag=tag,
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
            tag=tag,
            rois=rois,
        )


class TargetMatchTracker:
    """Count consecutive valid frames matching one LED target."""

    def __init__(
        self,
        target_states: Sequence[bool],
        required_frames: int = 3,
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
