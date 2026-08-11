"""ROS-independent target-selection helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DetectionCandidate:
    """One YOLO detection in image pixel coordinates."""

    class_id: int
    class_name: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def center_x(self) -> float:
        return (self.x1 + self.x2) * 0.5

    @property
    def center_y(self) -> float:
        return (self.y1 + self.y2) * 0.5

    @property
    def area(self) -> float:
        return max(0.0, self.x2 - self.x1) * max(0.0, self.y2 - self.y1)


def parse_name_filter(value: str) -> tuple[str, ...]:
    """Parse a comma-separated class-name filter."""
    names = tuple(part.strip() for part in value.split(",") if part.strip())
    if len(set(names)) != len(names):
        raise ValueError("target.class_names contains duplicate names")
    return names


def parse_id_filter(value: str) -> tuple[int, ...]:
    """Parse a comma-separated non-negative class-id filter."""
    if not value.strip():
        return ()
    try:
        ids = tuple(int(part.strip()) for part in value.split(","))
    except ValueError as exc:
        raise ValueError(
            "target.class_ids must be comma-separated integers"
        ) from exc
    if any(class_id < 0 for class_id in ids):
        raise ValueError("target.class_ids must not contain negative values")
    if len(set(ids)) != len(ids):
        raise ValueError("target.class_ids contains duplicate values")
    return ids


def inference_wait_seconds(
    rate_hz: float,
    last_started_at: float | None,
    now: float,
) -> float:
    """Return the delay needed to limit inference start-to-start rate."""
    if not math.isfinite(rate_hz) or rate_hz <= 0.0:
        raise ValueError("inference rate must be finite and positive")
    if not math.isfinite(now):
        raise ValueError("current monotonic time must be finite")
    if last_started_at is None:
        return 0.0
    if not math.isfinite(last_started_at):
        raise ValueError("last inference start time must be finite")
    elapsed = max(0.0, now - last_started_at)
    return max(0.0, 1.0 / rate_hz - elapsed)


def validate_candidate(candidate: DetectionCandidate) -> None:
    values = (
        candidate.confidence,
        candidate.x1,
        candidate.y1,
        candidate.x2,
        candidate.y2,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("detection contains a non-finite value")
    if not 0.0 <= candidate.confidence <= 1.0:
        raise ValueError("detection confidence must be in [0, 1]")
    if candidate.x2 < candidate.x1 or candidate.y2 < candidate.y1:
        raise ValueError("detection bounding box is inverted")


def select_target(
    candidates: list[DetectionCandidate],
    class_names: tuple[str, ...],
    class_ids: tuple[int, ...],
    previous: DetectionCandidate | None,
    image_width: int,
    image_height: int,
    max_track_distance_ratio: float,
    switch_confidence_margin: float,
) -> DetectionCandidate | None:
    """Select one allowed target while avoiding unnecessary target switching."""
    if image_width <= 0 or image_height <= 0:
        raise ValueError("image dimensions must be positive")
    diagonal = math.hypot(image_width, image_height)
    allowed: list[DetectionCandidate] = []
    for candidate in candidates:
        validate_candidate(candidate)
        name_allowed = not class_names or candidate.class_name in class_names
        id_allowed = not class_ids or candidate.class_id in class_ids
        if name_allowed and id_allowed:
            allowed.append(candidate)
    if not allowed:
        return None

    best = max(allowed, key=lambda item: (item.confidence, item.area))
    if previous is None:
        return best

    same_class = [
        item for item in allowed if item.class_id == previous.class_id
    ]
    if not same_class:
        return best
    nearest = min(
        same_class,
        key=lambda item: math.hypot(
            item.center_x - previous.center_x,
            item.center_y - previous.center_y,
        ),
    )
    distance_ratio = math.hypot(
        nearest.center_x - previous.center_x,
        nearest.center_y - previous.center_y,
    ) / diagonal
    if (
        distance_ratio <= max_track_distance_ratio
        and nearest.confidence + switch_confidence_margin >= best.confidence
    ):
        return nearest
    return best
