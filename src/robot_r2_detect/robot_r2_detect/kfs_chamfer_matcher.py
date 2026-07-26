"""Runtime loading and classification for offline KFS Chamfer features."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from robot_r2_detect.kfs_shape_features import (
    ShapeFeatureParameters,
    calculate_distance_transform,
    extract_normalized_mask,
    symmetric_chamfer_distance,
)


@dataclass(frozen=True)
class KfsMatchResult:
    """Best class and template for one normalized ROI."""

    class_name: str
    class_id: int
    template_name: str
    template_index: int
    best_distance: float
    class_margin: float
    confidence: float
    accepted: bool
    query_mask: np.ndarray
    template_mask: np.ndarray


class KfsChamferMatcher:
    """Match an ROI against the version-two offline feature archive."""

    def __init__(
        self,
        archive_path: Path,
        max_chamfer_distance: float,
        min_class_margin: float,
        confidence_threshold: float,
    ) -> None:
        if max_chamfer_distance <= 0.0:
            raise ValueError("max_chamfer_distance must be positive")
        if min_class_margin <= 0.0:
            raise ValueError("min_class_margin must be positive")
        if not 0.0 <= confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold must be in [0, 1]")
        self.archive_path = Path(archive_path)
        self.max_chamfer_distance = float(max_chamfer_distance)
        self.min_class_margin = float(min_class_margin)
        self.confidence_threshold = float(confidence_threshold)
        self._load_archive()

    def _load_archive(self) -> None:
        if not self.archive_path.exists():
            raise FileNotFoundError(
                f"KFS feature archive not found: {self.archive_path}"
            )
        with np.load(self.archive_path, allow_pickle=False) as archive:
            if int(archive["format_version"]) != 3:
                raise ValueError(
                    "KFS feature archive format_version must be 3"
                )
            if str(archive["feature_type"]) != (
                "binary_mask_symmetric_chamfer"
            ):
                raise ValueError("unsupported KFS feature archive type")

            count = int(archive["template_count"])
            labels = archive["labels"].astype(str).tolist()
            class_ids = archive["class_ids"].astype(np.int32).tolist()
            names = archive["names"].astype(str).tolist()
            if (
                count != 31
                or len(labels) != count
                or len(class_ids) != count
                or len(names) != count
            ):
                raise ValueError("invalid KFS feature archive metadata")
            expected_classes = {"r2": 0, "fake": 1, "r1": 2}
            for label, class_id in zip(labels, class_ids):
                if expected_classes.get(label) != class_id:
                    raise ValueError(
                        "KFS feature archive class mapping is invalid"
                    )
            if (
                str(archive["opening_operation"]) != "morph_open"
                or str(archive["opening_kernel_shape"]) != "rect"
            ):
                raise ValueError(
                    "unsupported KFS opening operation"
                )

            parameters = ShapeFeatureParameters(
                template_size=int(archive["template_size"]),
                content_size=int(archive["content_size"]),
                adaptive_block_size=int(
                    archive["adaptive_block_size"]
                ),
                adaptive_c=float(archive["adaptive_c"]),
                opening_kernel_size=int(
                    archive["opening_kernel_size"]
                ),
                cleared_border=int(archive["cleared_border"]),
            )
            parameters.validate()
            expected_shape = (
                parameters.template_size,
                parameters.template_size,
            )
            masks = []
            distances = []
            for index in range(count):
                mask = np.ascontiguousarray(
                    archive[f"mask_{index:02d}"],
                    dtype=np.uint8,
                )
                distance = np.ascontiguousarray(
                    archive[f"distance_{index:02d}"],
                    dtype=np.float32,
                )
                if mask.shape != expected_shape:
                    raise ValueError(
                        f"mask_{index:02d} has invalid dimensions"
                    )
                if distance.shape != expected_shape:
                    raise ValueError(
                        f"distance_{index:02d} has invalid dimensions"
                    )
                masks.append(mask)
                distances.append(distance)

        self.parameters = parameters
        self.labels = labels
        self.class_ids = class_ids
        self.names = names
        self.masks = masks
        self.distance_transforms = distances

    def match(self, image: np.ndarray) -> KfsMatchResult:
        query_mask = extract_normalized_mask(image, self.parameters)
        query_distance = calculate_distance_transform(query_mask)
        template_scores = np.asarray(
            [
                symmetric_chamfer_distance(
                    query_mask,
                    query_distance,
                    template_mask,
                    template_distance,
                )
                for template_mask, template_distance in zip(
                    self.masks,
                    self.distance_transforms,
                )
            ],
            dtype=np.float64,
        )

        class_candidates = []
        for class_name, class_id in (
            ("r2", 0),
            ("fake", 1),
            ("r1", 2),
        ):
            indices = [
                index
                for index, label in enumerate(self.labels)
                if label == class_name
            ]
            template_index = min(
                indices,
                key=lambda index: (
                    float(template_scores[index]),
                    index,
                ),
            )
            class_candidates.append(
                (
                    float(template_scores[template_index]),
                    class_id,
                    class_name,
                    template_index,
                )
            )
        class_candidates.sort(key=lambda item: (item[0], item[1]))
        (
            best_distance,
            class_id,
            class_name,
            template_index,
        ) = class_candidates[0]
        class_margin = class_candidates[1][0] - best_distance

        distance_score = 1.0 / (
            1.0 + best_distance / self.max_chamfer_distance
        )
        margin_score = class_margin / (
            class_margin + self.min_class_margin
        )
        confidence = float(min(distance_score, margin_score))
        accepted = (
            best_distance <= self.max_chamfer_distance
            and class_margin >= self.min_class_margin
            and confidence >= self.confidence_threshold
        )
        return KfsMatchResult(
            class_name=class_name,
            class_id=class_id,
            template_name=self.names[template_index],
            template_index=template_index,
            best_distance=best_distance,
            class_margin=class_margin,
            confidence=confidence,
            accepted=accepted,
            query_mask=query_mask,
            template_mask=self.masks[template_index],
        )
