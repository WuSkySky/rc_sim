#!/usr/bin/env python3
"""Visualize and evaluate KFS binary-mask Chamfer features."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys
from typing import Callable

import cv2
import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from robot_r2_detect.kfs_shape_features import (  # noqa: E402
    ShapeFeatureParameters,
    calculate_distance_transform,
    normalize_foreground_mask,
    symmetric_chamfer_distance,
)


CLASS_COLORS = {
    "r2": (40, 150, 40),
    "fake": (200, 100, 30),
    "r1": (40, 40, 210),
}
DEFAULT_ARCHIVE = PACKAGE_ROOT / "features" / "kfs_shape_templates.npz"
DEFAULT_OUTPUT = PACKAGE_ROOT / "features" / "visualization"


def load_archive(
    path: Path,
) -> tuple[
    list[np.ndarray],
    list[np.ndarray],
    list[str],
    list[str],
    ShapeFeatureParameters,
]:
    with np.load(path, allow_pickle=False) as archive:
        if int(archive["format_version"]) != 3:
            raise ValueError("visualizer requires feature archive version 3")
        count = int(archive["template_count"])
        labels = archive["labels"].astype(str).tolist()
        names = archive["names"].astype(str).tolist()
        parameters = ShapeFeatureParameters(
            template_size=int(archive["template_size"]),
            content_size=int(archive["content_size"]),
            adaptive_block_size=int(archive["adaptive_block_size"]),
            adaptive_c=float(archive["adaptive_c"]),
            opening_kernel_size=int(archive["opening_kernel_size"]),
            cleared_border=int(archive["cleared_border"]),
        )
        masks = [
            np.ascontiguousarray(
                archive[f"mask_{index:02d}"],
                dtype=np.uint8,
            )
            for index in range(count)
        ]
        distances = [
            np.ascontiguousarray(
                archive[f"distance_{index:02d}"],
                dtype=np.float32,
            )
            for index in range(count)
        ]
    return masks, distances, labels, names, parameters


def calculate_distance_matrix(
    masks: list[np.ndarray],
    distance_transforms: list[np.ndarray],
) -> np.ndarray:
    count = len(masks)
    matrix = np.zeros((count, count), dtype=np.float64)
    for first in range(count):
        for second in range(first + 1, count):
            distance = symmetric_chamfer_distance(
                masks[first],
                distance_transforms[first],
                masks[second],
                distance_transforms[second],
            )
            matrix[first, second] = distance
            matrix[second, first] = distance
    return matrix


def draw_mask(
    canvas: np.ndarray,
    mask: np.ndarray,
    origin: tuple[int, int],
    color: tuple[int, int, int],
    scale: int = 1,
) -> None:
    origin_x, origin_y = origin
    height, width = mask.shape
    rendered = np.zeros((height, width, 3), dtype=np.uint8)
    rendered[mask != 0] = color
    if scale != 1:
        rendered = cv2.resize(
            rendered,
            (width * scale, height * scale),
            interpolation=cv2.INTER_NEAREST,
        )
    rendered_height, rendered_width = rendered.shape[:2]
    canvas[
        origin_y:origin_y + rendered_height,
        origin_x:origin_x + rendered_width,
    ] = rendered
    cv2.rectangle(
        canvas,
        (origin_x, origin_y),
        (
            origin_x + rendered_width - 1,
            origin_y + rendered_height - 1,
        ),
        (180, 180, 180),
        1,
    )


def make_mask_overview(
    output: Path,
    masks: list[np.ndarray],
    labels: list[str],
    names: list[str],
) -> None:
    columns = 6
    rows = (len(masks) + columns - 1) // columns
    size = masks[0].shape[0]
    tile_width = size + 42
    tile_height = size + 43
    canvas = np.full(
        (rows * tile_height + 42, columns * tile_width, 3),
        248,
        dtype=np.uint8,
    )
    cv2.putText(
        canvas,
        "Normalized complete binary stroke masks",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (30, 30, 30),
        1,
        cv2.LINE_AA,
    )

    for index, (mask, label, name) in enumerate(
        zip(masks, labels, names)
    ):
        row, column = divmod(index, columns)
        origin = (
            column * tile_width + 20,
            row * tile_height + 38,
        )
        draw_mask(canvas, mask, origin, CLASS_COLORS[label])
        density = np.count_nonzero(mask) / mask.size
        cv2.putText(
            canvas,
            f"{index + 1:02d} {name} d={density:.2f}",
            (
                column * tile_width + 6,
                row * tile_height + size + 58,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (35, 35, 35),
            1,
            cv2.LINE_AA,
        )

    if not cv2.imwrite(str(output), canvas):
        raise RuntimeError(f"failed to write {output}")


def make_distance_heatmap(
    output: Path,
    matrix: np.ndarray,
    labels: list[str],
) -> None:
    count = matrix.shape[0]
    cell_size = 20
    left_margin = 60
    top_margin = 55
    bottom_margin = 80
    matrix_size = count * cell_size
    canvas = np.full(
        (
            top_margin + matrix_size + bottom_margin,
            left_margin + matrix_size + 35,
            3,
        ),
        250,
        dtype=np.uint8,
    )

    values = matrix[~np.eye(count, dtype=bool)]
    display_max = float(np.percentile(values, 95))
    normalized = np.clip(matrix / display_max, 0.0, 1.0)
    heatmap = cv2.applyColorMap(
        np.rint(normalized * 255.0).astype(np.uint8),
        cv2.COLORMAP_VIRIDIS,
    )
    heatmap[np.eye(count, dtype=bool)] = (255, 255, 255)
    heatmap = cv2.resize(
        heatmap,
        (matrix_size, matrix_size),
        interpolation=cv2.INTER_NEAREST,
    )
    canvas[
        top_margin:top_margin + matrix_size,
        left_margin:left_margin + matrix_size,
    ] = heatmap

    for index in range(count):
        coordinate = index * cell_size
        cv2.putText(
            canvas,
            f"{index + 1:02d}",
            (left_margin + coordinate + 3, top_margin - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.34,
            (35, 35, 35),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"{index + 1:02d}",
            (left_margin - 28, top_margin + coordinate + 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.34,
            (35, 35, 35),
            1,
            cv2.LINE_AA,
        )

    boundaries = [
        index
        for index in range(1, count)
        if labels[index] != labels[index - 1]
    ]
    for boundary in boundaries:
        coordinate = boundary * cell_size
        cv2.line(
            canvas,
            (left_margin + coordinate, top_margin),
            (left_margin + coordinate, top_margin + matrix_size),
            (0, 0, 230),
            2,
        )
        cv2.line(
            canvas,
            (left_margin, top_margin + coordinate),
            (left_margin + matrix_size, top_margin + coordinate),
            (0, 0, 230),
            2,
        )

    cv2.putText(
        canvas,
        "Symmetric Chamfer distance (dark = similar)",
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (30, 30, 30),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        f"Display range: 0 to p95={display_max:.5f}",
        (left_margin, top_margin + matrix_size + 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (35, 35, 35),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        "01-15=r2, 16-30=fake, 31=r1; red lines separate classes",
        (left_margin, top_margin + matrix_size + 55),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (35, 35, 35),
        1,
        cv2.LINE_AA,
    )
    if not cv2.imwrite(str(output), canvas):
        raise RuntimeError(f"failed to write {output}")


def closest_cross_class_pairs(
    matrix: np.ndarray,
    labels: list[str],
    limit: int,
) -> list[tuple[float, int, int]]:
    pairs = []
    for first in range(len(labels)):
        for second in range(first + 1, len(labels)):
            if labels[first] != labels[second]:
                pairs.append((matrix[first, second], first, second))
    return sorted(pairs)[:limit]


def make_closest_pairs(
    output: Path,
    masks: list[np.ndarray],
    labels: list[str],
    names: list[str],
    pairs: list[tuple[float, int, int]],
) -> None:
    size = masks[0].shape[0]
    row_height = size + 42
    canvas = np.full(
        (45 + len(pairs) * row_height, 520, 3),
        248,
        dtype=np.uint8,
    )
    cv2.putText(
        canvas,
        "Closest cross-class pairs (smaller = higher confusion risk)",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (30, 30, 30),
        1,
        cv2.LINE_AA,
    )

    for row, (distance, first, second) in enumerate(pairs):
        y = 42 + row * row_height
        draw_mask(
            canvas,
            masks[first],
            (42, y),
            CLASS_COLORS[labels[first]],
        )
        draw_mask(
            canvas,
            masks[second],
            (205, y),
            CLASS_COLORS[labels[second]],
        )
        cv2.putText(
            canvas,
            f"{names[first]} vs {names[second]}",
            (335, y + 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (35, 35, 35),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"distance={distance:.6f}",
            (335, y + 68),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (0, 0, 180),
            1,
            cv2.LINE_AA,
        )
    if not cv2.imwrite(str(output), canvas):
        raise RuntimeError(f"failed to write {output}")


def transform_affine(
    mask: np.ndarray,
    parameters: ShapeFeatureParameters,
    matrix: np.ndarray,
) -> np.ndarray:
    transformed = cv2.warpAffine(
        mask,
        matrix,
        (parameters.template_size, parameters.template_size),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    return normalize_foreground_mask(transformed, parameters)


def centered_linear_matrix(
    size: int,
    scale_x: float = 1.0,
    scale_y: float = 1.0,
    shear_x: float = 0.0,
) -> np.ndarray:
    center = (size - 1) / 2.0
    linear = np.asarray(
        [[scale_x, shear_x], [0.0, scale_y]],
        dtype=np.float32,
    )
    center_point = np.asarray([center, center], dtype=np.float32)
    translation = center_point - linear @ center_point
    return np.hstack((linear, translation.reshape(2, 1)))


def rotation_matrix(size: int, angle: float) -> np.ndarray:
    center = ((size - 1) / 2.0, (size - 1) / 2.0)
    return cv2.getRotationMatrix2D(center, angle, 1.0)


def evaluate_synthetic_robustness(
    masks: list[np.ndarray],
    distances: list[np.ndarray],
    labels: list[str],
    parameters: ShapeFeatureParameters,
) -> list[tuple[str, int, int, float, float]]:
    size = parameters.template_size
    scenarios: list[
        tuple[str, Callable[[np.ndarray], np.ndarray]]
    ] = [
        (
            "anisotropic_5pct",
            lambda mask: transform_affine(
                mask,
                parameters,
                centered_linear_matrix(size, 1.05, 0.95),
            ),
        ),
        (
            "anisotropic_10pct",
            lambda mask: transform_affine(
                mask,
                parameters,
                centered_linear_matrix(size, 1.10, 0.90),
            ),
        ),
        (
            "shear_5pct",
            lambda mask: transform_affine(
                mask,
                parameters,
                centered_linear_matrix(size, shear_x=0.05),
            ),
        ),
        (
            "shear_10pct",
            lambda mask: transform_affine(
                mask,
                parameters,
                centered_linear_matrix(size, shear_x=0.10),
            ),
        ),
        (
            "rotation_3deg",
            lambda mask: transform_affine(
                mask,
                parameters,
                rotation_matrix(size, 3.0),
            ),
        ),
        (
            "rotation_6deg",
            lambda mask: transform_affine(
                mask,
                parameters,
                rotation_matrix(size, 6.0),
            ),
        ),
    ]

    results = []
    for name, transform in scenarios:
        exact_count = 0
        class_count = 0
        best_distances = []
        class_margins = []
        for expected, mask in enumerate(masks):
            query_mask = transform(mask)
            query_distance = calculate_distance_transform(query_mask)
            scores = np.asarray(
                [
                    symmetric_chamfer_distance(
                        query_mask,
                        query_distance,
                        candidate_mask,
                        candidate_distance,
                    )
                    for candidate_mask, candidate_distance in zip(
                        masks,
                        distances,
                    )
                ],
                dtype=np.float64,
            )
            selected = int(np.argmin(scores))
            exact_count += selected == expected
            class_count += labels[selected] == labels[expected]
            best_distances.append(float(scores[selected]))
            per_class = {
                label: float(
                    min(
                        score
                        for score, candidate_label in zip(scores, labels)
                        if candidate_label == label
                    )
                )
                for label in set(labels)
            }
            ordered_classes = sorted(per_class.values())
            class_margins.append(
                ordered_classes[1] - ordered_classes[0]
            )
        results.append(
            (
                name,
                exact_count,
                class_count,
                float(np.median(best_distances)),
                float(np.median(class_margins)),
            )
        )
    return results


def save_distance_csv(
    output: Path,
    matrix: np.ndarray,
    names: list[str],
) -> None:
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["template", *names])
        for name, row in zip(names, matrix):
            writer.writerow([name, *(f"{value:.9f}" for value in row)])


def distribution_summary(values: list[float]) -> str:
    data = np.asarray(values, dtype=np.float64)
    return (
        f"min={data.min():.6f}, "
        f"p10={np.percentile(data, 10):.6f}, "
        f"median={np.median(data):.6f}, "
        f"p90={np.percentile(data, 90):.6f}, "
        f"max={data.max():.6f}"
    )


def save_quality_report(
    output: Path,
    masks: list[np.ndarray],
    labels: list[str],
    names: list[str],
    matrix: np.ndarray,
    pairs: list[tuple[float, int, int]],
    robustness: list[tuple[str, int, int, float, float]],
) -> None:
    within_r2 = []
    within_fake = []
    cross_r2_fake = []
    r1_other = []
    for first in range(len(labels)):
        for second in range(first + 1, len(labels)):
            distance = float(matrix[first, second])
            pair_labels = {labels[first], labels[second]}
            if labels[first] == labels[second] == "r2":
                within_r2.append(distance)
            elif labels[first] == labels[second] == "fake":
                within_fake.append(distance)
            elif pair_labels == {"r2", "fake"}:
                cross_r2_fake.append(distance)
            elif "r1" in pair_labels:
                r1_other.append(distance)

    densities = [
        np.count_nonzero(mask) / mask.size
        for mask in masks
    ]
    lines = [
        "KFS binary-mask Chamfer feature quality report",
        "==============================================",
        f"template_count: {len(masks)}",
        (
            "class_counts: "
            f"r2={labels.count('r2')}, "
            f"fake={labels.count('fake')}, "
            f"r1={labels.count('r1')}"
        ),
        (
            "foreground_density: "
            f"min={min(densities):.4f}, "
            f"median={np.median(densities):.4f}, "
            f"max={max(densities):.4f}"
        ),
        "",
        "Pairwise symmetric Chamfer distance distributions",
        f"within_r2:       {distribution_summary(within_r2)}",
        f"within_fake:     {distribution_summary(within_fake)}",
        f"cross_r2_fake:   {distribution_summary(cross_r2_fake)}",
        f"r1_vs_other:     {distribution_summary(r1_other)}",
        "",
        "Closest cross-class pairs",
    ]
    lines.extend(
        (
            f"- {names[first]} vs {names[second]}: "
            f"{distance:.9f}"
        )
        for distance, first, second in pairs
    )
    lines.extend(
        [
            "",
            "Synthetic robustness",
            (
                "scenario: exact_template/31, correct_class/31, "
                "median_best_distance, median_class_margin"
            ),
        ]
    )
    lines.extend(
        (
            f"- {name}: {exact_count}/31, {class_count}/31, "
            f"{best_distance:.6f}, {class_margin:.6f}"
        )
        for (
            name,
            exact_count,
            class_count,
            best_distance,
            class_margin,
        ) in robustness
    )
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize KFS binary-mask Chamfer features."
    )
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--closest-pair-count", type=int, default=10)
    args = parser.parse_args()
    if args.closest_pair_count <= 0:
        parser.error("--closest-pair-count must be positive")
    return args


def main() -> None:
    args = parse_args()
    (
        masks,
        distance_transforms,
        labels,
        names,
        parameters,
    ) = load_archive(args.archive)
    matrix = calculate_distance_matrix(masks, distance_transforms)
    pairs = closest_cross_class_pairs(
        matrix,
        labels,
        args.closest_pair_count,
    )
    robustness = evaluate_synthetic_robustness(
        masks,
        distance_transforms,
        labels,
        parameters,
    )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    make_mask_overview(
        args.output_dir / "mask_overview.png",
        masks,
        labels,
        names,
    )
    make_distance_heatmap(
        args.output_dir / "chamfer_distance_heatmap.png",
        matrix,
        labels,
    )
    make_closest_pairs(
        args.output_dir / "closest_cross_class_pairs.png",
        masks,
        labels,
        names,
        pairs,
    )
    save_distance_csv(
        args.output_dir / "chamfer_distances.csv",
        matrix,
        names,
    )
    save_quality_report(
        args.output_dir / "quality_report.txt",
        masks,
        labels,
        names,
        matrix,
        pairs,
        robustness,
    )
    print(f"Saved KFS Chamfer visualizations to {args.output_dir}")


if __name__ == "__main__":
    main()
