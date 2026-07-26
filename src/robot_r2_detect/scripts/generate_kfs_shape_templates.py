#!/usr/bin/env python3
"""Generate normalized KFS stroke masks and Chamfer features."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import sys

import cv2
import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from robot_r2_detect.kfs_shape_features import (  # noqa: E402
    ShapeFeatureParameters,
    calculate_distance_transform,
    extract_normalized_mask,
    symmetric_chamfer_distance,
)


SHEET_COLUMNS = 5
SHEET_ROWS = 6
CELL_SIZE = 108
FORMAT_VERSION = 3
OPENING_KERNEL_SIZE = 3
DEFAULT_SHEET = PACKAGE_ROOT / "reference" / "kfs_r2_fake_sheet.png"
DEFAULT_R1 = PACKAGE_ROOT / "reference" / "kfs_r1.png"
DEFAULT_OUTPUT = PACKAGE_ROOT / "features" / "kfs_shape_templates.npz"


def load_color_image(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(f"failed to read image: {path}")
    return image


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_source_images(
    sheet: np.ndarray,
    r1_image: np.ndarray,
) -> tuple[list[np.ndarray], list[str], list[int], list[str]]:
    expected_shape = (
        SHEET_ROWS * CELL_SIZE,
        SHEET_COLUMNS * CELL_SIZE,
    )
    if sheet.shape[:2] != expected_shape:
        raise ValueError(
            "R2/fake sheet must be "
            f"{expected_shape[1]}x{expected_shape[0]}, got "
            f"{sheet.shape[1]}x{sheet.shape[0]}"
        )

    images: list[np.ndarray] = []
    labels: list[str] = []
    class_ids: list[int] = []
    names: list[str] = []
    for row in range(SHEET_ROWS):
        for column in range(SHEET_COLUMNS):
            top = row * CELL_SIZE
            left = column * CELL_SIZE
            images.append(
                sheet[
                    top:top + CELL_SIZE,
                    left:left + CELL_SIZE,
                ]
            )
            is_r2 = row < SHEET_ROWS // 2
            label = "r2" if is_r2 else "fake"
            index = (
                row * SHEET_COLUMNS + column + 1
                if is_r2
                else (
                    row - SHEET_ROWS // 2
                ) * SHEET_COLUMNS + column + 1
            )
            labels.append(label)
            class_ids.append(0 if is_r2 else 1)
            names.append(f"{label}_{index:02d}")

    images.append(r1_image)
    labels.append("r1")
    class_ids.append(2)
    names.append("r1_01")
    return images, labels, class_ids, names


def build_features(
    images: list[np.ndarray],
    parameters: ShapeFeatureParameters,
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    masks = [
        extract_normalized_mask(image, parameters)
        for image in images
    ]
    distance_transforms = [
        calculate_distance_transform(mask)
        for mask in masks
    ]
    return masks, distance_transforms


def save_archive(
    output: Path,
    masks: list[np.ndarray],
    distance_transforms: list[np.ndarray],
    labels: list[str],
    class_ids: list[int],
    names: list[str],
    parameters: ShapeFeatureParameters,
    sheet_path: Path,
    r1_path: Path,
) -> None:
    archive: dict[str, np.ndarray] = {
        "format_version": np.asarray(FORMAT_VERSION, dtype=np.int32),
        "feature_type": np.asarray("binary_mask_symmetric_chamfer"),
        "template_count": np.asarray(len(masks), dtype=np.int32),
        "template_size": np.asarray(
            parameters.template_size,
            dtype=np.int32,
        ),
        "content_size": np.asarray(
            parameters.content_size,
            dtype=np.int32,
        ),
        "adaptive_block_size": np.asarray(
            parameters.adaptive_block_size,
            dtype=np.int32,
        ),
        "adaptive_c": np.asarray(
            parameters.adaptive_c,
            dtype=np.float64,
        ),
        "opening_operation": np.asarray("morph_open"),
        "opening_kernel_shape": np.asarray("rect"),
        "opening_kernel_size": np.asarray(
            parameters.opening_kernel_size,
            dtype=np.int32,
        ),
        "cleared_border": np.asarray(
            parameters.cleared_border,
            dtype=np.int32,
        ),
        "adaptive_method": np.asarray("gaussian_c"),
        "threshold_type": np.asarray("binary_inv"),
        "labels": np.asarray(labels),
        "class_ids": np.asarray(class_ids, dtype=np.int32),
        "names": np.asarray(names),
        "sheet_sha256": np.asarray(sha256(sheet_path)),
        "r1_sha256": np.asarray(sha256(r1_path)),
    }
    for index, (mask, distance_transform) in enumerate(
        zip(masks, distance_transforms)
    ):
        archive[f"mask_{index:02d}"] = mask
        archive[f"distance_{index:02d}"] = distance_transform

    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output, **archive)


def validate_archive(output: Path) -> None:
    with np.load(output, allow_pickle=False) as archive:
        count = int(archive["template_count"])
        labels = archive["labels"].astype(str).tolist()
        if int(archive["format_version"]) != FORMAT_VERSION:
            raise ValueError("unexpected feature archive version")
        if str(archive["feature_type"]) != (
            "binary_mask_symmetric_chamfer"
        ):
            raise ValueError("unexpected feature archive type")
        if (
            str(archive["opening_operation"]) != "morph_open"
            or str(archive["opening_kernel_shape"]) != "rect"
            or int(archive["opening_kernel_size"])
            != OPENING_KERNEL_SIZE
        ):
            raise ValueError("unexpected opening operation metadata")
        if count != 31:
            raise ValueError(f"archive has {count} templates, expected 31")
        if (
            labels.count("r2") != 15
            or labels.count("fake") != 15
            or labels.count("r1") != 1
        ):
            raise ValueError("archive class counts are incorrect")

        expected_shape = (
            int(archive["template_size"]),
            int(archive["template_size"]),
        )
        for index in range(count):
            mask = archive[f"mask_{index:02d}"]
            distance_transform = archive[f"distance_{index:02d}"]
            if mask.dtype != np.uint8 or mask.shape != expected_shape:
                raise ValueError(f"mask_{index:02d} is invalid")
            if (
                distance_transform.dtype != np.float32
                or distance_transform.shape != expected_shape
            ):
                raise ValueError(f"distance_{index:02d} is invalid")
            if not np.any(mask):
                raise ValueError(f"mask_{index:02d} is empty")
            self_distance = symmetric_chamfer_distance(
                mask,
                distance_transform,
                mask,
                distance_transform,
            )
            if self_distance != 0.0:
                raise ValueError(
                    f"mask_{index:02d} self distance is not zero"
                )


def print_summary(
    output: Path,
    masks: list[np.ndarray],
    labels: list[str],
    names: list[str],
) -> None:
    print(f"Saved {len(masks)} KFS Chamfer features to {output}")
    print(
        "Classes: "
        f"r2={labels.count('r2')}, "
        f"fake={labels.count('fake')}, "
        f"r1={labels.count('r1')}"
    )
    for name, label, mask in zip(names, labels, masks):
        foreground = int(np.count_nonzero(mask))
        print(
            f"{name:8s} class={label:4s} "
            f"foreground={foreground:4d} "
            f"density={foreground / mask.size:.4f}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate normalized KFS binary masks and distance transforms."
        )
    )
    parser.add_argument("--sheet", type=Path, default=DEFAULT_SHEET)
    parser.add_argument("--r1", type=Path, default=DEFAULT_R1)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--template-size", type=int, default=108)
    parser.add_argument("--content-size", type=int, default=96)
    parser.add_argument("--adaptive-block-size", type=int, default=21)
    parser.add_argument("--adaptive-c", type=float, default=5.0)
    parser.add_argument("--cleared-border", type=int, default=2)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    parameters = ShapeFeatureParameters(
        template_size=args.template_size,
        content_size=args.content_size,
        adaptive_block_size=args.adaptive_block_size,
        adaptive_c=args.adaptive_c,
        opening_kernel_size=OPENING_KERNEL_SIZE,
        cleared_border=args.cleared_border,
    )
    parameters.validate()

    sheet = load_color_image(args.sheet)
    r1_image = load_color_image(args.r1)
    images, labels, class_ids, names = build_source_images(
        sheet,
        r1_image,
    )
    masks, distance_transforms = build_features(images, parameters)
    save_archive(
        args.output,
        masks,
        distance_transforms,
        labels,
        class_ids,
        names,
        parameters,
        args.sheet,
        args.r1,
    )
    validate_archive(args.output)
    print_summary(args.output, masks, labels, names)


if __name__ == "__main__":
    main()
