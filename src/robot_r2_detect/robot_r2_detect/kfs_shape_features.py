"""Shared OpenCV preprocessing and Chamfer matching for KFS symbols."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class ShapeFeatureParameters:
    """Parameters that must remain identical offline and online."""

    template_size: int = 108
    content_size: int = 96
    adaptive_block_size: int = 21
    adaptive_c: float = 5.0
    opening_kernel_size: int = 3
    cleared_border: int = 2

    def validate(self) -> None:
        if self.template_size <= 0:
            raise ValueError("template_size must be positive")
        if not 0 < self.content_size <= self.template_size:
            raise ValueError(
                "content_size must be in the range "
                "[1, template_size]"
            )
        if (
            self.adaptive_block_size <= 1
            or self.adaptive_block_size % 2 == 0
        ):
            raise ValueError(
                "adaptive_block_size must be odd and greater than 1"
            )
        if (
            self.opening_kernel_size <= 0
            or self.opening_kernel_size % 2 == 0
        ):
            raise ValueError(
                "opening_kernel_size must be odd and positive"
            )
        if not 0 <= self.cleared_border < self.template_size // 2:
            raise ValueError(
                "cleared_border must be non-negative and smaller than "
                "half the template size"
            )


def extract_normalized_mask(
    image: np.ndarray,
    parameters: ShapeFeatureParameters,
) -> np.ndarray:
    """Convert one BGR symbol image into a centered binary stroke mask."""
    parameters.validate()
    if image is None or image.size == 0:
        raise ValueError("feature image is empty")
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("feature image must be a BGR image")

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    interpolation = (
        cv2.INTER_AREA
        if max(gray.shape[:2]) > parameters.template_size
        else cv2.INTER_LINEAR
    )
    normalized_gray = cv2.resize(
        gray,
        (parameters.template_size, parameters.template_size),
        interpolation=interpolation,
    )
    binary = cv2.adaptiveThreshold(
        normalized_gray,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        parameters.adaptive_block_size,
        parameters.adaptive_c,
    )
    opening_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (
            parameters.opening_kernel_size,
            parameters.opening_kernel_size,
        ),
    )
    binary = cv2.morphologyEx(
        binary,
        cv2.MORPH_OPEN,
        opening_kernel,
    )

    border = parameters.cleared_border
    if border:
        binary[:border, :] = 0
        binary[-border:, :] = 0
        binary[:, :border] = 0
        binary[:, -border:] = 0

    return normalize_foreground_mask(binary, parameters)


def normalize_foreground_mask(
    binary: np.ndarray,
    parameters: ShapeFeatureParameters,
) -> np.ndarray:
    """Scale and center all foreground pixels in an existing binary mask."""
    parameters.validate()
    _validate_mask(binary)

    rows, columns = np.nonzero(binary)
    if rows.size == 0:
        raise ValueError("feature image produced an empty binary mask")
    top = int(rows.min())
    bottom = int(rows.max()) + 1
    left = int(columns.min())
    right = int(columns.max()) + 1
    content = binary[top:bottom, left:right]

    scale = min(
        parameters.content_size / content.shape[1],
        parameters.content_size / content.shape[0],
    )
    width = max(1, int(round(content.shape[1] * scale)))
    height = max(1, int(round(content.shape[0] * scale)))
    resized = cv2.resize(
        content,
        (width, height),
        interpolation=cv2.INTER_NEAREST,
    )

    result = np.zeros(
        (parameters.template_size, parameters.template_size),
        dtype=np.uint8,
    )
    offset_x = (parameters.template_size - width) // 2
    offset_y = (parameters.template_size - height) // 2
    result[
        offset_y:offset_y + height,
        offset_x:offset_x + width,
    ] = resized
    return result


def calculate_distance_transform(mask: np.ndarray) -> np.ndarray:
    """Return each background pixel's distance to the nearest stroke."""
    _validate_mask(mask)
    if not np.any(mask):
        raise ValueError("cannot transform an empty feature mask")
    return cv2.distanceTransform(
        cv2.bitwise_not(mask),
        cv2.DIST_L2,
        cv2.DIST_MASK_3,
    ).astype(np.float32)


def symmetric_chamfer_distance(
    first_mask: np.ndarray,
    first_distance: np.ndarray,
    second_mask: np.ndarray,
    second_distance: np.ndarray,
) -> float:
    """Calculate normalized bidirectional Chamfer distance."""
    _validate_mask(first_mask)
    _validate_mask(second_mask)
    if first_mask.shape != second_mask.shape:
        raise ValueError("feature masks must have identical dimensions")
    if first_distance.shape != first_mask.shape:
        raise ValueError(
            "first distance transform does not match its mask"
        )
    if second_distance.shape != second_mask.shape:
        raise ValueError(
            "second distance transform does not match its mask"
        )

    first_points = first_mask != 0
    second_points = second_mask != 0
    if not np.any(first_points) or not np.any(second_points):
        raise ValueError("Chamfer distance requires non-empty masks")

    first_to_second = float(second_distance[first_points].mean())
    second_to_first = float(first_distance[second_points].mean())
    normalization = 2.0 * max(first_mask.shape)
    return (first_to_second + second_to_first) / normalization


def _validate_mask(mask: np.ndarray) -> None:
    if mask.ndim != 2 or mask.dtype != np.uint8:
        raise ValueError("feature mask must be a two-dimensional uint8 array")
