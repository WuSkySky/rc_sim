"""Pure OpenCV KFS ROI extraction shared by the ROS node and tests."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class KfsRoiResult:
    """ROI coordinates and mask derived from one source image."""

    valid: bool
    mask: np.ndarray
    roi: np.ndarray | None = None
    x1: int = 0
    y1: int = 0
    x2: int = 0
    y2: int = 0
    center_u: int = 0
    center_v: int = 0
    center_offset_x: int = 0
    center_offset_y: int = 0


def extract_kfs_roi(
    image: np.ndarray,
    blue_lower: np.ndarray,
    blue_upper: np.ndarray,
    red_low_lower: np.ndarray,
    red_low_upper: np.ndarray,
    red_high_lower: np.ndarray,
    red_high_upper: np.ndarray,
    column_threshold_ratio: float,
) -> KfsRoiResult:
    """Extract the only externally guaranteed red/blue KFS region."""
    if image is None or image.size == 0:
        raise ValueError("ROI source image is empty")
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("ROI source image must be BGR")
    if not 0.0 < column_threshold_ratio <= 1.0:
        raise ValueError("column_threshold_ratio must be in (0, 1]")

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
    red_low_mask = cv2.inRange(hsv, red_low_lower, red_low_upper)
    red_high_mask = cv2.inRange(hsv, red_high_lower, red_high_upper)
    mask = cv2.bitwise_or(
        blue_mask,
        cv2.bitwise_or(red_low_mask, red_high_mask),
    )

    column_lengths = np.count_nonzero(mask, axis=0)
    max_column_length = int(column_lengths.max())
    if max_column_length <= 0:
        return KfsRoiResult(valid=False, mask=mask)

    threshold = max_column_length * column_threshold_ratio
    valid_columns = np.flatnonzero(column_lengths >= threshold)
    if valid_columns.size == 0:
        return KfsRoiResult(valid=False, mask=mask)
    left = int(valid_columns[0])
    right = int(valid_columns[-1])

    longest_columns = np.flatnonzero(
        column_lengths == max_column_length
    )
    horizontal_center = (left + right) / 2.0
    height_column = int(
        min(
            longest_columns,
            key=lambda column: (
                abs(float(column) - horizontal_center),
                int(column),
            ),
        )
    )
    valid_rows = np.flatnonzero(mask[:, height_column] != 0)
    if valid_rows.size == 0:
        return KfsRoiResult(valid=False, mask=mask)
    top = int(valid_rows[0])
    bottom = int(valid_rows[-1])

    center_u = (left + right) // 2
    center_v = (top + bottom) // 2
    image_height, image_width = image.shape[:2]
    return KfsRoiResult(
        valid=True,
        mask=mask,
        roi=np.ascontiguousarray(
            image[top:bottom + 1, left:right + 1]
        ),
        x1=left,
        y1=top,
        x2=right,
        y2=bottom,
        center_u=center_u,
        center_v=center_v,
        center_offset_x=center_u - image_width // 2,
        center_offset_y=center_v - image_height // 2,
    )
