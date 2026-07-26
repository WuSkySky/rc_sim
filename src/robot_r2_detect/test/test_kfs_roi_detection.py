import cv2
import numpy as np

from robot_r2_detect.kfs_roi_detection import extract_kfs_roi


BLUE_LOWER = np.asarray([90, 80, 60], dtype=np.uint8)
BLUE_UPPER = np.asarray([130, 255, 255], dtype=np.uint8)
RED_LOW_LOWER = np.asarray([0, 80, 60], dtype=np.uint8)
RED_LOW_UPPER = np.asarray([10, 255, 255], dtype=np.uint8)
RED_HIGH_LOWER = np.asarray([170, 80, 60], dtype=np.uint8)
RED_HIGH_UPPER = np.asarray([179, 255, 255], dtype=np.uint8)


def extract(image, ratio=0.8):
    return extract_kfs_roi(
        image,
        BLUE_LOWER,
        BLUE_UPPER,
        RED_LOW_LOWER,
        RED_LOW_UPPER,
        RED_HIGH_LOWER,
        RED_HIGH_UPPER,
        ratio,
    )


def test_roi_uses_valid_columns_and_center_nearest_longest_column():
    image = np.zeros((100, 120, 3), dtype=np.uint8)
    image[15:85, 20:80] = (255, 0, 0)
    image[10:91, 49] = (255, 0, 0)

    result = extract(image)

    assert result.valid
    assert (result.x1, result.y1, result.x2, result.y2) == (
        20,
        10,
        79,
        90,
    )
    assert (result.center_u, result.center_v) == (49, 50)
    assert (result.center_offset_x, result.center_offset_y) == (
        -11,
        0,
    )
    assert result.roi.shape == (81, 60, 3)


def test_roi_accepts_red_high_hue():
    hsv = np.zeros((40, 50, 3), dtype=np.uint8)
    hsv[5:35, 10:40] = (175, 255, 255)
    image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    result = extract(image)

    assert result.valid
    assert (result.x1, result.y1, result.x2, result.y2) == (
        10,
        5,
        39,
        34,
    )


def test_roi_returns_invalid_when_no_color_region_exists():
    image = np.zeros((40, 50, 3), dtype=np.uint8)

    result = extract(image)

    assert not result.valid
    assert result.roi is None
    assert np.count_nonzero(result.mask) == 0
