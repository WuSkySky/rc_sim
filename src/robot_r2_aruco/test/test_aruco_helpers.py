import math

import numpy as np
import pytest

from robot_r2_aruco.aruco_camera_tf import _read_vector
from robot_r2_aruco.aruco_detect import (
    _advance_rate_limit,
    _rotation_matrix_to_quaternion,
)
from robot_r2_aruco.usb_camera_bridge import _estimate_camera_info


def test_rate_limit_advances_from_actual_start_time():
    assert _advance_rate_limit(1.0, 1.1, 0.05) == (False, 1.1)
    allowed, next_allowed = _advance_rate_limit(1.1, 1.1, 0.05)
    assert allowed is True
    assert next_allowed == pytest.approx(1.15)


def test_identity_rotation_maps_to_identity_quaternion():
    quaternion = _rotation_matrix_to_quaternion(np.eye(3))
    assert quaternion == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_camera_mount_vector_rejects_non_finite_values():
    with pytest.raises(ValueError, match="finite"):
        _read_vector("camera_xyz", [0.0, math.inf, 0.0])


def test_estimated_camera_info_uses_documented_fallback():
    info = _estimate_camera_info(640, 480, "camera")
    assert info.k == pytest.approx(
        [640.0, 0.0, 320.0, 0.0, 640.0, 240.0, 0.0, 0.0, 1.0]
    )
    assert info.d == pytest.approx([0.0] * 5)
