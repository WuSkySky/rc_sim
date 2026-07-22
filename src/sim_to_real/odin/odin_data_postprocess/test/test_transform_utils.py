import math

import pytest

from odin_data_postprocess.transform_utils import (
    compose_transforms,
    invert_transform,
    normalize_quaternion,
    quaternion_from_rpy,
)


def _assert_vector_close(actual, expected):
    assert actual == pytest.approx(expected, abs=1.0e-12)


def test_zero_quaternion_is_rejected():
    assert normalize_quaternion((0.0, 0.0, 0.0, 0.0)) is None


def test_transform_composed_with_inverse_is_identity():
    translation = (1.2, -0.4, 0.8)
    rotation = quaternion_from_rpy(0.2, -0.3, 0.7)
    inverse_translation, inverse_rotation = invert_transform(
        translation,
        rotation,
    )

    result_translation, result_rotation = compose_transforms(
        translation,
        rotation,
        inverse_translation,
        inverse_rotation,
    )

    _assert_vector_close(result_translation, (0.0, 0.0, 0.0))
    _assert_vector_close(result_rotation, (0.0, 0.0, 0.0, 1.0))


def test_map_to_odom_places_base_at_target_pose():
    odom_to_base_translation = (2.0, -1.0, 0.5)
    odom_to_base_rotation = quaternion_from_rpy(0.1, 0.2, -0.4)
    target_translation = (-0.3, 1.7, 0.2)
    target_rotation = quaternion_from_rpy(-0.2, 0.1, math.pi / 3.0)

    base_to_odom_translation, base_to_odom_rotation = invert_transform(
        odom_to_base_translation,
        odom_to_base_rotation,
    )
    map_to_odom_translation, map_to_odom_rotation = compose_transforms(
        target_translation,
        target_rotation,
        base_to_odom_translation,
        base_to_odom_rotation,
    )
    result_translation, result_rotation = compose_transforms(
        map_to_odom_translation,
        map_to_odom_rotation,
        odom_to_base_translation,
        odom_to_base_rotation,
    )

    _assert_vector_close(result_translation, target_translation)
    _assert_vector_close(result_rotation, target_rotation)
