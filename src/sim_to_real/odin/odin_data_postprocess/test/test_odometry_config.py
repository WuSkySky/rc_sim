import math

import pytest

from odin_data_postprocess.odometry_config import (
    DEFAULT_PUBLISH_RATE,
    validate_publish_rate,
    validate_vector,
)


def test_default_publish_rate_is_50_hz():
    assert DEFAULT_PUBLISH_RATE == 50.0


@pytest.mark.parametrize('value', [50, 50.0])
def test_publish_rate_accepts_positive_numbers(value):
    assert validate_publish_rate(value) == 50.0


@pytest.mark.parametrize(
    'value',
    [True, 0.0, -1.0, math.inf, math.nan, '50.0'],
)
def test_publish_rate_rejects_invalid_values(value):
    with pytest.raises(ValueError):
        validate_publish_rate(value)


def test_vector_is_converted_to_finite_float_tuple():
    assert validate_vector('offset', [1, 2.5, 3.0]) == (1.0, 2.5, 3.0)


@pytest.mark.parametrize(
    'value',
    [[], [1.0, 2.0], [1.0, 2.0, 3.0, 4.0], [True, 0.0, 0.0],
     [math.nan, 0.0, 0.0], [1.0, 2.0, '3.0'], '1,2,3'],
)
def test_vector_rejects_invalid_values(value):
    with pytest.raises(ValueError):
        validate_vector('offset', value)
