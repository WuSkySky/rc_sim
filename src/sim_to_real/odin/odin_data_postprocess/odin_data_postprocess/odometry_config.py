import math
from collections.abc import Sequence
from numbers import Real

from odin_data_postprocess.transform_utils import is_finite


DEFAULT_PUBLISH_RATE = 50.0


def validate_vector(name, values):
    if (
        not isinstance(values, Sequence)
        or isinstance(values, (str, bytes))
        or len(values) != 3
    ):
        raise ValueError(f'{name} must contain exactly three values')
    if any(
        isinstance(value, bool) or not isinstance(value, Real)
        for value in values
    ):
        raise ValueError(f'{name} values must be numbers')

    vector = tuple(float(value) for value in values)
    if not is_finite(vector):
        raise ValueError(f'{name} values must be finite')
    return vector


def validate_publish_rate(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError('publish_rate must be a number')
    publish_rate = float(value)
    if not math.isfinite(publish_rate) or publish_rate <= 0.0:
        raise ValueError('publish_rate must be finite and positive')
    return publish_rate
