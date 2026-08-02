import numpy as np
import pytest

from robot_r2_detect.resnet_tensorrt import (
    preprocess_resnet_image,
    softmax,
    validate_model_configuration,
)


MEAN = (0.485, 0.456, 0.406)
STD = (0.229, 0.224, 0.225)
CLASSES = ("R1", "Unlabeled", "fake", "true")


def test_preprocess_returns_contiguous_nchw_float32():
    image = np.zeros((480, 640, 3), dtype=np.uint8)

    tensor = preprocess_resnet_image(image, 224, MEAN, STD)

    assert tensor.shape == (3, 224, 224)
    assert tensor.dtype == np.float32
    assert tensor.flags.c_contiguous


def test_preprocess_converts_bgr_to_rgb_before_normalization():
    blue_bgr = np.zeros((224, 224, 3), dtype=np.uint8)
    blue_bgr[:, :, 0] = 255

    tensor = preprocess_resnet_image(blue_bgr, 224, (0.0,) * 3, (1.0,) * 3)

    assert np.allclose(tensor[:, 0, 0], (0.0, 0.0, 1.0))


def test_softmax_is_stable_and_normalized():
    probabilities = softmax(
        np.asarray([[1000.0, 1001.0, 999.0, -1000.0]], dtype=np.float32)
    )

    assert probabilities.shape == (1, 4)
    assert np.isfinite(probabilities).all()
    assert probabilities.sum() == pytest.approx(1.0)
    assert int(np.argmax(probabilities[0])) == 1


def test_model_configuration_rejects_duplicate_class_names():
    with pytest.raises(ValueError, match="unique"):
        validate_model_configuration(
            224,
            ("R1", "R1"),
            MEAN,
            STD,
        )
