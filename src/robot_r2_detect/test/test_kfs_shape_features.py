from pathlib import Path

import cv2
import numpy as np

from robot_r2_detect.kfs_chamfer_matcher import KfsChamferMatcher
from robot_r2_detect.kfs_shape_features import (
    calculate_distance_transform,
    extract_normalized_mask,
    normalize_foreground_mask,
    symmetric_chamfer_distance,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ARCHIVE_PATH = (
    PACKAGE_ROOT / "features" / "kfs_shape_templates.npz"
)
SHEET_PATH = (
    PACKAGE_ROOT / "reference" / "kfs_r2_fake_sheet.png"
)
R1_PATH = PACKAGE_ROOT / "reference" / "kfs_r1.png"


def source_images():
    sheet = cv2.imread(str(SHEET_PATH), cv2.IMREAD_COLOR)
    assert sheet is not None
    images = []
    expected_labels = []
    for row in range(6):
        for column in range(5):
            images.append(
                sheet[
                    row * 108:(row + 1) * 108,
                    column * 108:(column + 1) * 108,
                ]
            )
            expected_labels.append("r2" if row < 3 else "fake")
    r1 = cv2.imread(str(R1_PATH), cv2.IMREAD_COLOR)
    assert r1 is not None
    images.append(r1)
    expected_labels.append("r1")
    return images, expected_labels


def make_matcher() -> KfsChamferMatcher:
    return KfsChamferMatcher(
        ARCHIVE_PATH,
        max_chamfer_distance=0.015,
        min_class_margin=0.004,
        confidence_threshold=0.5,
    )


def test_archive_has_no_pickle_arrays():
    with np.load(ARCHIVE_PATH, allow_pickle=False) as archive:
        assert int(archive["format_version"]) == 3
        assert int(archive["template_count"]) == 31
        assert str(archive["opening_operation"]) == "morph_open"
        assert str(archive["opening_kernel_shape"]) == "rect"
        assert int(archive["opening_kernel_size"]) == 3
        assert not any(
            archive[key].dtype.hasobject for key in archive.files
        )


def test_online_preprocessing_matches_offline_archive():
    matcher = make_matcher()
    images, _ = source_images()
    for index, image in enumerate(images):
        online_mask = extract_normalized_mask(
            image,
            matcher.parameters,
        )
        assert np.array_equal(online_mask, matcher.masks[index])


def test_all_standard_images_are_accepted_as_their_class():
    matcher = make_matcher()
    images, expected_labels = source_images()
    for image, expected_label in zip(images, expected_labels):
        result = matcher.match(image)
        assert result.accepted
        assert result.class_name == expected_label
        assert result.best_distance == 0.0
        assert result.confidence >= 0.5


def classify_mask(matcher, query_mask):
    query_distance = calculate_distance_transform(query_mask)
    scores = np.asarray(
        [
            symmetric_chamfer_distance(
                query_mask,
                query_distance,
                mask,
                distance,
            )
            for mask, distance in zip(
                matcher.masks,
                matcher.distance_transforms,
            )
        ]
    )
    class_scores = {}
    for label in ("r2", "fake", "r1"):
        class_scores[label] = min(
            score
            for score, candidate_label in zip(scores, matcher.labels)
            if candidate_label == label
        )
    ordered = sorted(class_scores.items(), key=lambda item: item[1])
    best_label, best_distance = ordered[0]
    margin = ordered[1][1] - best_distance
    distance_score = 1.0 / (
        1.0 + best_distance / matcher.max_chamfer_distance
    )
    margin_score = margin / (
        margin + matcher.min_class_margin
    )
    accepted = (
        best_distance <= matcher.max_chamfer_distance
        and margin >= matcher.min_class_margin
        and min(distance_score, margin_score)
        >= matcher.confidence_threshold
    )
    return best_label, accepted


def anisotropic_mask(matcher, mask, amount):
    size = matcher.parameters.template_size
    center = (size - 1) / 2.0
    linear = np.asarray(
        [[1.0 + amount, 0.0], [0.0, 1.0 - amount]],
        dtype=np.float32,
    )
    center_point = np.asarray([center, center], dtype=np.float32)
    translation = center_point - linear @ center_point
    matrix = np.hstack((linear, translation.reshape(2, 1)))
    transformed = cv2.warpAffine(
        mask,
        matrix,
        (size, size),
        flags=cv2.INTER_NEAREST,
        borderValue=0,
    )
    return normalize_foreground_mask(
        transformed,
        matcher.parameters,
    )


def test_five_percent_anisotropic_masks_are_all_accepted_correctly():
    matcher = make_matcher()
    for mask, expected_label in zip(matcher.masks, matcher.labels):
        query = anisotropic_mask(matcher, mask, 0.05)
        selected_label, accepted = classify_mask(matcher, query)
        assert accepted
        assert selected_label == expected_label


def test_default_thresholds_never_accept_wrong_ten_percent_result():
    matcher = make_matcher()
    for mask, expected_label in zip(matcher.masks, matcher.labels):
        query = anisotropic_mask(matcher, mask, 0.10)
        selected_label, accepted = classify_mask(matcher, query)
        assert not accepted or selected_label == expected_label
