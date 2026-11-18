"""Color semantics, clipping limits, and the legacy HSV contract."""
import cv2
import numpy as np
import pytest

from clothoid_common.color import ColorParams, ColorPipeline


@pytest.mark.parametrize('method', ['hsv', 'lab', 'rgb'])
def test_yellow_white_blue_and_floor_have_distinct_labels(method):
    pixels = np.array([[[0, 220, 220], [240, 240, 240], [230, 40, 30], [50, 50, 50]]], np.uint8)
    y, w = ColorPipeline(ColorParams(color_method=method)).masks(pixels)
    assert y.tolist() == [[True, False, False, False]]
    assert w.tolist() == [[False, True, False, False]]


def test_rgb_weak_yellow_priority_and_mission_white_are_explicit():
    pixels = np.array([[[200, 210, 210], [220, 10, 10]]], np.uint8)
    pipeline = ColorPipeline(ColorParams(color_method='rgb'))
    yellow, white = pipeline.masks(pixels)
    assert yellow.tolist() == [[True, False]]  # signed subtraction must not wrap blue into yellow
    assert not white.any()
    _, mission_white = pipeline.masks(pixels, exclude_yellow=False)
    assert mission_white.tolist() == [[True, False]]


def test_default_hsv_matches_existing_masks_exactly():
    pixels = np.random.default_rng(18).integers(0, 256, (240, 320, 3), dtype=np.uint8)
    hsv = cv2.cvtColor(pixels, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, np.array([15, 20, 160]), np.array([40, 255, 255])) > 0
    white = (hsv[..., 1] <= 50) & (hsv[..., 2] >= 190) & ~yellow
    pipeline = ColorPipeline()
    assert np.array_equal(pipeline.preprocess(pixels), pixels)
    y, w = pipeline.masks(pixels)
    assert np.array_equal(y, yellow) and np.array_equal(w, white)


def test_gamma_convention_endpoints_and_input_ownership():
    pixels = np.array([[[0, 128, 255]]], np.uint8)
    saved = pixels.copy()
    result = ColorPipeline(ColorParams(gamma_enabled=True, gamma=2.)).preprocess(pixels)
    assert result.tolist() == [[[0, 64, 255]]]
    assert np.array_equal(pixels, saved)
    neutral = np.full((40, 40, 3), 120, np.uint8)
    corrected = ColorPipeline(ColorParams(clahe_enabled=True)).preprocess(neutral)
    assert corrected.shape == neutral.shape and corrected.dtype == np.uint8
    assert np.all(neutral == 120)


@pytest.mark.parametrize('values', [dict(color_method='guess'), dict(gamma=float('nan')),
    dict(gamma=0.), dict(gamma_enabled=1), dict(clahe_grid_size=2.5),
    dict(rgb_yellow_difference_min=-1), dict(lab_white_l_min=101.)])
def test_invalid_parameters_rejected(values):
    with pytest.raises(ValueError):
        ColorParams(**values)
