import numpy as np
import pytest

from clothoid_lidar.red_ratio import RedRatioParams, hsv_groups, red_ratio


def frame(red_rows=0, h=480, w=640):
    img = np.full((h, w, 3), (60, 60, 60), np.uint8)     # grey floor
    img[h - red_rows:] = (30, 60, 230)                    # BGR red-orange at the bottom
    return img


def test_share_of_red_rows():
    assert red_ratio(frame(0), RedRatioParams()) == 0.0
    assert red_ratio(frame(120), RedRatioParams()) == pytest.approx(0.25, abs=0.01)
    assert red_ratio(frame(480), RedRatioParams()) == pytest.approx(1.0)


def test_roi_lower_half():
    assert red_ratio(frame(120), RedRatioParams(roi_top=0.5)) == pytest.approx(0.5, abs=0.02)


def test_hsv_ranges_are_adjustable():
    blue_only = (100, 100, 100, 130, 255, 255)
    assert red_ratio(frame(240), RedRatioParams(hsv_ranges=blue_only)) == 0.0


def test_bad_ranges():
    with pytest.raises(ValueError):
        hsv_groups((0, 1, 2))
