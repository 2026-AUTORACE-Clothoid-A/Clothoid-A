"""Offline tests for the slow-zone and stop-line detectors: `pytest ros2_ws/src/clothoid_lane/test`."""
import os
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from clothoid_lane.lane_detector import LaneParams  # noqa: E402
from clothoid_lane.speed_zone import (  # noqa: E402
    SpeedZoneDetector, SpeedZoneHold, SpeedZoneParams)
from clothoid_lane.stopline import StopLineDetector, StopLineParams  # noqa: E402

RED = (0, 0, 255)  # BGR


def frame_with_red_patch(w=200, h=60):
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    # inside the default ROI: x 160..480, y 360..480
    cv2.rectangle(frame, (200, 380), (200 + w, 380 + h), RED, -1)
    return frame


def test_speed_zone_detects_red_patch_in_roi():
    result = SpeedZoneDetector(SpeedZoneParams()).detect(frame_with_red_patch())
    assert result.found
    assert result.area > 1800


def test_speed_zone_ignores_patch_outside_roi():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(frame, (200, 100), (400, 160), RED, -1)  # upper half, outside the ROI
    assert not SpeedZoneDetector(SpeedZoneParams()).detect(frame).found


def test_speed_zone_ignores_small_patch():
    assert not SpeedZoneDetector(SpeedZoneParams()).detect(frame_with_red_patch(20, 20)).found


def test_speed_zone_debug_image_is_full_frame():
    result = SpeedZoneDetector(SpeedZoneParams()).detect(frame_with_red_patch(), debug=True)
    assert result.debug_image.shape == (480, 640, 3)


@pytest.mark.parametrize('ranges', [(), (1, 2, 3), (0, 100, 100, 10, 255)])
def test_speed_zone_rejects_bad_hsv_ranges(ranges):
    with pytest.raises(ValueError):
        SpeedZoneDetector(SpeedZoneParams(hsv_ranges=ranges))


def test_speed_zone_rejects_bad_roi_and_keeps_previous_config():
    det = SpeedZoneDetector(SpeedZoneParams())
    before = det.p
    with pytest.raises(ValueError):
        det.configure(SpeedZoneParams(roi_x0=0.8, roi_x1=0.2))
    assert det.p is before


def test_speed_zone_hold_keeps_active_then_releases():
    hold = SpeedZoneHold(hold_sec=1.0)
    assert not hold.update(False, 0.0)       # never seen
    assert hold.update(True, 1.0)
    assert hold.update(False, 1.9)            # inside the hold window
    assert not hold.update(False, 2.2)        # expired
    assert hold.update(True, 3.0)             # re-armed by a new detection


# --- stop line -----------------------------------------------------------------------

IDENTITY = LaneParams(x_h=0, x_l=0, y_h=-240, y_l=0)  # whole frame -> 320x240


def stopline_detector(**kw):
    return StopLineDetector(StopLineParams(warp=IDENTITY, **kw))


def frame_with_stripes(xs, width=20):
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    for x in xs:
        cv2.rectangle(frame, (x, 0), (x + width, 479), (255, 255, 255), -1)
    return frame


def test_stopline_found_with_enough_stripes():
    result = stopline_detector().detect(frame_with_stripes([120, 200, 280, 360, 440]))
    assert result.found
    assert result.count >= 4
    assert result.white_ratio > 10.0


def test_stopline_not_found_with_too_few_stripes():
    result = stopline_detector().detect(frame_with_stripes([200, 360]))
    assert not result.found
    assert result.count == 2


def test_stopline_not_found_on_dark_frame():
    result = stopline_detector().detect(np.zeros((480, 640, 3), dtype=np.uint8))
    assert not result.found
    assert result.count == 0


def test_stopline_debug_image_matches_cropped_area():
    result = stopline_detector().detect(frame_with_stripes([120, 200, 280, 360, 440]), debug=True)
    # warped 320x240 cropped by crop_y=80 rows and crop_x=40 columns per side
    assert result.debug_image.shape == (160, 240, 3)


def test_stopline_rejects_degenerate_warp_and_keeps_previous_config():
    det = stopline_detector()
    before = det.p
    with pytest.raises(ValueError):
        det.configure(StopLineParams(warp=LaneParams(x_h=0, x_l=0, y_h=-240, y_l=480)))
    assert det.p is before
