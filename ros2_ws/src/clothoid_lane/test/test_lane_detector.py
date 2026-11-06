"""Offline tests for the lane detector (no ROS needed): `pytest ros2_ws/src/clothoid_lane/test`."""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from clothoid_lane.lane_detector import (  # noqa: E402
    SIDE_BOTH, SIDE_LEFT, SIDE_RIGHT, LaneDetector, LaneParams)

# Use the identity-like warp so a synthetic image maps straight through: the whole
# 640x480 frame is scaled to 320x240.
IDENTITY = dict(x_h=0, x_l=0, y_h=-240, y_l=0)


def make_frame(left_x=None, right_x=None):
    """Yellow vertical lane lines on a dark frame (coordinates in the 640x480 image)."""
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    yellow = (0, 230, 230)  # BGR, inside the default HSV range
    for x in (left_x, right_x):
        if x is not None:
            cv2.line(frame, (x, 0), (x, 479), yellow, 6)
    return frame


def detector():
    return LaneDetector(LaneParams(**IDENTITY))


def test_both_lanes_center_target():
    result = detector().detect(make_frame(160, 480), SIDE_BOTH, lane_width=103)
    assert result.found
    assert abs(result.target_x - 160) <= 4  # centered between the two lines (320 px wide image)


def test_right_lane_only_uses_lane_width_offset():
    lane_width = 100
    result = detector().detect(make_frame(160, 480), SIDE_RIGHT, lane_width=lane_width)
    assert result.found
    assert abs(result.target_x - (240 - lane_width // 2)) <= 4


def test_left_lane_only_uses_lane_width_offset():
    lane_width = 100
    result = detector().detect(make_frame(160, 480), SIDE_LEFT, lane_width=lane_width)
    assert result.found
    assert abs(result.target_x - (80 + lane_width // 2)) <= 4


def test_followed_side_missing_falls_back_to_other_side():
    # following the right lane, but only the left line is in view (right-hand corner)
    lane_width = 100
    result = detector().detect(make_frame(left_x=160), SIDE_RIGHT, lane_width=lane_width)
    assert result.found
    assert result.fallback
    assert abs(result.target_x - (80 + lane_width // 2)) <= 4


def test_followed_side_present_is_not_fallback():
    result = detector().detect(make_frame(right_x=480), SIDE_RIGHT, lane_width=100)
    assert result.found
    assert not result.fallback


def test_yellow_mask_ignores_white_marks():
    # yellow right line plus a white block (slow-zone text) inside the right half
    frame = make_frame(160, 480)
    cv2.rectangle(frame, (380, 200), (420, 300), (255, 255, 255), -1)
    clean = detector().detect(make_frame(160, 480), SIDE_RIGHT, lane_width=100)
    marked = detector().detect(frame, SIDE_RIGHT, lane_width=100)
    assert marked.yellow_mask
    assert abs(marked.target_x - clean.target_x) <= 2


def test_white_lanes_use_canny_when_no_yellow():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    for x in (160, 480):
        cv2.line(frame, (x, 0), (x, 479), (255, 255, 255), 6)
    result = detector().detect(frame, SIDE_RIGHT, lane_width=100)
    assert not result.yellow_mask
    assert result.found


def test_lane_lost_returns_not_found():
    result = detector().detect(np.zeros((480, 640, 3), dtype=np.uint8), SIDE_BOTH, 103)
    assert not result.found


def test_debug_image_is_three_channel():
    result = detector().detect(make_frame(160, 480), SIDE_BOTH, 103, debug=True)
    assert result.debug_image is not None
    assert result.debug_image.shape == (240, 320, 3)


def test_bad_parameters_keep_previous_config():
    det = detector()
    before = det.p
    # all four source corners collapse to one line -> degenerate transform
    degenerate = LaneParams(x_h=0, x_l=0, y_h=-240, y_l=480)
    try:
        det.configure(degenerate)
        raised = False
    except (ValueError, cv2.error):
        raised = True
    assert raised
    assert det.p is before


def test_default_2025_warp_is_valid_and_runs():
    det = LaneDetector(LaneParams())
    result = det.detect(make_frame(160, 480), SIDE_RIGHT, lane_width=103)
    assert isinstance(result.found, bool)
