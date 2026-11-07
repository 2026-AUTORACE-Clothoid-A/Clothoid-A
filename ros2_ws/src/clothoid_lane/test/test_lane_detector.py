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


# --- track2026 (lane_tracker.py) -------------------------------------------------------------
from clothoid_lane.lane_tracker import LaneTracker  # noqa: E402

YELLOW = (0, 200, 230)   # BGR, H ~26, S ~255
WHITE = (240, 240, 240)


def tracker():
    return LaneTracker(LaneParams(**IDENTITY, hsv_lower=(18, 60, 90), hsv_upper=(35, 255, 255)))


def draw(lines):
    """lines: (x in the 640 px image, colour, thickness, y0, y1)."""
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    for x, colour, thick, y0, y1 in lines:
        cv2.line(frame, (x, y0), (x, y1), colour, thick)
    return frame


def test_tracker_follows_the_right_line_with_a_half_lane_offset():
    frame = draw([(160, YELLOW, 8, 0, 479), (480, YELLOW, 8, 0, 479)])
    result = tracker().detect(frame, SIDE_RIGHT, lane_width=160)
    assert result.found
    assert abs(result.target_x - (240 - 80)) <= 4


def test_tracker_uses_the_left_line_when_the_right_one_is_missing():
    result = tracker().detect(draw([(160, YELLOW, 8, 0, 479)]), SIDE_RIGHT, lane_width=160)
    assert result.found
    assert abs(result.target_x - (80 + 80)) <= 4


def test_tracker_ignores_a_white_marking_in_the_middle_of_the_lane():
    lanes = [(160, YELLOW, 8, 0, 479), (480, YELLOW, 8, 0, 479)]
    clean = tracker().detect(draw(lanes), SIDE_BOTH, lane_width=160)
    # a long thin white stroke half a lane from both lines, like the letters of "천천히"
    marked = tracker().detect(draw(lanes + [(330, WHITE, 8, 200, 479)]), SIDE_BOTH, lane_width=160)
    assert marked.found and abs(marked.target_x - clean.target_x) <= 2


def test_tracker_takes_a_white_divider_one_lane_from_the_yellow_line():
    # yellow on the left, white divider one lane width (160 px warped) to its right, nothing else
    frame = draw([(100, YELLOW, 8, 0, 479), (420, WHITE, 8, 0, 479)])
    result = tracker().detect(frame, SIDE_RIGHT, lane_width=160)
    assert result.found
    assert abs(result.right_points[0][0] - 210) <= 4      # the divider is the right boundary
    assert abs(result.target_x - (210 - 80)) <= 4


def test_tracker_ignores_wide_white_bars():
    # a crosswalk bar (30 px warped) where a divider would be
    frame = draw([(100, YELLOW, 8, 0, 479), (420, WHITE, 60, 0, 479)])
    result = tracker().detect(frame, SIDE_RIGHT, lane_width=160)
    assert result.right_points == []


def test_tracker_lost_and_debug_image():
    det = tracker()
    assert not det.detect(np.zeros((480, 640, 3), np.uint8), SIDE_BOTH, 160).found
    result = det.detect(draw([(160, YELLOW, 8, 0, 479)]), SIDE_BOTH, 160, debug=True)
    assert result.debug_image.shape == (240, 320, 3)


def test_tracker_takes_the_centre_of_zigzag_lines():
    # school zone: both lines are triangle waves around x = 160 and 480 (640 px image)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    for centre in (160, 480):
        pts = np.array([(centre + (20 if k % 2 else -20), k * 60) for k in range(9)], np.int32)
        cv2.polylines(frame, [pts], False, YELLOW, 8)
    det = tracker()
    result = det.detect(frame, SIDE_BOTH, lane_width=160)
    assert result.found
    assert abs(result.target_x - 160) <= 4      # halfway between the two zigzag centre lines
    assert result.zigzag
    assert not det.detect(draw([(160, YELLOW, 8, 0, 479), (480, YELLOW, 8, 0, 479)]),
                          SIDE_BOTH, lane_width=160).zigzag


def test_tracker_keeps_the_longer_trace_of_one_line():
    # 2026-11-07 on limo2: the same line traced from two seeds with a different number of windows
    # made `list.index` compare the numpy point arrays and lane_main died with a ValueError
    frame = draw([(160, YELLOW, 8, 0, 479), (480, YELLOW, 8, 200, 479)])
    det = tracker()
    det._sides[SIDE_RIGHT].bottom_x = 245.0     # a seed beside the right line, as from the last frame
    det._sides[SIDE_LEFT].bottom_x = 85.0
    for _ in range(3):
        assert det.detect(frame, SIDE_RIGHT, lane_width=160).found


def test_tracker_debug_image_shows_windows_and_reference():
    det = tracker()
    result = det.detect(draw([(160, YELLOW, 8, 0, 479), (480, YELLOW, 8, 0, 479)]), SIDE_RIGHT,
                        lane_width=160, debug=True)
    assert result.found and result.reference == 'right'
    assert result.debug_image is not None and result.debug_image.shape[:2] == (240, 320)
    # the right boundary's sliding windows are drawn in its colour
    assert (result.debug_image == np.array(LaneTracker.RIGHT_COLOUR, np.uint8)).all(axis=2).any()
    assert det.detect(np.zeros((480, 640, 3), np.uint8), SIDE_RIGHT, 160).reference == ''


def test_steering_law_matches_2025():
    import math
    from clothoid_lane.steering import angular_z, steer_angle
    assert steer_angle(160, 120, 320, 240) == 0.0
    # target 60 px right, 60 px up: -atan(1) / (pi / 2) = -0.5 rad, clamped to 28 deg
    assert steer_angle(220, 180, 320, 240) == -0.48869
    assert abs(steer_angle(190, 120, 320, 240) - (-math.atan2(30, 120) / (math.pi / 2))) < 1e-12
    assert abs(angular_z(0.3, 0.2) - 0.3 * math.tan(0.2) / 0.2) < 1e-12


def test_target_step_limit():
    from clothoid_lane.steering import limit_step
    assert limit_step(None, (10.0, 20.0), 8.0) == (10.0, 20.0)          # nothing before: as detected
    assert limit_step((0.0, 0.0), (3.0, 4.0), 8.0) == (3.0, 4.0)        # small moves pass
    x, y = limit_step((0.0, 0.0), (30.0, 40.0), 8.0)                    # a 50 px jump moves 8 px
    assert abs(x - 4.8) < 1e-9 and abs(y - 6.4) < 1e-9
    assert limit_step((0.0, 0.0), (30.0, 40.0), 0.0) == (30.0, 40.0)    # 0 = off
