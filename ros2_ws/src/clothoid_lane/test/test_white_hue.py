"""White hue gates keep neutral paint while rejecting equally bright tinted pixels."""
import cv2
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter

from clothoid_lane.lane_detector import LaneParams
from clothoid_lane.lane_main import LaneMainNode
from clothoid_lane.lane_tracker import LaneTracker, TrackerParams


@pytest.mark.parametrize('lower,upper,expected', [
    (0, 179, [True, True, True]),
    (0, 60, [True, True, False]),
    (60, 179, [False, False, True]),
])
def test_white_hue_bounds_filter_low_saturation_pixels(lower, upper, expected):
    hsv = np.full((480, 640, 3), (0, 30, 235), np.uint8)
    hsv[:, 213:426, 0] = 50
    hsv[:, 426:, 0] = 120
    frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    detector = LaneTracker(LaneParams(x_h=0, x_l=0, y_h=-240, y_l=0),
                          TrackerParams(white_h_min=lower, white_h_max=upper))
    _, white = detector._masks(frame)
    assert white[100, [50, 160, 270]].tolist() == expected


@pytest.mark.parametrize('lower,upper', [(-1, 179), (0, 180), (100, 50)])
def test_invalid_white_hue_preserves_the_working_detector(lower, upper):
    detector = LaneTracker(LaneParams())
    previous = detector.t
    with pytest.raises(ValueError, match='white hue'):
        detector.configure(detector.p, TrackerParams(white_h_min=lower, white_h_max=upper))
    assert detector.t is previous


def test_live_white_hue_parameters_rebuild_and_reject_inverted_bounds():
    rclpy.init()
    node = LaneMainNode()
    try:
        assert node.set_parameters_atomically([Parameter('white_h_max', value=45)]).successful
        assert node.detector.t.white_h_max == 45
        working = node.detector
        result = node.set_parameters_atomically([Parameter('white_h_min', value=60)])
        assert not result.successful
        assert node.get_parameter('white_h_min').value == 0
        assert node.detector is working
    finally:
        node.destroy_node()
        rclpy.shutdown()
