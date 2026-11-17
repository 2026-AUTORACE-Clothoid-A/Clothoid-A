import numpy as np
import rclpy
from rclpy.parameter import Parameter

from clothoid_lane.lane_main import LaneMainNode


def test_runtime_color_selection_and_atomic_rejection():
    rclpy.init()
    node = LaneMainNode()
    try:
        frame = np.full((480, 640, 3), (200, 210, 210), np.uint8)
        before, _ = node.detector._masks(frame)
        assert not before.any()
        result = node.set_parameters_atomically([Parameter('color_method', value='rgb'),
                    Parameter('gamma_enabled', value=True)])
        assert result.successful
        yellow, _ = node.detector._masks(frame)
        assert yellow.any()
        working = node.detector
        result = node.set_parameters_atomically([Parameter('gamma', value=0.)])
        assert not result.successful and node.detector is working
        assert node.get_parameter('gamma').value == 1.2
        result = node.set_parameters_atomically([Parameter('detector', value='sliding2025')])
        assert not result.successful and node.detector is working
    finally:
        node.destroy_node()
        rclpy.shutdown()
