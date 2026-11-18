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


def test_faded_yellow_recovery_requires_blue_floor_and_connected_paint():
    from clothoid_lane.lane_tracker import _recover_faded_yellow

    frame = np.full((64, 80, 3), (100, 55, 55), np.uint8)
    frame[:, 20:24] = (225, 180, 210)  # faded tape
    frame[:, 50:54] = (225, 180, 210)  # disconnected paint
    frame[30:34, :] = (200, 200, 200)  # white marking crossing both strips
    seed = np.zeros(frame.shape[:2], bool)
    seed[15:20, 20:24] = True

    assert np.array_equal(_recover_faded_yellow(frame, seed), seed)
    seed[:32, 2] = True
    seed[:32, 76] = True
    mask = _recover_faded_yellow(frame, seed)
    assert mask[25, 22]
    assert not mask[:, 50:54].any()
    assert not mask[30:, 20:24].any()

    frame[:, :20] = frame[:, 24:50] = frame[:, 54:] = (60, 60, 60)
    # A small tinted patch beside the seed does not indicate widespread blue lighting.
    frame[21:25, 10:14] = frame[21:25, 30:34] = (100, 55, 55)
    frame[:, 50:54] = (60, 60, 60)
    frame[:, 20:24] = (200, 180, 210)
    assert np.array_equal(_recover_faded_yellow(frame, seed), seed)
