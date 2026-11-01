"""Offline tests for the parking sequence and start line: `pytest ros2_ws/src/clothoid_parking/test`."""
import math
import os
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from clothoid_parking.parking_sequence import (  # noqa: E402
    ParkingSequence, SequenceParams, Step, step_to_twist_values, steps_from_lists)
from clothoid_parking.start_line import StartLineDetector, StartLineParams  # noqa: E402

# 2025 table (config/parking.yaml)
DURATION = [0.5, 0.0, 0.5, 2.2, 0.5, 0.8, 0.5, 3.0, 0.5, 1.0, 0.5, 2.2]
SPEED = [0.195, 0.2, -0.195, -0.2, -0.195, -0.2, 0.195, -0.195, 0.195, 0.2, 0.195, 0.2]
STEER = [-0.34, -0.34, 0.17, 0.17, -0.34, -0.34, 0.34, 0.34, -0.34, -0.34, 0.34, 0.17]


def sequence(**kw):
    params = SequenceParams(steps=steps_from_lists(DURATION, SPEED, STEER), sensor_step=1, **kw)
    return ParkingSequence(params)


FAR = (-1.0, 0.8)      # closest point angle (rad), range (m): lidar condition satisfied
NEAR = (-1.0, 0.2)     # closer than range_limit: keep turning


def test_idle_until_started():
    assert sequence().update(0.0, *FAR) is None


def test_fixed_steps_follow_the_table():
    seq = sequence()
    seq.start(10.0)
    assert seq.update(10.2, *NEAR) == Step(0.5, 0.195, -0.34)  # step 0
    assert seq.update(10.5, *NEAR) == Step(0.0, 0.2, -0.34)    # lidar step is reached, waits
    assert seq.update(10.5, *FAR) == Step(0.5, -0.195, 0.17)   # condition met: skipped at once


def test_lidar_step_waits_then_shifts_the_rest_of_the_timeline():
    seq = sequence()
    seq.start(0.0)
    assert seq.update(0.6, *NEAR).speed == 0.2    # still turning
    assert seq.update(1.2, *NEAR).speed == 0.2
    # condition satisfied at t=1.3: the lidar step lasted 1.3 - 0.5 = 0.8 s
    assert seq.update(1.3, *FAR) == Step(0.5, -0.195, 0.17)   # step 2 starts
    assert seq.update(1.7, *FAR) == Step(0.5, -0.195, 0.17)   # step 2 ends at 1.3 + 0.5
    assert seq.update(1.9, *FAR) == Step(2.2, -0.2, 0.17)     # step 3


def test_sequence_finishes_after_last_step_and_stays_done():
    seq = sequence()
    seq.start(0.0)
    seq.update(0.5, *FAR)                           # lidar condition met immediately
    total = sum(DURATION)                           # lidar step lasted 0 s, rows add up to 12.2 s
    assert seq.update(total - 0.01, *FAR) == Step(2.2, 0.2, 0.17)
    assert seq.update(total + 0.01, *FAR) is None
    assert seq.done
    assert seq.update(100.0, *FAR) is None


def test_lidar_step_gives_up_without_scan():
    seq = sequence(sensor_step_max_sec=3.0)
    seq.start(0.0)
    assert seq.update(0.5 + 2.9, None, None).speed == 0.2   # no scan yet: still waiting
    assert seq.update(0.5 + 3.1, None, None).speed == -0.195  # timeout: moved on to step 2


def test_angle_limit_alone_keeps_the_lidar_step_going():
    seq = sequence()
    seq.start(0.0)
    assert seq.update(0.6, -2.5, 0.8).speed == 0.2   # angle below the limit
    assert seq.update(0.7, -1.5, 0.8).speed == -0.195


def test_start_again_resets_the_sequence():
    seq = sequence()
    seq.start(0.0)
    seq.update(0.5, *FAR)
    seq.update(100.0, *FAR)
    assert seq.done
    seq.start(200.0)
    assert not seq.done
    assert seq.update(200.1, *FAR) == Step(0.5, 0.195, -0.34)


def test_table_validation():
    with pytest.raises(ValueError):
        steps_from_lists([0.5, 1.0], [0.1], [0.0, 0.0])
    with pytest.raises(ValueError):
        ParkingSequence(SequenceParams(steps=steps_from_lists(DURATION, SPEED, STEER),
                                       sensor_step=12))


def test_step_to_twist_uses_bicycle_model():
    v, w = step_to_twist_values(Step(1.0, 0.2, 0.34), wheelbase=0.2)
    assert v == 0.2
    assert w == pytest.approx(0.2 * math.tan(0.34) / 0.2)
    v, w = step_to_twist_values(Step(1.0, 0.2, 0.0), wheelbase=0.2)
    assert w == 0.0


# --- start line ----------------------------------------------------------------------

IDENTITY = dict(x_h=0, x_l=0, y_h=-240, y_l=0)  # whole frame -> 320x240


def frame_with_line(y):
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.line(frame, (0, y), (639, y), (0, 230, 230), 8)   # yellow, inside the default HSV range
    return frame


def test_start_line_seen_and_triggers_when_close():
    det = StartLineDetector(StartLineParams(**IDENTITY))
    result = det.detect(frame_with_line(400))
    assert result.center_y is not None and result.center_y > 100
    assert det.should_start(result)


def test_start_line_far_away_does_not_trigger():
    det = StartLineDetector(StartLineParams(**IDENTITY))
    result = det.detect(frame_with_line(163))   # top rows of the cropped bird's-eye view
    assert result.center_y is not None
    assert not det.should_start(result)


def test_no_start_line_on_dark_frame():
    det = StartLineDetector(StartLineParams(**IDENTITY))
    result = det.detect(np.zeros((480, 640, 3), dtype=np.uint8))
    assert result.center_y is None
    assert not det.should_start(result)


def test_start_line_debug_image_shape():
    det = StartLineDetector(StartLineParams(**IDENTITY))
    result = det.detect(frame_with_line(400), debug=True)
    assert result.debug_image.shape == (160, 240, 3)


def test_start_line_rejects_degenerate_warp_and_keeps_previous_config():
    det = StartLineDetector(StartLineParams(**IDENTITY))
    before = det.p
    with pytest.raises(ValueError):
        det.configure(StartLineParams(x_h=0, x_l=0, y_h=-240, y_l=480))
    assert det.p is before
