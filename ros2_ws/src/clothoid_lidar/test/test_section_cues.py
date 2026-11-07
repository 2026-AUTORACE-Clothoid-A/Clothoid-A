import math

import cv2
import numpy as np
import pytest

from clothoid_lidar.section_cues import (CameraModel, EntryLineTracker, WhiteParams,
                                         find_entry_line)

CAM = CameraModel()


def floor_image(polygons, size=(480, 640)):
    """Dark floor with white floor polygons (base_link metres) drawn through the camera model."""
    img = np.full(size + (3,), 40, np.uint8)
    for poly in polygons:
        pts = np.array([CAM.floor_to_image(x, y) for x, y in poly], np.int32)
        cv2.fillPoly(img, [pts], (235, 235, 235))
    return img


def band(x, width=0.05, half=0.4, yaw=0.0):
    """A white line across the lane at x ahead, rotated by yaw (car yawed by -yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(-width / 2, -half), (width / 2, -half), (width / 2, half), (-width / 2, half)]
    return [(x + c * a - s * b, s * a + c * b) for a, b in pts]


def test_projection_round_trip():
    for x, y in ((0.45, 0.0), (0.6, 0.2), (1.0, -0.3)):
        px, py = CAM.floor_to_image(x, y)
        fx, fy = CAM.image_to_floor(px, py)
        assert fx == pytest.approx(x, abs=1e-6) and fy == pytest.approx(y, abs=1e-6)


@pytest.mark.parametrize('x', [0.6, 0.75, 0.9])   # limo2 sees the floor from 0.52 m
def test_entry_line_distance(x):
    line = find_entry_line(floor_image([band(x)]), CAM, WhiteParams())
    assert line is not None
    assert line.ahead == pytest.approx(x, abs=0.02)
    assert abs(line.heading) < math.radians(2)


def test_heading_from_the_slant():
    yaw = math.radians(6)
    line = find_entry_line(floor_image([band(0.7, yaw=yaw)]), CAM, WhiteParams())
    assert line is not None
    assert line.heading == pytest.approx(-yaw, abs=math.radians(1.5))


def test_dashes_and_letters_are_not_the_entry_line():
    dashes = [[(x0, y - 0.01), (x0 + 0.1, y - 0.01), (x0 + 0.1, y + 0.01), (x0, y + 0.01)]
              for x0 in (0.6, 0.8) for y in (0.25, -0.25)]
    letter = [[(0.65, -0.04), (0.73, -0.04), (0.73, 0.04), (0.65, 0.04)]]
    assert find_entry_line(floor_image(dashes + letter), CAM, WhiteParams()) is None


def test_white_threshold_follows_the_lane_tuning():
    img = floor_image([band(0.7)])
    img[img > 200] = 160                       # dimmer line, e.g. other lighting
    assert find_entry_line(img, CAM, WhiteParams()) is None
    assert find_entry_line(img, CAM, WhiteParams(v_min=140)) is not None


def test_tracker_turns_the_last_sighting_into_a_crossing():
    tr = EntryLineTracker()
    t, odo = 0.0, 10.0
    for x in (0.8, 0.75, 0.7, 0.65):           # line coming closer while the car drives 5 cm/frame
        line = find_entry_line(floor_image([band(x)]), CAM, WhiteParams())
        assert tr.update(t, odo, line) is None
        t, odo = t + 0.1, odo + 0.05
    assert tr.update(t, odo, None) is None     # just lost: not yet
    c = tr.update(t + 0.4, odo + 0.2, None)
    assert c is not None
    assert c.odometer == pytest.approx(10.15 + 0.65, abs=0.03)
    assert tr.recent(c.odometer + 0.7, 3.0) is c
    assert tr.recent(c.odometer + 3.5, 3.0) is None


def test_far_line_is_not_a_crossing():
    tr = EntryLineTracker()
    tr.update(0.0, 0.0, find_entry_line(floor_image([band(0.8)]), CAM, WhiteParams()))
    tr.last = (tr.last[0], tr.last[1], type(tr.last[2])(row=0, ahead=1.4, heading=0.0))
    assert tr.update(1.0, 0.5, None) is None
