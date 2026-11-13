import math

import cv2
import numpy as np
import pytest

from clothoid_lidar.section_cues import (CameraModel, ColourParams, EntryLineTracker, WhiteParams,
                                         find_entry_line, find_lane_lines)

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


def _sighting(ahead, heading=0.3):
    from clothoid_lidar.section_cues import EntryLine
    return EntryLine(row=0, ahead=ahead, heading=heading)


def test_another_mark_seen_once_does_not_move_the_crossing():
    """1108 run 6: the entry line seen down to 0.65 m (crossing 2.39), then a single sighting of
    another mark 0.64 m ahead when the car was at 2.13 (2.77) must not replace it."""
    tr = EntryLineTracker()
    t, odo = 0.0, 1.60
    for ahead in (0.95, 0.89, 0.83, 0.78, 0.73, 0.68, 0.65):
        assert tr.update(t, odo, _sighting(ahead)) is None
        t, odo = t + 0.05, odo + 0.02
    c = tr.update(t + 0.2, 2.13, _sighting(0.64))
    assert c is not None and c.odometer == pytest.approx(1.72 + 0.65, abs=0.01)
    assert tr.update(t + 0.8, 2.30, None) is None          # the single sighting is no crossing
    assert tr.recent(2.40, 3.0) is c


def test_two_marks_seen_in_turn_do_not_reset_each_other():
    tr = EntryLineTracker()
    t, odo = 0.0, 0.0
    for k in range(8):                          # entry line at 0.66, another mark at 1.30, in turn
        ahead = (0.66 - odo) if k % 2 == 0 else (1.30 - odo)
        tr.update(t, odo, _sighting(ahead))
        t, odo = t + 0.1, odo + 0.02
    assert tr.last is not None and tr.count >= 2
    assert tr.last[1] + tr.last[2].ahead == pytest.approx(0.66, abs=0.01)


def test_a_line_seen_in_one_frame_only_is_no_crossing():
    tr = EntryLineTracker()
    tr.update(0.0, 1.0, _sighting(0.6))
    assert tr.update(0.5, 1.2, None) is None


def test_far_line_is_not_a_crossing():
    tr = EntryLineTracker()
    tr.update(0.0, 0.0, find_entry_line(floor_image([band(0.8)]), CAM, WhiteParams()))
    tr.last = (tr.last[0], tr.last[1], type(tr.last[2])(row=0, ahead=1.4, heading=0.0))
    assert tr.update(1.0, 0.5, None) is None


def lane_strip(y, yaw=0.0, half=0.012, x0=0.5, x1=1.5):
    """A floor line through (0, y) at angle yaw from the car's x axis."""
    t = math.tan(yaw)
    return [(x0, y + t * x0 - half), (x1, y + t * x1 - half), (x1, y + t * x1 + half),
            (x0, y + t * x0 + half)]


@pytest.mark.parametrize('yaw_deg, offset', [(0.0, 0.0), (6.0, 0.0), (-9.0, 0.05), (4.0, -0.08)])
def test_lane_lines_give_direction_and_offset(yaw_deg, offset):
    yaw = math.radians(yaw_deg)
    img = floor_image([lane_strip(0.25 + offset, yaw), lane_strip(-0.25 + offset, yaw)])
    obs = find_lane_lines(img, CAM, ColourParams(), 0.0, math.radians(15))
    assert obs is not None
    assert obs.angle == pytest.approx(yaw, abs=math.radians(1.5))
    ys = sorted(ln.y0 for ln in obs.lines)
    assert len(ys) == 2
    assert ys[0] == pytest.approx(-0.25 + offset, abs=0.03)
    assert ys[1] == pytest.approx(0.25 + offset, abs=0.03)


def test_cross_line_does_not_turn_the_lane_direction():
    img = floor_image([lane_strip(0.25), lane_strip(-0.25), band(0.8, width=0.08)])
    obs = find_lane_lines(img, CAM, ColourParams(), 0.0, math.radians(15))
    assert obs is not None and abs(obs.angle) < math.radians(2)


def test_white_only_ignores_yellow_paint_and_cones():
    from clothoid_lidar.section_cues import find_lane_lines
    cam = CameraModel()
    img = np.zeros((480, 640, 3), np.uint8)
    for y in (0.25, -0.25):
        a = cam.floor_to_image(0.6, y)
        b = cam.floor_to_image(1.4, y)
        cv2.line(img, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), (255, 255, 255), 5)
    a, b = cam.floor_to_image(0.6, -0.1), cam.floor_to_image(1.4, 0.3)      # a yellow slant
    cv2.line(img, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), (0, 220, 220), 40)
    obs = find_lane_lines(img, cam, ColourParams(), 0.0, math.radians(20.0), white_only=True)
    assert obs is not None and abs(obs.angle) < math.radians(2.0)
    assert obs.lines and all(line.colour == 'white' for line in obs.lines)



def _crossed(tr, odo):
    from clothoid_lidar.section_cues import Crossing
    c = Crossing(odometer=odo, heading=0.0, t=0.0)
    tr.crossings.append(c)
    return c


def test_start_line_is_where_the_lane_line_pair_starts():
    from clothoid_lidar.section_cues import EntryLineTracker
    tr = EntryLineTracker()
    c = _crossed(tr, 2.3)
    for k in range(30):                                   # corner exit: no pair
        tr.observe_pair(0.3 + 0.05 * k, False)
    decided = []
    for k in range(12):                                   # from 0.6 m before it: the pair
        decided += tr.observe_pair(1.7 + 0.08 * k, k % 2 == 0)
    assert c.start is True and decided == [c]
    for k in range(20):                                   # beside the island and on: the pair
        tr.observe_pair(5.6 + 0.05 * k, True)
    end = _crossed(tr, 7.5)                               # the section's end line, 5.2 m on
    tr.observe_pair(7.6, True)
    assert tr.is_start(end, 7.6) is False                 # the pair was there before it
    assert tr.recent(8.0, 7.0, start_only=True) is c


def test_a_crosswalk_line_is_not_the_start_line():
    from clothoid_lidar.section_cues import EntryLineTracker
    tr = EntryLineTracker()
    c = _crossed(tr, 0.6)
    for k in range(25):                                   # no lane-line pair around it
        tr.observe_pair(0.0 + 0.06 * k, False)
    assert c.start is False
    assert tr.recent(1.0, 2.0, start_only=True) is None and tr.recent(1.0, 2.0) is c


def test_pair_seen_before_the_line_is_not_a_start():
    from clothoid_lidar.section_cues import EntryLineTracker
    tr = EntryLineTracker()
    c = _crossed(tr, 5.0)
    for k in range(40):                                   # inside the section already
        tr.observe_pair(3.0 + 0.06 * k, True)
    assert c.start is False
