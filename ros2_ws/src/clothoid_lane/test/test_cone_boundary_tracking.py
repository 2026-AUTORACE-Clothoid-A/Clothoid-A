"""A boundary must retain its side when a slanted fit crosses the image centre."""
import numpy as np

from clothoid_lane.lane_detector import LaneParams, SIDE_LEFT, SIDE_RIGHT
from clothoid_lane.lane_tracker import LaneTracker, Line


def line(slope, intercept, color='yellow', rows=(80., 160.)):
    ys = np.linspace(*rows, 8)
    coeff = np.array([slope, intercept])
    return Line(np.column_stack((np.polyval(coeff, ys), ys)), coeff, color,
                bottom_x=float(np.polyval(coeff, 239)))


def test_short_slanted_boundary_keeps_side_when_bottom_extrapolation_crosses_centre():
    tracker = LaneTracker(LaneParams())
    tracker.set_boundary_continuity(True)
    old, new = line(.8, 20.), line(.5, 37.)
    assert old.bottom_x > 160 > new.bottom_x
    assert tracker._pick([old], 170., 320) == (None, old)
    # The boundary barely moved on the rows actually seen in both frames.
    assert tracker._pick([new], 170., 320) == (None, new)


def test_one_observed_line_is_never_assigned_to_both_sides():
    tracker = LaneTracker(LaneParams())
    tracker.set_boundary_continuity(True)
    first = line(0., 180.)
    tracker._pick([first], 170., 320)
    for x in (170., 160., 150., 140.):
        current = line(0., x)
        left, right = tracker._pick([current], 170., 320)
        assert left is None and right is current


def test_short_occlusion_and_colour_change_keep_boundary_identity():
    tracker = LaneTracker(LaneParams())
    tracker.set_boundary_continuity(True)
    tracker._pick([line(0., 175.)], 170., 320)
    for _ in range(3):
        assert tracker._pick([], 170., 320) == (None, None)
    current = line(0., 155., color='white')
    assert tracker._pick([current], 170., 320) == (None, current)


def test_expired_observation_does_not_invent_a_boundary_or_block_reacquisition():
    tracker = LaneTracker(LaneParams())
    tracker.set_boundary_continuity(True)
    tracker._pick([line(0., 180.)], 170., 320)
    for _ in range(tracker.t.track_hold+1):
        assert tracker._pick([], 170., 320) == (None, None)
    current = line(0., 100.)
    assert tracker._pick([current], 170., 320) == (current, None)


def test_exit_reacquisition_clears_inside_course_seeds_once():
    tracker = LaneTracker(LaneParams())
    tracker._pick([line(0., 180.)], 170., 320)
    tracker.set_boundary_continuity(True)
    assert tracker._sides[SIDE_RIGHT].line is None
    current = line(0., 100.)
    tracker._pick([current], 170., 320)
    tracker.set_boundary_continuity(True)
    assert tracker._sides[SIDE_LEFT].line is current


def test_line_without_common_observed_rows_cannot_claim_previous_identity():
    tracker = LaneTracker(LaneParams())
    tracker.set_boundary_continuity(True)
    tracker._pick([line(0., 180., rows=(80., 120.))], 170., 320)
    # Same extrapolated bottom, but no common observed support: wait, do not match.
    assert tracker._pick([line(0., 180., rows=(120., 230.))], 170., 320) == (None, None)


def test_distant_fragment_cannot_replace_boundary_by_extrapolation():
    tracker = LaneTracker(LaneParams())
    tracker.set_boundary_continuity(True)
    assert tracker._pick([line(0., 180., rows=(20., 80.))], 170., 320) == (None, None)
