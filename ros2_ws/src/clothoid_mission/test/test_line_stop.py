"""LineStopCore (FinishCore, cross line in white) on synthetic camera frames rendered as in
clothoid_finish/test/test_finish.py: a lane with a white stop line and crosswalk bars past it, driven
with FinishCore's own command until STOP."""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(__file__)
for rel in ('..', '../../clothoid_finish', '../../clothoid_lane', '../../clothoid_parking'):
    sys.path.insert(0, os.path.join(HERE, rel))

from clothoid_finish.bev import Bev, CameraModel  # noqa: E402
from clothoid_finish.finish_core import FinishParams, Phase  # noqa: E402
from clothoid_lane.lane_detector import LaneParams  # noqa: E402
from clothoid_mission.line_stop import LineStopCore  # noqa: E402

YELLOW = (0, 190, 240)
WHITE = (235, 235, 235)
HALF_LANE, LINE_W = 0.26, 0.02
STOP_X = 2.0                       # near edge of the white stop line, world x


def test_white_hue_rejects_a_tinted_stop_line_with_the_same_saturation_and_brightness():
    import cv2
    cam = CameraModel()
    bev = Bev(LaneParams(x_h=60, x_l=327, y_h=91, y_l=2), cam)
    image = render(cam, 1.3, 0., 0., letters=False)
    paint = np.all(image == WHITE, axis=2)
    tint = cv2.cvtColor(np.array([[[120, 30, 235]]], np.uint8), cv2.COLOR_HSV2BGR)[0, 0]
    image[paint] = tint
    original = LineStopCore(bev)
    filtered = LineStopCore(bev, white_h_min=0, white_h_max=45)
    original.image(0., image)
    filtered.image(0., image)
    assert original.cross_x is not None
    assert filtered.cross_x is None
    image[paint] = WHITE
    filtered.image(.1, image)
    assert filtered.cross_x is not None


def render(cam, x, y, yaw, crosswalk=True, letters=True, boundary=True, diamond=False):
    h, w = 480, 640
    v, u = np.mgrid[0:h, 0:w].astype(np.float64)
    below = np.arctan((v - cam.cy) / cam.fy) + cam.pitch
    ok = below > 0.01
    fwd = np.where(ok, cam.height / np.tan(np.where(ok, below, 1.0)), 0)
    zc = fwd * math.cos(cam.pitch) + cam.height * math.sin(cam.pitch)
    left = -(u - cam.cx) / cam.fx * zc
    bx, by = fwd + cam.x, left + cam.y
    c, s = math.cos(yaw), math.sin(yaw)
    wx, wy = x + bx * c - by * s, y + bx * s + by * c
    img = np.full((h, w, 3), (40, 45, 50), np.uint8)
    inside = ok & (np.abs(wy) < HALF_LANE)
    img[ok & (np.abs(np.abs(wy) - HALF_LANE) < LINE_W / 2)] = YELLOW
    white = inside & (wx >= STOP_X) & (wx < STOP_X + 0.05) & boundary
    if diamond:
        white |= inside & (np.abs(np.abs(wx-(STOP_X-.2))+np.abs(wy)-.14) < .012)
    if crosswalk:                   # 4 bars along the lane from 6 cm past the line, far edge line
        bars = (wx >= STOP_X + 0.11) & (wx < STOP_X + 0.51)
        for cy in (-0.165, -0.055, 0.055, 0.165):
            white |= inside & bars & (np.abs(wy - cy) < 0.035)
        white |= inside & (wx >= STOP_X + 0.57) & (wx < STOP_X + 0.60)
    if letters:   # "천천히" before the signal's stop line: strokes across, gaps between the letters
        for x0 in (STOP_X - 0.25, STOP_X - 0.20, STOP_X - 0.15):
            rows = (wx >= x0) & (wx < x0 + 0.012)
            for c0 in (-0.14, -0.045, 0.05):
                white |= inside & rows & (wy >= c0) & (wy < c0 + 0.08)
    img[white] = WHITE
    return img


def run(x0=0.8, y0=0.0, yaw0_deg=0.0, params=None, **scene):
    cam = CameraModel()
    core = LineStopCore(Bev(LaneParams(x_h=60, x_l=327, y_h=91, y_l=2), cam),
                        params or FinishParams(rear_offset=0.1, odom_scale=1.0))
    x, y, yaw, v, steer = x0, y0, math.radians(yaw0_deg), 0.0, 0.0
    dt, t = 0.01, 0.0
    core.start(t)
    for i in range(3000):
        t += dt
        if i % 7 == 0:   # ~14 Hz camera, as test_finish
            core.image(t, render(cam, x, y, yaw, **scene))
        if i % 2 == 0:
            core.imu(t, v * math.tan(steer) / 0.2)
            core.odom(t, v)
        if i % 5 == 0:
            out = core.command(t)
            if out is not None:
                tv, steer = out
                v += max(-0.05, min(0.05, tv - v))
        if core.phase in (Phase.STOP, Phase.ABORT) and abs(v) < 1e-6:
            break
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += v * math.tan(steer) / 0.2 * dt
    return core, x


def test_bumper_stops_at_the_white_line_before_the_crosswalk():
    # as finish on its end line: stop_gap 0.03, minus the ~3 cm the line reads further while it
    # leaves the bottom of the view -> the bumper ends near the line, not over the crosswalk
    core, x = run()
    assert core.phase == Phase.STOP
    bumper_gap = STOP_X - (x + 0.161)
    assert -0.01 <= bumper_gap <= 0.04, bumper_gap


def test_same_stop_without_the_crosswalk():
    a = STOP_X - (run()[1] + 0.161)
    b = STOP_X - (run(crosswalk=False)[1] + 0.161)
    assert abs(a - b) < 0.01


def test_yellow_cross_line_is_not_the_stop_line():
    # the white stop line is what counts: with no white line there is nothing to stop at
    cam = CameraModel()
    core = LineStopCore(Bev(LaneParams(x_h=60, x_l=327, y_h=91, y_l=2), cam), FinishParams())
    core.start(0.0)
    img = render(cam, 1.2, 0.0, 0.0)
    img[np.all(img == WHITE, axis=2)] = YELLOW
    for k in range(4):
        core.image(0.1 * k, img)
    assert core.phase == Phase.FOLLOW and core.cross_x is None


def test_letters_before_the_line_are_not_the_line():
    core, x = run(letters=True)
    assert core.phase == Phase.STOP
    assert -0.01 <= STOP_X - (x + 0.161) <= 0.04      # at the white line, not 20+ cm short


def test_committed_target_cannot_jump_to_a_nearer_marking():
    core = LineStopCore(Bev(LaneParams(), CameraModel()))
    core.start(0.)
    for t in (0., .05, .1):
        core._sighting(t, 1.)
    assert core.phase == Phase.COMMIT
    target = core.target
    core._sighting(.15, .7)
    assert core.target == target


def test_rejected_later_line_is_not_reported_as_the_active_stop_measurement():
    cam = CameraModel()
    core = LineStopCore(Bev(LaneParams(x_h=60, x_l=327, y_h=91, y_l=2), cam),
                        FinishParams(band=.22))
    core.start(0.)
    for t in (0., .05, .1):
        core._sighting(t, 1.)
    target = core.target
    core.dist = .6  # the committed line is now below the camera
    core.image(.2, render(cam, 1.3, 0., 0., crosswalk=False, letters=False))
    assert core.observation.lines  # another white line is present in the image
    assert core.cross_x is None and core.target == target


def test_stale_line_sightings_do_not_form_a_confirmation():
    core = LineStopCore(Bev(LaneParams(), CameraModel()))
    core.start(0.)
    for t in (0., .5, 1.):
        core._sighting(t, 1.)
    assert core.phase == Phase.FOLLOW


def test_mission_line_requires_crosswalk_context():
    cam = CameraModel()
    core = LineStopCore(Bev(LaneParams(x_h=60, x_l=327, y_h=91, y_l=2), cam),
                        FinishParams(band=.22))
    core.require_crosswalk = True
    core.start(0.)
    for k in range(5):
        core.image(k*.05, render(cam, 1.3, 0., 0., crosswalk=False, letters=False))
    assert core.phase == Phase.FOLLOW
    for k in range(5, 10):
        core.image(k*.05, render(cam, 1.3, 0., 0., crosswalk=True, letters=False))
    assert core.phase == Phase.COMMIT


def test_explicit_reference_does_not_change_finish_defaults():
    params = FinishParams(front=.1, stop_gap=.03, rear_offset=.1, odom_scale=1.)
    core, x = run(params=params)
    assert core.phase == Phase.STOP
    assert -.01 <= STOP_X-(x+.1) <= .04
    assert FinishParams().front == .161


def test_oblique_mission_boundary_is_visible_without_becoming_a_crosswalk_stop():
    cam = CameraModel()
    core = LineStopCore(Bev(LaneParams(x_h=60, x_l=327, y_h=91, y_l=2), cam),
                        FinishParams(band=.22))
    core.require_crosswalk = True
    core.observe_entry = True
    core.start(0.)
    for k in range(3):
        core.image(k*.05, render(cam, 1.3, 0., math.radians(8), crosswalk=False, letters=False))
    assert core.phase == Phase.FOLLOW and core.cross_x is None
    assert len(core.observation.entry_lines) == 1
    assert .65 < core.observation.entry_lines[0] < .75


def test_entry_component_fit_rejects_diamond_and_letter_markings():
    cam = CameraModel()
    core = LineStopCore(Bev(LaneParams(x_h=60, x_l=327, y_h=91, y_l=2), cam),
                        FinishParams(band=.22))
    core.observe_entry = True
    for yaw in (0., .15):
        core.image(0., render(cam, 1.1, 0., yaw, crosswalk=False,
                              boundary=False, letters=True, diamond=True))
        assert not core.observation.entry_lines


def _grace_core(grace):
    cam = CameraModel()
    core = LineStopCore(Bev(LaneParams(x_h=60, x_l=327, y_h=91, y_l=2), cam), FinishParams(band=.22))
    core.require_crosswalk, core.crosswalk_grace = True, grace
    core.start(0.)
    return cam, core


@pytest.mark.parametrize('grace,gap,committed', [(.35, .05, True), (0., .05, False), (.35, .5, False)])
def test_crosswalk_just_after_the_line_confirms_it_within_the_grace(grace, gap, committed):
    """1118 1607: blue light; the line was seen without its bars, the bars only in the next image."""
    cam, core = _grace_core(grace)
    for k in range(4):                                           # the line alone
        core.image(k*.05, render(cam, 1.3, 0., 0., crosswalk=False, letters=False))
    assert core.phase == Phase.FOLLOW
    core.image(.15+gap, render(cam, 1.3, 0., 0., crosswalk=True, boundary=False, letters=False))
    assert (core.phase == Phase.COMMIT) == committed


def test_crosswalk_alone_confirms_nothing():
    cam, core = _grace_core(.35)
    for k in range(5):
        core.image(k*.05, render(cam, 1.3, 0., 0., crosswalk=True, boundary=False, letters=False))
    assert core.phase == Phase.FOLLOW
