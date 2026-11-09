"""LineStopCore (FinishCore, cross line in white) on synthetic camera frames rendered as in
clothoid_finish/test/test_finish.py: a lane with a white stop line and crosswalk bars past it, driven
with FinishCore's own command until STOP."""
import math
import os
import sys

import numpy as np

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


def render(cam, x, y, yaw, crosswalk=True, letters=True):
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
    white = inside & (wx >= STOP_X) & (wx < STOP_X + 0.05)
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


def run(x0=0.8, y0=0.0, yaw0_deg=0.0, **scene):
    cam = CameraModel()
    core = LineStopCore(Bev(LaneParams(x_h=60, x_l=327, y_h=91, y_l=2), cam),
                        FinishParams(rear_offset=0.1, odom_scale=1.0))
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
