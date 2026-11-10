"""FinishCore on synthetic camera frames: a straight lane drawn on the floor through the same pinhole
model, rendered into a 640x480 image, then driven with a bicycle model to the end line."""
import math
import os
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'clothoid_lane'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'clothoid_parking'))

from clothoid_finish.bev import Bev, CameraModel  # noqa: E402
from clothoid_finish.finish_core import FinishCore, FinishParams, Phase  # noqa: E402
from clothoid_lane.lane_detector import LaneParams  # noqa: E402

YELLOW = (0, 190, 240)          # BGR, inside lane.yaml's HSV range
WHITE = (235, 235, 235)
LEFT_Y, RIGHT_Y, END_X = 0.26, -0.26, 2.0     # world lines (lane along +x, line centres)
LINE_W = 0.02


def render(cam: CameraModel, x, y, yaw, white_right=True):
    """Camera image of the floor for the car's base_link at (x, y, yaw)."""
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
    lane = ok & (wx < END_X + 0.6)
    yel = lane & ((np.abs(wy - LEFT_Y) < LINE_W / 2) |
                  ((np.abs(wx - (END_X + LINE_W / 2)) < LINE_W / 2) & (wy > RIGHT_Y) & (wy < LEFT_Y)))
    img[yel] = YELLOW
    if white_right:   # parking lines on the right: white, with stubs pointing into the lane side
        wht = lane & ((np.abs(wy - RIGHT_Y) < LINE_W / 2) |
                      ((np.abs((wx % 0.52) - 0.26) < LINE_W / 2) & (wy < RIGHT_Y) & (wy > RIGHT_Y - 0.6)))
        img[wht & ~yel] = WHITE
    return img


def run(x0=0.3, y0=0.03, yaw0_deg=1.0, gap=0.03, odom_scale=1.0, seconds=20.0):
    cam = CameraModel()
    bev = Bev(LaneParams(x_h=60, x_l=327, y_h=91, y_l=2), cam)
    core = FinishCore(bev, FinishParams(stop_gap=gap, rear_offset=0.1, odom_scale=1.0))
    x, y, yaw, v, steer = x0, y0, math.radians(yaw0_deg), 0.0, 0.0
    dt, t = 0.01, 0.0
    core.start(t)
    for i in range(int(seconds / dt)):
        t += dt
        if i % 7 == 0:   # ~14 Hz camera
            core.image(t, render(cam, x, y, yaw))
        if i % 2 == 0:
            core.odom(t, v * odom_scale)
        core.imu(t, v * math.tan(steer) / 0.2)
        if i % 5 == 0:
            out = core.command(t)
            if out is not None:
                v, steer = out
        # rear-axle bicycle, base_link 0.1 ahead
        rx, ry = x - 0.1 * math.cos(yaw), y - 0.1 * math.sin(yaw)
        yaw += v * math.tan(steer) / 0.2 * dt
        rx += v * math.cos(yaw) * dt
        ry += v * math.sin(yaw) * dt
        x, y = rx + 0.1 * math.cos(yaw), ry + 0.1 * math.sin(yaw)
        if core.phase in (Phase.STOP, Phase.ABORT) and v == 0:
            break
    return core, x, y, yaw


def test_bev_metres_match_the_floor():
    cam = CameraModel()
    bev = Bev(LaneParams(x_h=60, x_l=327, y_h=91, y_l=2), cam)
    img = bev.warp_image(render(cam, 0.0, 0.0, 0.0))
    m = cv2.inRange(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), np.array([18, 60, 90]), np.array([35, 255, 255])) > 0
    ys = bev.gy[m & (bev.gy > 0)]
    assert np.median(ys) == pytest.approx(LEFT_Y, abs=0.01)


@pytest.mark.parametrize('y0,yaw0', [(0.0, 0.0), (0.04, 2.0), (-0.04, -3.0)])
def test_stops_at_the_end_line(y0, yaw0):
    core, x, y, yaw = run(y0=y0, yaw0_deg=yaw0)
    assert core.phase == Phase.STOP, core.reason
    front = x + 0.161 * math.cos(yaw)
    assert END_X - front == pytest.approx(0.03, abs=0.02)      # stop_gap 3 cm before the near edge
    assert abs(y) < 0.05 and abs(math.degrees(yaw)) < 4


def test_long_odom_still_stops_close():
    core, x, y, yaw = run(odom_scale=1.02)
    assert core.phase == Phase.STOP
    assert END_X - (x + 0.161 * math.cos(yaw)) == pytest.approx(0.03, abs=0.03)


def test_no_end_line_aborts():
    global END_X
    keep, END_X = END_X, 50.0
    try:
        core, *_ = run(seconds=25.0)
    finally:
        END_X = keep
    assert core.phase == Phase.ABORT
