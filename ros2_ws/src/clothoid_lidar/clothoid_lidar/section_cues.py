"""Camera cues for the static obstacle section, ROS-free.

The white entry line of the 3-lane section is the reference point of the section: the painted island
starts 4.40 m after it (rule book p.4 drawing at scale). The camera sees the line until it is about
0.5 m ahead of base_link; the base crosses it after driving that far, measured with the odometer.

Camera geometry is the lane package's per-car profile (camera_limo1/2.yaml: camera_height,
camera_pitch, camera_x, camera_y) with decision_viz's intrinsics, and the same flat-floor projection
as clothoid_debug.geometry.GroundProjection.image_to_floor. White = the lane tracker's white_s_max /
white_v_min, so tuning the lane at the venue also tunes this.
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class CameraModel:
    fx: float = 489.14            # px, 640x480 image (decision_viz)
    fy: float = 489.14
    cx: float = 322.0
    cy: float = 217.0
    height: float = 0.19          # m, limo2 = car 7 (camera_limo2.yaml)
    pitch: float = -0.021         # rad, + looks down
    x: float = 0.15               # m ahead of base_link
    y: float = -0.01              # m, left positive
    width: int = 640              # image size the intrinsics belong to
    height_px: int = 480

    def image_to_floor(self, px: float, py: float) -> Optional[Tuple[float, float]]:
        """Full-resolution pixel -> floor point (x forward, y left) in base_link, None above the
        horizon."""
        x, y = (px - self.cx) / self.fx, (py - self.cy) / self.fy
        c, s = math.cos(self.pitch), math.sin(self.pitch)
        forward, down = c - y * s, y * c + s
        if down <= 1e-6:
            return None
        scale = self.height / down
        return self.x + scale * forward, self.y - scale * x

    def floor_to_image(self, fx_: float, fy_: float) -> Tuple[float, float]:
        """Inverse of image_to_floor (tests, drawing)."""
        X, Y = fx_ - self.x, fy_ - self.y            # from the lens, X forward, Y left
        c, s = math.cos(self.pitch), math.sin(self.pitch)
        h = self.height
        y = (h * c - X * s) / (X * c + h * s)          # normalised image coordinates
        x = -Y * (y * c + s) / h
        return self.cx + self.fx * x, self.cy + self.fy * y


@dataclass
class WhiteParams:
    s_max: int = 50               # lane_main white_s_max
    v_min: int = 170              # lane_main white_v_min
    roi_top: float = 0.7          # search the image below this fraction of its height
    min_width: float = 0.25       # of the image width
    max_angle_deg: float = 30.0   # in the image, from horizontal
    min_area: int = 100           # px after shrinking
    work_width: int = 320         # shrink to this before searching


@dataclass
class EntryLine:
    row: float                    # full-resolution image row of the band's middle
    ahead: float                  # m from base_link to the line, along x, at the image centre column
    heading: float                # rad, car heading relative to the lane (+ left) from the line's slant


def find_entry_line(bgr: np.ndarray, cam: CameraModel, p: WhiteParams) -> Optional[EntryLine]:
    """The nearest long, nearly horizontal white band low in the image, or None. Dashes, letters
    and the white wall foot are too narrow or too high."""
    h, w = bgr.shape[:2]
    scale = w / float(p.work_width)
    small = cv2.resize(bgr, (p.work_width, max(1, int(round(h / scale)))),
                       interpolation=cv2.INTER_AREA)
    sh = small.shape[0]
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, (0, 0, int(p.v_min)), (180, int(p.s_max), 255))
    white[: int(sh * p.roi_top)] = 0
    n, lab, st, _ = cv2.connectedComponentsWithStats(white)
    best = None
    for k in range(1, n):
        ww, area = st[k, cv2.CC_STAT_WIDTH], st[k, cv2.CC_STAT_AREA]
        if ww < p.min_width * p.work_width or area < p.min_area:
            continue
        ys, xs = np.nonzero(lab == k)
        slope, intercept = np.polyfit(xs, ys, 1)
        if abs(math.degrees(math.atan(slope))) > p.max_angle_deg:
            continue
        row = float(np.median(ys))
        if best is None or row > best[0]:
            best = (row, slope, intercept, xs.min(), xs.max())
    if best is None:
        return None
    row, slope, intercept, x0, x1 = best
    sx = cam.width / float(p.work_width)
    sy = cam.height_px / float(sh)
    centre = cam.image_to_floor(cam.cx, (slope * cam.cx / sx + intercept) * sy)
    a = cam.image_to_floor(x0 * sx, (slope * x0 + intercept) * sy)
    b = cam.image_to_floor(x1 * sx, (slope * x1 + intercept) * sy)
    if centre is None or a is None or b is None:
        return None
    # the line is across the lane; seen from a car yawed by psi it points along (sin psi, cos psi)
    vx, vy = a[0] - b[0], a[1] - b[1]     # a is left in the image -> larger y
    heading = math.atan2(vx, vy) if vy > 0 else math.atan2(-vx, -vy)
    return EntryLine(row=row * sy, ahead=centre[0], heading=heading)


@dataclass
class Crossing:
    odometer: float               # m, where base_link crossed the line
    heading: float                # rad, heading estimate from the last sighting
    t: float


class EntryLineTracker:
    """Turns per-frame sightings into crossings: when a line seen close ahead disappears, the base
    crosses it `ahead` metres (odometer) after the last sighting."""

    def __init__(self, max_last_ahead: float = 0.9, lost_after: float = 0.3, keep: float = 20.0):
        self.max_last_ahead = max_last_ahead   # the last sighting must be this close (not a far wall)
        self.lost_after = lost_after           # s without a sighting = passed
        self.keep = keep                       # m of odometer to remember crossings
        self.last: Optional[Tuple[float, float, EntryLine]] = None   # (t, odometer, line)
        self.crossings: List[Crossing] = []

    def update(self, t: float, odometer: float, line: Optional[EntryLine]) -> Optional[Crossing]:
        if line is not None:
            self.last = (t, odometer, line)
            return None
        if self.last is None or t - self.last[0] < self.lost_after:
            return None
        t0, odo0, ln = self.last
        self.last = None
        if ln.ahead > self.max_last_ahead:
            return None
        c = Crossing(odometer=odo0 + ln.ahead, heading=ln.heading, t=t0)
        self.crossings.append(c)
        self.crossings = [x for x in self.crossings if odometer - x.odometer <= self.keep]
        return c

    def recent(self, odometer: float, max_back: float) -> Optional[Crossing]:
        """Last crossing at most max_back metres behind (or slightly ahead of) the car."""
        for c in reversed(self.crossings):
            if -0.3 <= odometer - c.odometer <= max_back:
                return c
        return None
