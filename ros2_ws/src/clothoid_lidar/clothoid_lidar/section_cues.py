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

    def image_to_floor_array(self, px: np.ndarray, py: np.ndarray) -> np.ndarray:
        """image_to_floor for arrays: (N, 2) floor points of the pixels below the horizon."""
        x, y = (px - self.cx) / self.fx, (py - self.cy) / self.fy
        c, s = math.cos(self.pitch), math.sin(self.pitch)
        forward, down = c - y * s, y * c + s
        ok = down > 1e-6
        scale = self.height / down[ok]
        return np.column_stack((self.x + scale * forward[ok], self.y - scale * x[ok]))

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
    # the static obstacle section's start line: the pair of white dashed lane lines (one lane
    # width apart) starts there. None until decided (see EntryLineTracker.is_start)
    start: Optional[bool] = None
    pairs: str = ''               # 'k/n' frames with the pair around the line, for the log
    where: str = ''               # course map position, when the course map decides


class EntryLineTracker:
    """Turns per-frame sightings into crossings: when a line seen close ahead disappears, the base
    crosses it `ahead` metres (odometer) after the last sighting."""

    def __init__(self, max_last_ahead: float = 0.9, lost_after: float = 0.3, keep: float = 20.0,
                 max_slant: float = math.radians(40.0), same_line: float = 0.15, min_sightings: int = 2,
                 view_min: float = 0.45):
        self.max_last_ahead = max_last_ahead   # the last sighting must be this close (not a far wall)
        # the entry line looked slanted by +6..+31 deg in the 11-05 / 11-08 bags (the car leaves the
        # corner before it; the slant is NOT the car's heading to the lane, the lidar frame matched
        # the tape best without using it); other white marks that pass the shape test were 43..60 deg
        self.max_slant = max_slant
        self.lost_after = lost_after           # s without a sighting = passed
        self.keep = keep                       # m of odometer to remember crossings
        # one line = sightings whose crossing estimates (odometer + ahead) agree within same_line; a
        # sighting that does not is another mark (1108 run 6: one frame of a mark 0.38 m further on,
        # when the entry line was already below the camera's view, replaced the crossing). A line
        # needs min_sightings frames: a single one is not trusted (2: right after the corner the line
        # is in view for ~0.15 m only, 2~3 frames on the 6 Hz image of a dark venue)
        self.same_line = same_line
        self.min_sightings = min_sightings
        # m: closer than this the camera cannot see the floor; until the tracked line is that close,
        # a sighting of another mark is ignored (two marks seen in turn must not reset each other)
        self.view_min = view_min
        self.last: Optional[Tuple[float, float, EntryLine]] = None   # (t, odometer, line)
        self.count = 0                         # sightings of the line in self.last
        self.crossings: List[Crossing] = []
        # the section's start line among all the white cross lines (every mission has start / end
        # lines, the crosswalks too): the pair of dashed lane lines one lane width apart is seen from
        # start_near[0] to start_near[1] m around it in at least start_ratio of start_frames+ frames,
        # and was not start_before m before it (where frames exist: a line inside the section, its
        # end ~5.2 m on). The 33 section bags 11-08~11-13: 33 start lines confirmed, every crosswalk
        # and end line not. The whole-course bags also confirm lines at the slow zone, the zigzag /
        # diamond marks and the parking area (white lines 0.5 m apart there too); they matter only
        # if the avoidance starts there, which is a false start anyway. No "no new start line within
        # 7 m of one" rule: the zigzag line at 9.9 m hid the real one at 14.1 m
        self.start_near = (-0.7, 0.8)
        self.start_before = (-2.0, -1.0)
        self.start_ratio = 0.30
        self.start_before_max = 0.10
        self.start_frames = 5
        self.pairs: List[Tuple[float, bool]] = []   # (odometer, pair seen)
        # set by the caller with the course map on: crossing -> start line? (None: not yet known);
        # the lane-line pair is then not looked for
        self.start_by_map = None

    def update(self, t: float, odometer: float, line: Optional[EntryLine]) -> Optional[Crossing]:
        if line is not None:
            if self.last is not None:
                t0, odo0, ln0 = self.last
                tracked = odo0 + ln0.ahead
                if abs((odometer + line.ahead) - tracked) > self.same_line:
                    if tracked - odometer > self.view_min:
                        return None                     # another mark while the line is in view
                    c = self._finish(odometer)          # the line is below the view: it is done
                    self.last, self.count = (t, odometer, line), 1
                    return c
            self.last = (t, odometer, line)
            self.count += 1
            return None
        if self.last is None or t - self.last[0] < self.lost_after:
            return None
        return self._finish(odometer)

    def _finish(self, odometer: float) -> Optional[Crossing]:
        t0, odo0, ln = self.last
        count, self.last, self.count = self.count, None, 0
        if ln.ahead > self.max_last_ahead or abs(ln.heading) > self.max_slant or count < self.min_sightings:
            return None
        c = Crossing(odometer=odo0 + ln.ahead, heading=ln.heading, t=t0)
        self.crossings.append(c)
        self.crossings = [x for x in self.crossings if odometer - x.odometer <= self.keep]
        return c

    def wants_pairs(self, odometer: float) -> bool:
        """The lane-line pair is looked for only around white cross lines (5 ms a frame on the PC)."""
        if self.last is not None:
            return True
        return any(-1.0 <= odometer - c.odometer <= self.start_near[1] + 0.1 and c.start is None
                   for c in self.crossings[-3:])

    def observe_pair(self, odometer: float, seen: bool) -> List[Crossing]:
        """One frame's answer at its odometer; returns the crossings decided by it."""
        self.pairs.append((odometer, seen))
        self.pairs = [q for q in self.pairs if odometer - q[0] <= self.keep]
        return [c for c in self.crossings if c.start is None and self.is_start(c, odometer) is not None]

    def is_start(self, c: Crossing, odometer: float) -> Optional[bool]:
        if c.start is not None:
            return c.start
        if self.start_by_map is not None:
            c.start = self.start_by_map(c)
            return c.start
        near = [v for o, v in self.pairs if self.start_near[0] <= o - c.odometer <= self.start_near[1]]
        before = [v for o, v in self.pairs if self.start_before[0] <= o - c.odometer <= self.start_before[1]]
        c.pairs = '%d/%d' % (sum(near), len(near))
        if len(before) >= 3 and sum(before) / len(before) > self.start_before_max:
            c.start = False
        elif len(near) >= self.start_frames and sum(near) / len(near) >= self.start_ratio:
            c.start = True
        elif odometer - c.odometer > self.start_near[1]:
            c.start = False
        return c.start

    def recent(self, odometer: float, max_back: float, max_ahead: float = 1.0,
               start_only: bool = False) -> Optional[Crossing]:
        """The latest crossing at most max_back metres behind the car, or already seen but still up
        to max_ahead metres ahead of it (the base crosses it soon). A line seen ahead wins over an
        older one behind: in the 1108 bags the stop line before the corner is 1.7 m before the entry
        line, and the start often comes just before the entry line."""
        best = None
        for c in self.crossings:
            if start_only and c.start is not True:
                continue
            if -max_ahead <= odometer - c.odometer <= max_back:
                if best is None or c.odometer > best.odometer:
                    best = c
        return best


def dashed_pair(obs: Optional['LaneObservation'], lane_width: float, tol: float = 0.08) -> bool:
    """Two white lines along the road one lane width apart (the three-lane section's dividers)."""
    if obs is None:
        return False
    ys = sorted(ln.y0 for ln in obs.lines if ln.colour == 'white')
    return any(abs(b - a - lane_width) <= tol for i, a in enumerate(ys) for b in ys[i + 1:])


def check_start_line(entry: 'EntryLineTracker', bgr, cam, colours, odometer: float,
                     lane_width: float) -> List[Crossing]:
    """Feed one frame to the start-line check (only near white cross lines); the crossings it
    decided. With the course map the crossings are decided by their position, no image work."""
    if entry.start_by_map is not None:
        return [c for c in entry.crossings if c.start is None and entry.is_start(c, odometer) is not None]
    if not entry.wants_pairs(odometer):
        return []
    obs = find_lane_lines(bgr, cam, colours, 0.0, math.radians(25.0), white_only=True)
    return entry.observe_pair(odometer, dashed_pair(obs, lane_width))


@dataclass
class ColourParams:
    """Lane paint, same thresholds as lane_main (white_s_max, white_v_min, hsv_lower, hsv_upper)."""
    white_s_max: int = 50
    white_v_min: int = 170
    yellow_lower: Tuple[int, int, int] = (18, 60, 90)    # 11-05 tape, namgyu/lane
    yellow_upper: Tuple[int, int, int] = (35, 255, 255)
    roi_top: float = 0.55         # fraction of the image height
    pixel_step: int = 3           # use every n-th painted pixel
    x_min: float = 0.45           # m, floor window in base_link
    x_max: float = 1.5
    y_max: float = 1.0
    min_total: int = 30           # floor points in a frame
    min_points: int = 12          # floor points on one line
    min_extent: float = 0.12      # m of floor along x covered by one line
    max_lines: int = 6            # per colour


@dataclass
class LaneLine:
    y0: float                     # m, line's lateral position at base_link (x = 0), left positive
    colour: str                   # 'white' | 'yellow'
    n: int
    extent: float


@dataclass
class LaneObservation:
    angle: float                  # rad, direction of the parallel lane lines relative to the car
    lines: List[LaneLine]


def _floor_points(mask: np.ndarray, cam: CameraModel, p: ColourParams, sx: float,
                  sy: float) -> np.ndarray:
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return np.zeros((0, 2))
    pts = cam.image_to_floor_array(xs[::p.pixel_step] * sx, ys[::p.pixel_step] * sy)
    ok = (pts[:, 0] > p.x_min) & (pts[:, 0] < p.x_max) & (np.abs(pts[:, 1]) < p.y_max)
    return pts[ok]


def find_lane_lines(bgr: np.ndarray, cam: CameraModel, p: ColourParams, centre: float = 0.0,
                    half_range: float = math.radians(10.0),
                    white_only: bool = False) -> Optional[LaneObservation]:
    """White and yellow paint projected on the floor; one common direction for all the parallel lane
    lines (searched around `centre`, relative to the car), then a peak per line. Cross lines and
    letters do not pile up along that direction. white_only: leave yellow out, the yellow cones standing
    on the road at the section start would count as paint."""
    h, w = bgr.shape[:2]
    sx, sy = cam.width / float(w), cam.height_px / float(h)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, (0, 0, int(p.white_v_min)), (180, int(p.white_s_max), 255))
    yellow = cv2.inRange(hsv, tuple(int(v) for v in p.yellow_lower),
                         tuple(int(v) for v in p.yellow_upper))
    top = int(h * p.roi_top)
    white[:top] = 0
    yellow[:top] = 0
    if white_only:
        yellow[:] = 0
    groups = (('white', _floor_points(white, cam, p, sx, sy)),
              ('yellow', _floor_points(yellow, cam, p, sx, sy)))
    pts = np.vstack([g[1] for g in groups])
    if len(pts) < p.min_total:
        return None
    edges = np.linspace(-1.5, 1.5, 201)
    best_score, best_angle = -1.0, centre
    for a in centre + np.radians(np.arange(-math.degrees(half_range),
                                           math.degrees(half_range) + 0.5, 1.0)):
        y0 = pts[:, 1] - math.tan(a) * pts[:, 0]
        hist = np.histogram(y0, bins=edges)[0].astype(float)
        score = float((hist ** 2).sum())
        if score > best_score:
            best_score, best_angle = score, float(a)
    lines: List[LaneLine] = []
    tan_a = math.tan(best_angle)
    for colour, q in groups:
        if len(q) < p.min_points:
            continue
        y0 = q[:, 1] - tan_a * q[:, 0]
        hist = np.histogram(y0, bins=edges)[0]
        for i in np.argsort(hist)[::-1][:p.max_lines]:
            if hist[i] < p.min_points:
                break
            c = float(edges[i] + edges[i + 1]) / 2.0
            if any(abs(c - ln.y0) < 0.08 for ln in lines):
                continue
            on = np.abs(y0 - c) < 0.025
            extent = float(np.ptp(q[on, 0]))
            if extent >= p.min_extent:
                lines.append(LaneLine(y0=c, colour=colour, n=int(on.sum()), extent=extent))
    return LaneObservation(angle=best_angle, lines=lines)
