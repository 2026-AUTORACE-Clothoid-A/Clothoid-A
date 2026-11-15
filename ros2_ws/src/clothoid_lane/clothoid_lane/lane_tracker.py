"""Lane detection for the 2026 track: yellow outer lines, white lane dividers between them.

ROS-free. Same input, output and warp as `lane_detector.LaneDetector` (the 2025 port, kept as
`detector: sliding2025`); lane_main picks one with its `detector` parameter.

Why a second detector (2026-11-05 bags, 02/03_slow_all):
  * 2025 merged an HSV mask with Canny edges. The 2025 HSV range (S >= 215) matches none of the yellow
    tape (S 70~180), so it ran on Canny alone and took the edges of every white floor marking
    ("천천히" letters, diamonds, crosswalks, stop lines) and of the wall as lane lines.
  * In right curves the right line leaves the view, the right window took the left line or a
    crosswalk bar and the target landed a lane width left of it: 1~5 % of the right curve frames
    steered right.

Pipeline:
  1. Warp the colour image to the bird's-eye view, then mask yellow (HSV) and white (low S, high V).
  2. Yellow lines: sliding windows from histogram peaks and from last frame's boundaries.
  3. White components are kept only if they look like a lane divider: thin, long, parallel to the
     yellow lines, on the road side of every yellow line and about one lane width from the nearest
     one. With no yellow line in view (obstacle section) the shape alone decides.
  4. Lines from yellow and kept white pixels; the boundaries of our lane are the nearest lines left
     and right of the car, tracked from frame to frame.
  5. Target: a point half a lane width from the preferred boundary, measured perpendicular to the
     line (a horizontal offset is too short on a slanted line), or the middle of both for SIDE_BOTH.
     If the preferred side is missing the other one is used.
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional

import cv2
import numpy as np

from clothoid_lane.lane_detector import (SIDE_BOTH, SIDE_LEFT, SIDE_RIGHT, LaneParams, LaneResult,
                                         make_warp_matrix)

YELLOW, WHITE = 'yellow', 'white'


@dataclass
class TrackerParams:
    white_s_max: int = 50             # white = S <= this and V >= white_v_min (floor V is 35~65)
    white_v_min: int = 170
    white_min_length: float = 40.0    # px, a divider dash is >= 40 px in the warped image
    white_max_width: float = 14.0     # px, dividers 8 px; crosswalk bars 25+, diamond 38+
    white_max_angle_deg: float = 25.0  # from the nearest yellow line (stop lines are ~90)
    white_offset_tol: float = 0.3     # divider at (1 +- tol) lane widths from the nearest yellow
    min_line_windows: int = 4         # windows with pixels for a line to count
    max_lines: int = 4                # histogram peaks tried per colour
    look_row: int = 120               # warped row of the target (2025 used the window mean, 120)
    track_gate: float = 40.0          # px, a boundary matches last frame's within this at the bottom
    track_hold: int = 5               # frames a lost boundary is still used as a search seed
    min_gap_ratio: float = 0.6        # left/right boundaries closer than this x lane width = same line
    max_gap_ratio: float = 1.5
    zigzag_min_amp: float = 4.0       # px, half swing of the window centres around the fit
    zigzag_min_flips: int = 3         # sign changes of that swing along the line
    zigzag_hold: int = 15             # frames the zone lasts with no boundary in view
    zigzag_width_ratio: float = 1.07  # zone lane width / normal: bags 182/170 px, drawing 570/520 mm


@dataclass(eq=False)   # compared by identity: the generated __eq__ would compare numpy arrays
class Line:
    points: np.ndarray                # (N, 2) window centres with pixels, warped x, y
    coeff: np.ndarray                 # x = polyval(coeff, y)
    color: str
    zigzag: bool = False
    bottom_x: float = 0.0
    # every sliding window tried, (x_lo, y_top, x_hi, y_bottom, had pixels), for the debug image
    windows: list = field(default_factory=list, repr=False)

    def straighten(self) -> None:
        """Replace the fit by a straight line (the centre line of a zigzag)."""
        self.coeff = np.polyfit(self.points[:, 1], self.points[:, 0], 1)

    def x_at(self, y: float) -> float:
        return float(np.polyval(self.coeff, y))

    def slope_at(self, y: float) -> float:
        return float(np.polyval(np.polyder(self.coeff), y)) if len(self.coeff) > 1 else 0.0


@dataclass
class _Side:
    bottom_x: Optional[float] = None
    missed: int = 0
    line: Optional[Line] = None


@dataclass
class _Components:
    kept: np.ndarray                  # mask of the white pixels accepted as dividers
    rejected: np.ndarray = field(default=None, repr=False)


class LaneTracker:
    def __init__(self, params: LaneParams, tracker: Optional[TrackerParams] = None):
        self.configure(params, tracker)

    def configure(self, params: LaneParams, tracker: Optional[TrackerParams] = None) -> None:
        matrix = make_warp_matrix(params)   # raises first, so a bad set keeps the old one
        self._matrix, self.p = matrix, params
        self.t = tracker or getattr(self, 't', None) or TrackerParams()
        size = (params.warp_w, params.warp_h)
        ones = np.full((params.height, params.width), 255, np.uint8)
        # pixels of the warped image that come from inside the camera image
        self._valid = cv2.warpPerspective(ones, matrix, size, flags=cv2.INTER_NEAREST) > 0
        self.reset()

    def reset(self) -> None:
        self._sides = {SIDE_LEFT: _Side(), SIDE_RIGHT: _Side()}
        self._continuity = False
        self._clear_reference()
        self._zigzag_left = 0     # frames the school zone state still lasts
        self._attempts = []       # (windows, became a line) of every trace this frame, debug image

    def set_boundary_continuity(self, enabled: bool) -> None:
        """Start exit reacquisition without inheriting lines seen inside the cone course."""
        if enabled and not self._continuity:
            self._sides = {SIDE_LEFT: _Side(), SIDE_RIGHT: _Side()}
            self._clear_reference()
        self._continuity = enabled

    def _clear_reference(self):
        self._reference = self._pending_reference = ''
        self._reference_target = self._pending_target = None
        self._reference_misses = self._pending_hits = 0

    def _confirmed_target(self, target, reference):
        if target is None:
            self._reference_misses += 1
            self._pending_reference, self._pending_target, self._pending_hits = '', None, 0
            if self._reference_misses > self.t.track_hold:
                self._clear_reference()
            return False
        self._reference_misses = 0
        if (self._reference_target is not None and reference != self._reference
                and math.dist(target, self._reference_target) > self.t.track_gate):
            if (reference != self._pending_reference or self._pending_target is None
                    or math.dist(target, self._pending_target) > self.t.track_gate):
                self._pending_hits = 0
            self._pending_reference, self._pending_target = reference, target
            self._pending_hits += 1
            if self._pending_hits < 3:
                return False
        self._reference, self._reference_target = reference, target
        self._pending_reference, self._pending_target, self._pending_hits = '', None, 0
        return True

    def _continuous_pick(self, lines, lane_width, w):
        # Match on common *observed* rows. A bottom extrapolation of a short,
        # slanted line can cross the car centre even while it is the same boundary.
        # A distant short fragment must not acquire control through extrapolation.
        lines = [line for line in lines if line.points[:, 1].min() <= self.t.look_row
                 <= line.points[:, 1].max()]
        matches = []
        for side, state in self._sides.items():
            old = state.line
            if old is None:
                continue
            for i, line in enumerate(lines):
                lo = max(old.points[:, 1].min(), line.points[:, 1].min())
                hi = min(old.points[:, 1].max(), line.points[:, 1].max())
                if hi - lo < self.p.warp_h / self.p.nwindows:
                    continue
                rows = np.linspace(lo, hi, 5)
                distance = float(np.median(abs(np.polyval(old.coeff, rows)
                                              - np.polyval(line.coeff, rows))))
                if distance <= self.t.track_gate:
                    matches.append((distance + (0 if line.color == old.color else self.p.margin/4),
                                    side, i))
        chosen, used = {}, set()
        for _, side, i in sorted(matches):
            if side not in chosen and i not in used:
                chosen[side] = lines[i]
                used.add(i)
        # Reserve matched boundaries before acquiring another side. Do not give
        # the same line two labels or immediately replace a temporarily lost side.
        for side in (SIDE_LEFT, SIDE_RIGHT):
            if side in chosen or self._sides[side].line is not None:
                continue
            candidates = [(abs(line.bottom_x-w/2), i) for i, line in enumerate(lines)
                          if i not in used and (line.bottom_x < w/2 if side == SIDE_LEFT
                                                else line.bottom_x > w/2)]
            if candidates:
                _, i = min(candidates)
                chosen[side] = lines[i]
                used.add(i)
        left, right = chosen.get(SIDE_LEFT), chosen.get(SIDE_RIGHT)
        if left is not None and right is not None:
            # Check actual common support, not the unobserved image bottom.
            lo = max(left.points[:, 1].min(), right.points[:, 1].min())
            hi = min(left.points[:, 1].max(), right.points[:, 1].max())
            row = float(np.clip(self.t.look_row, lo, hi))
            gap = right.x_at(row)-left.x_at(row)
            if lo > hi or not self.t.min_gap_ratio*lane_width <= gap <= self.t.max_gap_ratio*lane_width:
                # Prefer a continued observation over a newly acquired boundary.
                if self._sides[SIDE_RIGHT].line is not None and self._sides[SIDE_LEFT].line is None:
                    left = None
                else:
                    right = None
        self._remember_sides(left, right)
        return left, right

    def _remember_sides(self, left, right):
        for side, line in ((SIDE_LEFT, left), (SIDE_RIGHT, right)):
            state = self._sides[side]
            if line is not None:
                state.bottom_x, state.line, state.missed = line.bottom_x, line, 0
            else:
                state.missed += 1
                if state.missed > self.t.track_hold:
                    state.bottom_x = state.line = None

    # --- masks ------------------------------------------------------------------
    def _masks(self, bgr: np.ndarray):
        p, t = self.p, self.t
        warped = cv2.warpPerspective(cv2.GaussianBlur(bgr, (5, 5), 0), self._matrix,
                                     (p.warp_w, p.warp_h), flags=cv2.INTER_LINEAR)
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        yellow = cv2.inRange(hsv, np.array(p.hsv_lower), np.array(p.hsv_upper)) > 0
        white = (hsv[..., 1] <= t.white_s_max) & (hsv[..., 2] >= t.white_v_min)
        return yellow & self._valid, white & self._valid & ~yellow

    # --- lines ------------------------------------------------------------------
    def _is_zigzag(self, pts: np.ndarray, coeff: np.ndarray) -> bool:
        """Window centres swing back and forth around the smooth fit, like the school zone zigzag.

        Track drawing: each zigzag line is a triangle wave, +-46 mm with a 185 mm period and 45 deg
        teeth. The window centres follow the teeth: they swing around the fit by several pixels and
        change sign every half period (about 60 px in the warped image). One line alone can also
        wobble like this in a curve, so `detect` only acts when both boundaries do.
        """
        t = self.t
        if len(pts) < 8:
            return False
        r = pts[:, 0] - np.polyval(coeff, pts[:, 1])
        amp = (np.percentile(r, 90) - np.percentile(r, 10)) / 2.0
        signs = np.sign(r[np.abs(r) >= 1.0])
        flips = int(np.sum(signs[1:] != signs[:-1]))
        return amp >= t.zigzag_min_amp and flips >= t.zigzag_min_flips

    def _trace(self, mask: np.ndarray, seed: float, color: str) -> Optional[Line]:
        p = self.p
        h, w = mask.shape
        win_h = h // p.nwindows
        x, dx = float(seed), 0.0
        points, windows = [], []
        for k in range(p.nwindows):
            y1, y0 = h - k * win_h, h - (k + 1) * win_h
            lo, hi = int(round(x - p.margin)), int(round(x + p.margin))
            if hi <= 0 or lo >= w:
                break
            ys, xs = np.nonzero(mask[y0:y1, max(lo, 0):min(hi, w)])
            windows.append((lo, y0, hi, y1, len(xs) > p.min_pixels))
            if len(xs) > p.min_pixels:
                new_x = float(xs.mean()) + max(lo, 0)
                if points:
                    dx = new_x - points[-1][0]
                points.append((new_x, (y0 + y1) / 2.0))
                x = new_x
            else:
                x += dx   # follow the curve through gaps (dashes)
        found_line = len(points) >= self.t.min_line_windows
        self._attempts.append((windows, found_line))
        if not found_line:
            return None
        pts = np.array(points)
        degree = 2 if len(pts) >= 6 else 1
        coeff = np.polyfit(pts[:, 1], pts[:, 0], degree)
        line = Line(pts, coeff, color, zigzag=self._is_zigzag(pts, coeff), windows=windows)
        line.bottom_x = line.x_at(h - 1)
        return line

    def _lines(self, mask: np.ndarray, color: str, seeds: List[float]) -> List[Line]:
        p, t = self.p, self.t
        h = mask.shape[0]
        hist = mask[h // 2:].sum(axis=0).astype(float)
        starts = list(seeds)
        for _ in range(t.max_lines):
            peak = int(np.argmax(hist))
            if hist[peak] <= p.min_pixels:
                break
            starts.append(float(peak))
            hist[max(0, peak - 2 * p.margin):peak + 2 * p.margin] = 0.0
        lines: List[Line] = []
        for seed in starts:
            line = self._trace(mask, seed, color)
            if line is None:
                continue
            # the same line traced from two seeds: keep the one with more windows
            same = [o for o in lines if np.mean(np.abs(line.points[:, 0] - np.polyval(
                o.coeff, line.points[:, 1]))) < p.margin]
            if same:
                if len(line.points) > len(same[0].points):
                    lines = [line if o is same[0] else o for o in lines]
                continue
            lines.append(line)
        return lines

    # --- white dividers ---------------------------------------------------------
    def _dividers(self, white: np.ndarray, yellow_lines: List[Line], lane_width: float) -> _Components:
        t = self.t
        h, w = white.shape
        count, labels, stats, _ = cv2.connectedComponentsWithStats(white.astype(np.uint8), 8)
        kept = np.zeros_like(white)
        rejected = np.zeros_like(white)
        if count <= 1:
            return _Components(kept, rejected)
        # per-component moments in one pass
        ys, xs = np.nonzero(labels)
        lab = labels[ys, xs]
        n = np.bincount(lab, minlength=count).astype(float)
        n[n == 0] = 1.0
        mx, my = np.bincount(lab, xs, count) / n, np.bincount(lab, ys, count) / n
        cxx = np.bincount(lab, xs * xs, count) / n - mx * mx
        cyy = np.bincount(lab, ys * ys, count) / n - my * my
        cxy = np.bincount(lab, xs * ys, count) / n - mx * my
        car_x = w / 2.0
        for k in range(1, count):
            if stats[k, cv2.CC_STAT_AREA] < 20:
                continue
            ok = self._is_divider(mx[k], my[k], cxx[k], cyy[k], cxy[k], yellow_lines,
                                  lane_width, car_x, h)
            (kept if ok else rejected)[labels == k] = True
        return _Components(kept, rejected)

    def _is_divider(self, x, y, cxx, cyy, cxy, yellow_lines, lane_width, car_x, h) -> bool:
        t = self.t
        cov = np.array([[cxx, cxy], [cxy, cyy]])
        values, vectors = np.linalg.eigh(cov)
        length = 4.0 * math.sqrt(max(values[1], 0.0))
        width = 4.0 * math.sqrt(max(values[0], 0.0))
        if length < t.white_min_length or width > t.white_max_width:
            return False
        vx, vy = vectors[:, 1]
        if vy < 0:
            vx, vy = -vx, -vy
        angle = math.atan2(vx, vy)              # 0 = along the warped image (driving direction)
        if not yellow_lines:
            return abs(angle) <= math.radians(t.white_max_angle_deg)
        nearest, nearest_dist = None, math.inf
        for line in yellow_lines:
            # on the road: the same side of every yellow line as the car
            car_side = car_x - line.bottom_x
            line_x = line.x_at(y)
            if car_side * (x - line_x) <= 0:
                return False
            slope = line.slope_at(y)
            dist = abs(x - line_x) / math.sqrt(1.0 + slope * slope)
            if dist < nearest_dist:
                nearest, nearest_dist = line, dist
        ref = math.atan(nearest.slope_at(y))
        if abs(angle - ref) > math.radians(t.white_max_angle_deg):
            return False
        return abs(nearest_dist / lane_width - 1.0) <= t.white_offset_tol

    # --- boundaries -------------------------------------------------------------
    def _pick(self, lines: List[Line], lane_width: float, w: int):
        if self._continuity:
            return self._continuous_pick(lines, lane_width, w)
        t = self.t
        car_x = w / 2.0
        chosen = {}
        for side in (SIDE_LEFT, SIDE_RIGHT):
            prev = self._sides[side].bottom_x
            line = None
            if prev is not None:
                near = [l for l in lines if abs(l.bottom_x - prev) <= t.track_gate]
                if near:
                    line = min(near, key=lambda l: abs(l.bottom_x - prev))
            if line is None:
                cands = [l for l in lines if (l.bottom_x < car_x if side == SIDE_LEFT
                                              else l.bottom_x > car_x)]
                if cands:
                    line = min(cands, key=lambda l: abs(l.bottom_x - car_x))
            chosen[side] = line
        left, right = chosen[SIDE_LEFT], chosen[SIDE_RIGHT]
        if left is not None and right is not None:
            gap = right.bottom_x - left.bottom_x
            if not t.min_gap_ratio * lane_width <= gap <= t.max_gap_ratio * lane_width:
                # one of them is wrong: keep the one nearer to where its side is expected
                exp_l = self._expected(SIDE_LEFT, car_x, lane_width)
                exp_r = self._expected(SIDE_RIGHT, car_x, lane_width)
                if abs(left.bottom_x - exp_l) <= abs(right.bottom_x - exp_r):
                    right = None
                else:
                    left = None
        self._remember_sides(left, right)
        return left, right

    def _expected(self, side: int, car_x: float, lane_width: float) -> float:
        prev = self._sides[side].bottom_x
        if prev is not None:
            return prev
        return car_x - lane_width / 2.0 if side == SIDE_LEFT else car_x + lane_width / 2.0

    def _in_zigzag_zone(self, left: Optional[Line], right: Optional[Line]) -> bool:
        """School zone state. Entered when both boundaries are zigzag (one zigzag line alone can be a
        wobbly curve), kept while every boundary in view is zigzag, so a frame with one side out of
        view is still straightened; left as soon as a plain boundary shows up."""
        seen = [line for line in (left, right) if line is not None]
        if len(seen) == 2 and all(line.zigzag for line in seen):
            self._zigzag_left = self.t.zigzag_hold
        elif self._zigzag_left > 0 and seen and all(line.zigzag for line in seen):
            self._zigzag_left = self.t.zigzag_hold
        elif any(not line.zigzag for line in seen):
            self._zigzag_left = 0
        else:
            self._zigzag_left -= 1   # nothing in view
        return self._zigzag_left > 0

    def _offset_point(self, line: Line, toward: int, lane_width: float):
        """Half a lane width from the line at look_row, perpendicular to it, toward +1 right / -1 left."""
        y = float(self.t.look_row)
        x = line.x_at(y)
        slope = line.slope_at(y)
        norm = math.sqrt(1.0 + slope * slope)
        half = lane_width / 2.0
        return x + toward * half / norm, y - toward * half * slope / norm

    # --- main -------------------------------------------------------------------
    def detect(self, bgr: np.ndarray, side: int, lane_width: int, debug: bool = False) -> LaneResult:
        p = self.p
        h, w = p.warp_h, p.warp_w
        self._attempts = []
        yellow, white = self._masks(bgr)
        seeds = [s.bottom_x for s in self._sides.values() if s.bottom_x is not None]
        yellow_lines = self._lines(yellow, YELLOW, seeds)
        comps = self._dividers(white, yellow_lines, float(lane_width))
        white_lines = self._lines(comps.kept, WHITE, seeds)
        lines = yellow_lines + [l for l in white_lines if not any(
            np.mean(np.abs(l.points[:, 0] - np.polyval(y.coeff, l.points[:, 1]))) < p.margin
            for y in yellow_lines)]
        left, right = self._pick(lines, float(lane_width), w)
        zigzag_zone = self._in_zigzag_zone(left, right)
        if zigzag_zone:
            # the drawing has the zigzag only along a straight: its centre is a straight line, a
            # curved fit would follow the teeth at the ends of the image
            for line in (left, right):
                if line is not None:
                    line.straighten()

        # the zigzag lane is wider: centre lines 570 mm apart against 520 mm elsewhere (drawing and
        # rules: 450 mm between the inner tips, 690 mm between the outer tips)
        half_lane = float(lane_width) * (self.t.zigzag_width_ratio if zigzag_zone else 1.0)
        target = None
        heading = None
        reference = ''      # which boundary the target was built from
        bases = []          # point(s) on the boundary at look_row the target is measured from
        if side == SIDE_BOTH and left is not None and right is not None:
            y = float(self.t.look_row)
            target = ((left.x_at(y) + right.x_at(y)) / 2.0, y)
            reference = 'both'
            if all(line.points[:, 1].min() <= y <= line.points[:, 1].max()
                   for line in (left, right)):
                heading = math.degrees(math.atan((left.slope_at(y)+right.slope_at(y))/2))
            bases = [(left.x_at(y), y), (right.x_at(y), y)]
        else:
            order = (right, left) if side == SIDE_RIGHT else (left, right)
            for line in order:
                if line is not None:
                    toward = +1 if line is left else -1
                    target = self._offset_point(line, toward, half_lane)
                    reference = 'left' if line is left else 'right'
                    # A fitted line beyond its observed support is not alignment evidence.
                    y = float(self.t.look_row)
                    if line.points[:, 1].min() <= y <= line.points[:, 1].max():
                        heading = math.degrees(math.atan(line.slope_at(y)))
                    bases = [(line.x_at(float(self.t.look_row)), float(self.t.look_row))]
                    break
        if self._continuity and not self._confirmed_target(target, reference):
            target, reference, heading, bases = None, '', None, []
        found = target is not None
        tx, ty = target if found else (w / 2.0, h / 2.0)

        canvas = None
        if debug:
            canvas = self._draw(yellow, comps, lines, left, right, bases, (tx, ty), found)

        def as_points(line):
            return [] if line is None else [(int(x), int(y)) for x, y in line.points]

        return LaneResult(found=found, target_x=float(tx), target_y=float(ty), debug_image=canvas,
                          left_points=as_points(left), right_points=as_points(right),
                          zigzag=zigzag_zone, reference=reference, heading_deg=heading)

    # --- debug image ------------------------------------------------------------
    LEFT_COLOUR, RIGHT_COLOUR = (255, 80, 0), (0, 0, 255)       # BGR: blue, red
    MISS_DIM = 0.45                                              # windows that found no pixels

    def _draw(self, yellow, comps, lines, left, right, bases, target, found):
        """Masks, sliding windows, the look row and how the target was built.

        yellow / white: line pixels (white = accepted divider), grey: white rejected as a divider.
        Rectangles: sliding windows of the left (blue) and right (red) boundary, thick where the
        window found pixels, thin and dark where it only carried on; dots: window centres.
        Dark magenta windows: lines that were found but not used as a boundary.
        Thin grey / orange windows: a trace that found fewer than min_line_windows windows with
        pixels (orange) and so gave no line, e.g. a line only in the top of the image.
        Grey row: look_row. White dot: the boundary at look_row; green dot: the target, half a lane
        from it (red ring when the lane is lost and nothing is published).
        """
        h, w = self.p.warp_h, self.p.warp_w
        canvas = np.zeros((h, w, 3), np.uint8)
        canvas[yellow] = (0, 200, 255)
        canvas[comps.kept] = (255, 255, 255)
        canvas[comps.rejected] = (90, 90, 90)
        row = int(self.t.look_row)
        cv2.line(canvas, (0, row), (w - 1, row), (110, 110, 110), 1)

        for wins, became_line in self._attempts:
            if not became_line:
                for lo, y0, hi, y1, hit in wins:
                    cv2.rectangle(canvas, (lo, y0), (hi, y1 - 1),
                                  (0, 140, 255) if hit else (70, 70, 70), 1)

        def windows(line, colour, hit_width):
            dim = tuple(int(c * self.MISS_DIM) for c in colour)
            for lo, y0, hi, y1, hit in line.windows:
                cv2.rectangle(canvas, (lo, y0), (hi, y1 - 1), colour if hit else dim,
                              hit_width if hit else 1)
            for x, y in line.points:
                cv2.circle(canvas, (int(x), int(y)), 2, colour, -1)
            ys = np.arange(0, h, 4, dtype=float)
            fit = np.column_stack((np.polyval(line.coeff, ys), ys)).astype(np.int32)
            cv2.polylines(canvas, [fit], False, colour, 1)

        for line in lines:
            if line is not left and line is not right:
                windows(line, (140, 0, 140), 1)
        for line, colour in ((left, self.LEFT_COLOUR), (right, self.RIGHT_COLOUR)):
            if line is not None:
                windows(line, colour, 2)

        tx, ty = int(round(target[0])), int(round(target[1]))
        if found:
            for bx, by in bases:
                cv2.line(canvas, (int(round(bx)), int(round(by))), (tx, ty), (0, 255, 0), 1)
                cv2.circle(canvas, (int(round(bx)), int(round(by))), 3, (255, 255, 255), -1)
            cv2.circle(canvas, (tx, ty), 5, (0, 255, 0), -1)
        else:
            cv2.circle(canvas, (tx, ty), 6, (0, 0, 255), 2)
        return canvas
