"""Floor-mark slow zone detection.

ROS-free. The detector only says "a slow-zone mark is in view"; the node turns that into a
`/speed_limit` value and `lane_main` applies it with min(base_speed, limit).

method `diamond` (default, 2026 track): the slow-zone mark is a white diamond outline with
"천천히" next to it, in all three slow zones of the track drawing. A white blob counts as the
diamond when its outline simplifies to 4 corners, it is convex, it fills about half of its
bounding box (a rhombus fills exactly half, bars and stop lines fill most of it) and it has a hole
(the outline is hollow, crosswalk bars and stop lines are solid). The letters are not needed.

method `color` (2025 `red_road.cpp`): a large blob of the `hsv_ranges` colour in the ROI at the
bottom of the image. Dropped from 2025: the blue "speed up" colour and the steering pass-through.
"""
from dataclasses import dataclass, field
from typing import List, Optional

import cv2
import numpy as np

METHODS = ('diamond', 'color')


@dataclass
class SpeedZoneParams:
    method: str = 'diamond'
    # --- method `diamond`, measured on the 2026-11-05 bags 02 and 03 (camera image) ---
    # white = S <= white_s_max and V >= white_v_min (floor S 0~35, V 35~65)
    white_s_max: int = 60
    white_v_min: int = 170
    # only the image below this fraction of its height is searched (road ahead, no wall)
    diamond_y0: float = 0.40
    diamond_min_area: float = 400.0   # px^2: the diamond is ~440 px^2 when it first gets this far
    # area / bounding-box area: diamonds 0.47~0.52; crosswalk bars, stop lines 0.75+
    diamond_fill_min: float = 0.44
    diamond_fill_max: float = 0.56
    # hole area / outer area: diamonds 0.37~0.57; solid marks 0~0.05, letters mostly < 0.3
    diamond_hole_min: float = 0.33
    # approxPolyDP tolerance as a fraction of the outline length
    diamond_approx: float = 0.04
    # --- method `color` (2025) ---
    # ROI as fractions of the image (2025: x 1/4..3/4, y 3/4..1)
    roi_x0: float = 0.25
    roi_x1: float = 0.75
    roi_y0: float = 0.75
    roi_y1: float = 1.0
    # flat list of HSV ranges, 6 numbers each (h_lo, s_lo, v_lo, h_hi, s_hi, v_hi), OR-ed.
    # 2025 red values (red wraps around hue 0). The 2026 floor mark colour is not known yet.
    hsv_ranges: tuple = (0, 100, 100, 10, 255, 255, 170, 100, 0, 180, 255, 255)
    min_area: float = 1800.0   # px^2 at 640x480, 2025 value
    # --- both ---
    slow_speed: float = 0.2    # rules: 0.2 m/s or less in slow zones
    hold_sec: float = 1.0      # keep slowing after the mark is gone (retune)


@dataclass
class SpeedZoneResult:
    found: bool
    area: float
    debug_image: Optional[np.ndarray] = field(default=None, repr=False)
    # method `diamond`: outlines (camera image coordinates) of the diamonds found
    diamonds: List[np.ndarray] = field(default_factory=list, repr=False)


class SpeedZoneDetector:
    def __init__(self, params: SpeedZoneParams):
        self.configure(params)

    def configure(self, params: SpeedZoneParams) -> None:
        if params.method not in METHODS:
            raise ValueError(f'method must be one of {METHODS}, got {params.method!r}')
        ranges = list(params.hsv_ranges)
        if not ranges or len(ranges) % 6 != 0:
            raise ValueError('hsv_ranges must hold groups of 6 numbers')
        if not (0.0 <= params.roi_x0 < params.roi_x1 <= 1.0
                and 0.0 <= params.roi_y0 < params.roi_y1 <= 1.0):
            raise ValueError('invalid ROI fractions')
        if not 0.0 <= params.diamond_y0 < 1.0:
            raise ValueError('diamond_y0 must be in [0, 1)')
        if not 0.0 <= params.diamond_fill_min < params.diamond_fill_max <= 1.0:
            raise ValueError('diamond_fill_min must be below diamond_fill_max, both in [0, 1]')
        self._ranges = [(np.array(ranges[i:i + 3]), np.array(ranges[i + 3:i + 6]))
                        for i in range(0, len(ranges), 6)]
        self.p = params

    def detect(self, bgr: np.ndarray, debug: bool = False) -> SpeedZoneResult:
        if self.p.method == 'diamond':
            return self._detect_diamond(bgr, debug)
        return self._detect_color(bgr, debug)

    def _detect_diamond(self, bgr: np.ndarray, debug: bool) -> SpeedZoneResult:
        p = self.p
        h = bgr.shape[0]
        y0 = int(h * p.diamond_y0)
        hsv = cv2.cvtColor(cv2.GaussianBlur(bgr[y0:], (5, 5), 0), cv2.COLOR_BGR2HSV)
        white = cv2.inRange(hsv, np.array((0, 0, p.white_v_min)),
                            np.array((180, p.white_s_max, 255)))
        # RETR_CCOMP: outer outlines and the outlines of their holes, one level each
        contours, hierarchy = cv2.findContours(white, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        diamonds = []
        best = 0.0
        if hierarchy is not None:
            nxt, child, parent = 0, 2, 3
            for i, outline in enumerate(contours):
                link = hierarchy[0][i]
                if link[parent] != -1 or link[child] == -1:
                    continue  # a hole itself, or a solid blob
                area = cv2.contourArea(outline)
                if area < p.diamond_min_area:
                    continue
                corners = cv2.approxPolyDP(
                    outline, p.diamond_approx * cv2.arcLength(outline, True), True)
                if len(corners) != 4 or not cv2.isContourConvex(corners):
                    continue
                _, _, bw, bh = cv2.boundingRect(outline)
                if not p.diamond_fill_min <= area / float(bw * bh) <= p.diamond_fill_max:
                    continue
                hole, k = 0.0, link[child]
                while k != -1:
                    hole = max(hole, cv2.contourArea(contours[k]))
                    k = hierarchy[0][k][nxt]
                if hole / area < p.diamond_hole_min:
                    continue
                diamonds.append(outline + np.array([[0, y0]]))
                best = max(best, area)

        canvas = None
        if debug:
            canvas = bgr.copy()
            cv2.line(canvas, (0, y0), (canvas.shape[1] - 1, y0), (255, 0, 0), 1)
            cv2.drawContours(canvas, diamonds, -1, (0, 255, 0), 3)
            cv2.putText(canvas, 'SLOW' if diamonds else 'no mark', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0) if diamonds else (255, 0, 0), 2)
        return SpeedZoneResult(found=bool(diamonds), area=best, debug_image=canvas,
                               diamonds=diamonds)

    def _detect_color(self, bgr: np.ndarray, debug: bool) -> SpeedZoneResult:
        p = self.p
        h, w = bgr.shape[:2]
        x0, x1 = int(w * p.roi_x0), int(w * p.roi_x1)
        y0, y1 = int(h * p.roi_y0), int(h * p.roi_y1)
        roi = cv2.GaussianBlur(bgr[y0:y1, x0:x1], (5, 5), 0)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in self._ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areas = [cv2.contourArea(c) for c in contours]
        best = max(areas) if areas else 0.0
        found = best > p.min_area

        canvas = None
        if debug:
            canvas = bgr.copy()
            color = (0, 255, 0) if found else (255, 0, 0)
            cv2.rectangle(canvas, (x0, y0), (x1, y1), color, 2)
            for c in contours:
                if cv2.contourArea(c) > p.min_area:
                    cv2.drawContours(canvas, [c + np.array([[x0, y0]])], -1, (0, 255, 0), 3)
        return SpeedZoneResult(found=found, area=best, debug_image=canvas)


class SpeedZoneHold:
    """Stays active for `hold_sec` after the last detection (2025 used a 0.1~0.2 s tail)."""

    def __init__(self, hold_sec: float):
        self.hold_sec = hold_sec
        self._last_seen: Optional[float] = None

    def update(self, found: bool, now: float) -> bool:
        if found:
            self._last_seen = now
            return True
        return self._last_seen is not None and (now - self._last_seen) <= self.hold_sec
