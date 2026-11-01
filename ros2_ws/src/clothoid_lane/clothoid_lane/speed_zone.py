"""Floor-mark slow zone detection, ported from 2025 `red_road.cpp`.

ROS-free. 2025 detected red/blue road colour in a ROI at the bottom of the image and
overrode the speed. Here the detector only says "a mark is under the car"; the node turns
that into a `/speed_limit` value and `lane_main` applies it with min(base_speed, limit).
Dropped from 2025: the blue "speed up" colour (2026 has no fast zone) and the steering
pass-through.
"""
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np


@dataclass
class SpeedZoneParams:
    # ROI as fractions of the image (2025: x 1/4..3/4, y 3/4..1)
    roi_x0: float = 0.25
    roi_x1: float = 0.75
    roi_y0: float = 0.75
    roi_y1: float = 1.0
    # flat list of HSV ranges, 6 numbers each (h_lo, s_lo, v_lo, h_hi, s_hi, v_hi), OR-ed.
    # 2025 red values (red wraps around hue 0). The 2026 floor mark colour is not known yet.
    hsv_ranges: tuple = (0, 100, 100, 10, 255, 255, 170, 100, 0, 180, 255, 255)
    min_area: float = 1800.0   # px^2 at 640x480, 2025 value
    slow_speed: float = 0.2    # rules: 0.2 m/s or less in slow zones
    hold_sec: float = 1.0      # keep slowing after the mark is gone (retune)


@dataclass
class SpeedZoneResult:
    found: bool
    area: float
    debug_image: Optional[np.ndarray] = field(default=None, repr=False)


class SpeedZoneDetector:
    def __init__(self, params: SpeedZoneParams):
        self.configure(params)

    def configure(self, params: SpeedZoneParams) -> None:
        ranges = list(params.hsv_ranges)
        if not ranges or len(ranges) % 6 != 0:
            raise ValueError('hsv_ranges must hold groups of 6 numbers')
        if not (0.0 <= params.roi_x0 < params.roi_x1 <= 1.0
                and 0.0 <= params.roi_y0 < params.roi_y1 <= 1.0):
            raise ValueError('invalid ROI fractions')
        self._ranges = [(np.array(ranges[i:i + 3]), np.array(ranges[i + 3:i + 6]))
                        for i in range(0, len(ranges), 6)]
        self.p = params

    def detect(self, bgr: np.ndarray, debug: bool = False) -> SpeedZoneResult:
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
