"""Share of red/orange pixels in a camera frame (cone course check). No ROS.

The frame is shrunk first (cheap on the Jetson), optionally cropped to a band of rows, and the
share of pixels inside any HSV range is returned (0..1). HSV ranges use the speed_zone layout:
a flat list, 6 numbers per range (h_lo, s_lo, v_lo, h_hi, s_hi, v_hi), OR-ed, OpenCV H 0..180.
"""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class RedRatioParams:
    hsv_ranges: tuple = (0, 100, 100, 10, 255, 255, 170, 100, 0, 180, 255, 255)  # 2025 speed_zone red
    width: int = 160            # px after shrinking (height keeps the aspect ratio)
    roi_top: float = 0.0        # fraction of the height, rows above are ignored
    roi_bottom: float = 1.0     # fraction of the height, rows below are ignored


def hsv_groups(hsv_ranges):
    r = [int(v) for v in hsv_ranges]
    if len(r) == 0 or len(r) % 6:
        raise ValueError('hsv_ranges must hold groups of 6 numbers')
    return [(np.array(r[i:i + 3], np.uint8), np.array(r[i + 3:i + 6], np.uint8))
            for i in range(0, len(r), 6)]


def red_mask(bgr: np.ndarray, p: RedRatioParams) -> np.ndarray:
    h, w = bgr.shape[:2]
    if w > p.width:
        bgr = cv2.resize(bgr, (p.width, max(1, round(h * p.width / w))), interpolation=cv2.INTER_AREA)
    h = bgr.shape[0]
    top, bottom = int(round(h * p.roi_top)), int(round(h * p.roi_bottom))
    hsv = cv2.cvtColor(bgr[top:max(bottom, top + 1)], cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in hsv_groups(p.hsv_ranges):
        mask |= cv2.inRange(hsv, lo, hi)
    return mask


def red_ratio(bgr: np.ndarray, p: RedRatioParams) -> float:
    mask = red_mask(bgr, p)
    return float(np.count_nonzero(mask)) / max(1, mask.size)
