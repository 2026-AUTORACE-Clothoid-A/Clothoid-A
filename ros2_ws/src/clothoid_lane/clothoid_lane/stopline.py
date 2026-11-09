"""Stop line / crosswalk stripe detection, ported from 2025 `crosswalk.py`.

ROS-free. 2025 stopped the car itself for 5 s (publishing speed 0 on a mux input). Here the
detector only reports that a stop line is ahead; the mission node decides to stop, wait and go.
"""
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

from clothoid_lane.lane_detector import LaneParams, make_warp_matrix


@dataclass
class StopLineParams:
    # warp (same geometry as the lane detector; retune together)
    warp: LaneParams = field(default_factory=LaneParams)
    # crop applied to the warped image (2025: rows from 80, columns 40 px in from both sides)
    crop_x: int = 40
    crop_y: int = 80
    # white mask (2025 values, retune on the LIMO)
    hsv_lower: tuple = (0, 0, 150)
    hsv_upper: tuple = (180, 60, 255)
    gray_low: int = 130
    gray_high: int = 255
    # stripe shape: tall (h > w, w > min_tall_w) or wide (w > h, w > min_wide_w)
    min_tall_w: int = 5
    min_wide_w: int = 80
    min_count: int = 4            # stripes needed
    min_white_ratio: float = 10.0  # % of the cropped area


@dataclass
class StopLineResult:
    found: bool
    count: int
    white_ratio: float
    debug_image: Optional[np.ndarray] = field(default=None, repr=False)


class StopLineDetector:
    def __init__(self, params: StopLineParams):
        self.configure(params)

    def configure(self, params: StopLineParams) -> None:
        self._matrix = make_warp_matrix(params.warp)
        self.p = params

    def detect(self, bgr: np.ndarray, debug: bool = False) -> StopLineResult:
        p = self.p
        wp = p.warp
        warped = cv2.warpPerspective(bgr, self._matrix, (wp.warp_w, wp.warp_h),
                                     flags=cv2.INTER_LINEAR)
        warped = warped[p.crop_y:, p.crop_x:wp.warp_w - p.crop_x]

        blur = cv2.GaussianBlur(warped, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(hsv, np.array(p.hsv_lower), np.array(p.hsv_upper))
        gray = cv2.GaussianBlur(cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        _, bright = cv2.threshold(gray, p.gray_low, p.gray_high, cv2.THRESH_BINARY)
        result = cv2.bitwise_and(white, bright)

        contours, _ = cv2.findContours(result, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        count = 0
        canvas = warped.copy() if debug else None
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if (h > w and w > p.min_tall_w) or (w > h and w > p.min_wide_w):
                count += 1
                if canvas is not None:
                    cv2.rectangle(canvas, (x, y), (x + w, y + h), (0, 255, 0), 2)

        white_ratio = float(np.count_nonzero(result)) / result.size * 100.0
        found = count >= p.min_count and white_ratio > p.min_white_ratio
        return StopLineResult(found=found, count=count, white_ratio=white_ratio,
                              debug_image=canvas)



@dataclass
class NearLineParams:
    """One stop line across the lane at the bottom of the warped image (parking trigger).

    Defaults are for the limo2 warp (320 x 240, rows 200..239 = 0.52..0.61 m ahead of base_link,
    120 px = 0.35 m across). A row counts when one unbroken white run covers `min_run` of the
    band width (crosswalk blocks have gaps); the line needs `min_rows` such rows; a stop line with
    crosswalk blocks beyond it (rows `beyond_rows`, white share over `max_beyond`) is not one.
    """
    warp: LaneParams = field(default_factory=LaneParams)
    band_rows: int = 40
    half_width: int = 60
    min_run: float = 0.7
    min_rows: int = 3
    beyond_top: int = 60
    beyond_bottom: int = 190
    max_beyond: float = 0.10
    hsv_lower: tuple = (0, 0, 150)
    hsv_upper: tuple = (180, 60, 255)
    gray_low: int = 130


def longest_runs(mask: np.ndarray) -> np.ndarray:
    """Longest unbroken run of True in every row (numpy: count since the last False)."""
    m = mask.astype(np.int32)
    count = np.cumsum(m, axis=1)
    run = count - np.maximum.accumulate(np.where(m == 0, count, 0), axis=1)
    return run.max(axis=1) if run.shape[1] else np.zeros(mask.shape[0], dtype=np.int32)


class NearLineDetector:
    def __init__(self, params: NearLineParams):
        self.configure(params)

    def configure(self, params: NearLineParams) -> None:
        self._matrix = make_warp_matrix(params.warp)
        self.p = params

    def detect(self, bgr: np.ndarray) -> bool:
        p = self.p
        warped = cv2.warpPerspective(bgr, self._matrix, (p.warp.warp_w, p.warp.warp_h),
                                     flags=cv2.INTER_LINEAR)
        return self.detect_warped(warped)

    def detect_warped(self, warped: np.ndarray) -> bool:
        p = self.p
        hsv = cv2.cvtColor(cv2.GaussianBlur(warped, (5, 5), 0), cv2.COLOR_BGR2HSV)
        white = cv2.inRange(hsv, np.array(p.hsv_lower), np.array(p.hsv_upper)) > 0
        gray = cv2.GaussianBlur(cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY), (5, 5), 0) > p.gray_low
        mask = white & gray
        h, w = mask.shape
        c0, c1 = w // 2 - p.half_width, w // 2 + p.half_width
        if mask[p.beyond_top:p.beyond_bottom, c0:c1].mean() >= p.max_beyond:
            return False      # crosswalk (or other markings) right past the line
        band = mask[h - p.band_rows:, c0:c1]
        need = p.min_run * band.shape[1]
        rows = int((longest_runs(band) >= need).sum())   # rows with one long enough white run
        return rows >= p.min_rows
