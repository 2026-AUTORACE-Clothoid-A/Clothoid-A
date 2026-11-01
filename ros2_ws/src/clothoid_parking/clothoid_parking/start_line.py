"""Parking start line detection, ported from 2025 `parking_rect.py` (`check_histogram`).

ROS-free. A horizontal line in the bird's-eye view marks where the manoeuvre starts. The
result is the row (y in the cropped bird's-eye image) of the line; larger = closer to the car.
The warp geometry is the same camera calibration as clothoid_lane (kept as a separate copy so
the mission packages do not import each other's algorithms: retune both together).
"""
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np


@dataclass
class StartLineParams:
    # input image / warped image size and warp corners (2025 values, retune on the LIMO)
    width: int = 640
    height: int = 480
    warp_w: int = 320
    warp_h: int = 240
    x_h: int = 70
    x_l: int = 550
    y_h: int = 70
    y_l: int = 40
    crop_x: int = 40
    crop_y: int = 80
    # line colour (2025 values) and brightness threshold
    hsv_lower: tuple = (20, 165, 50)
    hsv_upper: tuple = (60, 255, 255)
    gray_low: int = 80
    gray_high: int = 255
    # row histogram: columns [margin + shift, w - margin + shift), rows with more pixels count
    margin: int = 60
    shift: int = 45
    min_line_len: int = 20
    # the manoeuvre starts when the line is at least this far down (2025 stopline_thresh)
    trigger_row: int = 5


@dataclass
class StartLineResult:
    center_y: Optional[int]
    debug_image: Optional[np.ndarray] = field(default=None, repr=False)

    @property
    def triggered(self) -> bool:
        return self.center_y is not None


def make_warp_matrix(p: StartLineParams) -> np.ndarray:
    w, h = p.width, p.height
    src = np.array([[p.x_h, h // 2 + p.y_h], [-p.x_l, h - p.y_l],
                    [w - p.x_h, h // 2 + p.y_h], [w + p.x_l, h - p.y_l]], dtype=np.float32)
    dst = np.array([[0, 0], [0, p.warp_h], [p.warp_w, 0], [p.warp_w, p.warp_h]], dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(src, dst)
    if not np.all(np.isfinite(matrix)) or abs(np.linalg.det(matrix)) < 1e-12:
        raise ValueError('degenerate warp parameters (corners collapse)')
    return matrix


class StartLineDetector:
    def __init__(self, params: StartLineParams):
        self.configure(params)

    def configure(self, params: StartLineParams) -> None:
        self._matrix = make_warp_matrix(params)
        self.p = params

    def detect(self, bgr: np.ndarray, debug: bool = False) -> StartLineResult:
        p = self.p
        warped = cv2.warpPerspective(bgr, self._matrix, (p.warp_w, p.warp_h),
                                     flags=cv2.INTER_LINEAR)
        warped = warped[p.crop_y:, p.crop_x:p.warp_w - p.crop_x]

        blur = cv2.GaussianBlur(warped, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        colour = cv2.inRange(hsv, np.array(p.hsv_lower), np.array(p.hsv_upper))
        gray = cv2.GaussianBlur(cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        _, bright = cv2.threshold(gray, p.gray_low, p.gray_high, cv2.THRESH_BINARY)
        binary = cv2.bitwise_and(colour, bright)

        h, w = binary.shape
        histogram = np.sum(binary[:, p.margin + p.shift:w - p.margin + p.shift], axis=1) // 255
        rows = np.where(histogram > p.min_line_len)[0]
        center_y = int(np.median(rows)) if len(rows) else None

        canvas = None
        if debug:
            canvas = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
            for y, value in enumerate(histogram):
                color = (0, 255, 255) if value > p.min_line_len else (0, 0, 255)
                cv2.line(canvas, (0, y), (int(value), y), color, 1)
            if center_y is not None:
                cv2.line(canvas, (0, center_y), (w, center_y), (255, 0, 255), 2)
        return StartLineResult(center_y=center_y, debug_image=canvas)

    def should_start(self, result: StartLineResult) -> bool:
        return result.center_y is not None and result.center_y > self.p.trigger_row
