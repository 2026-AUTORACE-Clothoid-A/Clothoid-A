"""Lane detection ported from 2025 `lane_detection/lane_main.py`.

ROS-free so it can be unit-tested and tuned offline.

Pipeline (unchanged from 2025): blur -> HSV mask OR Canny -> perspective warp ->
histogram for the start x of each lane -> sliding windows -> target point.
"""
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

SIDE_LEFT = 0
SIDE_RIGHT = 1
SIDE_BOTH = 2


@dataclass
class LaneParams:
    # input image size
    width: int = 640
    height: int = 480
    # warped image size
    warp_w: int = 320
    warp_h: int = 240
    # warp source corners (2025 values, camera dependent -> retune on LIMO)
    x_h: int = 70
    x_l: int = 550
    y_h: int = 70
    y_l: int = 40
    # sliding window
    nwindows: int = 20
    margin: int = 20
    # 2025 counted pixels on a 3-channel image (3x), so its minpix=15 means 5 real pixels
    min_pixels: int = 5
    # HSV lane mask (2025 yellow lane values)
    hsv_lower: tuple = (20, 215, 0)
    hsv_upper: tuple = (40, 255, 255)
    canny_low: int = 50
    canny_high: int = 100


@dataclass
class LaneResult:
    found: bool
    # target point in the warped image
    target_x: float
    target_y: float
    debug_image: Optional[np.ndarray] = field(default=None, repr=False)
    # (x, y) of the windows that really held lane pixels, in the warped image (for visualization)
    left_points: list = field(default_factory=list, repr=False)
    right_points: list = field(default_factory=list, repr=False)
    # track2026 only: inside the school zone zigzag (mission 6, slow guide lines)
    zigzag: bool = False
    # track2026 only: boundary the target was built from, 'left', 'right', 'both' or '' (lost)
    reference: str = ''
    # Broad yellow paint inside the driving corridor cannot certify a clean bump approach.
    broad_yellow: bool = False


def make_warp_matrix(params: LaneParams) -> np.ndarray:
    """Perspective matrix from the camera image to the bird's-eye view (shared with stopline)."""
    w, h = params.width, params.height
    src = np.array([
        [params.x_h, h // 2 + params.y_h],   # top left
        [-params.x_l, h - params.y_l],       # bottom left
        [w - params.x_h, h // 2 + params.y_h],  # top right
        [w + params.x_l, h - params.y_l],    # bottom right
    ], dtype=np.float32)
    dst = np.array([
        [0, 0],
        [0, params.warp_h],
        [params.warp_w, 0],
        [params.warp_w, params.warp_h],
    ], dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(src, dst)
    # OpenCV does not raise for collapsed corners, it returns a singular or NaN matrix
    if not np.all(np.isfinite(matrix)) or abs(np.linalg.det(matrix)) < 1e-12:
        raise ValueError('degenerate warp parameters (corners collapse)')
    return matrix


class LaneDetector:
    def __init__(self, params: LaneParams):
        self.configure(params)

    def configure(self, params: LaneParams) -> None:
        # compute first so a bad parameter set leaves the previous configuration intact
        self._matrix = make_warp_matrix(params)
        self.p = params

    def detect(self, bgr: np.ndarray, side: int, lane_width: int,
               debug: bool = False) -> LaneResult:
        p = self.p
        blur = cv2.GaussianBlur(bgr, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        hsv_mask = cv2.inRange(hsv, np.array(p.hsv_lower), np.array(p.hsv_upper))
        canny = cv2.Canny(blur, p.canny_low, p.canny_high)
        merged = cv2.bitwise_or(canny, hsv_mask)
        warped = cv2.warpPerspective(merged, self._matrix, (p.warp_w, p.warp_h),
                                     flags=cv2.INTER_LINEAR)
        return self._sliding_window(warped, side, lane_width, debug)

    def _sliding_window(self, warped: np.ndarray, side: int, lane_width: int,
                        debug: bool) -> LaneResult:
        p = self.p
        h, w = warped.shape[:2]

        histogram = np.sum(warped[h // 2:, :], axis=0)
        midpoint = histogram.shape[0] // 2
        leftx_current = int(np.argmax(histogram[:midpoint]))
        rightx_current = int(np.argmax(histogram[midpoint:]) + midpoint)

        nonzeroy, nonzerox = warped.nonzero()
        window_height = h // p.nwindows

        canvas = None
        if debug:
            canvas = np.dstack((warped, warped, warped))

        lx, ly, rx, ry = [], [], [], []
        lefty = None
        righty = None
        left_points, right_points = [], []

        for window in range(p.nwindows):
            win_yl = h - (window + 1) * window_height
            win_yh = h - window * window_height
            win_xll = leftx_current - p.margin
            win_xlh = leftx_current + p.margin
            win_xrl = rightx_current - p.margin
            win_xrh = rightx_current + p.margin

            in_rows = (nonzeroy >= win_yl) & (nonzeroy < win_yh)
            good_left = (in_rows & (nonzerox >= win_xll) & (nonzerox < win_xlh)).nonzero()[0]
            good_right = (in_rows & (nonzerox >= win_xrl) & (nonzerox < win_xrh)).nonzero()[0]

            if debug:
                if side in (SIDE_LEFT, SIDE_BOTH):
                    cv2.rectangle(canvas, (win_xll, win_yl), (win_xlh, win_yh), (0, 255, 0), 1)
                if side in (SIDE_RIGHT, SIDE_BOTH):
                    cv2.rectangle(canvas, (win_xrl, win_yl), (win_xrh, win_yh), (0, 255, 0), 1)

            if len(good_left) > p.min_pixels:
                leftx_current = int(np.mean(nonzerox[good_left]))
                lefty = int(np.mean(nonzeroy[good_left]))
                left_points.append((leftx_current, lefty))
            if len(good_right) > p.min_pixels:
                rightx_current = int(np.mean(nonzerox[good_right]))
                righty = int(np.mean(nonzeroy[good_right]))
                right_points.append((rightx_current, righty))

            lx.append(leftx_current)
            ly.append((win_yl + win_yh) / 2)
            rx.append(rightx_current)
            ry.append((win_yl + win_yh) / 2)

        if side == SIDE_LEFT:
            avex = int(np.array(lx).mean() + lane_width // 2)
            avey = int(np.array(ly).mean())
        elif side == SIDE_RIGHT:
            avex = int(np.array(rx).mean() - lane_width // 2)
            avey = int(np.array(ry).mean())
        else:
            avex = int(np.array(lx).mean() + np.array(rx).mean()) // 2
            avey = int(np.array(ly).mean() + np.array(ry).mean()) // 2

        # 2025 behaviour: both windows must have found pixels at least once,
        # even when only one lane is being followed.
        found = lefty is not None and righty is not None
        if not found:
            avex = w // 2
            avey = h // 2

        if debug:
            cv2.circle(canvas, (avex, avey), 5, (0, 255, 255), -1)

        return LaneResult(found=found, target_x=float(avex), target_y=float(avey),
                          debug_image=canvas, left_points=left_points,
                          right_points=right_points)
