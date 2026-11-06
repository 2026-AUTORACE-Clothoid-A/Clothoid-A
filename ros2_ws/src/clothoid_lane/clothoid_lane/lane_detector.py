"""Lane detection ported from 2025 `lane_detection/lane_main.py`.

ROS-free so it can be unit-tested and tuned offline.

Pipeline: blur -> lane mask -> perspective warp -> histogram for the start x of each lane ->
sliding windows -> target point.

Changes from 2025 (1105 bag analysis):
- Lane mask: when enough yellow is in view, only the yellow mask and the Canny edges next to it
  are kept, so white marks (slow-zone text, diamonds, crosswalks) and cone outlines are dropped.
  With little yellow in view (white-lane sections) it falls back to the 2025 mask, Canny OR HSV.
- found: the followed side alone decides. If it has no pixels, the other side +- lane_width/2
  is used and the result is marked `fallback`. 2025 required both sides even when following one.
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
    # yellow pixels needed in the warped view to use the yellow-only mask (0 = always 2025 mask)
    yellow_min_pixels: int = 150
    # Canny edges within this many pixels of the yellow mask are kept in the yellow-only mask
    yellow_near_px: int = 4
    # a yellow blob thicker than this in the camera image (2 x its largest distance to an edge,
    # inside the warp band) is taken as a cone and removed with its outline (0 = keep every blob).
    # Measured in the camera image because the warp stretches far, near-horizontal lane lines.
    # 1105 bags: lane tape, corners and zigzag 10~41 px, yellow cones 52~113 px.
    yellow_max_thickness: int = 46


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
    # the followed side had no pixels and the target came from the other side
    fallback: bool = False
    # the yellow-only mask was used (False: 2025 mask, Canny OR HSV)
    yellow_mask: bool = False


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
        size = (p.warp_w, p.warp_h)

        cones = self._thick_yellow_blobs(hsv_mask)
        if cv2.countNonZero(cones):
            # drop the cone and its outline (Canny draws the edge just outside the yellow)
            hsv_mask = cv2.bitwise_and(hsv_mask, cv2.bitwise_not(cones))
            cone_zone = cv2.dilate(cones, np.ones((9, 9), np.uint8))
            canny = cv2.bitwise_and(canny, cv2.bitwise_not(cone_zone))

        yellow_in_view = cv2.countNonZero(cv2.warpPerspective(hsv_mask, self._matrix, size))
        use_yellow = p.yellow_min_pixels > 0 and yellow_in_view >= p.yellow_min_pixels
        if use_yellow:
            k = 2 * p.yellow_near_px + 1
            near_yellow = cv2.dilate(hsv_mask, np.ones((k, k), np.uint8))
            merged = cv2.bitwise_or(hsv_mask, cv2.bitwise_and(canny, near_yellow))
        else:
            merged = cv2.bitwise_or(canny, hsv_mask)
        warped = cv2.warpPerspective(merged, self._matrix, size, flags=cv2.INTER_LINEAR)
        result = self._sliding_window(warped, side, lane_width, debug)
        result.yellow_mask = use_yellow
        return result

    def _thick_yellow_blobs(self, hsv_mask: np.ndarray) -> np.ndarray:
        """Mask (camera image) of yellow blobs in the warp band too thick to be lane tape."""
        p = self.p
        out = np.zeros_like(hsv_mask)
        if p.yellow_max_thickness <= 0:
            return out
        y0 = max(0, p.height // 2 + p.y_h)
        y1 = min(hsv_mask.shape[0], p.height - p.y_l)
        band = (hsv_mask[y0:y1] > 0).astype(np.uint8)
        n, labels = cv2.connectedComponents(band)
        if n <= 1:
            return out
        dist = cv2.distanceTransform(band, cv2.DIST_L2, 5)
        # largest distance to an edge per blob = half its thickness
        half = np.zeros(n, np.float32)
        np.maximum.at(half, labels.ravel(), dist.ravel())
        thick = np.nonzero(2 * half[1:] > p.yellow_max_thickness)[0] + 1
        if thick.size:
            out[y0:y1][np.isin(labels, thick)] = 255
        return out

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
        left_rects, right_rects = [], []

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

            left_rects.append(((win_xll, win_yl), (win_xlh, win_yh)))
            right_rects.append(((win_xrl, win_yl), (win_xrh, win_yh)))

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

        has_left = lefty is not None
        has_right = righty is not None
        from_left = (int(np.array(lx).mean() + lane_width // 2), int(np.array(ly).mean()))
        from_right = (int(np.array(rx).mean() - lane_width // 2), int(np.array(ry).mean()))

        # The followed side decides; the other side is the fallback.
        # (2025 required pixels on both sides even when following only one.)
        fallback = False
        used = []
        if side == SIDE_LEFT:
            if has_left:
                target, used = from_left, ['left']
            elif has_right:
                target, used, fallback = from_right, ['right'], True
        elif side == SIDE_RIGHT:
            if has_right:
                target, used = from_right, ['right']
            elif has_left:
                target, used, fallback = from_left, ['left'], True
        else:
            if has_left and has_right:
                target = (int(np.array(lx).mean() + np.array(rx).mean()) // 2,
                          int(np.array(ly).mean() + np.array(ry).mean()) // 2)
                used = ['left', 'right']
            elif has_left:
                target, used, fallback = from_left, ['left'], True
            elif has_right:
                target, used, fallback = from_right, ['right'], True

        found = bool(used)
        avex, avey = target if found else (w // 2, h // 2)

        if debug:
            # windows of the side(s) the target came from: green, orange when it is the fallback.
            # Lost: the followed side's windows in green, as in 2025.
            if not found:
                used = {SIDE_LEFT: ['left'], SIDE_RIGHT: ['right']}.get(side, ['left', 'right'])
            color = (0, 165, 255) if fallback else (0, 255, 0)
            rects = {'left': left_rects, 'right': right_rects}
            for name in used:
                for a, b in rects[name]:
                    cv2.rectangle(canvas, a, b, color, 1)
            cv2.circle(canvas, (avex, avey), 5, (0, 255, 255), -1)

        return LaneResult(found=found, target_x=float(avex), target_y=float(avey),
                          debug_image=canvas, left_points=left_points,
                          right_points=right_points, fallback=fallback)
