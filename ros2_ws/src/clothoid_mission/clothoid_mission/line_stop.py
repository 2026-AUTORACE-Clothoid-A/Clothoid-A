"""Stop at the white stop line (traffic light, pedestrian) with clothoid_finish's FinishCore.

Everything is FinishCore as it is: the left yellow line follow, the cross line scan, `confirm_frames`
sightings, the target (line - front 0.161 - stop_gap 0.03; the line reads ~3 cm further while it
leaves the bottom of the view, so the bumper ends about on the line, as finish does), the speed
ramp, STOP / ABORT, the parking Tracker (odom x 0.98). Three changes:

  * `image`: the cross line is looked for in white (the 2026 stop lines) instead of yellow; the left
    line stays yellow
  * `image`: a cross line row needs one unbroken white run over `min_run` (0.7) of the band, the
    parking trigger's rule (clothoid_lane NearLineParams.min_run), instead of finish's 50 % filled.
    The "천천히" letters before the signal's stop line filled 50 % of a row with their strokes and
    were confirmed as the line (sim 2026-11-09: stopped 26 cm short); the letters have gaps.
  * `_sighting`: once committed, a line more than `confirm_tolerance` (4 cm) further than the current
    one is ignored. Past the stop line come the crosswalk bars (white, along the lane, ~55 % of the
    band) and the crosswalk's far line; when the stop line left the view FinishCore took them and
    stopped 56 cm late in the synthetic test. Nearer or equal readings update the target as before.
"""
import math

import cv2
import numpy as np

from clothoid_finish.bev import Bev
from clothoid_finish.finish_core import FinishCore, FinishParams, Phase  # noqa: F401  (re-exported)


def longest_runs(mask: np.ndarray) -> np.ndarray:
    """Longest unbroken run of True in every row."""
    padded = np.zeros((mask.shape[0], mask.shape[1] + 2), dtype=np.int8)
    padded[:, 1:-1] = mask
    edges = np.diff(padded, axis=1)
    out = np.zeros(mask.shape[0], dtype=int)
    for i in np.nonzero(mask.any(1))[0]:
        starts, ends = np.flatnonzero(edges[i] == 1), np.flatnonzero(edges[i] == -1)
        out[i] = int((ends - starts).max())
    return out


class LineStopCore(FinishCore):
    def __init__(self, bev: Bev, p: FinishParams = FinishParams(),
                 white_s_max: int = 60, white_v_min: int = 150, min_run: float = 0.7):
        super().__init__(bev, p)
        self.white_s_max, self.white_v_min = white_s_max, white_v_min
        self.min_run = min_run
        self.white = None

    def _sighting(self, now: float, line_at: float):
        if self.phase == Phase.COMMIT and self.target is not None and \
                line_at > self.target + self.p.front + self.p.stop_gap + self.p.confirm_tolerance:
            return                      # a line past the stop line: crosswalk bars / far line
        super()._sighting(now, line_at)

    def remaining(self):
        return None if self.target is None else self.target - self.dist

    def image(self, now: float, bgr: np.ndarray):
        """FinishCore.image with the cross line in white (the left line stays yellow)."""
        p = self.p
        warped = self.bev.warp_image(bgr)
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(p.hsv_lower), np.array(p.hsv_upper)) > 0
        mask &= self.bev.valid
        white = (hsv[..., 1] <= self.white_s_max) & (hsv[..., 2] >= self.white_v_min) & self.bev.valid
        self.mask, self.white = mask, white
        gx, gy = self.bev.gx, self.bev.gy

        # cross line: rows whose central band holds one unbroken white run over min_run of it
        band = (np.abs(gy) <= p.band) & self.bev.valid
        per_row = band.sum(1)
        runs = longest_runs(white & band)
        rows = np.nonzero((per_row > 10) & (runs >= self.min_run * np.maximum(per_row, 1)))[0]
        cross_rows = set()
        self.cross_x = None
        if len(rows):
            near = rows.max()                       # bottom of the warped image = closest to the car
            run = [r for r in rows if r >= near - 40]
            cross_rows = set(range(min(run) - 3, near + 4))
            self.cross_x = float(np.min(gx[near][band[near]]))    # near edge of the line
            if self.phase in (Phase.FOLLOW, Phase.COMMIT):
                self._sighting(now, self.dist + self.cross_x)

        # left line: as FinishCore
        keep = mask & (gy >= p.left_min) & (gy <= p.left_max)
        if cross_rows:
            keep[sorted(r for r in cross_rows if 0 <= r < keep.shape[0])] = False
        if self._sightings:
            keep &= gx < self._sightings[-1][1] - self.dist - 0.03
        ys, xs = np.nonzero(keep)
        self.left = None
        if len(ys) >= 30 and np.ptp(gx[ys, xs]) >= p.left_min_span:
            X, Y = gx[ys, xs], gy[ys, xs]
            fit = np.polyfit(X, Y, 1)
            for _ in range(2):
                res = np.abs(Y - np.polyval(fit, X))
                good = res < max(0.02, 2.5 * np.median(res))
                if good.sum() < 30:
                    break
                fit = np.polyfit(X[good], Y[good], 1)
            slope, offset = fit
            self.left = (float(offset), float(math.atan(slope)))
            self._t_left = now
