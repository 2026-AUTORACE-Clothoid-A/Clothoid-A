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
  * `image` skips the left yellow line: lane_main steers, FinishCore's steering is not used (and its
    left-line-lost abort is off in mission_node). Only the band's columns are converted and scanned
    (numpy, no per-row Python loop).

Candidates must have a plausible physical thickness and appear in consecutive images. After
commit, both nearer and farther unrelated markings are rejected using the odometry-compensated
target tolerance. MissionNode configures a 44 cm wide corridor for the 50 cm lane.
"""
import cv2
import numpy as np

from clothoid_finish.bev import Bev
from clothoid_finish.finish_core import FinishCore, FinishParams, Phase  # noqa: F401  (re-exported)


def longest_runs(mask: np.ndarray) -> np.ndarray:
    """Longest unbroken run of True in every row (numpy: count since the last False)."""
    m = mask.astype(np.int32)
    count = np.cumsum(m, axis=1)
    at_false = np.maximum.accumulate(np.where(m == 0, count, 0), axis=1)
    run = count - at_false
    return run.max(axis=1) if run.shape[1] else np.zeros(mask.shape[0], dtype=np.int32)


class LineStopCore(FinishCore):
    def __init__(self, bev: Bev, p: FinishParams = FinishParams(),
                 white_s_max: int = 60, white_v_min: int = 150, min_run: float = 0.7):
        super().__init__(bev, p)
        self.white_s_max, self.white_v_min = white_s_max, white_v_min
        self.min_run = min_run
        self.debug = False            # True: keep full-size masks for mission's debug image
        self.white = None
        self.min_thickness, self.max_thickness = 0.008, 0.12
        self.max_frame_gap = 0.25
        self._last_sighting = None
        # the band (|y| <= p.band ahead of base_link) never moves in the warped image: only its
        # columns are converted and scanned
        band = (np.abs(bev.gy) <= p.band) & bev.valid
        cols = np.nonzero(band.any(0))[0]
        self._c0, self._c1 = (int(cols.min()), int(cols.max()) + 1) if len(cols) else (0, 0)
        self._band = band[:, self._c0:self._c1]
        self._per_row = self._band.sum(1)
        self._gx = bev.gx[:, self._c0:self._c1]

    def _sighting(self, now: float, line_at: float):
        if self.phase == Phase.COMMIT and self.target is not None and abs(
                line_at - (self.target + self.p.front + self.p.stop_gap)) > self.p.confirm_tolerance:
            return                      # keep tracking this line, in either direction
        if self._last_sighting is not None and now-self._last_sighting > self.max_frame_gap:
            self._sightings.clear()
        self._last_sighting = now
        super()._sighting(now, line_at)

    def remaining(self):
        return None if self.target is None else self.target - self.dist

    def image(self, now: float, bgr: np.ndarray):
        """FinishCore.image's cross line scan, in white and with an unbroken run; no left line."""
        warped = self.bev.warp_image(bgr)[:, self._c0:self._c1]
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        white = (hsv[..., 1] <= self.white_s_max) & (hsv[..., 2] >= self.white_v_min) & self._band
        if self.debug:
            full = np.zeros(self.bev.valid.shape, bool)
            full[:, self._c0:self._c1] = white
            self.white, self.mask = full, np.zeros_like(full)

        # cross line: rows whose central band holds one unbroken white run over min_run of it
        per_row = self._per_row
        runs = longest_runs(white)
        rows = np.nonzero((per_row > 10) & (runs >= self.min_run * np.maximum(per_row, 1)))[0]
        self.cross_x = None
        groups = np.split(rows, np.where(np.diff(rows) > 1)[0] + 1)
        candidates = []
        for group in groups:
            if len(group) < 2:
                continue
            far, near = int(group[0]), int(group[-1])
            thickness = float(np.median(self._gx[far][self._band[far]]) -
                              np.median(self._gx[near][self._band[near]]))
            if self.min_thickness <= thickness <= self.max_thickness:
                candidates.append(near)
        if candidates:
            near = max(candidates)
            self.cross_x = float(np.min(self._gx[near][self._band[near]]))   # near edge of the line
            if self.phase in (Phase.FOLLOW, Phase.COMMIT):
                self._sighting(now, self.dist + self.cross_x)
        elif self.phase == Phase.FOLLOW:
            self._sightings.clear()                 # confirmation requires consecutive images
            self._last_sighting = None
        self.left = None                            # lane_main steers (see the module docstring)
