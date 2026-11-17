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
from clothoid_common.color import ColorPipeline, validate_hsv

from clothoid_finish.bev import Bev
from clothoid_finish.finish_core import FinishCore, FinishParams, Phase  # noqa: F401  (re-exported)
from clothoid_mission.scene_evidence import FloorObservation


def longest_runs(mask: np.ndarray) -> np.ndarray:
    """Longest unbroken run of True in every row (numpy: count since the last False)."""
    m = mask.astype(np.int32)
    count = np.cumsum(m, axis=1)
    at_false = np.maximum.accumulate(np.where(m == 0, count, 0), axis=1)
    run = count - at_false
    return run.max(axis=1) if run.shape[1] else np.zeros(mask.shape[0], dtype=np.int32)


class LineStopCore(FinishCore):
    def __init__(self, bev: Bev, p: FinishParams = FinishParams(),
                 white_s_max: int = 60, white_v_min: int = 150, min_run: float = 0.7,
                 *, white_h_min: int = 0, white_h_max: int = 179, color=None):
        validate_hsv((0, 0, 0), (179, 255, 255), white_h_min, white_h_max, white_s_max, white_v_min)
        super().__init__(bev, p)
        self.color_pipeline = ColorPipeline(color)
        self.white_h_min, self.white_h_max = white_h_min, white_h_max
        self.white_s_max, self.white_v_min = white_s_max, white_v_min
        self.min_run = min_run
        self.debug = False            # True: keep full-size masks for mission's debug image
        self.white = None
        self.min_thickness, self.max_thickness = 0.008, 0.12
        self.max_frame_gap = 0.25
        self._last_sighting = None
        self.require_crosswalk = False
        self.observe_scene = False
        self.observe_entry = False
        self.entry_min_span = .26  # partial boundary at the edge of view during a turn
        self.observation = FloorObservation()
        # the band (|y| <= p.band ahead of base_link) never moves in the warped image: only its
        # columns are converted and scanned
        band = (np.abs(bev.gy) <= p.band) & bev.valid
        cols = np.nonzero(band.any(0))[0]
        self._c0, self._c1 = (int(cols.min()), int(cols.max()) + 1) if len(cols) else (0, 0)
        self._band = band[:, self._c0:self._c1]
        self._per_row = self._band.sum(1)
        self._gx = bev.gx[:, self._c0:self._c1]
        self._row_x = np.array([np.median(x[b]) if b.any() else np.nan
                               for x, b in zip(self._gx, self._band)])

    def crosswalk_behind(self, white, far_row):
        """Longitudinal white bars with gaps, immediately beyond the transverse line."""
        starts = white & ~np.pad(white[:, :-1], ((0, 0), (1, 0)))
        count = starts.sum(1)
        fill = white.sum(1) / np.maximum(self._per_row, 1)
        dx = self._row_x - self._row_x[far_row]
        rows = np.flatnonzero((count >= 3) & (fill >= .3) & (fill <= .85) &
                              (dx >= .02) & (dx <= .45))
        for group in np.split(rows, np.flatnonzero(np.diff(rows) > 1) + 1):
            if len(group) >= 2 and np.ptp(self._row_x[group]) >= .06:
                return True
        return False

    def _crosswalk_spans(self, white):
        starts = white & ~np.pad(white[:, :-1], ((0, 0), (1, 0)))
        count = starts.sum(1)
        fill = white.sum(1) / np.maximum(self._per_row, 1)
        rows = np.flatnonzero((count >= 3) & (fill >= .3) & (fill <= .85))
        spans = []
        for group in np.split(rows, np.flatnonzero(np.diff(rows) > 1) + 1):
            if len(group) >= 2:
                near, far = float(self._row_x[group].min()), float(self._row_x[group].max())
                if far - near >= .06:
                    spans.append((near, far))
        return tuple(spans)

    def _sighting(self, now: float, line_at: float):
        if self.phase == Phase.COMMIT and self.target is not None and abs(
                line_at - (self.target + self.p.front + self.p.stop_gap)) > self.p.confirm_tolerance:
            return False                # unrelated markings must not appear as accepted measurements
        if self._last_sighting is not None and now-self._last_sighting > self.max_frame_gap:
            self._sightings.clear()
        self._last_sighting = now
        super()._sighting(now, line_at)
        return True

    def _entry_lines(self, white):
        """Fit transverse white components; entry lines need not be image-horizontal.

        The pedestrian section starts while the car is still turning. A long run
        in one image row misses that boundary even though it spans the lane.
        Crosswalk bars/letters are too narrow laterally; a diamond is too thick.
        """
        count, labels, stats, _ = cv2.connectedComponentsWithStats(white.astype(np.uint8), 8)
        gy = self.bev.gy[:, self._c0:self._c1]
        lines = []
        for label in range(1, count):
            if stats[label, cv2.CC_STAT_AREA] < 20:
                continue
            component = labels == label
            x, y = self._gx[component], gy[component]
            if np.ptp(y) < self.entry_min_span:
                continue
            centred = y-y.mean()
            slope = np.dot(centred, x-x.mean())/np.dot(centred, centred)
            if abs(slope) > np.tan(np.deg2rad(30.)):
                continue
            intercepts = x-slope*y
            near, far = np.percentile(intercepts, (5, 95))
            if self.min_thickness <= far-near <= self.max_thickness:
                lines.append(float(near))
        return tuple(sorted(lines))

    def remaining(self):
        return None if self.target is None else self.target - self.dist

    def image(self, now: float, bgr: np.ndarray):
        """FinishCore.image's cross line scan, in white and with an unbroken run; no left line."""
        warped = self.bev.warp_image(self.color_pipeline.preprocess(bgr))[:, self._c0:self._c1]
        # Mission white is independent from lane yellow; preserve this policy in all modes.
        _, white = self.color_pipeline.masks(warped, white_h_min=self.white_h_min,
            white_h_max=self.white_h_max, white_s_max=self.white_s_max,
            white_v_min=self.white_v_min, exclude_yellow=False)
        white &= self._band
        crosswalks = self._crosswalk_spans(white) if self.observe_scene or self.require_crosswalk else ()
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
        candidates, lines = [], []
        for group in groups:
            if len(group) < 2:
                continue
            far, near = int(group[0]), int(group[-1])
            thickness = float(np.median(self._gx[far][self._band[far]]) -
                              np.median(self._gx[near][self._band[near]]))
            if self.min_thickness <= thickness <= self.max_thickness:
                line_x = float(np.min(self._gx[near][self._band[near]]))
                lines.append(line_x)
                far_x = float(self._row_x[far])
                behind = any(min(end, far_x + .45) - max(start, far_x + .02) >= .06
                             for start, end in crosswalks)
                if (not self.require_crosswalk or self.phase == Phase.COMMIT or
                        behind):
                    candidates.append(near)
        entry_lines = self._entry_lines(white) if self.observe_entry else ()
        self.observation = FloorObservation(tuple(lines), crosswalks, entry_lines)
        if candidates:
            near = max(candidates)
            self.cross_x = float(np.min(self._gx[near][self._band[near]]))   # near edge of the line
            if self.phase in (Phase.FOLLOW, Phase.COMMIT):
                if not self._sighting(now, self.dist + self.cross_x):
                    self.cross_x = None
        elif self.phase == Phase.FOLLOW:
            self._sightings.clear()                 # confirmation requires consecutive images
            self._last_sighting = None
        self.left = None                            # lane_main steers (see the module docstring)
