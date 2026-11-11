"""Passive, single-scan bay evidence. Never drives or integrates high-rate sensors."""
import math
from dataclasses import dataclass

import numpy as np

from clothoid_parking.bay_finder import BayParams, find_bay


@dataclass
class ProbeParams:
    rate_hz: float = 5.
    confirm_scans: int = 3
    max_gap: float = .75
    tolerance: float = .08         # lateral face/width consistency, metres


class ParkingSceneProbe:
    def __init__(self, bay: BayParams, params=None, laser_x=.103):
        self.bay_params, self.p = bay, params or ProbeParams()
        if self.p.rate_hz <= 0 or self.p.confirm_scans < 1 or self.p.max_gap <= 0 or self.p.tolerance < 0:
            raise ValueError('invalid parking probe rate, confirmation, gap or tolerance')
        self.laser_x = laser_x
        self.reset()

    def reset(self):
        self._last_stamp, self._candidate, self._hits = None, None, 0

    def scan(self, stamp, ranges, angle_min, angle_increment, range_min=.02, range_max=12.):
        if self._last_stamp is not None:
            dt = stamp - self._last_stamp
            if dt + 1e-6 < 1. / self.p.rate_hz:
                return False, None
            if dt > self.p.max_gap:
                self._candidate, self._hits = None, 0
        self._last_stamp = stamp
        r = np.asarray(ranges, dtype=float)
        a = angle_min + np.arange(len(r)) * angle_increment
        keep = np.isfinite(r) & (r > max(.02, range_min)) & (r < range_max)
        points = np.column_stack((r[keep] * np.cos(a[keep]) + self.laser_x,
                                  r[keep] * np.sin(a[keep])))
        candidate = find_bay(points, 0., 0., 0., self.bay_params)
        if candidate is None:
            self._candidate, self._hits = None, 0
            return True, None
        old = self._candidate
        # Along-row position changes as lane_main drives. Confirm the row and gap,
        # without assuming a known approach speed or world parking coordinate.
        same = (old is not None and abs(candidate.face_y - old.face_y) <= self.p.tolerance and
                abs(candidate.width - old.width) <= self.p.tolerance and
                abs(candidate.row_yaw - old.row_yaw) <= math.radians(self.bay_params.max_row_angle_deg))
        self._hits = self._hits + 1 if same else 1
        self._candidate = candidate
        return True, candidate if self._hits >= self.p.confirm_scans else None
