"""Drive from the parking exit to the end line on the yellow lines only.

    IDLE     until /parking_done
    FOLLOW   follow the left yellow line at lane_offset (white is never looked at: the parking
             lines next to this lane are white); watch for a yellow line across the lane ahead
    COMMIT   the line was seen in `confirm_frames` frames: drive the remaining distance on odom +
             gyro (the line goes under the bumper before the car reaches it), still steering on
             the left line while it is in view; the distance is re-measured while the line is seen
    STOP     zero for good, /finish_done
    ABORT    zero: no end line within max_distance, or the left line lost for too long

The car's front bumper stops `stop_gap` before the near edge of the line (negative: past it).
"""
import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

import cv2
import numpy as np

from clothoid_finish.bev import Bev
from clothoid_parking.tracker import Tracker


class Phase(Enum):
    IDLE = 'idle'
    FOLLOW = 'follow'
    COMMIT = 'commit'
    STOP = 'stop'
    ABORT = 'abort'


@dataclass
class FinishParams:
    hsv_lower: tuple = (18, 60, 90)      # clothoid_lane lane.yaml yellow
    hsv_upper: tuple = (35, 255, 255)
    speed: float = 0.2
    min_speed: float = 0.06
    lane_offset: float = 0.25            # m, left line from base_link (0.5 m lane, line centre to centre)
    left_min: float = 0.08               # m, band searched for the left line
    left_max: float = 0.40               # 0.55 caught the outer road's line beyond the end line
    left_min_span: float = 0.12          # m of left line along x needed for a fit
    steer_gain: float = 2.0              # rad steering per rad of heading error to the line
    lateral_gain: float = 3.0            # rad steering per m off lane_offset
    max_steer: float = 0.41
    band: float = 0.15                   # m each side of base_link scanned for the cross line
    cross_fill: float = 0.5              # share of the band a cross-line row must cover
    confirm_frames: int = 3
    confirm_tolerance: float = 0.04      # m, consecutive sightings must agree (odom-compensated)
    front: float = 0.161                 # m, base_link to the front bumper
    stop_gap: float = 0.03               # m, bumper short of the line's near edge
    max_distance: float = 3.0            # m from the start without an end line -> ABORT
    lost_hold_sec: float = 1.5           # left line missing: hold the heading this long, then ABORT
    rear_offset: float = 0.02            # dead reckoning, as clothoid_parking plan.* (limo2)
    odom_scale: float = 0.98


class FinishCore:
    def __init__(self, bev: Bev, p: FinishParams = FinishParams()):
        self.bev = bev
        self.p = p
        self.tracker = Tracker(rear_offset=p.rear_offset, odom_scale=p.odom_scale)
        self.phase = Phase.IDLE
        self.reason = ''
        self.dist = 0.0                  # m driven since the start (odom, along the path)
        self._last_odom_pose = None
        self._t_phase = 0.0
        self._start_dist = 0.0
        self.target: Optional[float] = None   # self.dist at which to stop
        self._sightings = []             # (dist when seen, absolute dist of the line)
        self.left: Optional[Tuple[float, float]] = None   # (offset at base_link, heading) of the line
        self._t_left = None
        self._hold_yaw = None
        self.cross_x: Optional[float] = None    # last cross line distance ahead of base_link
        self.mask = None

    # ---- sensors -------------------------------------------------------------------------------
    def imu(self, now: float, gz: float):
        self.tracker.imu(now, gz)

    def odom(self, now: float, v: float):
        before = self.tracker.pose
        self.tracker.odom(now, v)
        after = self.tracker.pose
        self.dist += math.copysign(math.hypot(after.x - before.x, after.y - before.y), v)

    def start(self, now: float):
        if self.phase == Phase.IDLE:
            self._go(Phase.FOLLOW, now, 'parking done')
            self._start_dist = self.dist
            self._t_left = now
            self._hold_yaw = self.tracker.pose.yaw

    def _go(self, phase: Phase, now: float, reason: str):
        self.phase, self._t_phase, self.reason = phase, now, reason

    def image(self, now: float, bgr: np.ndarray):
        """One camera frame: left line and cross line."""
        p = self.p
        warped = self.bev.warp_image(bgr)
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(p.hsv_lower), np.array(p.hsv_upper)) > 0
        mask &= self.bev.valid
        self.mask = mask
        gx, gy = self.bev.gx, self.bev.gy

        # cross line: rows whose central band is mostly yellow
        band = (np.abs(gy) <= p.band) & self.bev.valid
        per_row = band.sum(1)
        filled = (mask & band).sum(1)
        rows = np.nonzero((per_row > 10) & (filled >= p.cross_fill * np.maximum(per_row, 1)))[0]
        cross_rows = set()
        self.cross_x = None
        if len(rows):
            near = rows.max()                       # bottom of the warped image = closest to the car
            run = [r for r in rows if r >= near - 40]
            cross_rows = set(range(min(run) - 3, near + 4))
            self.cross_x = float(np.min(gx[near][band[near]]))    # near edge of the line
            if self.phase in (Phase.FOLLOW, Phase.COMMIT):
                self._sighting(now, self.dist + self.cross_x)

        # left line: yellow left of the car, only short of the end line once it is known (beyond it
        # are other roads' lines), and away from the cross line rows
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
            for _ in range(2):                     # drop stray yellow (the end line's corner etc.)
                res = np.abs(Y - np.polyval(fit, X))
                good = res < max(0.02, 2.5 * np.median(res))
                if good.sum() < 30:
                    break
                fit = np.polyfit(X[good], Y[good], 1)
            slope, offset = fit
            self.left = (float(offset), float(math.atan(slope)))
            self._t_left = now

    def _sighting(self, now: float, line_at: float):
        p = self.p
        s = self._sightings
        if s and abs(s[-1][1] - line_at) > p.confirm_tolerance:
            s.clear()
        s.append((self.dist, line_at))
        if self.phase == Phase.FOLLOW and len(s) >= p.confirm_frames:
            self.target = line_at - p.front - p.stop_gap
            self._go(Phase.COMMIT, now, f'end line {line_at - self.dist:.2f} m ahead')
        elif self.phase == Phase.COMMIT:
            # nearer sightings are more accurate: keep the latest
            self.target = line_at - p.front - p.stop_gap

    # ---- command --------------------------------------------------------------------------------
    def command(self, now: float) -> Optional[Tuple[float, float]]:
        p = self.p
        if self.phase == Phase.IDLE:
            return None
        if self.phase in (Phase.STOP, Phase.ABORT):
            return 0.0, 0.0
        pose = self.tracker.pose
        if self.phase == Phase.FOLLOW and self.dist - self._start_dist > p.max_distance:
            self._go(Phase.ABORT, now, f'no end line within {p.max_distance:.1f} m')
            return 0.0, 0.0
        if self.phase == Phase.COMMIT:
            remaining = self.target - self.dist
            if remaining <= 0.005:
                self._go(Phase.STOP, now, f'at the end line ({remaining * 100:+.1f} cm)')
                return 0.0, 0.0
            v = max(p.min_speed, min(p.speed, 1.5 * remaining))
        else:
            v = p.speed
        if self.left is not None:
            offset, heading = self.left
            # line heading relative to the car + being off the wanted distance from it
            steer = p.steer_gain * heading + p.lateral_gain * (offset - p.lane_offset)
            self._hold_yaw = pose.yaw + heading     # the lane direction, for when the line is lost
        else:
            if now - self._t_left > p.lost_hold_sec and self.phase == Phase.FOLLOW:
                self._go(Phase.ABORT, now, 'left line lost')
                return 0.0, 0.0
            err = math.atan2(math.sin(self._hold_yaw - pose.yaw), math.cos(self._hold_yaw - pose.yaw))
            steer = p.steer_gain * err
        return v, max(-p.max_steer, min(p.max_steer, steer))
