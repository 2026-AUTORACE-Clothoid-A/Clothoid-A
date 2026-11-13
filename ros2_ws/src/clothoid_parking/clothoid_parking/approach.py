"""Bounded lane-speed cap while observing a visually registered parking area.

No camera-box thresholds, world coordinates or bag timestamps. This does not
approve a bay or command steering; handover still needs entry + free-space evidence.
"""
import math
from dataclasses import dataclass


@dataclass
class ApproachParams:
    enabled: bool = True
    speed: float = .20
    deceleration: float = .5
    reaction_sec: float = .25
    stop_margin: float = .08
    entry_to_grid: float = .470207  # same CAD relation as parking_landmark.entry_to_grid
    entry_match_tolerance: float = .15 # independent entry/grid uncertainty, metres
    hold_sec: float = 3.
    timeout_sec: float = 15.


class ParkingApproach:
    def __init__(self, params=None):
        self.p = params or ApproachParams()
        if (any(not math.isfinite(v) or v <= 0 for v in
                (self.p.speed, self.p.deceleration, self.p.hold_sec, self.p.timeout_sec,
                 self.p.entry_to_grid, self.p.entry_match_tolerance)) or
                any(not math.isfinite(v) or v < 0 for v in (self.p.reaction_sec, self.p.stop_margin))):
            raise ValueError('invalid parking approach speed/timing')
        self.reset()

    def reset(self):
        self.started = self.holding_since = None
        self.skipped = False
        self.blocked = False
        self.remaining = None
        self.reason = 'inactive'

    @property
    def observing(self):
        return self.started is not None and not self.skipped

    def _hold(self, now, reason, allow_skip=True):
        if self.holding_since is None:
            self.holding_since = now
        self.reason = reason
        if allow_skip and now-self.holding_since >= self.p.hold_sec:
            self.skipped = True
            self.reason = 'skipped: observation wait expired'
            return None
        return 0.

    def update(self, now, preparing, active, layout, pose, plan, sensors_fresh=True, entry_distance=None, speed=0., steer=0., wheelbase=.2):
        if (not self.p.enabled or not preparing or active or layout is None or
                not layout.p.control_enabled):
            self.reset()
            return None
        if self.skipped:
            return None
        entry_matches = False
        if layout.origin is not None and entry_distance is not None and math.isfinite(entry_distance):
            predicted = ((layout.origin[0]-pose.x)*math.cos(layout.yaw)+
                         (layout.origin[1]-pose.y)*math.sin(layout.yaw)-self.p.entry_to_grid)
            entry_matches = abs(predicted-entry_distance) <= self.p.entry_match_tolerance
        valid = (entry_matches and layout.origin is not None and
                 0 <= now-layout.last_observation <= layout.p.max_pose_age and
                 layout.support_count >= layout.p.confirm_scans and
                 0 <= now-layout.last_scan <= .5 and sensors_fresh)
        if not valid:
            if self.started is None:
                self.reason = 'waiting for matching non-crosswalk entry, grid and lidar support'
                return None
            return self._hold(now, 'holding: registration or sensors unavailable', allow_skip=not self.blocked)
        if self.started is None:
            self.started = now
        bay = layout.bays()[layout.p.target_bay-1]
        rx = pose.x-plan.rear_offset*math.cos(pose.yaw)
        ry = pose.y-plan.rear_offset*math.sin(pose.yaw)
        self.remaining = ((bay.face_x-rx)*math.cos(bay.row_yaw)+
                          (bay.face_y-ry)*math.sin(bay.row_yaw)-plan.turn_radius-plan.turn_lead)
        if self.remaining < -.10:
            self.skipped = True
            self.reason = 'skipped: turn point already passed'
            return None
        # A known obstacle in the lane must not be released merely because a
        # parking observation timer expired. Keep sensing until the sweep clears.
        speed_bound = max(abs(speed), self.p.speed)
        lookahead = max(.12, speed_bound*self.p.reaction_sec + speed_bound**2/(2*self.p.deceleration))
        if not layout.motion_clear(now, pose, self.p.speed, steer, wheelbase, lookahead):
            self.blocked = self.blocked or layout.motion_reason.startswith('occupied')
            return self._hold(now, 'holding approach: '+layout.motion_reason, allow_skip=not self.blocked)
        self.blocked = False
        if now-self.started >= self.p.timeout_sec:
            self.skipped = True
            self.reason = 'skipped: approach observation timeout'
            return None
        distance = self.remaining-self.p.stop_margin
        if distance <= .01:
            return self._hold(now, 'holding before turn: waiting for entry and clear bay')
        # v*t + v^2/(2*a) <= distance. Geometry sets where to brake.
        a, t = self.p.deceleration, self.p.reaction_sec
        cap = min(self.p.speed, math.sqrt((a*t)**2+2*a*distance)-a*t)
        self.holding_since = None
        self.reason = 'observing registered bay at reduced lane speed'
        return cap
