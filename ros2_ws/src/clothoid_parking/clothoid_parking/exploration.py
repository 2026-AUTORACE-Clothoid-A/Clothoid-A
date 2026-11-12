"""Lane-following observation before parking takes steering control.

Returns only a speed cap. Lane still owns steering; unseen bays are never free.
"""
import math
from dataclasses import dataclass


@dataclass
class ExploreParams:
    speed: float = .20
    deceleration: float = .5
    reaction_sec: float = .3
    observation_sec: float = 1.0


class Exploration:
    def __init__(self, params=None):
        self.p = params or ExploreParams()
        if min(self.p.speed, self.p.deceleration, self.p.reaction_sec, self.p.observation_sec) <= 0:
            raise ValueError('exploration parameters must be positive')
        self.engaged = False
        self.skipped = False
        self.reason = 'waiting for visual registration'
        self._scan = None
        self._cap = None

    def update(self, now, layout, pose, plan, speed):
        if self.skipped:
            return None
        if layout.origin is None or now-layout.last_observation > layout.p.max_pose_age:
            self.reason = 'holding: registration lost' if self.engaged else 'waiting for visual registration'
            return 0. if self.engaged else None
        self.engaged = True
        if not 0 <= now-layout.last_scan <= .5:
            self.reason = 'holding: lidar stale'
            return 0.
        if self._scan == layout.last_scan:
            return self._cap
        self._scan = layout.last_scan
        bays = layout.bays()
        remaining = [layout.turn_remaining(b, pose, plan) for b in bays]
        relevant = [remaining[layout.p.target_bay-1]] if layout.p.target_bay else remaining
        # A whole registered row has passed, not a timer or a hardcoded bag coordinate.
        passed = max(relevant) < 0.
        future = [d for d in relevant if d >= 0.]
        distance = min(future) if future else 0.
        reserve = self.p.speed*self.p.observation_sec + abs(speed)*self.p.reaction_sec
        cap = math.sqrt(self.p.speed**2 + 2*self.p.deceleration*max(0., distance-reserve))
        # Check the lane corridor through reaction + braking distance, independently
        # of the possibly occluded turn into the bay. No new steering controller.
        horizon = max(.20, abs(speed)*self.p.reaction_sec + speed*speed/(2*self.p.deceleration))
        if not layout.motion_clear(now, pose, max(abs(speed), self.p.speed), 0., .2, distance=horizon):
            self._cap = 0.
            self.reason = 'holding: lane corridor occupied or unobserved'
        elif passed:
            self.skipped = True
            self._cap = None
            self.reason = 'skipped: no verified bay before final turn point'
        else:
            self._cap = cap
            self.reason = 'observing from lane'
        return self._cap
