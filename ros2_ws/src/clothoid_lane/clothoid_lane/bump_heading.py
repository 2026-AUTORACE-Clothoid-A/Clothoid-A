"""Freeze a clean approach heading before yellow bump tiles contaminate the lane.

No ROS dependencies. All angles here are radians except named ``*_deg`` limits.
"""
import math
from collections import deque
from dataclasses import dataclass


@dataclass
class BumpHeadingParams:
    enabled: bool = True
    reference_age: float = 1.0
    min_frames: int = 3
    max_steer_deg: float = 5.0
    max_yaw_error_deg: float = 8.0
    max_lateral_error: float = 0.1
    max_distance: float = 1.8
    max_duration: float = 12.0
    yaw_gain: float = 1.5
    recovery_frames: int = 3
    target_tolerance_px: float = 8.0


@dataclass(frozen=True)
class HeadingReference:
    stamp: float
    yaw: float
    pose: tuple
    travel: float
    target: tuple


class BumpHeading:
    def __init__(self):
        self.history = deque(maxlen=30)
        self.preparing = False
        self.reference = None
        self.active = self.failed = False
        self.started = self.start_travel = None
        self.recovery_target = self.recovery_stamp = None
        self.recovery_hits = 0

    def clear(self):
        self.__init__()

    def invalidate(self):
        self.history.clear()
        if self.active:
            self.failed = True
        else:
            self.reference = None

    def prepare(self, active, now, params):
        if active and not self.preparing:
            # A heartbeat never replaces an already frozen reference.
            self.reference = self._candidate(now, params)
        self.preparing = active

    def _candidate(self, now, params):
        if len(self.history) < params.min_frames:
            return None
        samples = list(self.history)[-params.min_frames:]
        if not 0 <= now-samples[0].stamp <= params.reference_age:
            return None
        last = samples[-1]
        if any(abs(math.remainder(s.yaw-last.yaw, 2*math.pi)) > math.radians(2.)
               or math.dist(s.target, last.target) > params.target_tolerance_px for s in samples):
            return None
        return last

    def observe(self, sample, trusted, params):
        if self.active:
            return
        if not trusted:
            self.history.clear()
            return
        if self.history and not 0 < sample.stamp-self.history[-1].stamp <= .15:
            self.history.clear()
        self.history.append(sample)
        if self.preparing and self.reference is None:
            self.reference = self._candidate(sample.stamp, params)

    def update(self, now, travel, yaw, pose, *, frame_stamp, fresh, hazard, clean, target,
               locked, params):
        """Return (owns command, steering); None steering means stop until recovery.

        ``hazard`` must use the raw lane result, before the normal target slew filter.
        Frozen heading control continues through pitch lock and a short lane recovery.
        """
        if not params.enabled:
            self.clear()
            return False, None
        if not self.active and self.preparing and hazard:
            self.active = True
            self.started, self.start_travel = now, travel
        if not self.active:
            if not self.preparing:
                self.reference = None
            return False, None

        # Reacquire from clean, distinct, consecutive camera frames after mission exit.
        if not self.preparing and not locked and fresh and clean:
            if frame_stamp != self.recovery_stamp:
                if (self.recovery_stamp is None or not 0 < frame_stamp-self.recovery_stamp <= .15
                        or math.dist(target, self.recovery_target) > params.target_tolerance_px):
                    self.recovery_hits = 0
                self.recovery_target, self.recovery_stamp = target, frame_stamp
                self.recovery_hits += 1
            if self.recovery_hits >= params.recovery_frames:
                self.clear()
                return False, None
        else:
            self.recovery_hits = 0
            self.recovery_stamp = self.recovery_target = None

        ref = self.reference
        if not fresh or ref is None:
            self.failed = True
        elif not (0 <= now-self.started < params.max_duration
                  and 0 <= travel-self.start_travel < params.max_distance):
            self.failed = True
        else:
            error = math.remainder(yaw-ref.yaw, 2*math.pi)
            lateral = -math.sin(ref.pose[2])*(pose[0]-ref.pose[0]) + math.cos(
                ref.pose[2])*(pose[1]-ref.pose[1])
            if (abs(error) > math.radians(params.max_yaw_error_deg)
                    or abs(lateral) > params.max_lateral_error):
                self.failed = True
            if not self.failed:
                return True, self._steer(yaw, params)
        return True, None

    def _steer(self, yaw, params):
        error = math.remainder(yaw-self.reference.yaw, 2*math.pi)
        limit = math.radians(params.max_steer_deg)
        return max(-limit, min(limit, -params.yaw_gain*error))
