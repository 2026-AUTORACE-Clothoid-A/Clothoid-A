"""Speed bump (mission 5): slow down at the bump sign, back to normal speed once the bump is passed.
ROS-free (speed_bump_node.py wraps it). Built on the IMU pitch idea of the `eunsoo_bump` branch.

Rules: slow to about half the base speed (0.2 m/s at most) and pass the bump; no visible slowdown or
getting stuck on the bump fails the mission.

How the car's pitch moves over a bump (nose up = +):

    front wheels go up    -> nose up    (+)
    front wheels come down -> about level
    rear wheels go up      -> nose down  (-)
    rear wheels come down  -> level again  = the whole car is over

So the slowdown ends only after nose up, then nose down, then level for `level_sec`. Ending at the
first level moment would speed up while the rear wheels are still before the bump.

States:
    idle     -> sign_bump seen (confident, `confirm_frames` messages in a row)      -> slow
    slow     -> nose up by `up_deg`                                                 -> on_bump
             -> no bump `arm_timeout_sec` after the sign was last seen (fallback)   -> idle
    on_bump  -> nose down by `down_deg`, then level within `level_deg` for `level_sec` -> passed
             -> `on_bump_max_sec` without that (fallback)                           -> passed
    passed   -> `cooldown_sec` later (the sign is behind the car by then)          -> idle
`slow` and `on_bump` limit the speed.

The pitch is measured from a level baseline (the car's own pitch while driving in `idle`, averaged),
because the LIMO's IMU does not read exactly 0 on a flat floor (limo2 bag: -0.43 deg).
"""
import math
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

IDLE = 'idle'
SLOW = 'slow'
ON_BUMP = 'on_bump'
PASSED = 'passed'

SIGN = 'sign_bump'


@dataclass
class SpeedBumpParams:
    slow_speed: float = 0.2          # rules: half the base speed, 0.2 m/s at most
    min_confidence: float = 0.5      # YOLO confidence for the sign
    confirm_frames: int = 2          # sign seen in this many detection messages in a row
    up_deg: float = 3.0              # nose up this much above the baseline: front wheels climbing
    down_deg: float = 3.0            # nose down this much below the baseline: rear wheels climbing
    level_deg: float = 1.0           # within this of the baseline counts as level
    level_sec: float = 0.3           # level this long after the nose-down: over the bump
    arm_timeout_sec: float = 10.0    # no bump this long after the sign was last seen: give up
    on_bump_max_sec: float = 5.0     # on the bump longer than this: count it as passed
    cooldown_sec: float = 3.0        # ignore the sign for this long after passing
    baseline_tau_sec: float = 2.0    # averaging time of the level baseline


def quaternion_to_pitch_deg(x: float, y: float, z: float, w: float) -> float:
    """Pitch of an orientation quaternion in degrees, nose up positive (same as eunsoo_bump)."""
    s = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return math.degrees(math.asin(s))


class SpeedBump:
    def __init__(self, params: SpeedBumpParams):
        self.p = params
        self.state = IDLE
        self.since = 0.0                     # time the current state started
        self.sign_last_seen: Optional[float] = None
        self._sign_count = 0
        self.baseline: Optional[float] = None
        self._baseline_t: Optional[float] = None
        self.pitch: Optional[float] = None   # last pitch relative to the baseline
        self._went_down = False
        self._level_since: Optional[float] = None

    @property
    def limit(self) -> Optional[float]:
        """Speed limit to publish now, or None (no limit)."""
        return self.p.slow_speed if self.state in (SLOW, ON_BUMP) else None

    def _go(self, state: str, now: float) -> None:
        self.state, self.since = state, now
        self._went_down, self._level_since = False, None

    def on_detections(self, detections: Iterable[Tuple[str, float]], now: float) -> str:
        """(class_name, confidence) of one YOLO message."""
        seen = any(name == SIGN and conf >= self.p.min_confidence for name, conf in detections)
        self._sign_count = self._sign_count + 1 if seen else 0
        if self._sign_count >= self.p.confirm_frames:
            self.sign_last_seen = now
            if self.state == IDLE:
                self._go(SLOW, now)
        return self.update(now)

    def on_pitch(self, pitch_deg: float, now: float) -> str:
        """Raw IMU pitch in degrees (nose up positive)."""
        if self.baseline is None:
            self.baseline = pitch_deg
        elif self.state == IDLE:
            # average the flat-floor pitch only while not near a bump
            dt = max(0.0, now - self._baseline_t)
            k = min(1.0, dt / self.p.baseline_tau_sec)
            self.baseline += k * (pitch_deg - self.baseline)
        self._baseline_t = now
        self.pitch = pitch_deg - self.baseline

        d, p = self.pitch, self.p
        if self.state == SLOW and d >= p.up_deg:
            self._go(ON_BUMP, now)
        elif self.state == ON_BUMP:
            if d <= -p.down_deg:
                self._went_down = True
                self._level_since = None
            elif self._went_down and abs(d) <= p.level_deg:
                if self._level_since is None:
                    self._level_since = now
                elif now - self._level_since >= p.level_sec:
                    self._go(PASSED, now)
            else:
                self._level_since = None
        return self.update(now)

    def update(self, now: float) -> str:
        """Time-only transitions (fallbacks and cooldown); call it regularly."""
        p = self.p
        if (self.state == SLOW and self.sign_last_seen is not None
                and now - self.sign_last_seen > p.arm_timeout_sec):
            self._go(IDLE, now)
        elif self.state == ON_BUMP and now - self.since > p.on_bump_max_sec:
            self._go(PASSED, now)
        elif self.state == PASSED and now - self.since >= p.cooldown_sec:
            self._go(IDLE, now)
            self._sign_count = 0
        return self.state
