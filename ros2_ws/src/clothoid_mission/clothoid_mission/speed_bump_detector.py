"""Speed Bump Detection & Control using YOLO sign trigger and IMU pitch sensing.

FSM States:
  - IDLE: Normal driving, IMU monitoring inactive (avoids false triggers on floor bumps).
  - APPROACHING: YOLO saw 'sign_bump'. Armed state; speed reduced to 0.2 m/s.
  - ON_BUMP: Pitch > climb threshold (front wheels on bump). Speed 0.2 m/s, steering lock requested.
  - COOLDOWN: Fully cleared bump and leveled; cooldown timer active before returning to IDLE.
"""
import math
from dataclasses import dataclass
from typing import Optional


def quaternion_to_pitch_deg(x: float, y: float, z: float, w: float) -> float:
    """Convert orientation quaternion (x, y, z, w) to vehicle Pitch angle in degrees.
    
    Positive pitch: nose up (climbing bump).
    Negative pitch: nose down (descending bump).
    """
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch_rad = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch_rad = math.asin(sinp)
    return math.degrees(pitch_rad)


@dataclass
class SpeedBumpParams:
    slow_speed: float = 0.2            # Max regulation speed on bump (m/s)
    pitch_climb_thresh: float = 3.5    # Nose-up angle to confirm bump entry (deg)
    pitch_level_thresh: float = 1.2    # Abs angle to consider vehicle level (deg)
    level_duration_sec: float = 0.3    # Continuous level duration to confirm exit (sec)
    arm_timeout_sec: float = 6.0       # Auto-expire if bump is not hit after sign (sec)
    cooldown_sec: float = 4.0          # Ignore subsequent triggers after exit (sec)


IDLE = 'idle'
APPROACHING = 'approaching'
ON_BUMP = 'on_bump'
COOLDOWN = 'cooldown'


class SpeedBumpDetector:
    def __init__(self, params: Optional[SpeedBumpParams] = None):
        self.p = params or SpeedBumpParams()
        self.state: str = IDLE
        self.armed_at: float = -1.0
        self.entered_bump_at: float = -1.0
        self.level_started_at: Optional[float] = None
        self.cooldown_until: float = -1.0
        self.seen_negative_pitch: bool = False

    def on_sign_bump(self, now: float) -> None:
        """Triggered when YOLO detects 'sign_bump'."""
        if self.state == COOLDOWN and now < self.cooldown_until:
            return  # Still in cooldown

        if self.state == IDLE or (self.state == COOLDOWN and now >= self.cooldown_until):
            self.state = APPROACHING
            self.armed_at = now
            self.seen_negative_pitch = False
            self.level_started_at = None

    def update_pitch(self, pitch_deg: float, now: float) -> None:
        """Update detector with current IMU pitch in degrees."""
        if self.state == IDLE:
            return

        if self.state == COOLDOWN:
            if now >= self.cooldown_until:
                self.state = IDLE
            return

        if self.state == APPROACHING:
            # Check timeout
            if now - self.armed_at > self.p.arm_timeout_sec:
                self.state = IDLE
                return

            # Check front wheels climbing bump
            if pitch_deg >= self.p.pitch_climb_thresh:
                self.state = ON_BUMP
                self.entered_bump_at = now
                self.seen_negative_pitch = False
                self.level_started_at = None
            return

        if self.state == ON_BUMP:
            # Track descending phase (front wheels going down / rear climbing)
            if pitch_deg <= -1.0:
                self.seen_negative_pitch = True

            # Check level condition (rear wheels exited)
            if abs(pitch_deg) <= self.p.pitch_level_thresh:
                if self.level_started_at is None:
                    self.level_started_at = now
                elif now - self.level_started_at >= self.p.level_duration_sec:
                    # Successfully cleared bump!
                    self.state = COOLDOWN
                    self.cooldown_until = now + self.p.cooldown_sec
                    self.level_started_at = None
            else:
                self.level_started_at = None

    @property
    def is_active(self) -> bool:
        """Whether speed limit must be applied (approaching or on bump)."""
        return self.state in (APPROACHING, ON_BUMP)

    @property
    def should_lock_steering(self) -> bool:
        """Whether steering should be locked to prevent camera shake distortion."""
        return self.state == ON_BUMP

    @property
    def speed_limit(self) -> Optional[float]:
        """Speed limit in m/s, or None if no limit."""
        return self.p.slow_speed if self.is_active else None
