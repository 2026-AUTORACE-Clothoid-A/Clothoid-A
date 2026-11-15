"""Confirm lane handover separately from measured, aligned speed recovery."""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class HandoverParams:
    min_duration: float = 1.5       # seconds after the cone controller releases
    min_distance: float = .45      # metres of fresh wheel odometry after release
    alignment_sec: float = .6      # consecutive aligned lane observations
    ramp_sec: float = 1.0
    max_heading_deg: float = 15.0  # fitted boundary heading in the warped image
    max_steer_deg: float = 8.0     # unit-gain target steering angle

    def __post_init__(self):
        for name, value in self.__dict__.items():
            if not math.isfinite(value) or value < 0 or (name not in (
                    'min_duration', 'min_distance', 'alignment_sec') and value == 0):
                raise ValueError(f'cone exit {name} must be finite and positive (holds may be zero)')


class ConeHandover:
    def __init__(self, params=None):
        self.p = params or HandoverParams()
        self.exit_requested = False
        self.cone_active = False
        self.recovering = False
        self.handover_at = self.start_travel = self.travel = None
        self._clear_evidence()

    def _clear_evidence(self):
        self.since = self.last_frame = self.release_since = None
        self.hits = 0
        self.aligned_since = self.ramp_since = None

    def on_cone(self, active, now, travel=None):
        if active and not self.cone_active:
            self.exit_requested = False
            self.recovering = False
            self._clear_evidence()
            self.handover_at = self.start_travel = self.travel = None
        elif not active and self.cone_active:
            self.recovering = True
            self._clear_evidence()
            self.handover_at, self.start_travel = now, travel
        self.cone_active = active

    def request_exit(self, active):
        if active and not self.exit_requested:
            self.recovering = True
            self._clear_evidence()
        elif not active and self.exit_requested and self.cone_active:
            self.recovering = False
            self._clear_evidence()
        self.exit_requested = active

    @property
    def ready(self):
        return self.exit_requested and self.release_since is not None

    def observe(self, now, found, detected, used, tolerance, confirm_sec,
                *, aligned=False, travel=None):
        if not self.recovering:
            return
        valid = (found and used is not None and detected is not None
                 and all(math.isfinite(v) for v in (*detected, *used))
                 and math.dist(detected, used) <= max(0., tolerance))
        if not valid:
            self._clear_evidence()
            return
        contiguous = (self.last_frame is not None and 0 < now - self.last_frame <= confirm_sec)
        if not contiguous:
            self._clear_evidence()
            self.since = now
        self.last_frame = now
        self.hits += 1
        if self.hits >= 3 and now - self.since >= confirm_sec and self.release_since is None:
            self.release_since = now
        self.travel = travel if travel is not None and math.isfinite(travel) else None
        if not self.cone_active and self.travel is not None:
            if self.start_travel is None or self.travel < self.start_travel:
                self.start_travel = self.travel
                self.aligned_since = self.ramp_since = None
        if aligned and self.travel is not None:
            if self.aligned_since is None:
                self.aligned_since = now
        else:
            self.aligned_since = self.ramp_since = None

    def speed(self, now, requested, recovery_speed, confirm_sec):
        if not self.recovering or recovery_speed <= 0:
            return requested
        if self.cone_active:
            return min(requested, recovery_speed)
        fraction = 0.
        if self.release_since is not None:
            if now < self.release_since or now - self.last_frame > confirm_sec:
                self._clear_evidence()
            else:
                p = self.p
                ready = (self.handover_at is not None and now-self.handover_at >= p.min_duration
                         and self.travel is not None and self.start_travel is not None
                         and self.travel-self.start_travel >= p.min_distance
                         and self.aligned_since is not None
                         and now-self.aligned_since >= p.alignment_sec)
                if ready:
                    if self.ramp_since is None:
                        self.ramp_since = now
                    fraction = min(1., (now-self.ramp_since)/p.ramp_sec)
                    if fraction >= 1.:
                        self.recovering = False
                        self._clear_evidence()
                else:
                    self.ramp_since = None
        return min(requested, recovery_speed + max(0., requested - recovery_speed) * fraction)
