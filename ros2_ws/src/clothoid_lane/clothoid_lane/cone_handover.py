"""Release the cone-exit speed cap only after the lane target catches up."""
import math


class ConeHandover:
    def __init__(self):
        self.exit_requested = False
        self.cone_active = False
        self.recovering = False
        self._clear_evidence()

    def _clear_evidence(self):
        self.since = self.last_frame = self.release_since = None
        self.hits = 0

    def on_cone(self, active, now):
        if active and not self.cone_active:
            self.exit_requested = False
            self.recovering = False
            self._clear_evidence()
        elif not active and self.cone_active:
            self.recovering = True
            self._clear_evidence()
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

    def observe(self, now, found, detected, used, tolerance, confirm_sec):
        if not self.recovering:
            return
        valid = (found and used is not None
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
                fraction = min(1., (now - self.release_since) / confirm_sec)
                if fraction >= 1.:
                    self.recovering = False
                    self._clear_evidence()
        return min(requested, recovery_speed + max(0., requested - recovery_speed) * fraction)
