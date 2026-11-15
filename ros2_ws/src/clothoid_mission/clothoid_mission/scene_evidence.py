"""Local floor evidence, in metres along measured motion; no route or mission order.

The end of a tracked crosswalk remains a crosswalk marking after its bars leave the
image. A parking entry is a separate, consistently observed line that the front reaches.
"""
from dataclasses import dataclass
import math
from typing import Optional, Tuple


@dataclass(frozen=True)
class FloorObservation:
    lines: Tuple[float, ...] = ()
    crosswalks: Tuple[Tuple[float, float], ...] = ()  # (near, far), base_link metres
    entry_lines: Tuple[float, ...] = ()  # transverse components, including oblique boundaries


class MissionEntryEvidence:
    """Track the first separate mission boundary, never the crosswalk stop/tail line.

    Observations use measured travel so a line remains usable below the camera.
    Braking starts before the front axle reaches it; reaching it is a separate event.
    """
    def __init__(self, confirm_frames=3, tolerance=.04, max_frame_gap=.25,
                 crosswalk_margin=.12, crosswalk_ahead=.45):
        self.confirm_frames, self.tolerance = confirm_frames, tolerance
        self.max_frame_gap = max_frame_gap
        self.crosswalk_margin, self.crosswalk_ahead = crosswalk_margin, crosswalk_ahead
        self.clear()

    def clear(self):
        self.entry_at = None
        self.slowing = False
        self._candidate, self._hits, self._last_image = None, 0, None
        self._crosswalks = []

    def observe(self, stamp, distance, observation):
        if self._last_image is not None and stamp <= self._last_image:
            return
        if self._last_image is not None and stamp-self._last_image > self.max_frame_gap:
            self._candidate, self._hits = None, 0
        self._last_image = stamp
        # Remember bars after they disappear, so their far boundary cannot become entry.
        self._crosswalks = [(a, b) for a, b in self._crosswalks if b >= distance-.5]
        self._crosswalks.extend((distance+a, distance+b) for a, b in observation.crosswalks)
        self._crosswalks = self._crosswalks[-64:]

        def separate(line):
            return not any(a-self.crosswalk_ahead <= line <= b+self.crosswalk_margin
                           for a, b in self._crosswalks)

        if self.entry_at is not None:
            if not separate(self.entry_at):
                self.entry_at = None
                self.slowing = False
            else:
                return  # keep this boundary; do not jump to later markings
        lines = [distance+x for x in (observation.entry_lines or observation.lines)
                 if x > 0 and separate(distance+x)]
        if not lines:
            self._candidate, self._hits = None, 0
            return
        line = min(lines)
        if self._candidate is None or abs(line-self._candidate) > self.tolerance:
            self._candidate, self._hits = line, 1
        else:
            self._hits += 1
            self._candidate = line
        if self._hits >= self.confirm_frames:
            self.entry_at = line

    def remaining(self, distance, front_axle):
        return None if self.entry_at is None else self.entry_at-distance-front_axle

    def should_slow(self, distance, front_axle, speed, target_speed, reaction_sec, deceleration):
        remaining = self.remaining(distance, front_axle)
        if remaining is None:
            return False
        # Missing speed cannot justify postponing braking once a boundary is confirmed.
        if speed is None or not math.isfinite(speed):
            self.slowing = True
        else:
            speed = max(0., speed)
            braking = speed*reaction_sec + max(0., speed*speed-target_speed*target_speed)/(2*deceleration)
            self.slowing |= remaining <= braking+self.tolerance
        return self.slowing

    def reached(self, distance, front_axle):
        remaining = self.remaining(distance, front_axle)
        return remaining is not None and remaining <= 0.


class ParkingEntryEvidence:
    def __init__(self, confirm_frames=3, tolerance=.04, max_frame_gap=.25,
                 max_travel=3., crosswalk_margin=.12):
        self.confirm_frames, self.tolerance = confirm_frames, tolerance
        self.max_frame_gap, self.max_travel = max_frame_gap, max_travel
        self.crosswalk_margin = crosswalk_margin
        self.clear()

    def clear(self):
        self.crosswalk_end: Optional[float] = None
        self.entry_at: Optional[float] = None
        self._candidate: Optional[float] = None
        self._hits, self._last_image = 0, None

    def observe(self, stamp, distance, observation):
        if self._last_image is not None and stamp <= self._last_image:
            return
        if self._last_image is not None and stamp - self._last_image > self.max_frame_gap:
            self._hits, self._candidate = 0, None
        self._last_image = stamp
        for _, far in observation.crosswalks:
            end = distance + far + self.crosswalk_margin
            self.crosswalk_end = end if self.crosswalk_end is None else max(self.crosswalk_end, end)
        if self.entry_at is not None and self.crosswalk_end is not None and self.entry_at <= self.crosswalk_end:
            self.entry_at = None
        valid = [distance + x for x in observation.lines
                 if self.crosswalk_end is None or distance + x > self.crosswalk_end + self.tolerance]
        if not valid:
            self._candidate, self._hits = None, 0
            return
        line = min(valid)
        if self._candidate is None or abs(line - self._candidate) > self.tolerance:
            self._candidate, self._hits = line, 1
        else:
            self._candidate = line
            self._hits += 1
        if self._hits >= self.confirm_frames:
            self.entry_at = line

    def reached(self, distance, front):
        if self.entry_at is None:
            return False
        progress = distance + front - self.entry_at
        return -self.tolerance <= progress <= self.max_travel
