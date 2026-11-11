"""Local floor evidence, in metres along measured motion; no route or mission order.

The end of a tracked crosswalk remains a crosswalk marking after its bars leave the
image. A parking entry is a separate, consistently observed line that the front reaches.
"""
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class FloorObservation:
    lines: Tuple[float, ...] = ()
    crosswalks: Tuple[Tuple[float, float], ...] = ()  # (near, far), base_link metres


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
