"""Small, bounded box tracker. A track confirms observations, never a mission location.

Light colours share an identity; distinct objects and duplicate boxes cannot pool hits.
Only detector frames advance confirmation. No image processing or learned model is used.
"""
from collections import deque
from dataclasses import dataclass, field
from typing import Optional


def family(name):
    return 'traffic_light' if name.startswith('traffic_light_') else name


@dataclass(frozen=True)
class Observation:
    class_name: str
    confidence: float
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    @property
    def width(self):
        return self.x_max - self.x_min

    @property
    def height(self):
        return self.y_max - self.y_min

    @property
    def centre(self):
        return ((self.x_min + self.x_max) / 2, (self.y_min + self.y_max) / 2)


@dataclass
class SignTrack:
    id: int
    box: Observation
    first_seen: float
    last_seen: float
    history: deque = field(default_factory=deque)
    confirmed: bool = False
    hits: int = 0


class SignTracker:
    def __init__(self, hits=3, window=5, max_age=.5, retention=2., confidence=.6,
                 sign_height=14, light_height=8, edge_margin=2, max_tracks=32,
                 match_gate=1., scale_ratio=3.):
        if not 1 <= hits <= window or max_age <= 0 or retention < max_age or max_tracks < 1:
            raise ValueError('invalid tracker window, age or capacity')
        self.hits, self.window, self.max_age = hits, window, max_age
        self.retention, self.confidence = retention, confidence
        self.sign_height, self.light_height, self.edge_margin = sign_height, light_height, edge_margin
        if match_gate <= 0 or scale_ratio < 1:
            raise ValueError('invalid association gate or scale ratio')
        self.max_tracks, self.match_gate, self.scale_ratio = max_tracks, match_gate, scale_ratio
        self.tracks = {}
        self.pinned: Optional[int] = None
        self._next_id, self._last_frame = 1, None

    def pin(self, track_id):
        """Keep the selected signal through an occlusion; its colour still needs fresh hits."""
        self.pinned = track_id

    @staticmethod
    def overlap(a, b):
        inter = max(0., min(a.x_max, b.x_max) - max(a.x_min, b.x_min)) * max(
            0., min(a.y_max, b.y_max) - max(a.y_min, b.y_min))
        union = a.width * a.height + b.width * b.height - inter
        return inter / union if union > 0 else 0.

    def match_cost(self, a, b):
        if family(a.class_name) != family(b.class_name) or not 1. / self.scale_ratio <= a.height / b.height <= self.scale_ratio:
            return None
        ac, bc = a.centre, b.centre
        # A signal housing is tall and narrow. Its width alone is not a suitable
        # motion uncertainty scale when the car turns or the signal is occluded.
        sx = max(a.width, b.width, a.height, b.height) if family(a.class_name) == 'traffic_light' else max(a.width, b.width)
        dx, dy = abs(ac[0] - bc[0]) / sx, abs(ac[1] - bc[1]) / max(a.height, b.height)
        return dx + dy if dx <= self.match_gate and dy <= self.match_gate else None

    def _boxes(self, detections, width, height):
        boxes = []
        for d in detections:
            if not (d.class_name.startswith('sign_') or d.class_name.startswith('traffic_light_')):
                continue
            b = Observation(d.class_name, float(d.confidence), d.x_min, d.y_min, d.x_max, d.y_max)
            minimum = self.light_height if family(b.class_name) == 'traffic_light' else self.sign_height
            if (b.confidence >= self.confidence and b.width > 0 and b.height >= minimum and
                    b.x_min >= self.edge_margin and b.y_min >= self.edge_margin and
                    b.x_max <= width - self.edge_margin and b.y_max <= height - self.edge_margin):
                boxes.append(b)
        # One physical light may be returned twice with different colours. Conflicting
        # overlapping colours are unknown, rather than a green permission to depart.
        result = []
        for b in sorted(boxes, key=lambda b: -b.confidence):
            same = next((i for i, old in enumerate(result) if family(b.class_name) == family(
                old.class_name) and self.overlap(b, old) > .5), None)
            if same is None:
                result.append(b)
            elif family(b.class_name) == 'traffic_light' and b.class_name != result[same].class_name:
                old = result[same]
                result[same] = Observation('traffic_light_unknown', old.confidence,
                                           old.x_min, old.y_min, old.x_max, old.y_max)
        return result

    def update(self, stamp, detections, width=640, height=480):
        if self._last_frame is not None and stamp <= self._last_frame:
            return []                  # repeated/out-of-order source images add no evidence
        self._last_frame = stamp
        self.tracks = {i: t for i, t in self.tracks.items()
                       if i == self.pinned or stamp - t.last_seen <= self.retention}
        boxes = self._boxes(detections, width, height)
        pairs = []
        for index, b in enumerate(boxes):
            for track_id, t in self.tracks.items():
                cost = self.match_cost(b, t.box)
                if cost is not None:
                    pairs.append((cost, track_id, index))
        if self.pinned in self.tracks and stamp - self.tracks[self.pinned].last_seen > self.retention:
            matches = [pair for pair in pairs if pair[1] == self.pinned]
            if len(matches) > 1:
                pairs = [pair for pair in pairs if pair[1] != self.pinned]  # ambiguous reacquisition
        assigned, used = {}, set()
        for _, track_id, index in sorted(pairs):
            if track_id not in assigned and index not in used:
                assigned[track_id] = boxes[index]
                used.add(index)
        for index, b in enumerate(boxes):
            if index in used:
                continue
            if len(self.tracks) >= self.max_tracks:
                evict = min((t for t in self.tracks.values() if t.id != self.pinned),
                            key=lambda t: (t.last_seen, t.id), default=None)
                if evict is None:
                    continue
                self.tracks.pop(evict.id)
                assigned.pop(evict.id, None)
            track_id = self._next_id
            self._next_id += 1
            self.tracks[track_id] = SignTrack(track_id, b, stamp, stamp,
                                             deque(maxlen=self.window))
            assigned[track_id] = b
        confirmed = []
        for track_id, t in self.tracks.items():
            b = assigned.get(track_id)
            t.history.append((stamp, None if b is None else b.class_name))
            t.confirmed, t.hits = False, 0
            if b is None:
                continue
            t.box, t.last_seen = b, stamp
            t.hits = sum(name == b.class_name for old, name in t.history
                         if 0 <= stamp - old <= self.max_age)
            t.confirmed = t.hits >= self.hits and b.class_name != 'traffic_light_unknown'
            if t.confirmed:
                confirmed.append(t)
        return confirmed
