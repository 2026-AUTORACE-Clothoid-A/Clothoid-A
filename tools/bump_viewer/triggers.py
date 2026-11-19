"""Trigger lanes and the driven path, from recorded and replayed evidence. No ROS, no Qt.

A lane is one trigger (slow sign, stop line, parking, ...). Each lane holds segments
(t0, t1, level) for two sources:
  recorded  what the car did: /mission/state, recorded signals and logs of the run
  replay    what the current mission code does on the same inputs (backend replay)
`level` is 1 for "armed / looking" (sign seen, searching the line) and 2 for "acting"
(slowing, committed, stopped, parking flag on), so "received but never acted" is visible.

The path integrates IMU yaw rate with the wheel speed. /odom yaw is not used: it is computed
from the steering command and does not match the real rotation (see decision_viz.py).
"""
import math
import re

GAP = .35   # s: samples closer than this belong to one segment (state 20 Hz, signals 1 Hz refresh)

# key, label, colour (recorded tint is drawn darker by the panel)
LANES = (
    ('slow', '서행 표지', '#f6c85f'),
    ('bump', '방지턱', '#c9a0ff'),
    ('entry', '시작선', '#ffa25c'),
    ('line', '정지선', '#ff6f91'),
    ('stop', '정지', '#ff4d4d'),
    ('limit', '미션 감속', '#6fd3ff'),
    ('fork', '갈림길', '#9be37b'),
    ('cone', '라바콘', '#ff9f43'),
    ('avoid', '장애물', '#7aa7ff'),
    ('parking', '주차', '#4dd0c4'),
    ('rc', 'RC 조종', '#b0b0b0'),
)


def segments(samples, gap=GAP, end=None):
    """[(t, level)] sorted by t -> [(t0, t1, level)]; level 0 / None closes a segment."""
    out = []
    current = None   # [t0, t1, level]
    for t, level in samples:
        level = int(level or 0)
        if current is not None and (level != current[2] or t - current[1] > gap):
            out.append(tuple(current)); current = None
        if level:
            if current is None:
                current = [t, t, level]
            else:
                current[1] = t
    if current is not None:
        out.append(tuple(current))
    return out


def state_levels(state):
    """Per-lane level from one /mission/state dict (recorded or replayed)."""
    armed = state.get('armed') or []
    bump = state.get('bump_state')
    entry = state.get('entry_phase')
    line = state.get('line_phase')
    return {
        'slow': 1 if 'slow' in armed else 0,
        'bump': 2 if bump == 'on_bump' else 1 if bump == 'approaching' else 0,
        'entry': 2 if entry in ('braking', 'entered') else 1 if entry in ('search', 'confirmed') else 0,
        'line': 2 if line in ('commit', 'arrived') else 1 if line == 'search' else 0,
        'stop': 2 if state.get('state', 'drive') != 'drive' else 0,
        'limit': 2 if state.get('speed_limit') is not None else 0,
        'parking': 1 if state.get('parking_prepare') else 0,
    }


def from_states(states):
    """[(t, state)] -> {lane: [(t0, t1, level)]} for the lanes /mission/state carries."""
    samples = {}
    for t, state in states:
        for lane, level in state_levels(state).items():
            samples.setdefault(lane, []).append((t, level))
    return {lane: segments(s) for lane, s in samples.items()}


SIGNAL_LANES = {'/parking_flag': 'parking', '/cone/mission_active': 'cone', '/avoid/active': 'avoid'}


def from_signals(signals, gap=1.5):
    """[(t, topic, value)] -> lanes; a true Bool is level 2. Signals refresh ~1 Hz, hence the gap."""
    samples = {}
    for t, name, value in signals:
        lane = SIGNAL_LANES.get(name)
        if lane is not None:
            samples.setdefault(lane, []).append((t, 2 if value is True else 0))
        elif name == '/direction_flag' and value is not None:
            samples.setdefault('fork', []).append((t, 2))
    return {lane: segments(s, gap) for lane, s in samples.items()}


RC_ON = re.compile(r'control mode 3\b')
RC_OFF = re.compile(r'control mode [^3]\b')


def rc_segments(rows, end):
    """Manual RC control from limo_base log lines: 'control mode 3 (RC transmitter)' until the
    next other mode. Rows are log dicts with 't' and 'message' (both log sources merged)."""
    out, since = [], None
    for row in sorted((r for r in rows if r.get('t') is not None), key=lambda r: r['t']):
        message = row.get('message', '')
        if RC_ON.search(message) and since is None:
            since = row['t']
        elif RC_OFF.search(message) and since is not None:
            out.append((since, row['t'], 2)); since = None
    if since is not None:
        out.append((since, end, 2))
    return out


def merge_lanes(*parts):
    out = {}
    for part in parts:
        for lane, segs in part.items():
            out.setdefault(lane, []).extend(segs)
    return {lane: sorted(segs) for lane, segs in out.items()}


def build(data):
    """{'recorded': lanes, 'replay': lanes} from a backend result."""
    duration = data.get('duration', 0.)
    recorded = merge_lanes(from_states(data.get('recorded_states', [])),
                           from_signals(data.get('recorded_signals', [])),
                           {'rc': rc_segments(data.get('recorded_logs', []) + data.get('file_logs', []), duration)})
    replay = merge_lanes(from_states([(f['t'], f['state']) for f in data.get('frames', []) if f.get('state')]),
                         from_signals(data.get('replay_signals', [])))
    return {'recorded': recorded, 'replay': replay}


def active_at(lanes, t):
    """{lane: level} of the segments covering time t."""
    out = {}
    for lane, segs in lanes.items():
        for t0, t1, level in segs:
            if t0 <= t <= t1:
                out[lane] = max(out.get(lane, 0), level)
    return out


class Path2D:
    """Dead reckoning: yaw from the IMU yaw rate, distance from the wheel speed."""

    def __init__(self, max_dt=.2):
        self.x = self.y = self.yaw = 0.
        self.v = 0.
        self.max_dt = max_dt
        self._imu = self._odom = None

    def imu(self, t, gz):
        if self._imu is not None and 0 < t - self._imu <= self.max_dt:
            self.yaw += gz * (t - self._imu)
        self._imu = t

    def odom(self, t, v):
        if self._odom is not None and 0 < t - self._odom <= self.max_dt:
            dt = t - self._odom
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt
        self.v, self._odom = v, t

    def pose(self):
        return (self.x, self.y, self.yaw)
