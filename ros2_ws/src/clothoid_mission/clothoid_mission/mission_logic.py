"""Mission state logic: turns perception into the signals the driving nodes listen to.

ROS-free; a FIRST DRAFT from the 2026 rule summary (docs/MIGRATION_PLAN.md 1-3), not a port: 2025
had no state node (missions fought over twist_mux priorities). Rules still open are parameters.

Inputs each tick: YOLO detections, "stop line ahead", "parking finished".
Outputs: `stop` (state node holds the car), `speed_limit`, `direction`, `parking_flag`.

  * slow sign        -> speed limit for `slow_hold_sec` after the last sighting
  * pedestrian sign  -> arm; stop at the stop line, wait `pedestrian_wait_sec`, then go
  * traffic light    -> arm on red (and yellow if `stop_on_yellow`); stop at the stop line,
                        go when green is seen
    Where it stops (`use_stop_line`): while armed the node runs clothoid_finish's FinishCore with
    the cross line in white (line_stop.py); its speed is the speed limit for lane_main (which keeps
    steering) and the car is held when it reports `arrived` (STOP).
    `use_stop_line` false: stop the moment the 2025 stripe flag (/stopline_detected) is true; that
    flag is commented out in clothoid_lane since 2026-11-09, so false means no stop at all.
  * left / right sign (fork) -> direction value for lane_main
  * parking sign     -> arm; parking flag (latched) at the next single stop line right in front
                        (`parking_line`: /stopline_near, not the stripe detector, which needs a
                        crosswalk; the lidar search starts there, about 2 m before the bay;
                        `parking_at_stopline` false: at the sign)
Driving nodes keep working underneath: `stop` only means "publish zero on the state input".
"""
from dataclasses import dataclass, field
from typing import Iterable, Optional, Set
from collections import deque

DRIVE = 'drive'
STOPPED_PEDESTRIAN = 'stopped_pedestrian'
STOPPED_LIGHT = 'stopped_light'


class DetectionGate:
    """Confirm spatially consistent boxes on distinct detector frames, not timer ticks."""
    def __init__(self, hits=3, window=5, max_age=0.5, confidence=0.6,
                 sign_height=14, light_height=8, edge_margin=2):
        self.hits, self.window, self.max_age = hits, window, max_age
        self.confidence, self.sign_height = confidence, sign_height
        self.light_height, self.edge_margin = light_height, edge_margin
        self.history = deque(maxlen=window)

    @staticmethod
    def matches(a, b):
        # Allow detector jitter and the motion of a sign during corner entry.
        aw, ah = a.x_max-a.x_min, a.y_max-a.y_min
        bw, bh = b.x_max-b.x_min, b.y_max-b.y_min
        return (a.class_name == b.class_name and
                abs((a.x_min+a.x_max-b.x_min-b.x_max)/2) <= max(aw, bw) and
                abs((a.y_min+a.y_max-b.y_min-b.y_max)/2) <= max(ah, bh) and
                0.5 <= ah / bh <= 2.0)

    def update(self, now, detections, accepted, width=640, height=480):
        if self.history and now < self.history[-1][0]:
            self.history.clear()
        boxes = []
        for d in detections:
            minimum = self.light_height if d.class_name.startswith('traffic_light_') else self.sign_height
            if (d.class_name in accepted and d.confidence >= self.confidence and
                    d.y_max-d.y_min >= minimum and d.x_max > d.x_min and
                    d.x_min >= self.edge_margin and d.y_min >= self.edge_margin and
                    d.x_max <= width-self.edge_margin and d.y_max <= height-self.edge_margin):
                boxes.append(d)
        self.history.append((now, boxes))
        return {d.class_name for d in boxes if sum(
            any(self.matches(d, old) for old in frame)
            for stamp, frame in self.history if 0 <= now-stamp <= self.max_age
        ) >= self.hits}


@dataclass
class MissionParams:
    # slow sign
    slow_speed: float = 0.2          # rules: 0.2 m/s or less
    slow_hold_sec: float = 4.0       # bump length is not fixed yet (retune on the track)
    # stop, wait, go
    pedestrian_wait_sec: float = 3.5   # rules: wait 3 s or more, restart within 15 s
    stop_on_yellow: bool = True        # the rules do not say what yellow means
    arm_timeout_sec: float = 8.0       # a sign/light seen this long ago without a stop line expires
    cooldown_sec: float = 15.0         # ignore the same trigger after a stop
    use_stop_line: bool = True         # stop with FinishCore (see the docstring)
    approach_speed: float = 0.2        # m/s under lane_main while armed, before FinishCore drives
    # fork (A/B): lane side to follow when the sign points left / right
    fork_left_direction: int = 0       # lane_main: 0 left lane, 1 right lane, 2 both
    fork_right_direction: int = 1
    # parking: the sign is seen from afar, the stop line before the bays is where the search starts
    parking_at_stopline: bool = True


@dataclass
class Inputs:
    now: float
    seen: Set[str] = field(default_factory=set)   # class names visible right now
    stopline: bool = False
    line_phase: Optional[str] = None    # line stop phase (search/commit/arrived/abort)
    line_speed: Optional[float] = None  # speed limit from the line stop, if any
    parking_line: bool = False      # one stop line right in front (stopline node /stopline_near)
    parking_done: bool = False


@dataclass
class Outputs:
    state: str = DRIVE
    stop: bool = False
    speed_limit: Optional[float] = None
    direction: Optional[int] = None    # None until a fork sign was seen
    parking_flag: bool = False
    stop_arm: bool = False             # run the line stop (FinishCore)


class MissionLogic:
    def __init__(self, params: MissionParams):
        self.p = params
        self.state = DRIVE
        self.slow_until = -1.0
        self.direction: Optional[int] = None
        self.parking_flag = False
        self.parking_armed = False
        self.pedestrian_armed_at: Optional[float] = None
        self.light_armed_at: Optional[float] = None
        self.stopped_at = 0.0
        self.cooldown_until = {'pedestrian': -1.0, 'light': -1.0}

    def step(self, inputs: Inputs) -> Outputs:
        now, seen, p = inputs.now, inputs.seen, self.p

        if 'sign_slow' in seen or 'sign_bump' in seen:
            self.slow_until = now + p.slow_hold_sec
        if 'sign_left' in seen:
            self.direction = p.fork_left_direction
        elif 'sign_right' in seen:
            self.direction = p.fork_right_direction
        if 'sign_parking' in seen:
            self.parking_armed = True
        if self.parking_armed and (inputs.parking_line or not p.parking_at_stopline):
            self.parking_flag = True

        self._update_stop(inputs)

        limits = []
        if now < self.slow_until:
            limits.append(p.slow_speed)
        arm = p.use_stop_line and self.state == DRIVE and self._armed()
        if arm:
            limits.append(inputs.line_speed if inputs.line_speed is not None else p.approach_speed)
        return Outputs(
            state=self.state,
            stop=self.state != DRIVE,
            speed_limit=min(limits) if limits else None,
            direction=self.direction,
            parking_flag=self.parking_flag,
            stop_arm=arm)

    def _update_stop(self, inputs: Inputs) -> None:
        now, seen, p = inputs.now, inputs.seen, self.p

        if self.state == STOPPED_PEDESTRIAN:
            if now - self.stopped_at >= p.pedestrian_wait_sec:
                self._release('pedestrian', now)
            return
        if self.state == STOPPED_LIGHT:
            if 'traffic_light_green' in seen:
                self._release('light', now)
            return

        # arm on sightings, expire stale arms
        if 'sign_pedestrian' in seen and now >= self.cooldown_until['pedestrian']:
            self.pedestrian_armed_at = now
        red = 'traffic_light_red' in seen or (p.stop_on_yellow and 'traffic_light_yellow' in seen)
        if red and now >= self.cooldown_until['light']:
            self.light_armed_at = now
        if 'traffic_light_green' in seen:
            self.light_armed_at = None   # green on approach: no stop
        if inputs.line_phase not in ('commit', 'arrived'):   # a confirmed line keeps the arm
            self.pedestrian_armed_at = self._expire(self.pedestrian_armed_at, now)
            self.light_armed_at = self._expire(self.light_armed_at, now)

        if p.use_stop_line:
            if inputs.line_phase != 'arrived' or not self._armed():
                return
        elif not inputs.stopline:
            return
        if self.light_armed_at is not None:
            self._enter(STOPPED_LIGHT, now)
        elif self.pedestrian_armed_at is not None:
            self._enter(STOPPED_PEDESTRIAN, now)

    def _armed(self) -> bool:
        return self.pedestrian_armed_at is not None or self.light_armed_at is not None

    def _expire(self, armed_at: Optional[float], now: float) -> Optional[float]:
        if armed_at is not None and now - armed_at > self.p.arm_timeout_sec:
            return None
        return armed_at

    def _enter(self, state: str, now: float) -> None:
        self.state, self.stopped_at = state, now

    def _release(self, which: str, now: float) -> None:
        self.state = DRIVE
        self.cooldown_until[which] = now + self.p.cooldown_sec
        if which == 'pedestrian':
            self.pedestrian_armed_at = None
        else:
            self.light_armed_at = None


def classes_seen(detections: Iterable, camera_center_x: float = 319.9,
                 light_max_offset_height_ratio: float = 3.5,
                 sign_max_offset_height_ratio: float = 5.0,
                 fork_max_offset_height_ratio: float = 10.0) -> Set[str]:
    """Classes used by the mission, excluding boxes far to the side for their size.

    abs(box centre x - camera principal x) / box height is a distance-normalised
    horizontal offset. On the start straight, the adjacent track's red light was
    5.3; on our signal approach it was 2.9. Signs need a separate limit: the valid
    bump/pedestrian signs reached 3.9, the adjacent parking sign was over 9, and
    the fork sign reached 9.4 while approaching from the outer lane.
    A configured limit <= 0 disables that group's filter for recorded comparisons.
    """
    seen = set()
    for d in detections:
        if d.class_name.startswith('traffic_light_'):
            limit = light_max_offset_height_ratio
        elif d.class_name in ('sign_left', 'sign_right'):
            limit = fork_max_offset_height_ratio
        elif d.class_name.startswith('sign_'):
            limit = sign_max_offset_height_ratio
        else:
            limit = 0.0
        if limit > 0:
            height = d.y_max - d.y_min
            if height <= 0 or d.x_max <= d.x_min:
                continue
            offset = abs((d.x_min + d.x_max) / 2.0 - camera_center_x)
            if offset > limit * height:
                continue
        seen.add(d.class_name)
    return seen
