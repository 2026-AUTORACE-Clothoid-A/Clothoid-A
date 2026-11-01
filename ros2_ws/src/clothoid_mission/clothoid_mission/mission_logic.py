"""Mission state logic: turns perception into the signals the driving nodes listen to.

ROS-free; a FIRST DRAFT from the 2026 rule summary (docs/MIGRATION_PLAN.md 1-3), not a port: 2025
had no state node (missions fought over twist_mux priorities). Rules still open are parameters.

Inputs each tick: YOLO detections, "stop line ahead", "parking finished".
Outputs: `stop` (state node holds the car), `speed_limit`, `direction`, `parking_flag`.

  * slow sign        -> speed limit for `slow_hold_sec` after the last sighting
  * pedestrian sign  -> arm; stop at the stop line, wait `pedestrian_wait_sec`, then go
  * traffic light    -> arm on red (and yellow if `stop_on_yellow`); stop at the stop line,
                        go when green is seen
  * left / right sign (fork) -> direction value for lane_main
  * parking sign     -> parking flag (latched)
Driving nodes keep working underneath: `stop` only means "publish zero on the state input".
"""
from dataclasses import dataclass, field
from typing import Iterable, Optional, Set

DRIVE = 'drive'
STOPPED_PEDESTRIAN = 'stopped_pedestrian'
STOPPED_LIGHT = 'stopped_light'


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
    # fork (A/B): lane side to follow when the sign points left / right
    fork_left_direction: int = 0       # lane_main: 0 left lane, 1 right lane, 2 both
    fork_right_direction: int = 1


@dataclass
class Inputs:
    now: float
    seen: Set[str] = field(default_factory=set)   # class names visible right now
    stopline: bool = False
    parking_done: bool = False


@dataclass
class Outputs:
    state: str = DRIVE
    stop: bool = False
    speed_limit: Optional[float] = None
    direction: Optional[int] = None    # None until a fork sign was seen
    parking_flag: bool = False


class MissionLogic:
    def __init__(self, params: MissionParams):
        self.p = params
        self.state = DRIVE
        self.slow_until = -1.0
        self.direction: Optional[int] = None
        self.parking_flag = False
        self.pedestrian_armed_at: Optional[float] = None
        self.light_armed_at: Optional[float] = None
        self.stopped_at = 0.0
        self.cooldown_until = {'pedestrian': -1.0, 'light': -1.0}

    def step(self, inputs: Inputs) -> Outputs:
        now, seen, p = inputs.now, inputs.seen, self.p

        if 'sign_slow' in seen:
            self.slow_until = now + p.slow_hold_sec
        if 'sign_left' in seen:
            self.direction = p.fork_left_direction
        elif 'sign_right' in seen:
            self.direction = p.fork_right_direction
        if 'sign_parking' in seen:
            self.parking_flag = True

        self._update_stop(inputs)

        return Outputs(
            state=self.state,
            stop=self.state != DRIVE,
            speed_limit=p.slow_speed if now < self.slow_until else None,
            direction=self.direction,
            parking_flag=self.parking_flag)

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
        self.pedestrian_armed_at = self._expire(self.pedestrian_armed_at, now)
        self.light_armed_at = self._expire(self.light_armed_at, now)

        if not inputs.stopline:
            return
        if self.light_armed_at is not None:
            self._enter(STOPPED_LIGHT, now)
        elif self.pedestrian_armed_at is not None:
            self._enter(STOPPED_PEDESTRIAN, now)

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


def classes_seen(detections: Iterable) -> Set[str]:
    return {d.class_name for d in detections}
