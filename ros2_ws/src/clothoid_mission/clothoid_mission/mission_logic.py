"""Mission state logic: turns perception into the signals the driving nodes listen to.

Inputs each tick: YOLO detections, "stop line ahead", "parking finished", IMU pitch.
Outputs: `stop` (state node holds the car), `speed_limit`, `direction`, `parking_flag`, `steering_lock`.

  * slow sign        -> speed limit for `slow_hold_sec` after the last sighting
  * bump sign + IMU  -> armed slow speed, dynamic exit on leveling, steering lock on climb
  * pedestrian sign  -> arm; stop at the stop line, wait `pedestrian_wait_sec`, then go
  * traffic light    -> arm on red (and yellow if `stop_on_yellow`); stop at the stop line,
                        go when green is seen
    Where it stops (`use_stop_line`): while armed the node runs clothoid_finish's FinishCore with
    the cross line in white (line_stop.py); its speed is the speed limit for lane_main (which keeps
    steering) and the car is held when it reports `arrived` (STOP).
    `use_stop_line` false: stop the moment the 2025 stripe flag (/stopline_detected) is true; that
    flag is commented out in clothoid_lane since 2026-11-09, so false means no stop at all.
  * left / right sign -> preferred boundary; lane_main still follows the measured road
  * parking sign     -> prepare passive scene sensing; entry plus bay confirmation transfers control
  * require_scene_confirmation false retains the legacy immediate/timed policy for comparisons
No mission's success or place in a route is a prerequisite for another mission.
Driving nodes keep working underneath: `stop` only means "publish zero on the state input".
"""
import math
from dataclasses import dataclass, field
from collections import deque
from typing import Dict, Iterable, Optional, Set, Tuple

DRIVE = 'drive'
STOPPED_PEDESTRIAN = 'stopped_pedestrian'
STOPPED_LIGHT = 'stopped_light'

# Bump FSM states
BUMP_IDLE = 'idle'
BUMP_APPROACHING = 'approaching'
BUMP_ON = 'on_bump'
BUMP_COOLDOWN = 'cooldown'


def quaternion_to_pitch_deg(x: float, y: float, z: float, w: float) -> float:
    """Convert orientation quaternion (x, y, z, w) to vehicle Pitch angle in degrees.

    ROS right-handed body axes: negative pitch raises the nose. The bump
    detector accepts both signs to tolerate the hardware driver's convention.
    """
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch_rad = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch_rad = math.asin(sinp)
    return math.degrees(pitch_rad)


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
    # slow sign & bump speed
    slow_speed: float = 0.2            # rules: 0.2 m/s or less
    slow_hold_sec: float = 4.0         # general slow sign hold timer
    # speed bump (IMU Pitch + YOLO fusion)
    bump_pitch_thresh: float = 3.5      # deg, absolute pitch to detect contact
    bump_level_thresh: float = 1.2      # deg, abs angle to detect vehicle leveled
    bump_level_duration_sec: float = 0.3  # sec, continuous leveling to confirm exit
    bump_arm_timeout_sec: float = 12.0  # sec since last confirmed sign; tune to approach distance/speed
    bump_cooldown_sec: float = 4.0      # sec, cooldown before re-arming bump
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
    require_scene_confirmation: bool = True
    parking_enabled: bool = True
    slow_arms_bump: bool = False


@dataclass
class Inputs:
    now: float
    seen: Set[str] = field(default_factory=set)   # class names visible right now
    stopline: bool = False
    line_phase: Optional[str] = None    # line stop phase (search/commit/arrived/abort)
    line_speed: Optional[float] = None  # speed limit from the line stop, if any
    parking_line: bool = False      # one stop line right in front (stopline node /stopline_near)
    parking_done: bool = False
    imu_pitch: Optional[float] = None             # vehicle pitch in degrees
    parking_scene: bool = False    # fresh, repeated lidar bay observations


@dataclass
class Outputs:
    state: str = DRIVE
    stop: bool = False
    speed_limit: Optional[float] = None
    direction: Optional[int] = None    # None until a fork sign was seen
    parking_flag: bool = False
    stop_arm: bool = False             # run the line stop (FinishCore)
    steering_lock: bool = False        # lock steering during bump traversal
    parking_prepare: bool = False     # observe the parking scene, keep lane control


class MissionLogic:
    def __init__(self, params: MissionParams):
        self.p = params
        self.state = DRIVE
        self.slow_until = -1.0
        self.direction: Optional[int] = None
        self.parking_flag = False
        self.parking_armed = False
        self.parking_seen_at = None
        self.pedestrian_armed_at: Optional[float] = None
        self.light_armed_at: Optional[float] = None
        self.stopped_at = 0.0
        self.cooldown_until = {'pedestrian': -1.0, 'light': -1.0}

        # Bump internal state
        self.bump_state = BUMP_IDLE
        self.bump_armed_at = -1.0
        self.bump_level_started_at: Optional[float] = None
        self.bump_cooldown_until = -1.0
        self.bump_entry_sign = 0.
        self.bump_opposite_seen = False

    def step(self, inputs: Inputs) -> Outputs:
        now, seen, p = inputs.now, inputs.seen, self.p

        if 'sign_slow' in seen:
            self.slow_until = now + p.slow_hold_sec
        self._update_fork(inputs)
        if 'sign_parking' in seen and p.parking_enabled:
            self.parking_seen_at = now
        self.parking_armed = (p.parking_enabled and self.parking_seen_at is not None and
                              now - self.parking_seen_at <= p.arm_timeout_sec and
                              not self.parking_flag and not inputs.parking_done)
        parking_entry = (inputs.parking_line and inputs.parking_scene if p.require_scene_confirmation
                         else inputs.parking_line or not p.parking_at_stopline)
        if self.parking_armed and parking_entry and self.state == DRIVE:
            self.parking_flag = True

        self._update_stop(inputs)
        self._update_bump(inputs)

        limits = []
        bump_active = self.bump_state in (BUMP_APPROACHING, BUMP_ON)
        if now < self.slow_until or bump_active:
            limits.append(p.slow_speed)
        arm = p.use_stop_line and self.state == DRIVE and self._armed()
        if arm and (not p.require_scene_confirmation or inputs.line_phase == 'commit'):
            limits.append(inputs.line_speed if inputs.line_speed is not None else p.approach_speed)
        return Outputs(
            state=self.state,
            stop=self.state != DRIVE,
            speed_limit=min(limits) if limits else None,
            direction=self.direction,
            parking_flag=self.parking_flag,
            stop_arm=arm,
            parking_prepare=self.parking_armed and not self.parking_flag,
            # Keep the slow limit on unknown attitude, but let lane following
            # steer again if IMU updates disappear during a crossing.
            steering_lock=(self.bump_state == BUMP_ON and inputs.imu_pitch is not None and
                           math.isfinite(inputs.imu_pitch)))

    def _update_fork(self, inputs: Inputs) -> None:
        seen, p = inputs.seen, self.p
        if 'sign_left' in seen and 'sign_right' not in seen:
            self.direction = p.fork_left_direction
        elif 'sign_right' in seen and 'sign_left' not in seen:
            self.direction = p.fork_right_direction

    def _update_bump(self, inputs: Inputs) -> None:
        now, seen, pitch, p = inputs.now, inputs.seen, inputs.imu_pitch, self.p

        # Both official warning forms can prepare a slow traversal.
        if 'sign_bump' in seen or (p.slow_arms_bump and 'sign_slow' in seen):
            if self.bump_state == BUMP_COOLDOWN and now >= self.bump_cooldown_until:
                self.bump_state = BUMP_IDLE
            if self.bump_state == BUMP_IDLE:
                self.bump_state = BUMP_APPROACHING
                self.bump_level_started_at = None
            if self.bump_state == BUMP_APPROACHING:
                # A still-visible confirmed sign is fresh approach evidence.
                self.bump_armed_at = now

        # 2. State transitions using IMU pitch and timers
        if self.bump_state == BUMP_COOLDOWN:
            if now >= self.bump_cooldown_until:
                self.bump_state = BUMP_IDLE

        elif self.bump_state == BUMP_APPROACHING:
            # Expire if bump not hit within timeout
            if now - self.bump_armed_at > p.bump_arm_timeout_sec:
                self.bump_state = BUMP_IDLE
            elif pitch is not None and abs(pitch) >= p.bump_pitch_thresh:
                # Front wheels climbing bump
                self.bump_state = BUMP_ON
                self.bump_level_started_at = None
                self.bump_entry_sign = math.copysign(1., pitch)
                self.bump_opposite_seen = False

        elif self.bump_state == BUMP_ON:
            if pitch is not None and pitch*self.bump_entry_sign < -p.bump_level_thresh:
                self.bump_opposite_seen = True
            # The flat interval between front and rear axles is not the exit.
            if self.bump_opposite_seen and pitch is not None and abs(pitch) <= p.bump_level_thresh:
                if self.bump_level_started_at is None:
                    self.bump_level_started_at = now
                elif now - self.bump_level_started_at >= p.bump_level_duration_sec:
                    # Vehicle leveled out -> Bump fully cleared!
                    self.bump_state = BUMP_COOLDOWN
                    self.bump_cooldown_until = now + p.bump_cooldown_sec
                    self.bump_level_started_at = None
            else:
                self.bump_level_started_at = None

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
                 fork_max_offset_height_ratio: float = 10.0,
                 min_sizes: Optional[Dict[str, Tuple[float, float]]] = None,
                 bump_min_area: float = 0.0,
                 bump_min_height: float = 0.0,
                 bump_max_offset_height_ratio: Optional[float] = None) -> Set[str]:
    """Classes used by the mission, excluding boxes far to the side or smaller than thresholds.

    abs(box centre x - camera principal x) / box height is a distance-normalised
    horizontal offset. On the start straight, the adjacent track's red light was
    5.3; on our signal approach it was 2.9. Signs need a separate limit: the valid
    bump/pedestrian signs reached 3.9, the adjacent parking sign was over 9, and
    the fork sign reached 9.4 while approaching from the outer lane.
    A configured limit <= 0 disables that group's filter for recorded comparisons.
    min_sizes maps class_name -> (min_area, min_height).
    bump_max_offset_height_ratio overrides only the bump sign's lateral limit:
    None inherits the shared sign limit; <= 0 disables it for roadside bump signs.
    """
    filters = dict(min_sizes or {})
    if bump_min_area > 0 or bump_min_height > 0:
        filters.setdefault('sign_bump', (bump_min_area, bump_min_height))

    seen = set()
    for d in detections:
        if isinstance(d, str):
            seen.add(d)
            continue

        if not hasattr(d, 'class_name'):
            continue

        name = d.class_name

        # 1. BBox size filtering (min_area, min_height)
        if name in filters:
            min_area, min_h = filters[name]
            w = max(0, getattr(d, 'x_max', 0) - getattr(d, 'x_min', 0))
            h = max(0, getattr(d, 'y_max', 0) - getattr(d, 'y_min', 0))
            area = float(w * h)
            if area < min_area or h < min_h:
                continue

        # 2. Horizontal offset ratio filtering (exclude adjacent track signs)
        if name == 'sign_bump' and bump_max_offset_height_ratio is not None:
            limit = bump_max_offset_height_ratio
        elif name.startswith('traffic_light_'):
            limit = light_max_offset_height_ratio
        elif name in ('sign_left', 'sign_right'):
            limit = fork_max_offset_height_ratio
        elif name.startswith('sign_'):
            limit = sign_max_offset_height_ratio
        else:
            limit = 0.0
        if limit > 0:
            height = getattr(d, 'y_max', 0) - getattr(d, 'y_min', 0)
            width = getattr(d, 'x_max', 0) - getattr(d, 'x_min', 0)
            if height <= 0 or width <= 0:
                continue
            offset = abs((d.x_min + d.x_max) / 2.0 - camera_center_x)
            if offset > limit * height:
                continue
        seen.add(name)
    return seen
