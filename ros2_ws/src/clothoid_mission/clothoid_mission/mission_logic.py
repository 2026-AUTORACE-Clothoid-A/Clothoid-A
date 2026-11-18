"""Mission state logic: turns perception into the signals the driving nodes listen to.

Inputs each tick: YOLO detections, "stop line ahead", "parking finished", IMU pitch.
Outputs: `stop` (state node holds the car), `speed_limit`, `direction`, `parking_flag`, `steering_lock`.

  * slow sign        -> optionally brake before the mission start line, then hold after entry;
                        without `slow_at_stopline`, hold after the last sign sighting
  * bump sign + IMU  -> optionally brake before the mission start line before approaching slowly;
                        IMU contact also starts slowing, leveling ends it, and climb locks steering
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
  * require_scene_confirmation false allows parking's legacy sign/stop-line trigger for comparisons
Cone course entry gates parking preparation and blocks later bump triggers;
parking still needs its local entry and bay observations.
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
                 sign_height=14, light_height=8, edge_margin=2, class_hits=None, class_max_age=None,
                 edge_exempt=(), class_confidence=None, any_x=()):
        self.hits, self.window, self.max_age = hits, window, max_age
        # per-class overrides, e.g. fork signs: small, seen between the static obstacles, no other
        # fork sign around. class_max_age classes count every frame within their age (not only the
        # last `window`); edge_exempt classes keep boxes touching the image border
        self.class_hits = dict(class_hits or {})
        self.class_max_age = dict(class_max_age or {})
        self.edge_exempt = frozenset(edge_exempt)
        self.class_confidence = dict(class_confidence or {})
        # any_x classes match across frames wherever they are across the image (a fork sign sweeps
        # 80-100 px per 0.5 s while the car swerves; its box is ~20 px wide); y and size still match
        self.any_x = frozenset(any_x)
        self.confidence, self.sign_height = confidence, sign_height
        self.light_height, self.edge_margin = light_height, edge_margin
        self.history = deque()

    def matches(self, a, b):
        # Allow detector jitter and the motion of a sign during corner entry.
        aw, ah = a.x_max-a.x_min, a.y_max-a.y_min
        bw, bh = b.x_max-b.x_min, b.y_max-b.y_min
        return (a.class_name == b.class_name and
                (a.class_name in self.any_x or abs((a.x_min+a.x_max-b.x_min-b.x_max)/2) <= max(aw, bw)) and
                abs((a.y_min+a.y_max-b.y_min-b.y_max)/2) <= max(ah, bh) and
                0.5 <= ah / bh <= 2.0)

    def required(self, name: str) -> int:
        return self.class_hits.get(name, self.hits)

    def age(self, name: str) -> float:
        return self.class_max_age.get(name, self.max_age)

    def box_reason(self, d, width=640, height=480) -> Optional[str]:
        """Why the box itself cannot count (None: it can): conf, height or edge."""
        minimum = self.light_height if d.class_name.startswith('traffic_light_') else self.sign_height
        confidence = self.class_confidence.get(d.class_name, self.confidence)
        if d.confidence < confidence:
            return 'conf %.2f < %.2f' % (d.confidence, confidence)
        if d.y_max-d.y_min < minimum or d.x_max <= d.x_min:
            return 'h %d < %d' % (d.y_max-d.y_min, minimum)
        m = self.edge_margin
        on_edge = d.x_min < m or d.y_min < m or d.x_max > width-m or d.y_max > height-m
        if on_edge and d.class_name not in self.edge_exempt:
            return 'edge'
        return None

    def count(self, d, now) -> int:
        """Recent frames (within the class's age, this one included) holding a box matching d."""
        frames = self.history
        if d.class_name not in self.class_max_age:
            frames = list(frames)[-self.window:]
        age = self.age(d.class_name)
        return sum(any(self.matches(d, old) for old in frame)
                   for stamp, frame in frames if 0 <= now-stamp <= age)

    def update(self, now, detections, accepted, width=640, height=480):
        if self.history and now < self.history[-1][0]:
            self.history.clear()
        boxes = [d for d in detections
                 if d.class_name in accepted and self.box_reason(d, width, height) is None]
        self.history.append((now, boxes))
        keep = max([self.max_age] + list(self.class_max_age.values()))
        while len(self.history) > self.window and now - self.history[0][0] > keep:
            self.history.popleft()
        return {d.class_name for d in boxes if self.count(d, now) >= self.required(d.class_name)}


@dataclass
class MissionParams:
    # slow sign & bump speed
    slow_speed: float = 0.2            # rules: 0.2 m/s or less
    slow_hold_sec: float = 4.0         # general slow sign hold timer
    slow_at_stopline: bool = False     # legacy key: slow before the separate mission entry line
    # a pedestrian sign confirmed while the slow sign's hold runs keeps slow_speed until that stop is
    # released (1118: the zigzag zone's slow sign stands at its start, the crosswalk stop ~2 m later)
    slow_until_pedestrian_stop: bool = False
    # speed bump (IMU Pitch + YOLO fusion)
    bump_pitch_thresh: float = 3.5      # deg, absolute pitch to detect contact
    bump_level_thresh: float = 1.2      # deg, abs angle to detect vehicle leveled
    bump_level_duration_sec: float = 0.3  # sec, continuous leveling to confirm exit
    bump_arm_timeout_sec: float = 12.0  # sec since last confirmed sign; tune to approach distance/speed
    bump_cooldown_sec: float = 4.0      # sec, cooldown before re-arming bump
    bump_at_stopline: bool = False      # legacy key: slow before the separate mission entry line
    # hill / bump from the IMU alone (no sign: the 1118 PM rules replaced the start's bump sign with a
    # slow sign): slow_speed from |pitch| >= bump_pitch_thresh until the car is level again after the
    # opposite tilt (as the bump exit), at most hill_max_sec after the last tilt. Speed only, no
    # steering lock: nothing but the IMU says it is a bump.
    hill_slow_imu: bool = False
    hill_max_sec: float = 6.0
    # first slow zone (the start's hill, 1118 PM): lane_main holds the IMU yaw from the first slow sign
    # until the hill is over (hill_active rose and fell), at most yaw_hold_max_sec. The hill's yellow
    # paint pulled lane_main's target sideways.
    first_slow_yaw_hold: bool = False
    yaw_hold_max_sec: float = 10.0
    # odometry metres after the first slow sign before the hold starts: the heading is steady until
    # just before the hill and swings 10+ deg where its paint comes into view (1118 PM: sign at
    # 1.0~1.1 m, swing at 2.5 m, hill 2.6~2.8 m). Without odometry the hold starts at the sign.
    yaw_hold_after_m: float = 1.0
    # stop, wait, go
    pedestrian_wait_sec: float = 3.5   # rules: wait 3 s or more, restart within 15 s
    stop_on_yellow: bool = True        # the rules do not say what yellow means
    arm_timeout_sec: float = 8.0       # a sign/light seen this long ago without a stop line expires
    cooldown_sec: float = 15.0         # ignore the same trigger after a stop
    use_stop_line: bool = True         # stop with FinishCore (see the docstring)
    approach_speed: float = 0.2        # m/s target at the parking line; also the line stop fallback
    # fork (A/B): lane side to follow when the sign points left / right
    fork_left_direction: int = 0       # lane_main: 0 left lane, 1 right lane, 2 both
    fork_right_direction: int = 1
    # parking: the sign is seen from afar, the stop line before the bays is where the search starts
    parking_at_stopline: bool = True
    require_scene_confirmation: bool = True
    parking_enabled: bool = True
    slow_arms_bump: bool = False
    # Main route policy: latch cone entry before accepting parking signs.
    require_cone_for_parking: bool = True
    block_bump_after_cone: bool = True
    cone_confirm_sec: float = 1.5      # consecutive seconds of cone_active before latching cone_entered
    # cone_active does not count while static_avoid drives and this long after (1114_0028: a false cone
    # start during the avoidance latched cone_entered); same hold as cone_sorter avoid_release_hold
    avoid_release_hold: float = 2.0


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
    cone_active: bool = False                     # /cone/mission_active from cone_sorter
    parking_scene: bool = False    # fresh, repeated lidar bay observations
    parking_observing: bool = False # bounded approach observation, not permission to turn
    parking_skipped: bool = False   # observation failed; never means parking_done
    avoid_active: bool = False                    # /avoid/active from static_avoid (fresh)
    entry_slow: bool = False       # braking point before the separate mission boundary
    entry_reached: bool = False    # front wheels reached that boundary (not a stop request)
    travel: Optional[float] = None  # odometry metres (fresh), for distance based holds
    parking_speed_limit: Optional[float] = None  # braking envelope from the measured entry line


@dataclass
class Outputs:
    state: str = DRIVE
    stop: bool = False
    speed_limit: Optional[float] = None
    direction: Optional[int] = None    # None until a fork sign was seen
    parking_flag: bool = False
    stop_arm: bool = False             # run the line stop (FinishCore)
    stop_require_crosswalk: bool = True
    steering_lock: bool = False        # lock steering during bump traversal
    yaw_hold: bool = False             # lane_main keeps the IMU yaw (first slow zone, see MissionParams)
    light_stopped: bool = False        # latched: the car stopped at the traffic light once
    parking_prepare: bool = False     # observe the parking scene, keep lane control
    entry_arm: bool = False           # observe the mission boundary, independently of the stop line


class MissionLogic:
    def __init__(self, params: MissionParams):
        self.p = params
        self.state = DRIVE
        self.slow_until = -1.0
        self.slow_armed_at: Optional[float] = None
        self.slow_for_pedestrian = False
        self.direction: Optional[int] = None
        self.parking_flag = False
        self.parking_armed = False
        self.parking_seen_at = None
        self.cone_entered = False
        self.cone_active_since: Optional[float] = None
        self.parking_skip_until = -math.inf
        self.avoid_until = -1.0                     # cone_active ignored until then
        self.pedestrian_armed_at: Optional[float] = None
        self.light_armed_at: Optional[float] = None
        self.stopped_at = 0.0
        self.cooldown_until = {'pedestrian': -1.0, 'light': -1.0}

        # Bump internal state
        self.bump_state = BUMP_IDLE
        self.bump_armed_at = -1.0
        self.bump_line_armed_at: Optional[float] = None
        self.bump_level_started_at: Optional[float] = None
        self.bump_cooldown_until = -1.0
        self.bump_entry_sign = 0.
        self.bump_opposite_seen = False

        # IMU-only hill slow (hill_slow_imu)
        self.hill_active = False
        self.hill_tilt_at = -math.inf
        self.hill_entry_sign = 0.
        self.hill_opposite_seen = False
        self.hill_level_since: Optional[float] = None

        # first slow zone yaw hold (first_slow_yaw_hold)
        self.yaw_hold_since: Optional[float] = None
        self.yaw_hold_travel: Optional[float] = None
        self.yaw_hold_hill_seen = False
        self.yaw_hold_done = False
        self.light_stopped = False

    def _line_arrived(self, inputs: Inputs) -> bool:
        return inputs.line_phase == 'arrived' if self.p.use_stop_line else inputs.stopline

    def step(self, inputs: Inputs) -> Outputs:
        now, seen, p = inputs.now, inputs.seen, self.p

        if inputs.avoid_active:
            self.avoid_until = now + p.avoid_release_hold
        if inputs.cone_active and now >= self.avoid_until:
            if self.cone_active_since is None:
                self.cone_active_since = now
            if now-self.cone_active_since >= p.cone_confirm_sec:
                self.cone_entered = True
        else:
            self.cone_active_since = None

        if p.slow_at_stopline:
            if 'sign_slow' in seen and now >= self.slow_until:
                self.slow_armed_at = now
            if self.slow_armed_at is not None:
                if inputs.entry_reached:
                    self.slow_until = now + p.slow_hold_sec
                    self.slow_armed_at = None
                elif not inputs.entry_slow and now - self.slow_armed_at > p.arm_timeout_sec:
                    self.slow_armed_at = None
        elif 'sign_slow' in seen:
            self.slow_until = now + p.slow_hold_sec
        self._update_fork(inputs)
        if inputs.parking_skipped and not self.parking_flag:
            self.parking_seen_at = None
            self.parking_skip_until = now+p.cooldown_sec
        if ('sign_parking' in seen and p.parking_enabled and now >= self.parking_skip_until
                and (self.cone_entered or not p.require_cone_for_parking)):
            self.parking_seen_at = now
        self.parking_armed = (p.parking_enabled and self.parking_seen_at is not None and
                              (now - self.parking_seen_at <= p.arm_timeout_sec or
                               (self.parking_armed and inputs.parking_observing)) and
                              not self.parking_flag and not inputs.parking_done)
        parking_line_hit = inputs.parking_line or (p.use_stop_line and self._line_arrived(inputs))
        parking_entry = (inputs.parking_line and inputs.parking_scene if p.require_scene_confirmation
                         else parking_line_hit or not p.parking_at_stopline)
        if self.parking_armed and parking_entry and self.state == DRIVE:
            self.parking_flag = True

        self._update_stop(inputs)
        self._update_bump(inputs)
        self._update_hill(inputs)
        self._update_yaw_hold(inputs)
        if self.pedestrian_armed_at is None or self.state != DRIVE:
            self.slow_for_pedestrian = False      # stopped at the crosswalk (or the arm expired): done
        elif p.slow_until_pedestrian_stop and now < self.slow_until:
            self.slow_for_pedestrian = True

        limits = []
        bump_active = self.bump_state in (BUMP_APPROACHING, BUMP_ON)
        if now < self.slow_until or bump_active or self.hill_active or self.slow_for_pedestrian:
            limits.append(p.slow_speed)
        if self.slow_armed_at is not None and inputs.entry_slow:
            limits.append(p.slow_speed)
        parking_prepare = self.parking_armed and not self.parking_flag
        if p.require_scene_confirmation and parking_prepare and inputs.parking_speed_limit is not None:
            limits.append(inputs.parking_speed_limit)
        arm = p.use_stop_line and self.state == DRIVE and self._armed()
        parking_line_arm = p.use_stop_line and self.state == DRIVE and (
            not p.require_scene_confirmation and self.parking_armed and not self.parking_flag
            and p.parking_at_stopline)
        # Keep entry slowing separate from the final crosswalk stop. Its shorter
        # braking lead avoids crawling from a distant mission-start observation.
        if arm and (not p.require_scene_confirmation or inputs.entry_slow or inputs.line_phase == 'commit'):
            limits.append(inputs.line_speed if inputs.line_speed is not None else p.approach_speed)
        return Outputs(
            state=self.state,
            stop=self.state != DRIVE,
            speed_limit=min(limits) if limits else None,
            direction=self.direction,
            parking_flag=self.parking_flag,
            stop_arm=arm or parking_line_arm,
            stop_require_crosswalk=arm,
            entry_arm=p.use_stop_line and self.state == DRIVE and (
                arm or self.slow_armed_at is not None or self.bump_line_armed_at is not None),
            parking_prepare=parking_prepare,
            # Keep the slow limit on unknown attitude, but let lane following
            # steer again if IMU updates disappear during a crossing.
            steering_lock=(self.bump_state == BUMP_ON and inputs.imu_pitch is not None and
                           math.isfinite(inputs.imu_pitch)),
            yaw_hold=self._yaw_hold_on(inputs),
            light_stopped=self.light_stopped)

    def _yaw_hold_on(self, inputs: Inputs) -> bool:
        if self.yaw_hold_since is None or self.yaw_hold_done:
            return False
        if self.yaw_hold_travel is None or inputs.travel is None:
            return True                 # no odometry: from the sign on
        return inputs.travel - self.yaw_hold_travel >= self.p.yaw_hold_after_m

    def _update_yaw_hold(self, inputs: Inputs) -> None:
        """first_slow_yaw_hold: from the first slow sign until the hill rose and fell (or the cap)."""
        p, now = self.p, inputs.now
        if not p.first_slow_yaw_hold or self.yaw_hold_done:
            return
        if self.yaw_hold_since is None:
            if 'sign_slow' in inputs.seen:
                self.yaw_hold_since, self.yaw_hold_travel = now, inputs.travel
            return
        if self.hill_active:
            self.yaw_hold_hill_seen = True
        elif self.yaw_hold_hill_seen or now - self.yaw_hold_since > p.yaw_hold_max_sec:
            self.yaw_hold_done = True

    def _update_fork(self, inputs: Inputs) -> None:
        seen, p = inputs.seen, self.p
        if 'sign_left' in seen and 'sign_right' not in seen:
            self.direction = p.fork_left_direction
        elif 'sign_right' in seen and 'sign_left' not in seen:
            self.direction = p.fork_right_direction

    def _update_bump(self, inputs: Inputs) -> None:
        now, seen, pitch, p = inputs.now, inputs.seen, inputs.imu_pitch, self.p
        bump_blocked = self.cone_entered and p.block_bump_after_cone
        bump_seen = 'sign_bump' in seen or (p.slow_arms_bump and 'sign_slow' in seen)

        # Select the configured trigger even on the first tick after cooldown.
        if self.bump_state == BUMP_COOLDOWN and now >= self.bump_cooldown_until:
            self.bump_state = BUMP_IDLE

        # 1. YOLO trigger
        if bump_blocked:
            self.bump_line_armed_at = None
            if self.bump_state == BUMP_APPROACHING:
                self.bump_state = BUMP_IDLE
        elif p.bump_at_stopline and self.bump_state == BUMP_IDLE:
            if bump_seen:
                self.bump_line_armed_at = now
            if self.bump_line_armed_at is not None:
                if pitch is not None and abs(pitch) >= p.bump_pitch_thresh:
                    self.bump_line_armed_at = None
                    self.bump_state = BUMP_ON
                    self.bump_level_started_at = None
                    self.bump_entry_sign = math.copysign(1., pitch)
                    self.bump_opposite_seen = False
                elif inputs.entry_slow or inputs.entry_reached:
                    self.bump_line_armed_at = None
                    self.bump_state = BUMP_APPROACHING
                    self.bump_level_started_at = None
                    self.bump_armed_at = now
                elif now - self.bump_line_armed_at > p.bump_arm_timeout_sec:
                    self.bump_line_armed_at = None
        elif bump_seen:
            if self.bump_state == BUMP_IDLE:
                self.bump_state = BUMP_APPROACHING
                self.bump_level_started_at = None
            if self.bump_state == BUMP_APPROACHING:
                # A still-visible confirmed sign is fresh approach evidence.
                self.bump_armed_at = now

        # 2. State transitions using IMU pitch and timers
        if self.bump_state == BUMP_APPROACHING:
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

    def _update_hill(self, inputs: Inputs) -> None:
        """Slow on a tilt the IMU alone sees (hill_slow_imu); see MissionParams."""
        p, now, pitch = self.p, inputs.now, inputs.imu_pitch
        if not p.hill_slow_imu:
            self.hill_active = False
            return
        valid = pitch is not None and math.isfinite(pitch)
        if valid and abs(pitch) >= p.bump_pitch_thresh:
            if not self.hill_active:
                self.hill_active = True
                self.hill_entry_sign = math.copysign(1., pitch)
                self.hill_opposite_seen = False
            elif pitch*self.hill_entry_sign < 0:
                self.hill_opposite_seen = True
            self.hill_tilt_at = now
            self.hill_level_since = None
            return
        if not self.hill_active:
            return
        if valid and pitch*self.hill_entry_sign < -p.bump_level_thresh:
            self.hill_opposite_seen = True
        if now - self.hill_tilt_at > p.hill_max_sec:          # a ramp only, or the IMU went silent
            self.hill_active = False
        elif self.hill_opposite_seen and valid and abs(pitch) <= p.bump_level_thresh:
            if self.hill_level_since is None:
                self.hill_level_since = now
            elif now - self.hill_level_since >= p.bump_level_duration_sec:
                self.hill_active = False
        else:
            self.hill_level_since = None
        if not self.hill_active:
            self.hill_level_since = None

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
        if state == STOPPED_LIGHT:
            self.light_stopped = True

    def _release(self, which: str, now: float) -> None:
        self.state = DRIVE
        self.cooldown_until[which] = now + self.p.cooldown_sec
        # the stop ends that zone's slow sign: go at lane speed after the crosswalk / on green
        self.slow_until = min(self.slow_until, now)
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
