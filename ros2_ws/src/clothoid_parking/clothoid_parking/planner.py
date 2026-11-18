"""Nose-in T-parking in one forward arc.

    SEARCH   drive straight past the bay row (gyro heading hold) until the empty bay is known and
             the rear axle is one turning radius before the bay centre
    TURN     full lock towards the bays until the heading is 90 deg from the row
    ENTER    straight in or back (heading hold) until base_link sits at the bay centre depth
    PARKED   zero speed for `park_hold_sec` (judging)
    EXIT_BACK   straight back out along the bay (heading hold) to where the arc ended
    EXIT_TURN   back up at the same full lock, retracing the arc into the lane, until the heading is
                along the row again
    DONE     no command any more: the node stops publishing, twist_mux falls back to lane, and
             /parking_done goes out (with exit_enabled false DONE comes straight after PARKED)
    ABORT    zero speed for good: no bay found, it was already passed, or a phase took too long

Geometry (2026 track, limo2 2026-11-08): full lock rear-axle radius 0.46 m both sides, the lane
centre runs 0.27 m from the bay mouth, so the arc from the lane ends near the bay centre and
ENTER only corrects a few cm. Along the row there is about +-12 cm of room (500 mm bay, 220 mm
car), so the turn point is what decides the score.
"""
import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

from clothoid_parking.bay_finder import Bay
from clothoid_parking.tracker import Pose2D


class Phase(Enum):
    IDLE = 'idle'
    SEARCH = 'search'
    TURN = 'turn'
    ENTER = 'enter'
    PARKED = 'parked'
    EXIT_BACK = 'exit_back'
    EXIT_TURN = 'exit_turn'
    DONE = 'done'
    ABORT = 'abort'


@dataclass
class PlanParams:
    side: float = -1.0             # -1: bays on the right
    entry_heading_compensation: bool = False  # account for residual approach yaw in turn position
    # Dead-reckoning point: the point with no sideways speed, `rear_offset` behind base_link. The rear
    # axle is 0.1 behind, but at full lock limo2's rear slides out: 5 parking bags (2026-11-09) checked
    # against the lidar fit 0.02 behind with odom speed x0.98 (4 mm rms, 11.6 cm with 0.1 / x1.0).
    # simul/ moves base_link as if it were the rear axle and reads odom 1.5 % long: 0.0 and 0.985 there.
    rear_offset: float = 0.02      # m
    odom_scale: float = 0.98       # true speed / /odom speed
    turn_radius: float = 0.45      # m, that point at full lock: 0.46 (wheel speed / gyro, 2026-11-08) x 0.98
    search_speed: float = 0.2
    turn_speed: float = 0.2
    enter_speed: float = 0.1
    min_speed: float = 0.06        # slower than this the LIMO barely moves
    turn_lead: float = 0.03        # m, start turning this early: the wheels need ~0.2 s to reach lock
    turn_early_deg: float = 6.0    # stop the turn this short of 90 deg: the car keeps turning while the
                                   # steering slews back (3 left limo2 3-4 deg over, 2026-11-09)
    heading_gain: float = 2.0      # rad of steering per rad of heading error
    max_steer: float = 0.41        # rad, bicycle angle at full lock (23.3 deg)
    full_lock: float = 0.6         # rad, past the limit on purpose: limo_base clips it to full lock
    depth_tolerance: float = 0.02  # m
    max_search: float = 3.0        # m driven in SEARCH without a usable bay -> ABORT
    freeze_distance: float = 0.20  # m before the turn point: stop moving the target
    max_reversals: int = 2         # ENTER direction changes before accepting where it is
    turn_max_sec: float = 8.0
    enter_max_sec: float = 12.0    # a second-row bay is 0.6 m further in
    front_stop: float = 0.10       # m, lidar front clearance that ends a forward ENTER
    park_hold_sec: float = 3.0     # s standing in the bay before backing out
    strict_completion: bool = False  # registered layout: timeout/obstacle is not successful parking
    exit_enabled: bool = True
    exit_speed: float = 0.15
    exit_early_deg: float = 2.0    # end the reverse arc this short of the row heading (it stops right
                                   # there, unlike the turn in, which rolls on into ENTER)
    exit_max_sec: float = 12.0


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def arc_forward_span(pose, bay, plan):
    delta = wrap(pose.yaw-bay.row_yaw) if plan.entry_heading_compensation else 0.
    return plan.turn_radius*(1-plan.side*math.sin(delta))


def turn_remaining(bay, pose, plan):
    rx = pose.x-plan.rear_offset*math.cos(pose.yaw)
    ry = pose.y-plan.rear_offset*math.sin(pose.yaw)
    along = (bay.face_x-rx)*math.cos(bay.row_yaw)+(bay.face_y-ry)*math.sin(bay.row_yaw)
    return along-arc_forward_span(pose, bay, plan)-plan.turn_lead


class Planner:
    def __init__(self, params: PlanParams = PlanParams()):
        self.p = params
        self.phase = Phase.IDLE
        self.bay: Optional[Bay] = None
        self.reason = ''
        self._start = Pose2D()
        self._t_phase = 0.0
        self._enter_dir = 0.0
        self._reversals = 0
        self._arc_end: Optional[Pose2D] = None
        self.row_hint: Optional[float] = None   # row direction from the lidar before a bay is found

    @property
    def active(self) -> bool:
        return self.phase not in (Phase.IDLE,)

    def start(self, now: float, pose: Pose2D):
        self._start = Pose2D(pose.x, pose.y, pose.yaw)
        self.bay = None
        self._enter_dir, self._reversals, self._arc_end = 0.0, 0, None
        self.row_hint = None
        self._go(Phase.SEARCH, now, 'flag')

    def _go(self, phase: Phase, now: float, reason: str):
        self.phase, self._t_phase, self.reason = phase, now, reason

    def frozen(self, pose: Pose2D) -> bool:
        return self.phase != Phase.SEARCH or (
            self.bay is not None and self.turn_remaining(pose) < self.p.freeze_distance)

    def offer_bay(self, bay: Optional[Bay], pose: Pose2D) -> bool:
        """New estimate from the bay finder; ignored once the target is frozen."""
        if bay is None or self.frozen(pose):
            return False
        self.bay = bay
        return True

    def turn_remaining(self, pose: Pose2D) -> float:
        """m the rear axle still has to go along the row before turning."""
        return turn_remaining(self.bay, pose, self.p)

    def _hold(self, pose: Pose2D, yaw_ref: float, v: float) -> float:
        steer = self.p.heading_gain * wrap(yaw_ref - pose.yaw)
        steer = max(-self.p.max_steer, min(self.p.max_steer, steer))
        return steer if v >= 0 else -steer

    def step(self, now: float, pose: Pose2D, front_range: Optional[float] = None
             ) -> Optional[Tuple[float, float]]:
        """(speed m/s, bicycle steering rad, + left), or None while idle."""
        p = self.p
        if self.phase == Phase.IDLE:
            return None
        if self.phase == Phase.DONE:
            return None
        if self.phase == Phase.ABORT:
            return 0.0, 0.0
        if self.phase == Phase.PARKED:
            if now - self._t_phase < p.park_hold_sec:
                return 0.0, 0.0
            if not p.exit_enabled:
                self._go(Phase.DONE, now, 'parked, no exit')
                return None
            self._go(Phase.EXIT_BACK, now, 'backing out')
        if self.phase == Phase.EXIT_BACK:
            b, a = self.bay, self._arc_end
            # how far base_link still is deeper into the bay than where the arc ended
            deeper = (pose.x - a.x) * math.cos(b.entry_yaw) + (pose.y - a.y) * math.sin(b.entry_yaw)
            if deeper <= 0.0:
                self._go(Phase.EXIT_TURN, now, f'back at the arc end ({deeper:+.3f} m)')
            elif now - self._t_phase > p.exit_max_sec:
                self._go(Phase.ABORT, now, 'backing out took too long')
                return 0.0, 0.0
            else:
                v = -max(p.min_speed, min(p.exit_speed, 1.5 * deeper + 0.03))
                return v, self._hold(pose, b.entry_yaw, v)
        if self.phase == Phase.EXIT_TURN:
            left = wrap(pose.yaw - self.bay.row_yaw) * p.side     # > 0 while still turned into the bay
            if left <= math.radians(p.exit_early_deg):
                self._go(Phase.DONE, now, f'out, {math.degrees(left):+.1f} deg to the row heading')
                return None
            if now - self._t_phase > p.exit_max_sec:
                self._go(Phase.ABORT, now, 'reverse arc took too long')
                return 0.0, 0.0
            # same steering as the way in: backing up retraces the arc
            return -p.exit_speed, p.side * p.full_lock

        if self.phase == Phase.SEARCH:
            if self.bay is None:
                driven = math.hypot(pose.x - self._start.x, pose.y - self._start.y)
                if driven > p.max_search:
                    self._go(Phase.ABORT, now, f'no empty bay within {p.max_search:.1f} m')
                    return 0.0, 0.0
                yaw_ref = self._start.yaw if self.row_hint is None else self.row_hint
                return p.search_speed, self._hold(pose, yaw_ref, p.search_speed)
            remaining = self.turn_remaining(pose)
            if remaining <= 0.0:
                if remaining < -0.10:
                    self._go(Phase.ABORT, now, f'turn point passed by {-remaining:.2f} m')
                    return 0.0, 0.0
                self._go(Phase.TURN, now, f'turn at {remaining:+.3f} m')
            else:
                return p.search_speed, self._hold(pose, self.bay.row_yaw, p.search_speed)

        if self.phase == Phase.TURN:
            left = wrap(self.bay.entry_yaw - pose.yaw) * p.side   # > 0 while still turning
            if left <= math.radians(p.turn_early_deg):
                self._arc_end = Pose2D(pose.x, pose.y, pose.yaw)
                self._go(Phase.ENTER, now, f'turned, {math.degrees(left):+.1f} deg left')
            elif now - self._t_phase > p.turn_max_sec:
                self._go(Phase.ABORT, now, 'turn took too long')
                return 0.0, 0.0
            else:
                return p.turn_speed, p.side * p.full_lock

        if self.phase == Phase.ENTER:
            b = self.bay
            depth = (b.x - pose.x) * math.cos(b.entry_yaw) + (b.y - pose.y) * math.sin(b.entry_yaw)
            if abs(depth) <= p.depth_tolerance:
                self._go(Phase.PARKED, now, f'depth error {depth:+.3f} m')
                return 0.0, 0.0
            direction = 1.0 if depth > 0 else -1.0
            if self._enter_dir and direction != self._enter_dir:
                self._reversals += 1
            self._enter_dir = direction
            if self._reversals > p.max_reversals or now - self._t_phase > p.enter_max_sec:
                self._go(Phase.ABORT if p.strict_completion else Phase.PARKED, now, f'stopped at depth error {depth:+.3f} m')
                return 0.0, 0.0
            if direction > 0 and front_range is not None and front_range < p.front_stop:
                self._go(Phase.ABORT if p.strict_completion else Phase.PARKED, now, f'front {front_range:.2f} m, depth error {depth:+.3f} m')
                return 0.0, 0.0
            v = direction * max(p.min_speed, min(p.enter_speed, 1.5 * abs(depth)))
            return v, self._hold(pose, b.entry_yaw, v)
        return 0.0, 0.0
