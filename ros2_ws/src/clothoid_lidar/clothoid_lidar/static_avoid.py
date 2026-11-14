"""Static obstacle mission (3-lane section, parked LIMOs) and the A/B lane entry after it. ROS-free.

Design: docs/STATIC_AVOID.md. In short
  * trigger, decided on the spot (no sequence state): a LIMO-sized cluster blocks our lane ahead for
    a few scans, the cone course is not on (its flag, camera red share), no other mission holds the
    car (parking flag, mission stop commands) and the car is moving.
  * on activation a section frame is fixed: s along the heading at that moment (IMU yaw), d to the
    left, d = 0 at the centre of lane 2 where the section is entered. Lane centres d = +w (lane 1, A,
    left), 0 (lane 2), -w (lane 3, B, right); the yellow outer lines at +-1.5 w; white dashes between.
  * pose from /odom speed and /imu yaw (the /odom yaw is not usable, see ODOM_STEERING_TEST.md).
  * a light costmap in the section frame (hit/clear counts from the whole scan, occlusion, inflation)
    and ~100 candidate local paths (end offset x transition length) scored on it, with lane centre
    attraction, hysteresis and the target lane from /direction_flag. Pure pursuit follows the best.
  * hand-over to lane_main once the car is centred and straight in the target lane with that lane
    clear ahead; Twist(0) once, then silence.
"""
import bisect
import math
from dataclasses import dataclass, field, replace
from typing import List, Optional, Tuple

import numpy as np

from clothoid_lidar.lane_sequence import Sequence, plan_sequence
from scipy.ndimage import distance_transform_edt

from clothoid_lidar.section_cues import CameraModel, ColourParams, LaneObservation, find_lane_lines

LETHAL = 1000.0
IDLE, ACTIVE = 'idle', 'active'


@dataclass
class AvoidParams:
    # geometry (m)
    lane_width: float = 0.50          # line centre to centre, our track; rules say 0.55
    car_half_width: float = 0.11      # LIMO 0.22 m wide
    car_half_length: float = 0.16     # LIMO 0.32 m long, base_link in the middle
    footprint_offsets: Tuple[float, float, float] = (-0.1, 0.0, 0.1)  # circle centres along the car
    safety: float = 0.05              # extra clearance to obstacles
    line_margin: float = 0.03         # car side to yellow outer line
    wheelbase: float = 0.2
    rear_axle_x: float = 0.1          # rear axle behind base_link
    # trigger
    activate_range: float = 1.5       # m ahead
    block_half_width: float = 0.30    # |y| counted as our lane
    activate_scans: int = 3
    red_ratio_max: float = 0.10       # camera red share at or above this = cone course
    min_speed: float = 0.05           # m/s, car must be moving
    rearm_clear: float = 1.0          # s our lane must be clear before a new activation
    # hand-over
    min_travel: float = 3.0
    max_travel: float = 7.0
    release_tol: float = 0.10         # m from the target lane centre
    release_heading_deg: float = 10.0
    release_clear: float = 2.0        # m ahead, every lane: no obstacle left there to hand over
    # hand over beside the island, in the A / B lane (1 / 3), handover_past m past its front: lane_main
    # then starts on a road with yellow lines on both sides. Before, on the three-lane road, it
    # steered for lane 2 and hit the island's sign right after a hand-over in lane 1 (1112 auto 05).
    # Off: the hand-overs before the island (target lane, island close, a clear road) as before
    handover_after_island: bool = True
    handover_past: float = 0.6        # m past the island front (the island is 1.75 m long)
    handover_heading_deg: float = 15.0
    handover_overshoot: float = 0.6   # m further: hand over even if not straight
    # after a hand-over beside the island the section is behind us for good: no new start (1113
    # auto 02 / 05 started again at the cone course's white line and cones, 6 m past the fork, and
    # drove there with the cone follower on the same mux input). Restart the node for another run
    one_section_per_run: bool = True
    # lidar hits this far beyond the yellow outer lines are dropped once the camera measured the
    # lanes (1112 auto 05: a cone beside the road at 4 m was in the costmap)
    outside_margin: float = 0.10
    # painted island in lane 2. Measured from the white entry line when the camera saw it
    # (rule book p.4 drawing at scale), else guessed from the activation point (~1.5 m before the
    # first LIMO; the 11-05 bags put the true island at 3.6~5.5 m from activation)
    island_from_entry_min: float = 4.40
    island_from_entry_max: float = 6.15
    entry_max_back: float = 3.0       # m, an entry-line crossing this recent anchors the section
    # ... unless the car turned more than this since: that line was before the corner (the crosswalk;
    # the simulator's --corner runs anchored on it, 90 deg off the lanes, and stuck)
    entry_max_turn_deg: float = 45.0
    island_s_min: float = 4.65       # m from the start, entry line not seen; 1108 run 1 2 3 10 11: 4.61~4.74
    island_s_max: float = 6.40       # front + the island length of the rule drawing (1.75)
    island_tip: float = 0.25          # m, the painted island narrows to a point at both ends
    island_align_until: float = 0.3   # m before the island front: closer, hand over even if not aligned
    island_handover: float = 1.0      # m, hand over to lane_main when the island front is this close
    # past the island front the road is lane_main's whatever the state: hand over and lock. Only an
    # estimated island (no entry line seen) gets a margin
    island_past_estimate: float = 0.5
    # no obstacle ever blocked our lane: the section still counts as done at the island front, unless
    # the road turned this much after the white line (then that line was not the section's entry)
    watch_max_turn_deg: float = 45.0
    # past the entry line, something in lane 2 from here on is the island's sign stand, not a LIMO:
    # lane_main takes the car into A / B there (1108 run 4 started on it)
    island_object_margin: float = 0.25
    # course map (/imu yaw + /odom distance from where the node started: the race start, heading +x,
    # as docs 1105 map_03 / 1114 maps_1114): the section's start line is the white line crossed within
    # start_line_radius of start_line_xy, heading start_line_heading_deg +- tol. Before the section the
    # car stays on the course, so the lines of the slow zone, the zigzag marks, the crosswalks and
    # the parking area are told apart by where they are. Replays from the race start (1114 x3, 1112
    # whole course x2): the real line at x 8.07~8.32, y -4.25~-4.80, heading -172~180 deg, the
    # nearest other line 0.98 m away heading -92 deg (the crosswalk). Off: the dashed lane-line pair decides (a start
    # away from the race start, e.g. the avoid-only tests from the crosswalk)
    course_map: bool = True
    # where the node started (CourseMap.origin), from the first turn: the race start drives a straight
    # of ~9 m and turns right (1114 0217: map x 9.1); the avoid-only tests turn within 1.5 m (1113).
    # 'pending' until judged: no start on a blocker (1114 0231 started at 3.8 m on the parking area's
    # obstacles after losing the lane on the hill). 'race': the mission starts on the start line
    # itself. 'elsewhere': a blocker starts it anywhere, the dashed lane-line pair finds the start line
    course_turn_deg: float = 45.0         # heading this far off the first straight's: a turn
    course_corner_tol_deg: float = 20.0   # within this of the next 90 deg axis: the turn is made
    course_corner1_x_min: float = 7.5     # m of straight on the map before the first turn, at least
    course_corner1_odometer_max: float = 11.0   # no turn yet by this odometer: not the race start
    course_test_start: float = 1.0        # a LIMO in our lane within this odometer: an avoid test
    # race: from the start line until the first blocker (LIMO in our lane, obstacle cost over slow_cost
    # or a lane change) at this speed (lane_main's base speed), then speed / slow_speed to the end
    approach_speed: float = 0.8
    start_line_x: float = 8.25
    start_line_y: float = -4.65
    start_line_radius: float = 1.0
    start_line_heading_deg: float = 180.0
    start_line_heading_tol_deg: float = 35.0
    # the map's heading is snapped to the track axes on straights: a mean heading over course_straight
    # m that varied less than course_straight_spread_deg (lane following, no turn, no lane change)
    # and lies within course_snap_max_deg of a multiple of 90 deg is made that multiple (the car set
    # down 3 deg off at the start puts the line 0.7 m off at 14 m; the IMU drifts). Not while avoiding
    course_straight: float = 1.5
    course_straight_spread_deg: float = 6.0
    course_snap_max_deg: float = 15.0
    # no start line confirmed by the map although the car came within start_line_radius of the point
    # and is course_fallback_past m further off again, or the odometer passed start_line_odometer by
    # course_fallback_odometer: the dashed lane-line pair decides from then on (a wrong start pose)
    course_fallback_past: float = 1.0
    start_line_odometer: float = 14.2
    course_fallback_odometer: float = 3.0
    # m: a start line crossed after the start must be this close. Only lines confirmed as the
    # section's start line by the lane-line pair (section_cues.EntryLineTracker.is_start) count, so
    # this is a sanity bound (it was 1.2 m when any white line counted: 1108 run 4 took a line on
    # the A / B road for it)
    entry_late_max: float = 3.0
    fork_commit: float = 2.0          # m travelled before the A/B lane is pulled hard without a flag
    # costmap
    resolution: float = 0.02
    grid_s_min: float = -1.0
    grid_s_max: float = 8.0
    hit_add: int = 2
    hit_max: int = 10
    occupied_count: int = 3
    occlusion_depth: float = 0.30     # cells behind a hit taken as occupied (unseen LIMO body)
    inflation_radius: float = 0.35
    inflation_decay: float = 8.0
    scan_range_max: float = 2.5
    free_range_no_return: float = 2.0   # beams with no return clear cells up to this (m)
    # candidates
    end_step: float = 0.05            # lateral grid of path end offsets
    end_span: float = 1.2             # x lane_width each side
    transition_lengths: Tuple[float, ...] = (0.8, 1.2, 1.6)
    horizon: float = 2.0
    path_ds: float = 0.05
    # two-stage candidates (direction known, island measured): swerve now, then into the A / B lane
    # by route_margin before the island front; scored up to there + route_after. Off: in the simulation
    # it passed 71 of 80 random layouts (one-stage 68) but lost 2 of the 12 runs of the 1108 run 6-9
    # layout (tools/static_avoid, docs/STATIC_AVOID_LOCAL_PLAN.md)
    route: bool = False
    route_end_step: float = 0.10
    route_lengths: Tuple[float, ...] = (0.8, 1.2, 1.6)
    route_len_per_m: float = 1.6      # 2nd stage at least this long per m of offset: the car can follow it
    route_start_step: float = 0.25
    route_margin: float = 0.40
    route_after: float = 0.40
    # lane-sequence planner (lane_sequence.py): lane centre to lane centre with S-curves no sharper
    # than lc_radius, into the A / B lane by route_margin before the island (else over its pointed
    # tip: late), kept while it stays free. Used with the direction known and the island measured.
    # sim (tools/static_avoid/sim_layouts.py): 12/12 of the 1108 layouts and 80/80 random ones, the
    # one-stage planner 12/12 and 68/80
    sequence: bool = True
    seq_step: float = 0.10
    seq_sample: float = 0.05
    lc_radius: float = 0.55           # m, car 7 turns at 0.47 m (rear axle, full lock)
    lc_max_heading_deg: float = 50.0
    lc_blend: float = 0.40            # m over which the present heading is blended out
    w_change: float = 30.0            # per lane change
    w_keep: float = 15.0              # bonus for the change already chosen
    w_off_target: float = 8.0         # per m outside the A / B lane: change early, with margin
    late_tip: bool = True             # allow the island's pointed tip when the target lane is late
    # stuck (every path blocked for stuck_time): sharper transitions if recovery_sharp (off: in the
    # simulation the car could not follow them and hit the LIMO), then back up if allowed
    recovery_sharp: bool = False
    recovery_lengths: Tuple[float, ...] = (0.5, 0.8, 1.2)
    stuck_time: float = 0.5           # s with every path blocked
    stuck_log_after: float = 0.5      # s blocked before the one 'stuck' log line (no line for a blip)
    recovery_until: float = 0.5       # m driven on a plan again: back to the normal candidates
    recovery_reverse: bool = False    # also back up when still stuck (launch argument `reverse`)
    reverse_after: float = 1.5        # s stuck
    reverse_distance: float = 0.20    # m per back-up, straight back along the way the car came
    reverse_max: float = 0.40         # m in all, per mission
    reverse_speed: float = 0.15
    # stuck for stuck_time: search once (again every escape_retry s) curvature-limited paths of three
    # arcs (escape_curvatures x 1 / escape_radius, escape_lengths each) for one that is free and ends
    # about straight, and drive it slowly with a short look-ahead. 1113 auto 05 stopped 14 s where
    # only such a quick swerve (right at full lock, then left) went through; the lane-centre
    # S-curves never contain it
    # off for now (11-13 car: with reverse on, escape paths turned the car near 90 deg and the
    # back-up then drove it off the track; fixed on a later branch)
    escape: bool = False
    escape_radius: float = 0.47       # m, car 7 at full lock
    escape_curvatures: Tuple[float, ...] = (-1.0, -0.5, 0.0, 0.5, 1.0)
    escape_lengths: Tuple[float, ...] = (0.2, 0.4, 0.6)
    escape_end_heading_deg: float = 35.0
    escape_lookahead: float = 0.30
    escape_speed: float = 0.15
    escape_margin: float = 0.03       # m of clearance more than safety: full-lock arcs are cut a little
    escape_w_near: float = 0.02       # per obstacle cost point (0..100), against m of progress
    escape_retry: float = 1.0         # s
    # the first metres of every path start where the car is now: if the car is already inside the
    # safety margin (an obstacle seen late, or passed close), lethal cells there only cost, so the
    # car can still pick the way out instead of freezing
    lethal_free_start: float = 0.25
    # A lethal cell only forbids a path if it is closer than the car needs to swerve clear of it at
    # full lock; further ones are a large cost, the car can still turn away on a later plan (10 Hz).
    # escape = sqrt(2 R y - y^2) + margin: R the rear-axle radius at full lock (car 7 / limo2
    # measured 0.45~0.49 m with steering_scale 1.06), y the sideways move that clears a LIMO
    # straight ahead (car half width + LIMO half width + safety), margin for the replanning period,
    # the look-ahead and the car's front overhang.
    min_turn_radius: float = 0.48
    escape_offset: float = 0.30
    escape_margin: float = 0.15
    w_lethal_far: float = 400.0       # x share of path samples lethal beyond the escape distance
    w_obstacle: float = 1.0
    w_centre: float = 40.0            # per m^2 off the nearest lane centre
    w_outer: float = 200.0            # per m (path mean) beyond the lane 1 / lane 3 centre, outwards
    w_lateral: float = 2.0            # per m of offset change
    w_short: float = 0.5              # per 1/m of transition length
    w_hysteresis: float = 4.0         # per m away from the last chosen end offset
    # per m^2 away from the A/B lane centre, once /direction_flag is known
    # pull to the A / B lane. Weak at first: a strong pull steers into the gap between a lane 2 LIMO and
    # one in the target lane and dead-locks there (sim, 1108 run 1-3 layouts). Strong from target_near
    # before the island front (from about the entry line + 1.4 m), so that the car is back in time
    w_target: float = 20.0
    w_target_near: float = 400.0
    target_near: float = 3.0
    w_lane2: float = 3.0              # lane 2 end without a direction (the island ahead)
    # x share of path samples on the painted island; soft because its position is only estimated
    # and a hard limit can dead-lock the car in lane 2 when the fork sign comes late
    w_island: float = 200.0
    # lane frame from the camera lines (entry line = s 0, lane lines + IMU = direction, offset)
    lane_yaw_max_deg: float = 15.0    # largest turn of the lane frame against the section frame
    lane_offset_max: float = 0.10     # m, largest sideways correction
    lane_fit_window: float = 0.5      # m past the entry line whose frames set the lane frame
    lane_fit_give_up: float = 1.8     # m past the entry line (the car may still be turning there)
    lane_fit_frames: int = 5
    lane_fit_lines: int = 2           # lines per frame
    lane_fit_spread_deg: float = 3.0  # median absolute deviation of the direction
    lane_fit_search_deg: float = 40.0  # around the heading at the line; a fit on the edge is dropped
    # the start can come while the car still turns out of the corner (1112 runs: 25~30 deg off the
    # lane): frames taken while turning faster than this are not used for the direction, and a
    # direction more than lane_yaw_max_deg off the section frame turns the section frame (costmap
    # carried over) up to lane_ref_max_deg
    lane_ref_rate_max_deg: float = 25.0   # deg/s
    lane_ref_max_deg: float = 40.0
    reframe: bool = True
    # direction from the white dashes ahead, as soon as the mission starts (entry line not needed)
    lane_start_frames: int = 3
    lane_start_give_up: float = 1.0   # m from the start: go on with the start heading, tracked
    lane_track_search_deg: float = 10.0
    lane_match_deg: float = 8.0       # slow update: a measurement this far off is not a lane line
    lane_match_dist: float = 0.12     # m
    lane_gain: float = 0.2
    lane_step_deg: float = 0.5        # per update
    # m per update; 0 = off: on the 1108 bags it drifted up to 9 cm and worsened the LIMO positions
    lane_step_dist: float = 0.0
    # control
    lookahead: float = 0.45
    max_steer: float = 0.49           # rad, central angle
    speed: float = 0.30
    slow_speed: float = 0.20
    slow_cost: float = 10.0           # obstacle cost above this -> slow


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# --------------------------------------------------------------------------------------- pose
class PoseTracker:
    """Dead reckoning in a local world frame: /odom linear speed, /imu yaw. Keeps a short history so
    a scan can be placed with the pose at its own time."""

    def __init__(self, history: float = 2.0, distance_history: float = 10.0):
        self.history = history
        self.distance_history = distance_history     # m of odometer kept for poses by distance
        self.by_distance: List[Tuple[float, float, float, float]] = []   # (odometer, x, y, yaw)
        self.last_by_distance: Optional[float] = None
        self.yaw: Optional[float] = None
        self.last_t: Optional[float] = None
        self.x = self.y = 0.0
        self.course: Optional['CourseMap'] = None   # fed every step when the course map is on
        self.speed = 0.0
        self.distance = 0.0           # odometer, m
        self.last_pos: Optional[Tuple[float, float]] = None   # /odom position of the last message
        self.trace: List[Tuple[float, float, float, float]] = []
        self.dtrace: List[Tuple[float, float]] = []      # (t, odometer)
        self.ytrace: List[Tuple[float, float]] = []      # (t, IMU yaw unwrapped)

    def imu(self, t: float, yaw: float) -> None:
        # unwrap so interpolation never jumps by 2 pi
        self.yaw = yaw if self.yaw is None else self.yaw + wrap(yaw - self.yaw)
        # the heading at an image's time comes from here, not from `trace` (stored with /odom): a
        # gap in /odom (a busy node) must not turn the heading in a straight line through a corner
        if self.ytrace and t <= self.ytrace[-1][0]:
            return
        self.ytrace.append((t, self.yaw))
        while self.ytrace and self.ytrace[0][0] < t - self.history:
            self.ytrace.pop(0)

    def yaw_at(self, t: float) -> Optional[float]:
        """IMU heading at time t (unwrapped), interpolated, clamped to the history."""
        yt = self.ytrace
        if not yt:
            return None
        i = bisect.bisect_left(yt, (t,))
        if i <= 0:
            return yt[0][1]
        if i >= len(yt):
            return yt[-1][1]
        (ta, ya), (tb, yb) = yt[i - 1], yt[i]
        return ya + (t - ta) / max(tb - ta, 1e-9) * (yb - ya)

    def yaw_rate(self, t: float, window: float = 0.2) -> Optional[float]:
        """rad/s over the `window` before t; None without that much IMU history."""
        yt = self.ytrace
        if not yt or yt[0][0] > t - window + 1e-3:
            return None
        return (self.yaw_at(t) - self.yaw_at(t - window)) / window

    def odom(self, t: float, v: float, pos: Optional[Tuple[float, float]] = None,
             max_jump: float = 0.5, max_gap: float = 2.0) -> None:
        """pos = the driver's accumulated /odom position: the step is the distance between the last two
        messages received, so messages dropped by a busy node lose no distance (its heading is wrong
        on the LIMO, only the length is used; the direction is the IMU's). Without pos: v * dt, gaps
        capped at 0.1 s (simulation, old bags)."""
        self.speed = v
        if self.yaw is None:
            return
        step = None
        if pos is not None:
            if self.last_pos is not None and self.last_t is not None and t - self.last_t <= max_gap:
                d = math.hypot(pos[0] - self.last_pos[0], pos[1] - self.last_pos[1])
                if d <= max_jump:                       # a larger jump is a driver reset
                    step = math.copysign(d, v) if v != 0.0 else 0.0
            self.last_pos = (float(pos[0]), float(pos[1]))
        elif self.last_t is not None:
            step = v * min(max(t - self.last_t, 0.0), 0.1)
        if step:
            self.x += step * math.cos(self.yaw)
            self.y += step * math.sin(self.yaw)
            self.distance += step
        self.last_t = t
        if self.course is not None:
            self.course.step(self.distance, step or 0.0, self.yaw)
        self.trace.append((t, self.x, self.y, self.yaw))
        self.dtrace.append((t, self.distance))
        while self.trace and self.trace[0][0] < t - self.history:
            self.trace.pop(0)
        while self.dtrace and self.dtrace[0][0] < t - self.history:
            self.dtrace.pop(0)
        last = self.last_by_distance
        if last is None or abs(self.distance - last) >= 0.01:
            # sorted by odometer. Backing up (pushed back by hand before a run, the recovery reverse)
            # passes readings again: the latest pass counts, readings below the first are added
            bd = self.by_distance
            keys = [e[0] for e in bd]
            if last is not None:      # the readings passed again since the last entry (kept)
                lo, hi = sorted((last, self.distance))
                if self.distance < last:
                    i, j = bisect.bisect_left(keys, lo - 0.005), bisect.bisect_left(keys, hi - 0.005)
                else:
                    i, j = bisect.bisect_right(keys, lo + 0.005), bisect.bisect_right(keys, hi + 0.005)
                del bd[i:j]
                keys = keys[:i] + keys[j:]
            bd.insert(bisect.bisect_left(keys, self.distance), (self.distance, self.x, self.y, self.yaw))
            self.last_by_distance = self.distance
            while bd and bd[0][0] < self.distance - self.distance_history:
                bd.pop(0)
            while bd and bd[-1][0] > self.distance + self.distance_history:
                bd.pop()

    def line_pose(self, odometer: float, ahead_max: float = 1.5) -> Optional[Tuple[float, float, float]]:
        """Pose where the odometer reads `odometer`: from the history, or for a line still ahead (up
        to ahead_max) straight on from the latest pose. A start line confirmed 0.5 m ahead must count
        for a retry before the base reaches it (parking_1112_201114: 1.1 m late)."""
        pose = self.at_distance(odometer)
        if pose is not None or not self.trace:
            return pose
        ahead = odometer - self.distance
        if not 0.0 < ahead <= ahead_max:
            return None
        x, y, yaw = self.latest()
        return x + ahead * math.cos(yaw), y + ahead * math.sin(yaw), yaw

    def at_distance(self, odometer: float) -> Optional[Tuple[float, float, float]]:
        """Pose where the odometer read `odometer` (1 cm steps), None outside the kept range."""
        bd = self.by_distance
        if not bd or odometer < bd[0][0] - 0.02 or odometer > bd[-1][0] + 0.02:
            return None
        k = min(range(len(bd)), key=lambda i: abs(bd[i][0] - odometer))
        return bd[k][1:]

    @property
    def ready(self) -> bool:
        return bool(self.trace)

    def latest(self) -> Tuple[float, float, float]:
        return self.trace[-1][1:]

    def distance_at(self, t: float) -> float:
        """Odometer at time t (an image's stamp), interpolated, clamped to the history."""
        dt = self.dtrace
        if not dt:
            return self.distance
        ts = [x[0] for x in dt]
        return float(np.interp(t, ts, [x[1] for x in dt]))

    def at(self, t: float) -> Tuple[float, float, float]:
        """Pose at time t, interpolated, clamped to the history."""
        tr = self.trace
        if t <= tr[0][0]:
            return (*tr[0][1:3], self._imu_yaw(t, tr[0][3]))
        if t >= tr[-1][0]:
            return (*tr[-1][1:3], self._imu_yaw(t, tr[-1][3]))
        lo, hi = 0, len(tr) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if tr[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        a, b = tr[lo], tr[hi]
        f = (t - a[0]) / max(b[0] - a[0], 1e-9)
        x, y, yaw = (a[k] + f * (b[k] - a[k]) for k in (1, 2, 3))
        return x, y, self._imu_yaw(t, yaw)

    def _imu_yaw(self, t: float, fallback: float) -> float:
        yaw = self.yaw_at(t)
        return fallback if yaw is None else yaw


@dataclass
class SectionFrame:
    x0: float
    y0: float
    yaw0: float

    def to_section(self, x: float, y: float, yaw: float) -> Tuple[float, float, float]:
        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        dx, dy = x - self.x0, y - self.y0
        return c * dx + s * dy, -s * dx + c * dy, wrap(yaw - self.yaw0)

    def points_from(self, pts: np.ndarray) -> np.ndarray:
        """(N, 2) points given in this frame -> the parent frame."""
        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        return np.column_stack((self.x0 + c * pts[:, 0] - s * pts[:, 1],
                                self.y0 + s * pts[:, 0] + c * pts[:, 1]))

    def points_to(self, pts: np.ndarray) -> np.ndarray:
        """(N, 2) points given in the parent frame -> this frame."""
        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        dx, dy = pts[:, 0] - self.x0, pts[:, 1] - self.y0
        return np.column_stack((c * dx + s * dy, -s * dx + c * dy))


def base_to_section(points: np.ndarray, pose: Tuple[float, float, float]) -> np.ndarray:
    """base_link points (N, 2) -> section frame, car at pose (s, d, psi)."""
    s0, d0, psi = pose
    c, s = math.cos(psi), math.sin(psi)
    return np.column_stack((s0 + c * points[:, 0] - s * points[:, 1],
                            d0 + s * points[:, 0] + c * points[:, 1]))


def section_to_base(points: np.ndarray, pose: Tuple[float, float, float]) -> np.ndarray:
    s0, d0, psi = pose
    c, s = math.cos(psi), math.sin(psi)
    dx, dy = points[:, 0] - s0, points[:, 1] - d0
    return np.column_stack((c * dx + s * dy, -s * dx + c * dy))


# ----------------------------------------------------------------------------------- costmap
class Costmap:
    """Hit/clear counts on a grid in the section frame, inflated into a cost grid."""

    def __init__(self, p: AvoidParams):
        self.p = p
        # the lanes may turn against this grid by up to lane_yaw_max_deg over the section
        self.d_max = (1.5 * p.lane_width + 0.15
                      + math.tan(math.radians(p.lane_yaw_max_deg)) * p.max_travel)
        self.ns = int(round((p.grid_s_max - p.grid_s_min) / p.resolution))
        self.nd = int(round(2 * self.d_max / p.resolution))
        self.count = np.zeros((self.ns, self.nd), np.int16)
        self.cost = np.zeros((self.ns, self.nd))

    def cells(self, pts: np.ndarray):
        i = np.floor((pts[:, 0] - self.p.grid_s_min) / self.p.resolution).astype(int)
        j = np.floor((pts[:, 1] + self.d_max) / self.p.resolution).astype(int)
        ok = (i >= 0) & (i < self.ns) & (j >= 0) & (j < self.nd)
        return i, j, ok

    def update(self, hits: np.ndarray, origin: np.ndarray,
               free_dirs: Optional[np.ndarray] = None,
               clear_only: Optional[np.ndarray] = None) -> None:
        """hits: (N, 2) beam end points, origin: (2,) lidar position, both in the section frame.
        free_dirs: (M, 2) unit directions of beams with no return; they clear up to
        free_range_no_return, so an obstacle that left is forgotten even with nothing behind it.
        clear_only: end points that are no obstacle of the section (beyond the yellow lines): their
        beams clear, nothing is marked. Hits beyond scan_range_max clear up to it as well: dropped
        whole, the cells marked from further back (the unseen body behind a hit) were never cleared
        and closed the gap the driver took (1112 auto 05, 24 s, 20 s stopped)."""
        p = self.p
        if len(hits):
            dist = np.linalg.norm(hits - origin, axis=1)
            far = dist > p.scan_range_max
            if far.any():
                clear_only = hits[far] if clear_only is None else np.vstack((clear_only, hits[far]))
                hits = hits[~far]
        if clear_only is not None and len(clear_only):
            vec = clear_only - origin
            dist = np.linalg.norm(vec, axis=1)
            ok = dist > 1e-3
            unit, reach = vec[ok] / dist[ok, None], np.minimum(dist[ok] - 1.5 * p.resolution,
                                                              p.scan_range_max)
            if len(unit):
                steps = np.arange(0.0, reach.max(), p.resolution)
                fp = (origin + unit[:, None, :] * steps[None, :, None])[steps[None, :] < reach[:, None]]
                i, j, inside = self.cells(fp)
                self.count.reshape(-1)[np.unique(i[inside] * self.nd + j[inside])] -= 1
        if free_dirs is not None and len(free_dirs):
            steps = np.arange(0.0, p.free_range_no_return, p.resolution)
            fp = (origin + free_dirs[:, None, :] * steps[None, :, None]).reshape(-1, 2)
            i, j, ok = self.cells(fp)
            flat = np.unique(i[ok] * self.nd + j[ok])
            self.count.reshape(-1)[flat] -= 1
        if len(hits):
            vec = hits - origin
            dist = np.linalg.norm(vec, axis=1)
            keep = (dist > 1e-3) & (dist <= p.scan_range_max)
            hits, vec, dist = hits[keep], vec[keep], dist[keep]
        if len(hits):
            unit = vec / dist[:, None]
            # free cells along each beam, up to one cell before the hit
            steps = np.arange(0.0, dist.max(), p.resolution)
            t = steps[None, :]
            free = t < (dist[:, None] - 1.5 * p.resolution)
            fp = origin + unit[:, None, :] * t[..., None]
            fp = fp[free]
            i, j, ok = self.cells(fp)
            flat = np.unique(i[ok] * self.nd + j[ok])
            self.count.reshape(-1)[flat] -= 1
            # the hit and the unseen body behind it
            behind = np.arange(0.0, p.occlusion_depth + 1e-9, p.resolution)
            hp = (hits[:, None, :] + unit[:, None, :] * behind[None, :, None]).reshape(-1, 2)
            i, j, ok = self.cells(hp)
            flat = np.unique(i[ok] * self.nd + j[ok])
            self.count.reshape(-1)[flat] += p.hit_add
        np.clip(self.count, 0, p.hit_max, out=self.count)
        self._inflate()

    def resample(self, old: 'SectionFrame', new: 'SectionFrame') -> None:
        """Carry the hit / clear counts over to a turned section frame (same world points)."""
        p = self.p
        i, j = np.meshgrid(np.arange(self.ns), np.arange(self.nd), indexing='ij')
        centres = np.column_stack(((i.ravel() + 0.5) * p.resolution + p.grid_s_min,
                                   (j.ravel() + 0.5) * p.resolution - self.d_max))
        oi, oj, ok = self.cells(old.points_to(new.points_from(centres)))
        count = np.zeros_like(self.count)
        count.reshape(-1)[ok] = self.count[oi[ok], oj[ok]]
        self.count = count
        self._inflate()

    def _inflate(self) -> None:
        p = self.p
        occupied = self.count >= p.occupied_count
        if not occupied.any():
            self.cost[:] = 0.0
            return
        d = distance_transform_edt(~occupied) * p.resolution
        inscribed = p.car_half_width + p.safety
        cost = np.where(d < p.inflation_radius,
                        100.0 * np.exp(-p.inflation_decay * np.maximum(d - inscribed, 0.0)), 0.0)
        self.cost = np.where(d < inscribed, LETHAL, cost)

    def lookup(self, pts: np.ndarray) -> np.ndarray:
        i, j, ok = self.cells(pts)
        out = np.zeros(len(pts))
        out[ok] = self.cost[i[ok], j[ok]]
        return out

    def occupied_at(self, pts: np.ndarray) -> np.ndarray:
        i, j, ok = self.cells(pts)
        out = np.zeros(len(pts), bool)
        out[ok] = self.count[i[ok], j[ok]] >= self.p.occupied_count
        return out

    def occupied_in(self, s0, s1, d0, d1) -> bool:
        p = self.p
        i0 = max(0, int((s0 - p.grid_s_min) / p.resolution))
        i1 = min(self.ns, int((s1 - p.grid_s_min) / p.resolution) + 1)
        j0 = max(0, int((d0 + self.d_max) / p.resolution))
        j1 = min(self.nd, int((d1 + self.d_max) / p.resolution) + 1)
        if i0 >= i1 or j0 >= j1:
            return False
        return bool((self.count[i0:i1, j0:j1] >= p.occupied_count).any())


# --------------------------------------------------------------------------------- planner
@dataclass
class Plan:
    path: np.ndarray                  # (K, 2) section frame, the chosen local path
    end_offset: float
    cost: float
    obstacle_cost: float
    candidates: List[Tuple[np.ndarray, float]] = field(default_factory=list)  # (path, cost) for debug
    feasible: bool = True
    sequence: Optional['Sequence'] = None     # lane-sequence plan, if that planner made it
    blocked: Optional[dict] = None    # candidates out per reason (local candidates), for the stuck log


def lane_centres(p: AvoidParams) -> np.ndarray:
    return np.array([p.lane_width, 0.0, -p.lane_width])   # lanes 1, 2, 3


def lane_of(d: float, p: AvoidParams) -> int:
    """Lane number 1 (left, A), 2, 3 (right, B) of a lateral offset."""
    return int(min(3, max(1, 2 - round(d / p.lane_width))))


def target_centre(direction: Optional[int], p: AvoidParams) -> Optional[float]:
    """/direction_flag 0 = left lane (A, lane 1), 1 = right lane (B, lane 3); else unknown."""
    if direction == 0:
        return p.lane_width
    if direction == 1:
        return -p.lane_width
    return None


def escape_distance(p: AvoidParams) -> float:
    """Path length within which a lethal cell cannot be swerved around any more (m)."""
    r, y = p.min_turn_radius, min(p.escape_offset, p.min_turn_radius)
    return math.sqrt(max(2.0 * r * y - y * y, 0.0)) + p.escape_margin


def _hermite(ds: np.ndarray, d0: float, m0: float, E: np.ndarray, Lt: np.ndarray):
    """Offsets and slopes (P, S): leave d0 along slope m0, end straight at E after Lt (cubic Hermite in
    d(s)), then straight."""
    x = np.clip(ds[None, :] / Lt[:, None], 0.0, 1.0)
    h00, h10, h01 = 2 * x ** 3 - 3 * x ** 2 + 1, x ** 3 - 2 * x ** 2 + x, -2 * x ** 3 + 3 * x ** 2
    d = h00 * d0 + h10 * (Lt[:, None] * m0) + h01 * E[:, None]
    dh00, dh10, dh01 = 6 * x ** 2 - 6 * x, 3 * x ** 2 - 4 * x + 1, -6 * x ** 2 + 6 * x
    slope = np.where(x < 1.0, (dh00 * d0 + dh01 * E[:, None]) / Lt[:, None] + dh10 * m0, 0.0)
    return d, slope


def _route_candidates(ds: np.ndarray, d0: float, m0: float, s0: float, tc: float, front: float,
                      p: 'AvoidParams'):
    """Two-stage candidates: to E1 now (as the one-stage ones), then from s_b over L2 into the target
    lane centre tc, done by `front` (the island front less route_margin). Returns E1, Lt, d, slope."""
    w = p.lane_width
    out = []
    for E1 in np.arange(-p.end_span * w, p.end_span * w + 1e-9, p.route_end_step):
        for Lt in p.transition_lengths:
            for L2 in p.route_lengths:
                if abs(E1 - tc) < 1e-6:
                    if L2 == p.route_lengths[0]:
                        out.append((E1, Lt, None, None))          # already there: one stage
                    continue
                if L2 < p.route_len_per_m * abs(tc - E1) - 1e-9:
                    continue
                last = front - L2                                 # latest start of the 2nd stage
                first = s0 + Lt
                if last < first - 1e-9:
                    continue
                for sb in np.arange(first, last + 1e-9, p.route_start_step):
                    out.append((E1, Lt, sb, L2))
    if not out:
        return None
    E1 = np.array([o[0] for o in out])
    Lt = np.array([o[1] for o in out])
    d, slope = _hermite(ds, d0, m0, E1, Lt)
    for k, (_, _, sb, L2) in enumerate(out):
        if sb is None:
            continue
        x = np.clip((s0 + ds - sb) / L2, 0.0, 1.0)
        d[k] += (tc - E1[k]) * (3 * x ** 2 - 2 * x ** 3)
        slope[k] += np.where((x > 0.0) & (x < 1.0), (tc - E1[k]) * (6 * x - 6 * x ** 2) / L2, 0.0)
    return E1, Lt, d, slope


def _score(s0, d0, psi, ds, E, Lt, d, slope, costmap, travelled, direction, last_end, p, island,
           lane_in_costmap, forbid_island, route: bool):
    """Cost per candidate (inf = not allowed) and the obstacle term. With route, every candidate ends
    in the target lane before the island: no target-lane term."""
    w = p.lane_width
    theta = np.arctan(slope)
    s = np.broadcast_to(s0 + ds[None, :], d.shape)

    # footprint circles along the car at every path point
    off = np.asarray(p.footprint_offsets)
    fs = s[..., None] + off * np.cos(theta)[..., None]
    fd = d[..., None] + off * np.sin(theta)[..., None]
    query = np.column_stack((fs.ravel(), fd.ravel()))
    if lane_in_costmap is not None:
        query = lane_in_costmap.points_from(query)
    c = costmap.lookup(query).reshape(fs.shape)
    hit = (c >= LETHAL).any(axis=2) & (ds[None, :] > p.lethal_free_start)     # (P, S)
    near = ds[None, :] <= escape_distance(p)
    lethal = (hit & near).any(axis=1)              # cannot be swerved around: forbidden
    lethal_far = (hit & ~near).mean(axis=1)        # can still be swerved around later: cost
    # closeness over the first `horizon` m as for one-stage paths: a long route must not dilute it
    obstacle = np.where(c >= LETHAL, 100.0, c)[:, ds <= p.horizon + 1e-9].mean(axis=(1, 2))

    # yellow outer line, checked at the car's outer corners (a car at an angle reaches further out
    # than its centre): no path may go further out than the line - margin, or than the corners are
    # now when they are already beyond it (a hard limit from the first point would forbid coming
    # back)
    def corner(dd, th):
        return np.abs(dd) + p.car_half_length * np.abs(np.sin(th)) + p.car_half_width * np.cos(th)
    # turning back at full lock from heading psi still drifts out by R (1 - cos psi)
    d_limit = max(1.5 * w - p.line_margin, float(corner(np.array(d0), np.array(psi)))
                  + p.min_turn_radius * (1.0 - math.cos(psi)) + 0.01)
    out_of_road = (corner(d, theta) > d_limit + 1e-6).any(axis=1)
    beyond_outer = np.maximum(np.abs(d) - w, 0.0).mean(axis=1)   # past lane 1 / 3 centre
    on_island = ((s >= island[0]) & (s <= island[1])
                 & (np.abs(d) < w / 2.0)).mean(axis=1)
    # the car side must stay off it as well; only with a measured island position
    tip = np.clip(np.minimum(s - island[0], island[1] - s) / p.island_tip, 0.0, 1.0)  # hexagon
    island_hit = ((s >= island[0]) & (s <= island[1])
                  & (np.abs(d) < tip * w / 2.0 + p.car_half_width)).any(axis=1)

    centres = lane_centres(p)
    off_centre = np.min((d[..., None] - centres) ** 2, axis=2).mean(axis=1)
    cost = (p.w_obstacle * obstacle + p.w_centre * off_centre + p.w_lateral * np.abs(E - d0)
            + p.w_short / Lt + p.w_island * on_island + p.w_lethal_far * lethal_far
            + p.w_outer * beyond_outer)
    if last_end is not None:
        cost += p.w_hysteresis * np.abs(E - last_end)
    tc = target_centre(direction, p)
    if route:
        pass
    elif tc is not None:
        near_island = island[0] - s0 <= p.target_near
        cost += (p.w_target_near if near_island else p.w_target) * (E - tc) ** 2
    else:
        lane2 = np.abs(E) < w / 2.0
        weight = p.w_lane2 * (4.0 if travelled >= p.fork_commit else 1.0)
        cost += weight * lane2

    bad = lethal | out_of_road
    # the island is forbidden only while another way exists: it must never freeze the car. A route
    # over the island is never taken: the one-stage candidates are the other way then
    if route or (forbid_island and tc is not None and not (bad | island_hit).all()):
        bad |= island_hit
    if route:       # only a way that is clear all the way to the island: else wait as one-stage
        bad |= hit.any(axis=1)
    # why candidates are out, for the stuck log (a candidate can count more than once)
    blocked = {'obstacle margin': int(lethal.sum()), 'yellow line': int(out_of_road.sum()),
               'island': int((bad & island_hit).sum()), 'of': int(len(E))}
    return np.where(bad, np.inf, cost), obstacle, s, blocked


def plan_paths(pose: Tuple[float, float, float], travelled: float, costmap: Costmap,
               direction: Optional[int], last_end: Optional[float], p: AvoidParams,
               keep_candidates: bool = False,
               island: Optional[Tuple[float, float]] = None,
               lane_in_costmap: Optional[SectionFrame] = None,
               forbid_island: bool = False, recovery: bool = False,
               keep: Optional[Sequence] = None) -> Plan:
    """pose, island and the returned paths are in lane coordinates; lane_in_costmap is the lane frame
    expressed in the costmap's section frame (None = the same frame).

    Direction known and the island measured: first the two-stage candidates (swerve now, then into the
    A / B lane before the island), scored up to the island and taken only when one is clear all the way
    (nothing lethal on it, the island not touched); else the one-stage ones over
    `horizon` with the target-lane weight. recovery: the shorter transitions and the smaller safety of
    a car that has been stuck (every path blocked)."""
    s0, d0, psi = pose
    island = island or (p.island_s_min, p.island_s_max)
    if recovery and p.recovery_sharp:
        p = replace(p, transition_lengths=p.recovery_lengths, route_lengths=p.recovery_lengths)
    w = p.lane_width
    # Every candidate leaves along the car's present heading (slope tan psi, as when it is half way
    # through a lane change).
    m0 = math.tan(max(-1.0, min(1.0, psi)))
    tc = target_centre(direction, p)
    args = (costmap, travelled, direction, last_end, p, island, lane_in_costmap, forbid_island)

    # with the hand-over beside the island the lane centre is followed there too (the one-stage
    # candidates drifted to the yellow line, away from the island)
    seq_until = island[0] + (p.handover_past + p.handover_overshoot if p.handover_after_island else 0.0)
    if p.sequence and tc is not None and forbid_island and seq_until - s0 > 0.3:
        plan = _sequence_plan(pose, costmap, direction, p, island, lane_in_costmap, keep)
        if plan is not None:
            return plan

    front = island[0] - p.route_margin
    if p.route and tc is not None and forbid_island and front - s0 > min(p.transition_lengths):
        ds = np.arange(p.path_ds, max(p.horizon, front - s0 + p.route_after) + 1e-9, p.path_ds)
        cands = _route_candidates(ds, d0, m0, s0, tc, front, p)
        if cands is not None:
            E, Lt, d, slope = cands
            cost, obstacle, s, blocked = _score(s0, d0, psi, ds, E, Lt, d, slope, *args, route=True)
            if np.isfinite(cost).any():
                return _plan(s, d, E, cost, obstacle, keep_candidates, blocked)

    ends = np.arange(-p.end_span * w, p.end_span * w + 1e-9, p.end_step)
    E, Lt = np.meshgrid(ends, np.asarray(p.transition_lengths), indexing='ij')
    E, Lt = E.ravel(), Lt.ravel()                                   # (P,)
    ds = np.arange(p.path_ds, p.horizon + 1e-9, p.path_ds)          # (S,)
    d, slope = _hermite(ds, d0, m0, E, Lt)
    cost, obstacle, s, blocked = _score(s0, d0, psi, ds, E, Lt, d, slope, *args, route=False)
    return _plan(s, d, E, cost, obstacle, keep_candidates, blocked)


def _sequence_plan(pose, costmap, direction, p, island, lane_in_costmap, keep) -> Optional[Plan]:
    s0, d0, psi = pose
    w = p.lane_width
    off = np.asarray(p.footprint_offsets)

    def costs(s, d, h):
        fs = s[:, None] + off * np.cos(h)[:, None]
        fd = d[:, None] + off * np.sin(h)[:, None]
        q = np.column_stack((fs.ravel(), fd.ravel()))
        if lane_in_costmap is not None:
            q = lane_in_costmap.points_from(q)
        return costmap.lookup(q).reshape(fs.shape)

    def blocked(s, d, h, late):
        c = costs(s, d, h)
        lethal = (c >= LETHAL).any(axis=1) & (s - s0 > p.lethal_free_start)
        corner = np.abs(d) + p.car_half_length * np.abs(np.sin(h)) + p.car_half_width * np.cos(h)
        out = corner > 1.5 * w - p.line_margin + 1e-6
        tip = np.clip(np.minimum(s - island[0], island[1] - s) / p.island_tip, 0.0, 1.0)
        on = (s >= island[0]) & (s <= island[1]) & (np.abs(d) < tip * w / 2.0 + p.car_half_width)
        if late and p.late_tip:
            on &= s >= island[0] + p.island_tip          # the pointed tip may be crossed
        return lethal | out | on

    def closeness(s, d, h):
        return np.minimum(costs(s, d, h), 100.0).mean(axis=1)

    target = {0: 1, 1: 3}.get(direction)
    # the chosen sequence's path while it is still free: the car lags the S-curves a little, and a
    # sequence re-made from its pose every scan would come and go
    if keep is not None and keep.path[-1, 0] > s0 + 0.3:
        ahead = keep.path[keep.path[:, 0] >= s0]
        if len(ahead) > 2:
            h = np.arctan(np.gradient(ahead[:, 1], ahead[:, 0]))
            if not blocked(ahead[:, 0], ahead[:, 1], h, keep.late).any():
                e1 = float(ahead[min(len(ahead) - 1, int(p.horizon / p.seq_sample)), 1])
                c = closeness(ahead[:, 0], ahead[:, 1], h)
                return Plan(path=ahead, end_offset=e1, cost=keep.cost,
                            obstacle_cost=float(c[ahead[:, 0] <= s0 + p.horizon].mean()),
                            sequence=keep)
    after = p.island_tip + p.route_after
    if p.handover_after_island:                 # on beside the island, to the hand-over and more
        after = max(after, p.handover_past + 0.4)
    end = max(island[0] + after, s0 + p.horizon)
    seq = plan_sequence(s0, d0, psi, target, island[0] - p.route_margin, island[0] + p.island_tip,
                        end, p, blocked, closeness, keep)
    if seq is None:
        return None
    path = seq.path
    e1 = {1: w, 2: 0.0, 3: -w}[seq.lanes[0] if seq.lanes else seq.first_lane]
    near = path[path[:, 0] <= s0 + p.horizon]
    c = closeness(near[:, 0], near[:, 1], np.zeros(len(near)))
    return Plan(path=path, end_offset=e1, cost=seq.cost, obstacle_cost=float(c.mean()),
                sequence=seq)


def _plan(s, d, E, cost, obstacle, keep_candidates, blocked=None) -> Plan:
    best = int(np.argmin(cost))
    feasible = bool(np.isfinite(cost[best]))
    cands = []
    if keep_candidates:
        step = max(1, len(E) // 60)
        for k in range(0, len(E), step):
            cands.append((np.column_stack((s[k], d[k])), float(cost[k])))
    return Plan(path=np.column_stack((s[best], d[best])), end_offset=float(E[best]),
                cost=float(cost[best]), obstacle_cost=float(obstacle[best]), candidates=cands,
                feasible=feasible, blocked=blocked)


class CourseMap:
    """The race course as /imu yaw + /odom distance from where the node started (the race start,
    heading +x; docs: 1105 map_03, 1114 maps_1114), its heading snapped to the track axes on straights
    (AvoidParams.course_*). Poses by odometer for the start line check; `frozen` while avoiding."""

    def __init__(self, p: AvoidParams):
        self.p = p
        self.h0: Optional[float] = None       # IMU yaw at the start
        self.off = 0.0                        # rad, heading correction from the straights
        self.x = self.y = self.h = 0.0
        self.hist: List[Tuple[float, float, float, float]] = []   # (odometer, x, y, heading)
        self.win: List[Tuple[float, float, float, float]] = []    # the same over the last straight
        self.frozen = False
        self.snaps: List[Tuple[float, float]] = []                # (odometer, deg) for the log
        self.min_far = math.inf
        self.fell_back = False
        # where the node started (AvoidParams.course_turn_deg): 'pending' | 'race' | 'elsewhere'
        self.origin = 'pending'
        self.origin_why = ''
        self.corner1: Optional[Tuple[float, float]] = None        # (odometer, map x) of the right turn

    def set_origin(self, origin: str, why: str) -> None:
        if self.origin == 'pending':
            self.origin, self.origin_why = origin, why

    def _judge(self, odometer: float) -> None:
        """Race start or not, from the first turn (see AvoidParams.course_turn_deg)."""
        p = self.p
        if self.origin != 'pending':
            return
        deg = math.degrees(self.h)
        if self.corner1 is None:
            if abs(deg) > p.course_turn_deg and self.x < p.course_corner1_x_min:
                self.set_origin('elsewhere', 'turned %.0f deg after %.1f m of straight (the race start '
                                'drives %.1f m before its first turn)' % (deg, self.x, p.course_corner1_x_min))
            elif abs(deg + 90.0) <= p.course_corner_tol_deg:
                self.corner1 = (odometer, self.x)
            elif abs(deg) > 90.0 - p.course_corner_tol_deg and deg > 0.0:
                self.set_origin('elsewhere', 'the first turn went left (%.0f deg)' % deg)
            elif odometer > p.course_corner1_odometer_max:
                self.set_origin('elsewhere', 'no turn in the first %.1f m' % odometer)
        elif abs(deg) <= p.course_corner_tol_deg or abs(deg - 90.0) <= p.course_corner_tol_deg:
            self.set_origin('elsewhere', 'turned to %.0f deg after the first right turn (the race goes '
                            'on to 180 deg)' % deg)

    def step(self, odometer: float, step: float, yaw: float) -> None:
        if yaw is None:
            return
        if self.h0 is None:
            self.h0 = yaw
            self.hist.append((odometer, 0.0, 0.0, 0.0))
            return
        self.h = wrap(yaw - self.h0 - self.off)
        self.x += step * math.cos(self.h)
        self.y += step * math.sin(self.h)
        if odometer > self.hist[-1][0] + 0.01:
            self.hist.append((odometer, self.x, self.y, self.h))
        self._snap(odometer)
        self._judge(odometer)

    def _snap(self, odometer: float) -> None:
        p = self.p
        if self.frozen:
            self.win = []
            return
        self.win.append((odometer, self.x, self.y, self.h))
        if odometer - self.win[0][0] < p.course_straight:
            return
        while odometer - self.win[1][0] >= p.course_straight:
            self.win.pop(0)
        hs = np.unwrap([q[3] for q in self.win])
        if np.ptp(hs) > math.radians(p.course_straight_spread_deg):
            return
        mean = float(np.mean(hs))
        axis = round(mean / (math.pi / 2)) * (math.pi / 2)
        err = wrap(mean - axis)
        if abs(err) > math.radians(p.course_snap_max_deg) or abs(err) < math.radians(0.2):
            return
        # this straight lies along `axis`: turn what was driven on it about its first point
        ox, oy = self.win[0][1], self.win[0][2]
        c, sn = math.cos(-err), math.sin(-err)
        dx, dy = self.x - ox, self.y - oy
        self.x, self.y = ox + c * dx - sn * dy, oy + sn * dx + c * dy
        self.off += err
        self.h = wrap(self.h - err)
        self.snaps.append((odometer, math.degrees(err)))
        self.win = [(odometer, self.x, self.y, self.h)]

    def pose_at(self, odometer: float, ahead_max: float = 1.5) -> Optional[Tuple[float, float, float]]:
        """Map pose where the odometer read `odometer`; a line still ahead straight on from now."""
        if not self.hist:
            return None
        last = self.hist[-1][0]
        if odometer > last:
            ahead = odometer - last
            if ahead > ahead_max:
                return None
            return self.x + ahead * math.cos(self.h), self.y + ahead * math.sin(self.h), self.h
        keys = [q[0] for q in self.hist]
        k = bisect.bisect_left(keys, odometer)
        if k >= len(keys) or (k > 0 and odometer - keys[k - 1] < keys[k] - odometer):
            k -= 1
        if abs(keys[k] - odometer) > 0.05:
            return None
        return self.hist[k][1:]

    def should_fall_back(self, odometer: float, entry) -> Optional[str]:
        """Give the start line over to the dashed lane-line pair? Returns the reason once."""
        p = self.p
        if self.fell_back or any(c.start for c in entry.crossings):
            return None
        far = math.hypot(self.x - p.start_line_x, self.y - p.start_line_y)
        self.min_far = min(self.min_far, far)
        why = None
        if self.min_far <= p.start_line_radius and far > p.start_line_radius + p.course_fallback_past:
            why = ('passed the start line point (%.2f m at the closest, %.2f m now) without a white '
                   'line there' % (self.min_far, far))
        elif odometer > p.start_line_odometer + p.course_fallback_odometer:
            why = ('%.1f m driven, never within %.1f m of the start line point (closest %.2f m)'
                   % (odometer, p.start_line_radius, self.min_far))
        if why:
            self.fell_back = True
            self.set_origin('elsewhere', why)
        return why


def course_start_line(course: CourseMap, p: AvoidParams):
    """EntryLineTracker.start_by_map: is a crossed white line the section's start line, by where it is
    on the course map? None while its pose is not known yet."""
    def check(c) -> Optional[bool]:
        pose = course.pose_at(c.odometer)
        if pose is None:
            return None
        x, y, h = pose
        far = math.hypot(x - p.start_line_x, y - p.start_line_y)
        turn = abs(wrap(h - math.radians(p.start_line_heading_deg)))
        c.where = 'map (%.2f, %.2f), heading %.0f deg, %.2f m from the start line' % (
            x, y, math.degrees(h), far)
        return far <= p.start_line_radius and turn <= math.radians(p.start_line_heading_tol_deg)
    return check


def path_blocker(s0: float, costmap: 'Costmap', lane_rel: Optional['SectionFrame'],
                 island: Tuple[float, float], p: AvoidParams, margin: float = 0.0):
    """blocked(s, d, heading) per sample, lane coordinates: a footprint circle on a lethal cell (past
    lethal_free_start), a corner over the yellow outer line, or on the island (its pointed tip
    allowed), as the lane-sequence planner checks. margin: m of clearance more than safety."""
    w = p.lane_width
    off = np.asarray(p.footprint_offsets)
    limit = LETHAL if margin <= 0.0 else 100.0 * math.exp(-p.inflation_decay * margin) - 1e-6

    def blocked(s, d, h):
        fs = s[:, None] + off * np.cos(h)[:, None]
        fd = d[:, None] + off * np.sin(h)[:, None]
        q = np.column_stack((fs.ravel(), fd.ravel()))
        if lane_rel is not None:
            q = lane_rel.points_from(q)
        c = costmap.lookup(q).reshape(fs.shape)
        lethal = (c >= limit).any(axis=1) & (s - s0 > p.lethal_free_start)
        corner = np.abs(d) + p.car_half_length * np.abs(np.sin(h)) + p.car_half_width * np.cos(h)
        out = corner > 1.5 * w - p.line_margin + 1e-6
        tip = np.clip(np.minimum(s - island[0], island[1] - s) / p.island_tip, 0.0, 1.0)
        on = ((s >= island[0] + p.island_tip) & (s <= island[1])
              & (np.abs(d) < tip * w / 2.0 + p.car_half_width))
        return lethal | out | on
    return blocked


def escape_path(pose: Tuple[float, float, float], costmap: 'Costmap', lane_rel, island,
                direction: Optional[int], p: AvoidParams) -> Optional[Plan]:
    """Every curvature-limited path of three arcs (p.escape_*) from the car, with a straight
    0.4 m after: the free ones ending within escape_end_heading_deg of the lanes; the one going
    furthest along s, nearest to the target lane (any lane centre without a direction), with the
    least obstacle cost. None if none is free."""
    s0, d0, psi = pose
    ks = np.asarray(p.escape_curvatures) / p.escape_radius
    ls = np.asarray(p.escape_lengths)
    K = np.array(np.meshgrid(ks, ks, ks, indexing='ij')).reshape(3, -1).T          # (A, 3)
    L = np.array(np.meshgrid(ls, ls, ls, indexing='ij')).reshape(3, -1).T          # (B, 3)
    K = np.repeat(K, len(L), axis=0)
    L = np.tile(L, (len(ks) ** 3, 1))
    ds = 0.04
    n = int(math.ceil(L.sum(axis=1).max() / ds)) + int(0.4 / ds)
    u = (np.arange(n) + 1) * ds                                                     # (S,)
    edges = np.cumsum(L, axis=1)                                                    # (P, 3)
    seg = (u[None, :] > edges[:, 0:1]).astype(int) + (u[None, :] > edges[:, 1:2])
    kappa = np.where(u[None, :] > edges[:, 2:3], 0.0, np.take_along_axis(K, seg, axis=1))
    h = psi + np.cumsum(kappa * ds, axis=1)
    keep = np.abs(h[:, -1]) <= math.radians(p.escape_end_heading_deg)
    if not keep.any():
        return None
    h = h[keep]
    s = s0 + np.cumsum(np.cos(h) * ds, axis=1)
    d = d0 + np.cumsum(np.sin(h) * ds, axis=1)
    blocked = path_blocker(s0, costmap, lane_rel, island, p, p.escape_margin)
    bad = blocked(s.ravel(), d.ravel(), h.ravel()).reshape(s.shape).any(axis=1)
    if bad.all():
        return None
    s, d, h = s[~bad], d[~bad], h[~bad]
    tc = target_centre(direction, p)
    centres = lane_centres(p)
    end_off = (np.abs(d[:, -1] - tc) if tc is not None
               else np.min(np.abs(d[:, -1:] - centres[None, :]), axis=1))
    off = np.asarray(p.footprint_offsets)
    q = np.column_stack(((s[:, :, None] + off * np.cos(h)[:, :, None]).ravel(),
                         (d[:, :, None] + off * np.sin(h)[:, :, None]).ravel()))
    if lane_rel is not None:
        q = lane_rel.points_from(q)
    near = np.minimum(costmap.lookup(q), 100.0).reshape(len(s), -1).mean(axis=1)
    score = -(s[:, -1] - s0) + 2.0 * end_off + p.escape_w_near * near
    k = int(np.argmin(score))
    path = np.column_stack((np.concatenate(([s0], s[k])), np.concatenate(([d0], d[k]))))
    return Plan(path=path, end_offset=float(d[k, -1]), cost=float(score[k]),
                obstacle_cost=float(near[k]))


def pure_pursuit(path: np.ndarray, pose: Tuple[float, float, float], p: AvoidParams) -> float:
    """Central steering angle (rad, + left) towards the path point `lookahead` from the rear axle."""
    s0, d0, psi = pose
    rs, rd = s0 - p.rear_axle_x * math.cos(psi), d0 - p.rear_axle_x * math.sin(psi)
    dist = np.hypot(path[:, 0] - rs, path[:, 1] - rd)
    ahead = np.flatnonzero((dist >= p.lookahead) & (path[:, 0] > rs))
    if len(ahead):
        ts, td = path[ahead[0]]
    else:   # path shorter than the look-ahead: extend it straight along s
        ts, td = path[-1, 0] + p.lookahead, path[-1, 1]
    c, s = math.cos(psi), math.sin(psi)
    dx, dy = ts - rs, td - rd
    x, y = c * dx + s * dy, -s * dx + c * dy
    ld = math.hypot(x, y)
    alpha = math.atan2(y, x)
    delta = math.atan2(2.0 * p.wheelbase * math.sin(alpha), ld)
    return max(-p.max_steer, min(p.max_steer, delta))


# ---------------------------------------------------------------------------------- mission
@dataclass
class Inhibit:
    """Signals that keep the mission off (each one also aborts a running avoidance)."""
    cone_active: bool = False
    red_ratio: Optional[float] = None   # max over the last window, None = no camera signal
    parking: bool = False
    state_command: bool = False         # mission stop / go commands seen recently

    def reason(self, p: AvoidParams) -> Optional[str]:
        if self.cone_active:
            return 'cone course'
        if self.red_ratio is not None and self.red_ratio >= p.red_ratio_max:
            return 'red cones in view'
        if self.parking:
            return 'parking'
        if self.state_command:
            return 'mission stop'
        return None


class StaticAvoid:
    """Trigger, section frame, costmap, plan and control. Times in seconds, poses (x, y, yaw) in the
    PoseTracker world frame."""

    def __init__(self, p: AvoidParams):
        self.p = p
        self.state = IDLE
        self.blocked_scans = 0
        self.clear_since: Optional[float] = None
        self.armed = True
        # finished at the fork: no new start until a new white line with a blocker right after it
        self.locked = False
        self.seen_entry: Optional[Tuple[float, float, float]] = None
        self.lock_entry: Optional[Tuple[float, float, float]] = None
        self.finished = False                # handed over beside the island (one_section_per_run)
        self.frame: Optional[SectionFrame] = None
        self.lane_frame: Optional[SectionFrame] = None    # None = the section frame; else from the camera
        self.lane_source: Optional[SectionFrame] = None   # the fit it came from
        self.lane_shift = 0.0                # m of slow sideways correction so far
        self.costmap: Optional[Costmap] = None
        self.plan: Optional[Plan] = None
        self.last_end: Optional[float] = None
        self.direction: Optional[int] = None
        self.island = (p.island_s_min, p.island_s_max)   # section frame
        self.island_source = 'estimate'
        self.entry_s: Optional[float] = None
        self.event = ''                      # last state change, for the log
        self.reset_recovery()
        self.done = False                    # the section is behind us, avoided or not
        # course map, race start: start on the section start line (request_line_start), at
        # approach_speed until the first blocker
        self.line_start: Optional[Tuple[float, float, float]] = None    # the line's pose, pending
        self.approach = False

    # ---- trigger --------------------------------------------------------------------------
    def blocking(self, limos: np.ndarray, since_entry: Optional[float] = None) -> np.ndarray:
        """Mask of the LIMO-sized clusters in our lane ahead. With the entry line behind us
        (since_entry m), clusters at the island front or beyond are the sign stand / island."""
        p = self.p
        if len(limos) == 0:
            return np.zeros(0, bool)
        ahead = ((limos[:, 0] > 0.0) & (limos[:, 0] < p.activate_range)
                 & (np.abs(limos[:, 1]) < p.block_half_width))
        if since_entry is not None and -0.3 <= since_entry <= p.island_from_entry_max:
            at_island = since_entry + limos[:, 0] >= p.island_from_entry_min - p.island_object_margin
            ahead &= ~at_island
        return ahead

    def blocked(self, limos: np.ndarray, since_entry: Optional[float] = None) -> bool:
        return bool(self.blocking(limos, since_entry).any())

    def on_limos(self, t: float, limos: np.ndarray, inhibit: Inhibit, speed: float,
                 pose_world: Optional[Tuple[float, float, float]],
                 entry_back: Optional[float] = None,
                 entry_pose: Optional[Tuple[float, float, float]] = None,
                 allowed: bool = True) -> bool:
        """Call per LIMO cluster message (base_link). Returns True when the mission starts.
        entry_back: metres driven since the base crossed the white entry line, if it was seen
        (anchors the section within entry_max_back; tells the island's sign stand from a LIMO).
        entry_pose: world pose where the base crossed the line (frame origin and lane direction).
        allowed: False = a blocker does not start it (course map: not judged yet, or the race start
        where the start line starts it); a requested line start still does."""
        p = self.p
        blocked = self.blocked(limos, entry_back)
        self.seen_entry = entry_pose
        if self.state == ACTIVE:
            if blocked:
                self.approach = False
            return False
        if self.line_start is not None:
            return self._line_start(inhibit, pose_world, entry_back, entry_pose)
        self._watch(pose_world, entry_back, entry_pose)
        if self.finished and p.one_section_per_run:
            self.blocked_scans = 0
            return False
        # a new section start line ahead (up to 1 m, already confirmed) or just crossed: start as
        # usual. Waiting until the base crossed it started 0.4~0.6 m after it, 0.6 m from the
        # LIMO, after a wrong lock at the slow zone's lines (1114 car)
        retry = (self.locked and blocked and entry_pose is not None and entry_pose != self.lock_entry
                 and entry_back is not None and -1.0 <= entry_back <= p.entry_max_back)
        if self.locked and not retry:
            self.blocked_scans = 0
            return False
        if not blocked:
            self.blocked_scans = 0
            if self.clear_since is None:
                self.clear_since = t
            if t - self.clear_since >= p.rearm_clear:
                self.armed = True
            return False
        self.clear_since = None
        reason = inhibit.reason(p)
        if reason or speed < p.min_speed or not self.armed or pose_world is None \
                or (not retry and self.past_island(pose_world)):
            self.blocked_scans = 0
            return False
        self.blocked_scans += 1
        if self.blocked_scans < p.activate_scans or not allowed:
            return False
        if retry:      # a retry from the mission start: nothing of the old section is valid
            self.locked, self.frame, self.done = False, None, False
        self.activate(pose_world, entry_back, entry_pose)
        return True

    def request_line_start(self, line_pose: Tuple[float, float, float]) -> None:
        """Course map, race start: the section start line was confirmed. The mission starts on the
        next LIMO message (on_limos), as soon as no other mission inhibits it."""
        if self.state != ACTIVE and not self.finished and not self.locked:
            self.line_start = line_pose

    def _line_start(self, inhibit: Inhibit, pose_world, entry_back, entry_pose) -> bool:
        """entry_back / entry_pose: of the requested line itself (the node keeps its odometer). A line
        still ahead of the base waits: on a corner the line leaves the view sideways and is placed up
        to 1.5 m ahead; looking it up again within 1 m dropped the start (1114 23:39, no start at
        all, the car drove into the LIMOs). Dropped only once the base is entry_max_back past it."""
        p = self.p
        if entry_back is not None and entry_back > p.entry_max_back:
            self.line_start = None          # too far past it: no start from it
            self.event = 'section start line: no start (%.2f m past it)' % entry_back
            return False
        if entry_back is None or entry_pose is None:
            return False                    # not known yet: wait
        same = math.hypot(entry_pose[0] - self.line_start[0], entry_pose[1] - self.line_start[1]) < 0.1
        if not same:
            self.line_start = None
            self.event = 'section start line: no start (another line)'
            return False
        if inhibit.reason(p) or pose_world is None:
            return False
        self.line_start = None
        self.activate(pose_world, entry_back, entry_pose)
        self.approach = True
        self.event = 'start on the section start line'
        return True

    def _watch(self, pose_world, entry_back, entry_pose) -> None:
        """Idle, driven by lane_main: past the island front of the last entry line the section is done
        even though nothing blocked our lane (obstacles only in the side lanes). Locked like a hand-over,
        a retry from a new white line still releases it."""
        p = self.p
        if pose_world is None or entry_pose is None or entry_back is None or self.locked:
            return
        if not p.island_from_entry_min <= entry_back <= p.island_from_entry_max:
            return
        psi = wrap(pose_world[2] - entry_pose[2])
        if abs(psi) > math.radians(p.watch_max_turn_deg):
            return
        self.done, self.locked, self.lock_entry = True, True, entry_pose
        self.event = 'section done without avoiding, %.2f m past the entry line' % entry_back

    def activate(self, pose_world: Tuple[float, float, float],
                 entry_back: Optional[float] = None,
                 entry_pose: Optional[Tuple[float, float, float]] = None) -> None:
        """Start. Inside the section of an earlier start (car still within max_travel of its origin
        and on the road) that frame and costmap are kept: the car may now be in lane 1 or 3, or at an
        angle, and a new frame would wrongly put it at the centre of lane 2."""
        self.state = ACTIVE
        if self.frame is not None:
            s, d, _ = self.frame.to_section(*pose_world)
            if 0.0 <= s <= self.p.max_travel and abs(d) < 1.5 * self.p.lane_width:
                self.plan = None
                self.blocked_scans = 0
                self.event = 'start (same section)'
                return
        p = self.p
        self.costmap = Costmap(self.p)
        self.lane_frame, self.lane_source, self.lane_shift = None, None, 0.0
        # a line still ahead (entry_back < 0) is handled by reanchor() when the base reaches it
        anchored = (entry_back is not None and 0.0 <= entry_back <= p.entry_max_back
                    and (entry_pose is None or abs(wrap(pose_world[2] - entry_pose[2]))
                         <= math.radians(p.entry_max_turn_deg)))
        if anchored and entry_pose is not None:
            # the section frame starts on the entry line, along the car's heading there: the car may
            # still be turning out of the corner before the section when it starts (1108 bags), and a
            # frame along that heading would skew the lanes, the yellow lines and the island
            self.frame = SectionFrame(*entry_pose)
        else:
            self.frame = SectionFrame(*pose_world)
        if anchored:
            self.entry_s = 0.0 if entry_pose is not None else -entry_back
            self.island = (self.entry_s + p.island_from_entry_min,
                           self.entry_s + p.island_from_entry_max)
            self.island_source = 'entry line'
        else:
            self.entry_s = None
            self.island = (p.island_s_min, p.island_s_max)
            self.island_source = 'estimate'
        self.plan = None
        self.last_end = None
        self.blocked_scans = 0
        self.reset_recovery()
        self.event = 'start'

    def past_island(self, pose_world) -> bool:
        """Inside the last section's frame and already at its island: the island and the A/B
        roads are lane_main's, so nothing starts there (a position test, not a sequence latch)."""
        if self.frame is None:
            return False
        s = self.lane_pose(pose_world)[0]
        return self.island[0] - self.p.island_handover <= s <= self.p.max_travel + 2.0

    def stop(self, why: str, final: bool = False) -> None:
        """End. After an abort (another mission took over) a new start waits until our lane was
        clear for rearm_clear; after a hand-over nothing is left ahead, so it may start again at
        once (a LIMO seen later in the new lane must not be ignored). final = the car reached the
        fork: the mission is done, locked until a retry (see on_limos)."""
        self.state = IDLE
        self.approach, self.line_start = False, None
        if final:
            self.locked, self.lock_entry, self.done = True, self.seen_entry, True
        self.armed = why.startswith('hand over')
        self.clear_since = None
        self.event = why

    def on_entry_crossing(self, pose_world, ahead: float = 0.0) -> None:
        """The base crosses the white entry line `ahead` metres from pose_world while the mission
        runs (the first LIMO was close to the entry): place the island from it."""
        if self.frame is None or self.island_source != 'estimate' or pose_world is None:
            return              # None: that odometer reading is no longer kept (1112 run 01 crash)
        s, d, _ = self.frame.to_section(*pose_world)
        s += ahead
        # the entry line is before the LIMOs: a white line far past the start (e.g. on the A / B
        # road, 1108 run 4) is something else
        if -1.0 <= s <= self.p.entry_late_max and abs(d) < 1.5 * self.p.lane_width:
            self.entry_s = s
            self.island = (s + self.p.island_from_entry_min, s + self.p.island_from_entry_max)
            self.island_source = 'entry line'

    # ---- while active -----------------------------------------------------------------------
    def section_pose(self, pose_world) -> Tuple[float, float, float]:
        """Pose in the costmap's frame."""
        return self.frame.to_section(*pose_world)

    def lane_pose(self, pose_world) -> Tuple[float, float, float]:
        """Pose in lane coordinates: what the plans, lanes, island and hand-over use."""
        return (self.lane_frame or self.frame).to_section(*pose_world)

    def lane_in_costmap(self) -> Optional[SectionFrame]:
        if self.lane_frame is None:
            return None
        lf = self.lane_frame
        return SectionFrame(*self.frame.to_section(lf.x0, lf.y0, lf.yaw0))

    def on_lane_fit(self, fit: SectionFrame) -> bool:
        """A lane frame measured at an entry line. Used if that line is this section's and the turn is
        believable. Stop lines before the corner and lines on the A / B roads fall outside."""
        if self.state != ACTIVE or fit is self.lane_source:
            return False
        p = self.p
        s, d, yaw = self.frame.to_section(fit.x0, fit.y0, fit.yaw0)
        self.lane_source = fit                       # judged once
        if not -1.0 <= s <= p.entry_late_max or abs(d) >= 1.5 * p.lane_width:
            return False
        if abs(yaw) > math.radians(self.yaw_limit_deg()):
            self.event = 'lane fit rejected: %.1f deg' % math.degrees(yaw)
            return False
        turned = self.reframe_if_needed(fit.yaw0, yaw)
        self.lane_frame, self.lane_shift = fit, 0.0
        self.entry_s = 0.0
        self.last_end = None
        if self.island_source != 'lane tracker':
            self.island = (p.island_from_entry_min, p.island_from_entry_max)
            self.island_source = 'entry line'
        self.event = 'lane fit %+.1f deg%s' % (math.degrees(yaw), turned)
        return True

    def on_start_fit(self, fit: SectionFrame) -> bool:
        """The lane direction from the first frames after the start. Only the direction and sideways
        position: s and the island stay as they are until the entry line is crossed."""
        if self.state != ACTIVE or self.lane_frame is not None:
            return False
        yaw = self.frame.to_section(fit.x0, fit.y0, fit.yaw0)[2]
        if abs(yaw) > math.radians(self.yaw_limit_deg()):
            self.event = 'start fit rejected: %.1f deg' % math.degrees(yaw)
            return False
        turned = self.reframe_if_needed(fit.yaw0, yaw)
        self.lane_frame, self.lane_shift = fit, 0.0
        self.last_end = None
        self.event = 'start fit %+.1f deg%s' % (math.degrees(yaw), turned)
        return True

    def yaw_limit_deg(self) -> float:
        return self.p.lane_ref_max_deg if self.p.reframe else self.p.lane_yaw_max_deg

    def reframe_if_needed(self, lane_yaw: float, off: float) -> str:
        """A lane direction `off` rad from the section frame: beyond lane_yaw_max_deg the section
        frame (and its costmap, the lanes' grid) turns to it, about the same origin. The start came
        in the corner, along a heading that was not the lane's. Returns a note for the log."""
        if abs(off) <= math.radians(self.p.lane_yaw_max_deg):
            return ''
        old = self.frame
        new = SectionFrame(old.x0, old.y0, lane_yaw)
        if self.costmap is not None:
            self.costmap.resample(old, new)
        self.frame = new
        self.plan, self.sequence, self.last_end = None, None, None
        return ', section frame turned'

    def track_from_start_heading(self) -> None:
        """No start fit: track the lane lines from the start heading all the same."""
        if self.state == ACTIVE and self.lane_frame is None:
            self.lane_frame, self.lane_shift = self.frame, 0.0
            self.event = 'no start fit: tracking from the start heading'

    def update_lane(self, pose_world, obs: Optional[LaneObservation]) -> bool:
        """Slow correction of a fitted lane frame from one frame's lines."""
        if self.state != ACTIVE or self.lane_frame is None or obs is None:
            return False
        p = self.p
        s, d, psi = self.lane_frame.to_section(*pose_world)
        err = wrap(psi + obs.angle)
        residual = line_residuals(d, obs, p)
        if s < 0.0 or abs(err) > math.radians(p.lane_match_deg) or len(residual) < p.lane_fit_lines:
            return False
        step = math.radians(p.lane_step_deg)
        turn = max(-step, min(step, p.lane_gain * err))
        yaw_new = self.lane_frame.yaw0 + turn
        if abs(wrap(yaw_new - self.frame.yaw0)) > math.radians(p.lane_yaw_max_deg):
            turn, yaw_new = 0.0, self.lane_frame.yaw0
        # about the car, so that the sideways position is only changed by the term below
        cx, cy = pose_world[0], pose_world[1]
        c, sn = math.cos(turn), math.sin(turn)
        rx, ry = self.lane_frame.x0 - cx, self.lane_frame.y0 - cy
        x0, y0 = cx + c * rx - sn * ry, cy + sn * rx + c * ry
        r = float(np.median(residual))
        move = max(-p.lane_step_dist, min(p.lane_step_dist, p.lane_gain * r))
        shift = max(-p.lane_offset_max, min(p.lane_offset_max, self.lane_shift + move))
        move, self.lane_shift = shift - self.lane_shift, shift
        self.lane_frame = SectionFrame(x0 + move * math.sin(yaw_new), y0 - move * math.cos(yaw_new), yaw_new)
        return True

    def on_scan(self, hits_base: np.ndarray, lidar_base: np.ndarray, pose_world,
                pose_now_world, direction: Optional[int], keep_candidates: bool = False,
                free_dirs_base: Optional[np.ndarray] = None) -> None:
        """hits in base_link at the scan's own pose (pose_world); plan from the latest pose.
        free_dirs_base: base_link unit directions of beams that returned nothing."""
        if self.state != ACTIVE:
            return
        self.direction = direction if direction in (0, 1) else self.direction
        at_scan = self.section_pose(pose_world)
        hits = base_to_section(hits_base, at_scan) if len(hits_base) else np.zeros((0, 2))
        origin = base_to_section(lidar_base.reshape(1, 2), at_scan)[0]
        outside = None
        if len(hits) and self.lane_source is not None:
            # lanes measured by the camera: what lies beyond the yellow outer lines (cones put
            # beside the road, people, walls) is no obstacle of the section; its beams still clear
            rel = self.lane_in_costmap()
            d = (hits if rel is None else rel.points_to(hits))[:, 1]
            off = np.abs(d) > 1.5 * self.p.lane_width + self.p.outside_margin
            hits, outside = hits[~off], hits[off]
        free = None
        if free_dirs_base is not None and len(free_dirs_base):
            c, s = math.cos(at_scan[2]), math.sin(at_scan[2])
            free = free_dirs_base @ np.array([[c, s], [-s, c]])
        self.costmap.update(hits, origin, free, outside)
        now = self.lane_pose(pose_now_world)
        self.plan = plan_paths(now, now[0], self.costmap, self.direction, self.last_end,
                               self.p, keep_candidates, self.island, self.lane_in_costmap(),
                               self.island_source == 'entry line', self.recovering,
                               self.sequence)
        self.sequence = self.plan.sequence
        if self.plan.feasible:
            self.last_end = self.plan.end_offset

    def _escape(self, t: Optional[float], pose) -> Optional[Plan]:
        """The escape path while stuck (see AvoidParams.escape): kept while the rest of it stays
        free, searched again at most every escape_retry s."""
        p = self.p
        if not p.escape or t is None or self.costmap is None:
            return None
        if self.stuck_since is None:
            self.stuck_since = t
        if t - self.stuck_since < p.stuck_time:
            return None
        esc = self._escape_free(pose)
        if esc is None and (self.escape_tried is None or t - self.escape_tried >= p.escape_retry):
            self.escape_tried = t
            esc = escape_path(pose, self.costmap, self.lane_in_costmap(), self.island,
                              self.direction, p)
            if esc is not None:
                self.event = 'stuck: escape path to s %.2f d %+.2f' % (esc.path[-1, 0], esc.end_offset)
        self.escape_plan = esc
        return esc

    def _escape_free(self, pose) -> Optional[Plan]:
        """The escape path being driven, while the rest of it (to 0.3 m before its end) is free."""
        esc = self.escape_plan
        if esc is None:
            return None
        ahead = esc.path[esc.path[:, 0] > pose[0] + 0.05]
        if len(ahead) < 3 or ahead[-1, 0] - pose[0] < 0.3:
            self.escape_plan = None
            return None
        h = np.arctan2(np.gradient(ahead[:, 1]), np.gradient(ahead[:, 0]))
        if path_blocker(pose[0], self.costmap, self.lane_in_costmap(), self.island,
                        self.p)(ahead[:, 0], ahead[:, 1], h).any():
            self.escape_plan = None
            return None
        return esc

    def reset_recovery(self) -> None:
        self.escape_plan: Optional[Plan] = None
        self.escape_tried: Optional[float] = None
        self.sequence: Optional[Sequence] = None
        self.stuck_since: Optional[float] = None
        self.recovering = False
        self.recovered_from: Optional[float] = None   # s where a plan was found again
        self.backing_from: Optional[float] = None     # s where the back-up started
        self.reversed = 0.0
        self.stuck_log = None           # (t, where, logged) since every path got blocked (for the log)

    def _stuck(self, t: Optional[float], s: float) -> Tuple[float, float]:
        """Every path blocked: stop; after stuck_time plan with the sharper recovery candidates; after
        reverse_after back up (if allowed) straight along the way the car came, which is free."""
        p = self.p
        self.recovered_from = None
        if t is None:
            return 0.0, 0.0
        if self.stuck_since is None:
            self.stuck_since = t
        stuck = t - self.stuck_since
        if stuck >= p.stuck_time and not self.recovering:
            self.recovering = True
            self.event = 'stuck: recovery candidates'
        if p.recovery_reverse and stuck >= p.reverse_after and self.reversed < p.reverse_max:
            if self.backing_from is None:
                self.backing_from = s
                self.event = 'stuck: backing up'
            backed = self.backing_from - s
            if backed < min(p.reverse_distance, p.reverse_max - self.reversed):
                return -p.reverse_speed, 0.0
            self.reversed += backed
            self.backing_from, self.stuck_since = None, t          # wait for a plan again
        return 0.0, 0.0

    def control(self, pose_now_world, inhibit: Inhibit,
                t: Optional[float] = None) -> Optional[Tuple[float, float]]:
        """(v, wz) while active. None when idle. Ends the mission (state IDLE, event set) when it
        hands over or is inhibited; the caller then sends one Twist(0). t (s) enables the recovery
        when every path is blocked."""
        if self.state != ACTIVE:
            return None
        p = self.p
        reason = inhibit.reason(p)
        if reason:
            self.stop('abort: ' + reason)
            return None
        pose = self.lane_pose(pose_now_world)
        if p.handover_after_island:
            if self.past_island_ready(pose):
                return None
        elif self.target_ready(pose):
            self.stop('hand over in target lane %d, %.2f m before the island (%s)'
                      % (lane_of(pose[1], p), self.island[0] - pose[0], self.island_source), True)
            return None
        elif self.island_ready(pose):
            self.stop('hand over %.2f m before the island (%s), lane %d'
                      % (self.island[0] - pose[0], self.island_source, lane_of(pose[1], p)), True)
            return None
        elif self.release_ready(pose):
            self.stop('hand over in lane %d' % lane_of(pose[1], p))
            return None
        else:
            margin = p.island_past_estimate if self.island_source == 'estimate' else 0.0
            if pose[0] >= self.island[0] + margin:
                self.stop('hand over past the island front (%s), lane %d'
                          % (self.island_source, lane_of(pose[1], p)), True)
                return None
        if pose[0] > p.max_travel:
            self.stop('hand over: max travel', True)
            return None
        # an escape path is driven to its end while it stays free: a plan found half way out starts
        # inside the safety margin (lethal_free_start) and took the car too close
        esc = self._escape_free(pose)
        if esc is None and (self.plan is None or not self.plan.feasible):
            esc = self._escape(t, pose)
            if esc is None:
                # one log line when it lasts stuck_log_after s (blocked for a scan or two is normal),
                # with where it started and how far each lane was free then (see stuck_report)
                if self.stuck_log is None:
                    self.stuck_log = (t if t is not None else 0.0, self.stuck_report(pose), False)
                cmd = self._stuck(t, pose[0])
                t0, where, logged = self.stuck_log
                if not logged and (t is None or t - t0 >= p.stuck_log_after):
                    self.stuck_log = (t0, where, True)
                    self.event = ('stuck: every path blocked at ' + where
                                  + ('; recovery candidates' if self.recovering else ''))
                return cmd
        if esc is not None:
            delta = pure_pursuit(esc.path, pose, replace(p, lookahead=p.escape_lookahead))
            return p.escape_speed, p.escape_speed * math.tan(delta) / p.wheelbase
        if self.backing_from is not None:                  # a plan again while backing up
            self.reversed += max(self.backing_from - pose[0], 0.0)
            self.backing_from = None
        self.stuck_since = None
        if self.stuck_log is not None:
            if self.stuck_log[2]:
                self.event = ('stuck: moving again after %.1f s' % (t - self.stuck_log[0])
                              if t is not None else 'stuck: moving again')
            self.stuck_log = None
        if self.recovering:
            if self.recovered_from is None:
                self.recovered_from = pose[0]
            elif pose[0] - self.recovered_from >= p.recovery_until:
                self.recovering, self.recovered_from = False, None
        delta = pure_pursuit(self.plan.path, pose, p)
        changing = abs(self.plan.end_offset - pose[1]) > 0.10
        if changing or self.plan.obstacle_cost > p.slow_cost:
            self.approach = False
        v = p.slow_speed if (changing or self.plan.obstacle_cost > p.slow_cost) else p.speed
        if self.approach:
            v = p.approach_speed
        return v, v * math.tan(delta) / p.wheelbase

    def free_ahead(self, s: float, reach: float = 2.0) -> List[Optional[float]]:
        """Per lane (1, 2, 3): m along its centre from s to the first occupied costmap cell, None if
        none within reach."""
        p = self.p
        steps = np.arange(0.0, reach, p.resolution)
        rel = self.lane_in_costmap()
        out = []
        for c in lane_centres(p):
            pts = np.column_stack((s + steps, np.full(len(steps), c)))
            occ = self.costmap.occupied_at(pts if rel is None else rel.points_from(pts))
            out.append(float(steps[np.argmax(occ)]) if occ.any() else None)
        return out

    def stuck_report(self, pose) -> str:
        """Where the car stands and how far each lane is free ahead, for the log."""
        s, d, psi = pose
        free = self.free_ahead(s) if self.costmap is not None else [None] * 3
        lanes = ', '.join('lane %d %s' % (k + 1, '>2.0 m' if f is None else '%.2f m' % f)
                          for k, f in enumerate(free))
        why = ''
        b = self.plan.blocked if self.plan is not None else None
        if b:
            # 'obstacle margin': within the distance a full-lock swerve needs, every way passes too
            # close to an obstacle (1114_0028: 0.32 m behind a LIMO, lane 1 free but out of reach)
            why = '; %d paths blocked by: obstacle margin %d, yellow line %d, island %d' % (
                b['of'], b['obstacle margin'], b['yellow line'], b['island'])
        return ('s %.2f d %+.2f (lane %d, heading %+.0f deg); free ahead: %s%s'
                % (s, d, lane_of(d, self.p), math.degrees(psi), lanes, why))

    def release_target(self, d: float) -> float:
        """Centre of the lane the car is in: any lane, lane_main picks A/B at the island."""
        return float(lane_centres(self.p)[lane_of(d, self.p) - 1])

    def set_island_front(self, s_front: float, source: str) -> None:
        """A measured island front (section frame), e.g. from the lane tracker's island."""
        length = self.p.island_from_entry_max - self.p.island_from_entry_min
        self.island = (s_front, s_front + length)
        self.island_source = source

    def road_clear(self, s0: float, s1: float) -> bool:
        """No occupied cell across the whole road (all three lanes) between s0 and s1, lane
        coordinates."""
        p = self.p
        if s1 <= s0:
            return True
        half = 1.5 * p.lane_width
        ss, dd = np.meshgrid(np.arange(s0, s1, p.resolution), np.arange(-half, half, p.resolution),
                             indexing='ij')
        pts = np.column_stack((ss.ravel(), dd.ravel()))
        rel = self.lane_in_costmap()
        return not self.costmap.occupied_at(pts if rel is None else rel.points_from(pts)).any()

    def aligned(self, d: float, psi: float, centre: float) -> bool:
        return (abs(d - centre) <= self.p.release_tol
                and abs(psi) <= math.radians(self.p.release_heading_deg))

    def target_ready(self, pose) -> bool:
        """The direction is known: centred and straight in the A / B lane, the island within
        release_clear (so the whole way to it has been seen) and nothing left on it."""
        tc = target_centre(self.direction, self.p)
        if tc is None or self.plan is None:
            return False
        s, d, psi = pose
        if not self.aligned(d, psi, tc):
            return False
        if self.island[0] - s > self.p.release_clear:
            return False
        return self.road_clear(s + 0.2, self.island[0] - self.p.island_object_margin)

    def island_ready(self, pose) -> bool:
        """Direction unknown: island front within island_handover, the car centred and straight in
        a lane (not waited for inside island_align_until), and no obstacle left in any lane up to it.
        With a direction, target_ready decides."""
        s, d, psi = pose
        gap = self.island[0] - s
        if self.direction in (0, 1) or gap > self.p.island_handover or gap < -0.5:
            return False
        if gap > self.p.island_align_until and not self.aligned(d, psi, self.release_target(d)):
            return False
        return self.road_clear(s + 0.2, self.island[0])

    def past_island_ready(self, pose) -> bool:
        """handover_after_island: hand over (and lock) handover_past m beside the island in lane 1 / 3
        and about straight; handover_overshoot further on whatever the heading. Logs the sign's lane
        with it (whether lane_main's /direction_flag side matches is seen there). finished (no new
        start this run) only with the island from the entry line or the lane tracker: after a false
        start before the section the estimated island is reached too, and the real section that
        follows must still start (locked, the retry from a new white line applies)."""
        p = self.p
        s, d, psi = pose
        margin = p.island_past_estimate if self.island_source == 'estimate' else 0.0
        past = s - (self.island[0] + margin)
        if past < p.handover_past:
            return False
        lane = lane_of(d, p)
        sign = {0: 'sign lane 1', 1: 'sign lane 3'}.get(self.direction, 'no sign')
        if lane in (1, 3) and abs(psi) <= math.radians(p.handover_heading_deg):
            self.finished = self.island_source != 'estimate'
            self.stop('hand over beside the island, lane %d, %.2f m past its front (%s, %s)'
                      % (lane, past, self.island_source, sign), True)
            return True
        if past >= p.handover_past + p.handover_overshoot:
            self.finished = self.island_source != 'estimate'
            self.stop('hand over beside the island, lane %d, %.2f m past its front, heading %.0f deg '
                      '(%s, %s)' % (lane, past, math.degrees(psi), self.island_source, sign), True)
            return True
        return False

    def release_ready(self, pose) -> bool:
        p = self.p
        s, d, psi = pose
        if s < p.min_travel or self.direction in (0, 1):
            return False
        tc = self.release_target(d)
        if abs(d - tc) > p.release_tol:
            return False
        if abs(psi) > math.radians(p.release_heading_deg):
            return False
        return self.road_clear(s + 0.2, s + p.release_clear)

    def lane(self, pose_world) -> Optional[int]:
        if self.state != ACTIVE:
            return None
        return lane_of(self.lane_pose(pose_world)[1], self.p)


def line_residuals(d: float, obs: LaneObservation, p: AvoidParams) -> List[float]:
    """For each seen line near an expected one (+-w/2 dashes and island edge, +-1.5 w yellow): expected
    minus measured lateral position in lane coordinates, car at lateral d."""
    expected = p.lane_width * np.array([-1.5, -0.5, 0.5, 1.5])
    out = []
    for line in obs.lines:
        m = d + line.y0 * math.cos(obs.angle)
        r = expected - m
        k = int(np.argmin(np.abs(r)))
        if abs(r[k]) <= p.lane_match_dist:
            out.append(float(r[k]))
    return out


class LaneFitter:
    """The H at the section entry: the white entry line gives s = 0, the lane lines seen just after it
    (+ IMU yaw) the lane direction and sideways position. Lines before the entry line are the corner
    and are not used; nor is the slant of the entry line (it is not the car's heading, 1108 bags)."""

    def __init__(self, p: AvoidParams, cam: CameraModel, colours: ColourParams):
        self.p, self.cam, self.colours = p, cam, colours
        self.crossing: Optional[float] = None    # odometer of the line being measured
        self.closed: Optional[float] = None      # odometer of the last line finished
        self.samples: List[Tuple[Tuple[float, float, float], LaneObservation]] = []
        self.fit: Optional[SectionFrame] = None
        self.start_frame: Optional[SectionFrame] = None      # the start being measured
        self.start_samples: List[Tuple[Tuple[float, float, float], LaneObservation]] = []
        self.start_odometer = 0.0
        self.start_open = False

    def on_frame(self, bgr: np.ndarray, avoid: StaticAvoid, tracker: PoseTracker, entry,
                 t: Optional[float] = None) -> None:
        """t: the image's stamp; its pose and odometer are used (a frame processed late must not
        be placed where the car is now). None: the latest."""
        p = self.p
        pose = tracker.latest() if t is None else tracker.at(t)
        odometer = tracker.distance if t is None else tracker.distance_at(t)
        t_rate = t if t is not None else (tracker.ytrace[-1][0] if tracker.ytrace else None)
        rate = tracker.yaw_rate(t_rate) if t_rate is not None else None
        # turning hard (still in the corner): the lines' angle and the heading at that instant do not
        # go together well enough for the direction (1112 run 02: -37.8 deg)
        steady = rate is None or abs(rate) <= math.radians(p.lane_ref_rate_max_deg)
        c = entry.recent(odometer, p.lane_fit_give_up, 0.0, start_only=True)
        if c is not None and c.odometer != self.closed:
            entry_pose = tracker.at_distance(c.odometer)
            if entry_pose is not None:
                if c.odometer != self.crossing:
                    self.crossing, self.samples = c.odometer, []
                if steady:
                    obs = self._lines(bgr, wrap(entry_pose[2] - pose[2]))
                    if obs is not None:
                        self.samples.append((pose, obs))
                self._try_fit(entry_pose, odometer - c.odometer)
        if avoid.state == ACTIVE:
            self._start_fit(bgr, avoid, odometer, pose, steady)
            if self.fit is not None:
                avoid.on_lane_fit(self.fit)
            if avoid.lane_frame is not None:
                obs = find_lane_lines(bgr, self.cam, self.colours, wrap(avoid.lane_frame.yaw0 - pose[2]),
                                      math.radians(p.lane_track_search_deg))
                avoid.update_lane(pose, obs)

    def _lines(self, bgr, centre: float, white_only: bool = False) -> Optional[LaneObservation]:
        """Lane lines for a fit: enough of them, and not on the edge of the search (the lines'
        direction is then further off, 1112: -20.0 deg with a 20 deg search)."""
        p = self.p
        half = math.radians(p.lane_fit_search_deg)
        obs = find_lane_lines(bgr, self.cam, self.colours, centre, half, white_only=white_only)
        if obs is None or len(obs.lines) < p.lane_fit_lines:
            return None
        if abs(wrap(obs.angle - centre)) >= half - math.radians(0.5):
            return None
        return obs

    def _start_fit(self, bgr, avoid: StaticAvoid, odometer: float, pose, steady: bool = True) -> None:
        """The white dashes ahead give the lane direction right after the start, the car may still be
        coming out of the corner. If they do not, the tracking still runs (from the start heading)."""
        p = self.p
        frame = avoid.frame
        if frame is not self.start_frame:
            self.start_frame, self.start_samples = frame, []
            self.start_odometer, self.start_open = odometer, True
        if not self.start_open:
            return
        if avoid.lane_frame is not None:
            self.start_open = False
            return
        obs = self._lines(bgr, wrap(frame.yaw0 - pose[2]), white_only=True) if steady else None
        if obs is not None:
            self.start_samples.append((pose, obs))
        origin = (frame.x0, frame.y0, frame.yaw0)
        fit = self._fit(origin, self.start_samples, p.lane_start_frames)
        if fit is not None and avoid.on_start_fit(fit):
            self.start_open = False
        elif odometer - self.start_odometer >= p.lane_start_give_up:
            self.start_open = False
            avoid.track_from_start_heading()

    def _try_fit(self, entry_pose, past: float) -> None:
        p = self.p
        if past < p.lane_fit_window:
            return
        fit = self._fit(entry_pose)
        if fit is not None or past >= p.lane_fit_give_up:
            self.closed, self.samples = self.crossing, []
        if fit is not None:
            self.fit = fit

    def _fit(self, entry_pose, samples=None, frames: Optional[int] = None) -> Optional[SectionFrame]:
        p = self.p
        samples = self.samples if samples is None else samples
        if len(samples) < (p.lane_fit_frames if frames is None else frames):
            return None
        base = entry_pose[2]
        turns = np.array([wrap(q[2] + o.angle - base) for q, o in samples])
        turn = float(np.median(turns))
        if float(np.median(np.abs(turns - turn))) > math.radians(p.lane_fit_spread_deg):
            return None
        yaw = base + turn
        frame = SectionFrame(entry_pose[0], entry_pose[1], yaw)
        residual = []
        for q, o in samples:
            residual += line_residuals(frame.to_section(*q)[1], o, p)
        r = float(np.median(residual)) if residual else 0.0
        r = max(-p.lane_offset_max, min(p.lane_offset_max, r))
        # the true lane 2 centre is -r off the frame's axis (left positive)
        return SectionFrame(entry_pose[0] + r * math.sin(yaw), entry_pose[1] - r * math.cos(yaw), yaw)
