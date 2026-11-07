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
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt

LETHAL = 1000.0
IDLE, ACTIVE = 'idle', 'active'


@dataclass
class AvoidParams:
    # geometry (m)
    lane_width: float = 0.50          # line centre to centre, our track; rules say 0.55
    car_half_width: float = 0.11      # LIMO 0.22 m wide
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
    # painted island in lane 2. Measured from the white entry line when the camera saw it
    # (rule book p.4 drawing at scale), else guessed from the activation point (~1.5 m before the
    # first LIMO; the 11-05 bags put the true island at 3.6~5.5 m from activation)
    island_from_entry_min: float = 4.40
    island_from_entry_max: float = 6.15
    entry_max_back: float = 3.0       # m, an entry-line crossing this recent anchors the section
    island_s_min: float = 3.3         # fallback, from the activation point
    island_s_max: float = 5.0
    island_handover: float = 1.0      # m, hand over to lane_main when the island front is this close
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
    # the first metres of every path start where the car is now: if the car is already inside the
    # safety margin (an obstacle seen late, or passed close), lethal cells there only cost, so the
    # car can still pick the way out instead of freezing
    lethal_free_start: float = 0.25
    w_obstacle: float = 1.0
    w_centre: float = 40.0            # per m^2 off the nearest lane centre
    w_lateral: float = 2.0            # per m of offset change
    w_short: float = 0.5              # per 1/m of transition length
    w_hysteresis: float = 4.0         # per m away from the last chosen end offset
    # per m^2 away from the A/B lane centre; weak: lane_main chooses A/B at the island, this leans
    w_target: float = 20.0
    w_lane2: float = 3.0              # lane 2 end without a direction (the island ahead)
    # x share of path samples on the painted island; soft because its position is only estimated
    # and a hard limit can dead-lock the car in lane 2 when the fork sign comes late
    w_island: float = 200.0
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

    def __init__(self, history: float = 2.0):
        self.history = history
        self.yaw: Optional[float] = None
        self.last_t: Optional[float] = None
        self.x = self.y = 0.0
        self.speed = 0.0
        self.distance = 0.0           # odometer, m
        self.trace: List[Tuple[float, float, float, float]] = []

    def imu(self, t: float, yaw: float) -> None:
        # unwrap so interpolation never jumps by 2 pi
        self.yaw = yaw if self.yaw is None else self.yaw + wrap(yaw - self.yaw)

    def odom(self, t: float, v: float) -> None:
        self.speed = v
        if self.yaw is None:
            return
        if self.last_t is not None:
            dt = min(max(t - self.last_t, 0.0), 0.1)
            self.x += v * dt * math.cos(self.yaw)
            self.y += v * dt * math.sin(self.yaw)
            self.distance += v * dt
        self.last_t = t
        self.trace.append((t, self.x, self.y, self.yaw))
        while self.trace and self.trace[0][0] < t - self.history:
            self.trace.pop(0)

    @property
    def ready(self) -> bool:
        return bool(self.trace)

    def latest(self) -> Tuple[float, float, float]:
        return self.trace[-1][1:]

    def at(self, t: float) -> Tuple[float, float, float]:
        """Pose at time t, interpolated, clamped to the history."""
        tr = self.trace
        if t <= tr[0][0]:
            return tr[0][1:]
        if t >= tr[-1][0]:
            return tr[-1][1:]
        lo, hi = 0, len(tr) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if tr[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        a, b = tr[lo], tr[hi]
        f = (t - a[0]) / max(b[0] - a[0], 1e-9)
        return tuple(a[k] + f * (b[k] - a[k]) for k in (1, 2, 3))


@dataclass
class SectionFrame:
    x0: float
    y0: float
    yaw0: float

    def to_section(self, x: float, y: float, yaw: float) -> Tuple[float, float, float]:
        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        dx, dy = x - self.x0, y - self.y0
        return c * dx + s * dy, -s * dx + c * dy, wrap(yaw - self.yaw0)


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
        self.d_max = 1.5 * p.lane_width + 0.15
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
               free_dirs: Optional[np.ndarray] = None) -> None:
        """hits: (N, 2) beam end points, origin: (2,) lidar position, both in the section frame.
        free_dirs: (M, 2) unit directions of beams with no return; they clear up to
        free_range_no_return, so an obstacle that left is forgotten even with nothing behind it."""
        p = self.p
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


def plan_paths(pose: Tuple[float, float, float], travelled: float, costmap: Costmap,
               direction: Optional[int], last_end: Optional[float], p: AvoidParams,
               keep_candidates: bool = False,
               island: Optional[Tuple[float, float]] = None) -> Plan:
    s0, d0, _ = pose
    island = island or (p.island_s_min, p.island_s_max)
    w = p.lane_width
    ends = np.arange(-p.end_span * w, p.end_span * w + 1e-9, p.end_step)
    lengths = np.asarray(p.transition_lengths)
    E, Lt = np.meshgrid(ends, lengths, indexing='ij')
    E, Lt = E.ravel(), Lt.ravel()                                   # (P,)
    ds = np.arange(p.path_ds, p.horizon + 1e-9, p.path_ds)          # (S,)
    u = np.clip(ds[None, :] / Lt[:, None], 0.0, 1.0)
    d = d0 + (E - d0)[:, None] * (1.0 - np.cos(math.pi * u)) / 2.0  # (P, S)
    slope = np.where(u < 1.0, (E - d0)[:, None] * math.pi / (2.0 * Lt[:, None])
                     * np.sin(math.pi * u), 0.0)
    theta = np.arctan(slope)
    s = np.broadcast_to(s0 + ds[None, :], d.shape)

    # footprint circles along the car at every path point
    off = np.asarray(p.footprint_offsets)
    fs = s[..., None] + off * np.cos(theta)[..., None]
    fd = d[..., None] + off * np.sin(theta)[..., None]
    c = costmap.lookup(np.column_stack((fs.ravel(), fd.ravel()))).reshape(fs.shape)
    lethal = ((c >= LETHAL) & (ds[None, :, None] > p.lethal_free_start)).any(axis=(1, 2))
    obstacle = c.mean(axis=(1, 2))

    d_limit = 1.5 * w - p.car_half_width - p.line_margin
    out_of_road = (np.abs(d) > d_limit).any(axis=1)
    on_island = ((s >= island[0]) & (s <= island[1])
                 & (np.abs(d) < w / 2.0)).mean(axis=1)

    centres = lane_centres(p)
    off_centre = np.min((d[..., None] - centres) ** 2, axis=2).mean(axis=1)
    cost = (p.w_obstacle * obstacle + p.w_centre * off_centre + p.w_lateral * np.abs(E - d0)
            + p.w_short / Lt + p.w_island * on_island)
    if last_end is not None:
        cost += p.w_hysteresis * np.abs(E - last_end)
    tc = target_centre(direction, p)
    if tc is not None:
        cost += p.w_target * (E - tc) ** 2
    else:
        lane2 = np.abs(E) < w / 2.0
        weight = p.w_lane2 * (4.0 if travelled >= p.fork_commit else 1.0)
        cost += weight * lane2

    bad = lethal | out_of_road
    cost = np.where(bad, np.inf, cost)
    best = int(np.argmin(cost))
    feasible = bool(np.isfinite(cost[best]))
    path = np.column_stack((s[best], d[best]))
    cands = []
    if keep_candidates:
        for k in range(0, len(E), 2):
            cands.append((np.column_stack((s[k], d[k])), float(cost[k])))
    return Plan(path=path, end_offset=float(E[best]), cost=float(cost[best]),
                obstacle_cost=float(obstacle[best]), candidates=cands, feasible=feasible)


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
        self.frame: Optional[SectionFrame] = None
        self.costmap: Optional[Costmap] = None
        self.plan: Optional[Plan] = None
        self.last_end: Optional[float] = None
        self.direction: Optional[int] = None
        self.island = (p.island_s_min, p.island_s_max)   # section frame
        self.island_source = 'estimate'
        self.entry_s: Optional[float] = None
        self.event = ''                      # last state change, for the log

    # ---- trigger --------------------------------------------------------------------------
    def blocked(self, limos: np.ndarray) -> bool:
        p = self.p
        if len(limos) == 0:
            return False
        ahead = ((limos[:, 0] > 0.0) & (limos[:, 0] < p.activate_range)
                 & (np.abs(limos[:, 1]) < p.block_half_width))
        return bool(ahead.any())

    def on_limos(self, t: float, limos: np.ndarray, inhibit: Inhibit, speed: float,
                 pose_world: Optional[Tuple[float, float, float]],
                 entry_back: Optional[float] = None) -> bool:
        """Call per LIMO cluster message (base_link). Returns True when the mission starts.
        entry_back: metres driven since the base crossed the white entry line, if it was seen."""
        p = self.p
        blocked = self.blocked(limos)
        if self.state == ACTIVE:
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
                or self.past_island(pose_world):
            self.blocked_scans = 0
            return False
        self.blocked_scans += 1
        if self.blocked_scans < p.activate_scans:
            return False
        self.activate(pose_world, entry_back)
        return True

    def activate(self, pose_world: Tuple[float, float, float],
                 entry_back: Optional[float] = None) -> None:
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
        self.frame = SectionFrame(*pose_world)
        self.costmap = Costmap(self.p)
        p = self.p
        if entry_back is not None and -0.3 <= entry_back <= p.entry_max_back:
            self.entry_s = -entry_back
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
        self.event = 'start'

    def past_island(self, pose_world) -> bool:
        """Inside the last section's frame and already at its island: the island and the A/B
        roads are lane_main's, so nothing starts there (a position test, not a sequence latch)."""
        if self.frame is None:
            return False
        s, d, _ = self.frame.to_section(*pose_world)
        return (self.island[0] - self.p.island_handover <= s <= self.p.max_travel + 2.0
                and abs(d) < 2.0 * self.p.lane_width)

    def stop(self, why: str) -> None:
        """End. After an abort (another mission took over) a new start waits until our lane was
        clear for rearm_clear; after a hand-over nothing is left ahead, so it may start again at
        once (a LIMO seen later in the new lane must not be ignored)."""
        self.state = IDLE
        self.armed = why.startswith('hand over')
        self.clear_since = None
        self.event = why

    def on_entry_crossing(self, pose_world, ahead: float = 0.0) -> None:
        """The base crosses the white entry line `ahead` metres from pose_world while the mission
        runs (the first LIMO was close to the entry): place the island from it."""
        if self.frame is None or self.island_source != 'estimate':
            return
        s, d, _ = self.frame.to_section(*pose_world)
        s += ahead
        if -1.0 <= s <= self.p.max_travel and abs(d) < 1.5 * self.p.lane_width:
            self.entry_s = s
            self.island = (s + self.p.island_from_entry_min, s + self.p.island_from_entry_max)
            self.island_source = 'entry line'

    # ---- while active -----------------------------------------------------------------------
    def section_pose(self, pose_world) -> Tuple[float, float, float]:
        return self.frame.to_section(*pose_world)

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
        free = None
        if free_dirs_base is not None and len(free_dirs_base):
            c, s = math.cos(at_scan[2]), math.sin(at_scan[2])
            free = free_dirs_base @ np.array([[c, s], [-s, c]])
        self.costmap.update(hits, origin, free)
        now = self.section_pose(pose_now_world)
        self.plan = plan_paths(now, now[0], self.costmap, self.direction, self.last_end,
                               self.p, keep_candidates, self.island)
        if self.plan.feasible:
            self.last_end = self.plan.end_offset

    def control(self, pose_now_world, inhibit: Inhibit) -> Optional[Tuple[float, float]]:
        """(v, wz) while active. None when idle. Ends the mission (state IDLE, event set) when it
        hands over or is inhibited; the caller then sends one Twist(0)."""
        if self.state != ACTIVE:
            return None
        p = self.p
        reason = inhibit.reason(p)
        if reason:
            self.stop('abort: ' + reason)
            return None
        pose = self.section_pose(pose_now_world)
        if self.island_ready(pose):
            self.stop('hand over %.2f m before the island (%s), lane %d'
                      % (self.island[0] - pose[0], self.island_source, lane_of(pose[1], p)))
            return None
        if self.release_ready(pose):
            self.stop('hand over in lane %d' % lane_of(pose[1], p))
            return None
        if pose[0] > p.max_travel:
            self.stop('hand over: max travel')
            return None
        if self.plan is None:
            return 0.0, 0.0
        if not self.plan.feasible:
            return 0.0, 0.0
        delta = pure_pursuit(self.plan.path, pose, p)
        changing = abs(self.plan.end_offset - pose[1]) > 0.10
        v = p.slow_speed if (changing or self.plan.obstacle_cost > p.slow_cost) else p.speed
        return v, v * math.tan(delta) / p.wheelbase

    def release_target(self, d: float) -> float:
        """Centre of the lane the car is in: any lane, lane_main picks A/B at the island."""
        return float(lane_centres(self.p)[lane_of(d, self.p) - 1])

    def set_island_front(self, s_front: float, source: str) -> None:
        """A measured island front (section frame), e.g. from the lane tracker's island."""
        length = self.p.island_from_entry_max - self.p.island_from_entry_min
        self.island = (s_front, s_front + length)
        self.island_source = source

    def road_clear(self, s0: float, s1: float) -> bool:
        """No occupied cell across the whole road (all three lanes) between s0 and s1."""
        half = 1.5 * self.p.lane_width
        return not self.costmap.occupied_in(s0, s1, -half, half)

    def island_ready(self, pose) -> bool:
        """Island front within island_handover and no obstacle left in any lane up to it."""
        s = pose[0]
        gap = self.island[0] - s
        if gap > self.p.island_handover or gap < -0.5:
            return False
        return self.road_clear(s + 0.2, self.island[0])

    def release_ready(self, pose) -> bool:
        p = self.p
        s, d, psi = pose
        if s < p.min_travel:
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
        return lane_of(self.section_pose(pose_world)[1], self.p)
