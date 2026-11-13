"""Six painted bays registered by vision, verified by recent lidar rays.

Unknown is not free. There is no fixed world pose, bag timestamp, mission order,
or assumption that a gap between two returns is traversable. All units are SI.
"""
import math
from collections import deque
from dataclasses import dataclass
import numpy as np
from clothoid_parking.bay_finder import Bay
from clothoid_parking.planner import wrap


def unique_cells(points, resolution):
    """Sort one packed int64 per cell instead of structured two-column rows."""
    indices = np.floor(np.asarray(points)/resolution).astype(np.int64)
    packed = np.unique((indices[:, 0] << 32) | (indices[:, 1] & 0xffffffff))
    return np.column_stack((packed >> 32, (packed & 0xffffffff).astype(np.int32)))


@dataclass
class LayoutParams:
    enabled: bool = True
    control_enabled: bool = False  # observe until bag + closed-loop validation is sufficient
    target_bay: int = 0  # 0: observe all; control requires an explicitly designated bay (1..6)
    columns: int = 3
    rows: int = 2
    bay_width: float = .50
    bay_depth: float = .60
    line_width: float = .02
    side: float = -1.
    confirm_frames: int = 3
    position_tolerance: float = .07
    angle_tolerance: float = .07
    observation_gap: float = .65
    max_pose_age: float = 20.
    grid: float = .04
    scan_hz: float = 5.
    evidence_age: float = 1.5
    lidar_range: float = 3.5
    confirm_scans: int = 3
    body_length: float = .322
    body_width: float = .220
    clearance: float = .03
    observation_stop_margin: float = .08  # braking + callback latency before the turn point


class ParkingLayout:
    def __init__(self, params=None, min_hits=2):
        self.p = params or LayoutParams()
        self.min_hits = min_hits
        if self.p.grid <= 0 or self.p.scan_hz <= 0 or self.p.side not in (-1, 1):
            raise ValueError('invalid layout geometry/rate')
        if not isinstance(self.p.target_bay, int) or not 0 <= self.p.target_bay <= self.p.rows*self.p.columns:
            raise ValueError('target_bay outside layout')
        if not math.isfinite(self.p.observation_stop_margin) or self.p.observation_stop_margin < 0:
            raise ValueError('invalid observation stop margin')
        if self.p.control_enabled and not self.p.target_bay:
            raise ValueError('registered control requires a designated target_bay')
        p = self.p
        margin = p.clearance+p.position_tolerance
        u = np.linspace(-p.body_length/2-margin, p.body_length/2+margin,
                        math.ceil((p.body_length+2*margin)/(p.grid/2))+1)
        v = np.linspace(-p.body_width/2-margin, p.body_width/2+margin,
                        math.ceil((p.body_width+2*margin)/(p.grid/2))+1)
        u, v = np.meshgrid(u, v)
        self._footprint_offsets = np.column_stack((u.ravel(), v.ravel()))
        self.reset()

    def reset(self):
        self.origin = None
        self.yaw = 0.
        self.samples = deque(maxlen=self.p.confirm_frames)
        self.last_observation = -math.inf
        self.last_scan = -math.inf
        self.last_choice_scan = -math.inf
        self.cells = {}  # world grid -> (last evidence time, occupied)
        self.endpoint_hits = {}  # cell -> (last hit time, distinct scan count)
        self.retained_free = set()
        self.retaining = False
        self.states = ['unknown'] * (self.p.rows*self.p.columns)
        self.counts = [0] * len(self.states)
        self.selected = None
        self.reason = 'waiting for lane/entry/grid registration'
        self.support_count = 0
        self.motion_reason = ''

    def retain_free(self, now, enabled):
        """Keep measured free cells only for one committed entry manoeuvre."""
        self.retaining = enabled
        self.retained_free = ({k for k, (t, occupied) in self.cells.items()
                               if not occupied and 0 <= now-t <= self.p.evidence_age}
                              if enabled else set())

    def observe(self, stamp, origin, yaw, pose):
        if stamp <= self.last_observation:
            return False
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        world = np.array([pose.x+c*origin[0]-s*origin[1], pose.y+s*origin[0]+c*origin[1]])
        heading = wrap(yaw+pose.yaw)
        if not np.isfinite([*world, heading]).all():
            return False
        previous = self.samples[-1] if self.samples else None
        if previous is not None and (stamp-self.last_observation > self.p.observation_gap or
                np.linalg.norm(world-previous[0]) > self.p.position_tolerance or
                abs(wrap(heading-previous[1])) > self.p.angle_tolerance):
            self.samples.clear()
        if self.origin is not None and len(self.samples) == 0:
            self.retain_free(stamp, False)
            self.origin = None
            self.selected = None
            self.counts = [0]*len(self.counts)
            self.support_count = 0
        self.last_observation = stamp
        self.samples.append((world, heading))
        if len(self.samples) < self.p.confirm_frames:
            return False
        points = np.array([x[0] for x in self.samples])
        angles = np.array([wrap(x[1]-heading) for x in self.samples])
        if np.max(np.linalg.norm(points-np.median(points, axis=0), axis=1)) > self.p.position_tolerance:
            return False
        candidate = np.median(points, axis=0)
        angle = wrap(heading+float(np.median(angles)))
        if self.origin is not None and (np.linalg.norm(candidate-self.origin) > self.p.position_tolerance or
                                       abs(wrap(angle-self.yaw)) > self.p.angle_tolerance):
            # Contradictory anchors revoke the plan instead of silently moving a frozen target.
            self.retain_free(stamp, False)
            self.origin = None
            self.counts = [0]*len(self.counts)
            self.selected = None
            self.reason = 'inconsistent visual registration'
            return False
        self.origin, self.yaw = candidate, angle
        # Observations may move the reference during search, never preserve a
        # manoeuvre cache across such a reference update.
        self.retain_free(stamp, False)
        self.reason = 'registered; checking observed free space'
        return True

    def bays(self):
        if self.origin is None:
            return []
        p = self.p
        n = np.array([math.cos(self.yaw), math.sin(self.yaw)])
        inward = p.side*np.array([-n[1], n[0]])
        out = []
        for row in range(p.rows):
            for col in range(p.columns):
                face = self.origin + ((col+.5)*(p.bay_width+p.line_width))*n + row*(p.bay_depth+p.line_width)*inward
                centre = face + (p.bay_depth+p.line_width)/2*inward
                out.append(Bay(*centre, self.yaw, self.yaw+p.side*math.pi/2,
                               p.bay_width, *face))
        return out

    def scan(self, stamp, ranges, angle_min, angle_increment, range_min, range_max, pose, laser_x):
        if stamp-self.last_scan < 1/self.p.scan_hz-1e-6:
            return False
        self.last_scan = stamp
        p = self.p
        r = np.asarray(ranges, dtype=float)
        a = angle_min + np.arange(len(r))*angle_increment + pose.yaw
        good = np.isfinite(r) & (r > max(.02, range_min)) & (r < range_max)
        r, a = r[good], a[good]
        # Invalid/no-return readings never certify free space.
        start = np.array([pose.x+laser_x*math.cos(pose.yaw), pose.y+laser_x*math.sin(pose.yaw)])
        steps = np.arange(p.grid/2, p.lidar_range, p.grid/2)
        lengths = np.minimum(r, p.lidar_range)
        valid = steps[None, :] < (lengths[:, None]-p.grid)
        x = start[0]+np.cos(a)[:, None]*steps
        y = start[1]+np.sin(a)[:, None]*steps
        free = unique_cells(np.column_stack((x[valid], y[valid])), p.grid)
        end = r <= p.lidar_range
        endpoints = start + np.column_stack((np.cos(a[end]), np.sin(a[end])))*r[end, None]
        hits = set(map(tuple, np.floor(endpoints/p.grid).astype(int)))
        # Multiple beams in one scan are one observation. Endpoints win over
        # clearing rays from that same scan; a later clearing ray resets hits.
        for key in map(tuple, free):
            if key not in hits:
                self.cells[key] = (stamp, False)
                self.endpoint_hits.pop(key, None)
                if self.retaining:
                    self.retained_free.add(key)
        self.endpoint_hits = {k: v for k, v in self.endpoint_hits.items()
                              if stamp-v[0] <= p.evidence_age}
        for key in hits:
            # Even one contradictory return revokes remembered free space.
            self.retained_free.discard(key)
            count = min(self.min_hits, self.endpoint_hits.get(key, (stamp, 0))[1]+1)
            self.endpoint_hits[key] = (stamp, count)
            if count >= self.min_hits:
                self.cells[key] = (stamp, True)
            else:
                # An unconfirmed return is not evidence of free space.
                self.cells.pop(key, None)
        self.cells = {k: v for k, v in self.cells.items() if stamp-v[0] <= p.evidence_age}
        return True

    def coverage(self, points, now):
        keys = unique_cells(points, self.p.grid)
        values = [self.cells.get(tuple(k)) for k in keys]
        if any(v is not None and now-v[0] <= self.p.evidence_age and v[1] for v in values):
            return 'occupied'
        if not values or any(v is None or now-v[0] > self.p.evidence_age for v in values):
            return 'unknown'
        return 'free'

    def footprint(self, x, y, yaw):
        # Include registration tolerance in the envelope; uncertain poses cost usable clearance.
        c, s = math.cos(yaw), math.sin(yaw)
        return self._footprint_offsets @ np.array([[c, s], [-s, c]]) + [x, y]

    def inside_bay(self, bay, pose):
        """All four physical corners, not merely the longitudinal goal error."""
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        corners = np.array([(u, v) for u in (-self.p.body_length/2, self.p.body_length/2)
                            for v in (-self.p.body_width/2, self.p.body_width/2)])
        world = corners @ np.array([[c, s], [-s, c]]) + [pose.x-bay.x, pose.y-bay.y]
        n = np.array([math.cos(bay.row_yaw), math.sin(bay.row_yaw)])
        return bool((np.abs(world@n) <= self.p.bay_width/2).all() and
                    (np.abs(world@np.array([-n[1], n[0]])) <= self.p.bay_depth/2).all())

    def path(self, bay, pose, plan):
        """Swept envelope of the existing forward arc + straight entry, in world coordinates."""
        n = np.array([math.cos(bay.row_yaw), math.sin(bay.row_yaw)])
        inward = plan.side*np.array([-n[1], n[0]])
        rear = np.array([pose.x, pose.y])-plan.rear_offset*n
        along = float((np.array([bay.face_x, bay.face_y])-rear)@n)
        distance = along-plan.turn_radius
        if distance < -.1 or abs(wrap(pose.yaw-bay.row_yaw)) > .15:
            return None
        start = rear+distance*n
        poses = []
        for d in np.arange(0, max(0., distance), .04):
            q = rear+d*n+plan.rear_offset*n
            poses.append((*q, bay.row_yaw))
        for t in np.linspace(0, math.pi/2, 25):
            yaw = bay.row_yaw+plan.side*t
            q = start+plan.turn_radius*math.sin(t)*n+plan.turn_radius*(1-math.cos(t))*inward
            q += plan.rear_offset*np.array([math.cos(yaw), math.sin(yaw)])
            poses.append((*q, yaw))
        end = np.array(poses[-1][:2]); target = np.array([bay.x, bay.y])
        for f in np.linspace(0, 1, max(2, int(np.linalg.norm(target-end)/.04)+1)):
            poses.append((*(end+f*(target-end)), bay.entry_yaw))
        return np.concatenate([self.footprint(*q) for q in poses])

    def supported(self, now):
        """Corroborate visual registration with separated obstacle groups.

        Cars need not be centred or parallel. A continuous wall is not two
        groups; missing/occluded evidence cannot corroborate the registration.
        This check never estimates the target pose from obstacle faces.
        """
        if self.origin is None:
            return False
        pts = np.array([(np.array(k)+.5)*self.p.grid for k, v in self.cells.items()
                        if v[1] and now-v[0] <= self.p.evidence_age])
        if len(pts) < 4:
            return False
        n = np.array([math.cos(self.yaw), math.sin(self.yaw)])
        inward = self.p.side*np.array([-n[1], n[0]])
        u, depth = (pts-self.origin)@n, (pts-self.origin)@inward
        pitch = self.p.bay_width+self.p.line_width
        in_row = (u>0)&(u<self.p.columns*pitch)&(depth>.08)&(depth<self.p.rows*(self.p.bay_depth+self.p.line_width))
        u = u[in_row]
        if len(u) < 4:
            return False
        index = np.floor(u/pitch).astype(int)
        columns = {int(i) for i in index if np.sum(index == i) >= 2}
        ordered = np.sort(u)
        splits = np.flatnonzero(np.diff(ordered) > 2*self.p.grid)+1
        groups = np.split(ordered, splits)
        return len(columns) >= 2 and sum(len(g) >= 2 for g in groups) >= 2

    def choose(self, now, pose, plan):
        if self.origin is None or now-self.last_observation > self.p.max_pose_age:
            self.selected = None
            self.support_count = 0
            self.counts = [0]*len(self.counts)
            self.states = ['unknown']*len(self.states)
            self.reason = 'no recent consistent visual registration'
            return None
        if self.last_choice_scan == self.last_scan:
            bays = self.bays()
            return None if self.selected is None else bays[self.selected]
        self.last_choice_scan = self.last_scan
        self.support_count = self.support_count+1 if self.supported(now) else 0
        if self.support_count < self.p.confirm_scans:
            self.selected = None
            self.counts = [0]*len(self.counts)
            self.reason = 'visual candidate not corroborated by lidar layout'
            return None
        bays = self.bays()
        candidates = []
        for i, bay in enumerate(bays):
            space = self.coverage(self.footprint(bay.x, bay.y, bay.entry_yaw), now)
            if self.p.control_enabled and i != self.p.target_bay-1:
                self.states[i] = space
                self.counts[i] = 0
                continue
            # Expensive swept area only for an observed empty target.
            path = self.path(bay, pose, plan) if space == 'free' else None
            route = 'unreachable' if path is None else self.coverage(path, now)
            self.states[i] = space if space != 'free' else ('free' if route == 'free' else 'path_'+route)
            # Unknown distant approach cells are not declared free. Observe them
            # as we approach, checking the next swept envelope before every move.
            # A known obstacle anywhere on the nominal route still vetoes it.
            usable = space == 'free' and (route == 'free' or
                                         (self.p.control_enabled and route == 'unknown'))
            self.counts[i] = self.counts[i]+1 if usable else 0
            if self.counts[i] >= self.p.confirm_scans:
                if not self.p.target_bay or i == self.p.target_bay-1:
                    candidates.append(i)
        if self.selected not in candidates:
            self.selected = candidates[0] if candidates else None
        if self.selected is None:
            self.reason = 'no clear designated bay and collision-free nominal route'
        else:
            self.reason = f'bay {self.selected+1}: target observed free; '+(
                'approach checked incrementally' if self.p.control_enabled else 'nominal approach observed free')
        return None if self.selected is None else bays[self.selected]

    def motion_clear(self, now, pose, speed, steer, wheelbase, distance=.20):
        self.motion_reason = ''
        if not speed:
            return True
        direction = 1 if speed > 0 else -1
        curvature = math.tan(steer)/wheelbase
        envelope = []
        # Includes a braking/lookahead margin in both forward and reverse motion.
        for d in np.linspace(0, direction*distance, 8):
            if abs(curvature) < 1e-6:
                dx, dy, yaw = d, 0., pose.yaw
            else:
                dx, dy = math.sin(curvature*d)/curvature, (1-math.cos(curvature*d))/curvature
                yaw = pose.yaw+curvature*d
            c, s = math.cos(pose.yaw), math.sin(pose.yaw)
            envelope.append(self.footprint(pose.x+c*dx-s*dy, pose.y+s*dx+c*dy, yaw))
        points = np.concatenate(envelope)
        keys = unique_cells(points, self.p.grid)
        current = set(map(tuple, unique_cells(self.footprint(pose.x, pose.y, pose.yaw), self.p.grid)))
        for key in map(tuple, keys):
            value = self.cells.get(key)
            recent = value is not None and now-value[0] <= self.p.evidence_age
            if recent and value[1]:
                self.motion_reason = 'occupied swept footprint'
                return False
            # Existing safety-envelope cells are not newly entered space. Never
            # exempt a measured obstacle there, or certify a new unknown cell.
            if key not in current and not ((recent and not value[1]) or key in self.retained_free):
                self.motion_reason = 'unobserved swept footprint'
                return False
        return True
