"""Centre line and pure pursuit for the rubber cone course (2025 `rubber_cone_node.cpp`).

ROS-free. An optional observed-endpoint midpoint path handles asymmetric corners.
Otherwise the 2025 pipeline uses Delaunay triangulation of both boundaries -> midpoints of
edges that join a left and a right cone -> MST longest path to order them -> local cubic least
squares smoothing -> cubic spline resampling -> pure pursuit on the look-ahead point.
Steering is returned in rad, positive = left (the node converts it to a Twist).
"""
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra, minimum_spanning_tree
from scipy.spatial import Delaunay
from clothoid_lidar.cone_corner import endpoint_centerline
from clothoid_lidar.cone_corridor import CorridorMemory, usable_path, width_samples

try:
    from scipy.spatial import QhullError
except ImportError:  # scipy < 1.8 (older JetPack / apt images)
    from scipy.spatial.qhull import QhullError


@dataclass
class FollowerParams:
    speed: float = 0.3            # m/s (2025 launch value, node default was 0.4)
    reverse_speed: float = -0.2   # m/s, boundary recovery only; 0 disables recovery
    # deg, steering while backing off when only one boundary is visible (2025 `max_steer_deg`, 21).
    # Forward pure pursuit is not cut by this; the LIMO hardware limit is 28 deg inner wheel.
    reverse_steer_deg: float = 21.0
    # The 2025 car had one `vehicle_length` (0.25) for both of these. On the LIMO they differ:
    # base_link sits midway between the axles (URDF wheels at x = +-wheelbase/2).
    wheelbase: float = 0.2        # m, front to rear axle (steering gain of pure pursuit)
    rear_axle_x: float = 0.1      # m, rear axle behind base_link (origin of the look-ahead angle)
    lfd: float = 0.3              # m, look-ahead circle radius from the rear axle
    steering_gain: float = 1.0    # forward curvature multiplier; target/lfd stay unchanged
    centerline_ds: float = 0.05   # m, resampling step
    lsq_window_size: int = 9      # odd, >= 5
    # s, boundaries older than this are ignored. 2025 used 0.1, but the Tmini Plus scans at 10 Hz
    # and left/right arrive one after the other, so 0.1 would call the other side stale.
    boundary_timeout: float = 0.25
    corner_endpoint_enabled: bool = True

    def __post_init__(self):
        if not np.isfinite(self.lfd) or self.lfd <= 0.0:
            raise ValueError('lfd must be finite and positive')
        if not np.isfinite(self.steering_gain) or self.steering_gain <= 0.0:
            raise ValueError('steering_gain must be finite and positive')
        self.lsq_window_size = max(5, self.lsq_window_size)
        if self.lsq_window_size % 2 == 0:
            self.lsq_window_size += 1


@dataclass
class FollowResult:
    speed: float
    steering: float                # rad, + left
    reverse: bool = False
    center: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)), repr=False)
    target: Optional[np.ndarray] = field(default=None, repr=False)   # look-ahead point, base_link
    source: str = 'observed'


def _arc_length(points: np.ndarray) -> np.ndarray:
    s = np.zeros(len(points))
    s[1:] = np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))
    return s


def order_by_mst(points: np.ndarray) -> np.ndarray:
    """Order midpoints along the longest path of their minimum spanning tree.

    The path starts at the end nearer to the vehicle (origin).
    """
    n = len(points)
    if n < 2:
        return points
    dist = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2) + 1e-9
    np.fill_diagonal(dist, 0.0)
    tree = minimum_spanning_tree(csr_matrix(dist))
    tree = tree + tree.T

    far1 = int(np.argmax(dijkstra(tree, indices=0)))
    d, pred = dijkstra(tree, indices=far1, return_predecessors=True)
    far2 = int(np.argmax(np.where(np.isfinite(d), d, -1.0)))

    path = []
    node = far2
    while node >= 0:
        path.append(node)
        node = pred[node]
    ordered = points[path[::-1]]
    if np.sum(ordered[0] ** 2) > np.sum(ordered[-1] ** 2):
        ordered = ordered[::-1]
    return ordered


def lsq_cubic_smooth(points: np.ndarray, window: int) -> np.ndarray:
    """Local cubic least squares (Savitzky-Golay style) over an arc-length window."""
    n = len(points)
    if n <= 3:
        return points
    s = _arc_length(points)
    half = window // 2
    out = points.copy()
    for i in range(n):
        j0, j1 = max(0, i - half), min(n - 1, i + half)
        if j1 - j0 + 1 < 4:
            continue
        t = s[j0:j1 + 1] - s[i]
        a = np.column_stack((np.ones_like(t), t, t ** 2, t ** 3))
        coeff, *_ = np.linalg.lstsq(a, points[j0:j1 + 1], rcond=None)
        out[i] = coeff[0]  # value at t = 0
    return out


def spline_resample(points: np.ndarray, ds: float) -> np.ndarray:
    """Natural cubic spline over arc length, resampled at a constant step."""
    if len(points) < 3:
        return points
    keep = np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-9]
    points = points[keep]
    if len(points) < 3:
        return points
    s = _arc_length(points)
    total = s[-1]
    if total < 1e-6:
        return points
    if ds <= 1e-4:
        ds = total / (len(points) - 1)
    m = max(2, int(np.ceil(total / ds)) + 1)
    si = np.minimum(ds * np.arange(m), total)
    return np.column_stack((CubicSpline(s, points[:, 0], bc_type='natural')(si),
                            CubicSpline(s, points[:, 1], bc_type='natural')(si)))


def _smooth(raw: np.ndarray, params: FollowerParams) -> np.ndarray:
    return spline_resample(lsq_cubic_smooth(raw, params.lsq_window_size), params.centerline_ds)


def build_center_line(left: np.ndarray, right: np.ndarray, params: FollowerParams) -> np.ndarray:
    """Centre line from the left and right boundary cones (base_link, x forward)."""
    left, right = np.asarray(left, float).reshape(-1, 2), np.asarray(right, float).reshape(-1, 2)
    if len(left) == 0 and len(right) == 0:
        return np.zeros((0, 2))

    if params.corner_endpoint_enabled:
        # LIMO driver limits the INNER wheel to 28 degrees, with 175 mm track.
        min_radius = params.wheelbase / np.tan(np.deg2rad(28.)) + .175/2
        corner = endpoint_centerline(left, right, params.centerline_ds, min_radius)
        if corner is not None:
            return corner

    points = np.vstack((left, right))
    label = np.r_[np.zeros(len(left), int), np.ones(len(right), int)]

    if len(points) < 3:
        raw = _pairwise_center(left, right)
        return _smooth(raw, params) if len(raw) else np.zeros((0, 2))

    try:
        triangles = Delaunay(points).simplices
    except QhullError:  # collinear cones
        raw = _pairwise_center(left, right)
        return _smooth(raw, params) if len(raw) else np.zeros((0, 2))

    edges = set()
    for a, b, c in triangles:
        for u, v in ((a, b), (b, c), (c, a)):
            if label[u] != label[v]:
                edges.add((min(u, v), max(u, v)))
    if not edges:
        return np.zeros((0, 2))

    raw = np.array([(points[u] + points[v]) / 2 for u, v in sorted(edges)])
    return _smooth(order_by_mst(raw), params)


def _pairwise_center(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    if len(left) and len(right):
        m = min(len(left), len(right))
        return (left[:m] + right[:m]) / 2
    return left if len(left) else right


def retained_center_line(left, right, params):
    # A stored wall behind the car must not be joined diagonally to a newly
    # observed straight wall ahead. Require overlap or an observed corner arc.
    if width_samples(left, right):
        return build_center_line(left, right, params)
    radius = params.wheelbase/np.tan(np.deg2rad(28.))+.175/2
    corner = endpoint_centerline(left, right, params.centerline_ds, radius)
    return corner if corner is not None else np.zeros((0, 2))


def look_ahead_point(center: np.ndarray, params: FollowerParams) -> Optional[np.ndarray]:
    """First forward outward crossing of the rear-axle lookahead circle.

    Start at the closest projection onto the ordered path and interpolate the
    crossing inside a segment. A short remaining path uses its endpoint. If the
    path lies outside the circle, retain its nearest forward point instead of
    treating the missing intersection as missing boundary geometry.
    """
    points = np.asarray(center, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 2 or len(points) < 2
            or not np.isfinite(points).all() or not np.isfinite(params.rear_axle_x)):
        return None
    # Duplicate vertices are not a path segment (two isolated cone seeds can
    # produce a single midpoint, which must still use the existing recovery).
    points = points[np.r_[True, np.sum(np.diff(points, axis=0)**2, axis=1) > 1e-12]]
    if len(points) < 2:
        return None
    rear = np.array([-params.rear_axle_x, 0.])
    steps = np.diff(points, axis=0)
    lengths_squared = np.sum(steps*steps, axis=1)
    fractions = np.clip(np.sum((rear-points[:-1])*steps, axis=1)/lengths_squared, 0., 1.)
    projections = points[:-1]+fractions[:, None]*steps
    closest = int(np.argmin(np.sum((projections-rear)**2, axis=1)))
    remaining = np.vstack((projections[closest], points[closest+1:]))
    if np.sum(np.diff(remaining, axis=0)**2) <= 1e-12:
        return None  # the ordered path has already ended

    radius_squared = params.lfd**2
    for start, end in zip(remaining, remaining[1:]):
        step, local = end-start, start-rear
        length_squared = float(step@step)
        if length_squared <= 1e-12:
            continue
        projection = -float(local@step)/length_squared
        perpendicular_squared = max(0., float(local@local)-projection**2*length_squared)
        gap = radius_squared-perpendicular_squared
        if gap < -1e-12:
            continue
        # The positive root is the outward crossing, following path order.
        fraction = projection+np.sqrt(max(0., gap)/length_squared)
        if -1e-9 <= fraction <= 1.+1e-9:
            target = start+np.clip(fraction, 0., 1.)*step
            if target[0] > 0.:
                return target

    distances_squared = np.sum((remaining-rear)**2, axis=1)
    if np.all(distances_squared <= radius_squared+1e-12):
        return remaining[-1].copy() if remaining[-1, 0] > 0. else None
    forward = np.flatnonzero(remaining[:, 0] > 0.)
    if not len(forward):
        return None
    return remaining[forward[np.argmin(distances_squared[forward])]].copy()


def compute_steering(center: np.ndarray, params: FollowerParams) -> float:
    """Pure pursuit using the target's actual distance from the rear axle.

    The circle normally sets the target distance to lfd. Short-path and
    outside-circle fallbacks still use their actual rear-axle distance.
    tan(delta) = wheelbase * gain * 2*y/(x*x+y*y), in rear-axle coordinates.
    """
    target = look_ahead_point(center, params)
    if target is None:
        return 0.0
    x, y = float(target[0]) + params.rear_axle_x, float(target[1])
    distance_squared = x*x + y*y
    if not np.isfinite(distance_squared) or distance_squared <= 1e-12:
        return 0.0
    return float(np.arctan2(2.0 * params.wheelbase * params.steering_gain * y, distance_squared))


class ConeFollower:
    def __init__(self, params: FollowerParams):
        self.p = params
        self.corridor = CorridorMemory()
        self.last_forward_steering = None
        self.reverse_start_travel = None

    def reset(self):
        self.corridor.clear()
        self.last_forward_steering = None
        self.reverse_start_travel = None

    def _forward(self, center, source='observed'):
        self.last_forward_steering = compute_steering(center, self.p)
        self.reverse_start_travel = None
        return FollowResult(self.p.speed, self.last_forward_steering, center=center,
                            target=look_ahead_point(center, self.p), source=source)

    def _recover(self, left, right, pose, obstacles, allow_reverse):
        p = self.p
        if obstacles is None:
            return FollowResult(0., 0., source='stale_obstacles')
        if not len(obstacles):
            return FollowResult(0., 0., source='no_cones')
        if pose is None:
            return FollowResult(0., 0., source='stale_pose')
        if not allow_reverse or p.reverse_speed >= 0.:
            return FollowResult(0., 0., source='no_boundary_path')
        self.corridor.update_pose(pose)
        if self.reverse_start_travel is None:
            self.reverse_start_travel = self.corridor.travelled
        if self.corridor.travelled-self.reverse_start_travel >= p.lfd:
            return FollowResult(0., 0., source='recovery_limit')
        lock = np.deg2rad(p.reverse_steer_deg)
        steering = self.last_forward_steering
        if steering is None:
            steering = lock if len(left) and not len(right) else -lock if len(right) and not len(left) else 0.
        # Same steering and negative velocity retrace the most recent turn.
        return FollowResult(p.reverse_speed, float(np.clip(steering, -lock, lock)),
                            reverse=True, source='boundary_recovery')

    def decide(self, left: np.ndarray, right: np.ndarray,
               left_fresh: bool, right_fresh: bool, pose=None, obstacles=None,
               min_clearance: float = .2, allow_reverse: bool = False) -> Optional[FollowResult]:
        """None means "publish nothing" (no fresh boundary, or no centre line)."""
        p = self.p
        visible_left = left if left_fresh else np.zeros((0, 2))
        visible_right = right if right_fresh else np.zeros((0, 2))
        center = np.zeros((0, 2))
        if left_fresh and right_fresh and len(left) and len(right):
            center = build_center_line(left, right, p)
            target = look_ahead_point(center, p)
            if target is not None and np.isfinite(target).all() and target[0] > 0.:
                if usable_path(center, obstacles, min_clearance, target):
                    self.corridor.observe(left, right, center, pose, p.lfd)
                return self._forward(center)
        # An empty/one-point centre is not a steerable path, even if both
        # boundary messages are fresh. Try retained/width paths before recovery.
        if pose is not None:
            for center, source in self.corridor.candidates(
                    visible_left, visible_right, pose, p.centerline_ds,
                    lambda a, b: retained_center_line(a, b, p)):
                if len(center) < 2:
                    continue
                closest = int(np.argmin(np.linalg.norm(center, axis=1)))
                center = center[closest:]
                target = look_ahead_point(center, p)
                if usable_path(center, obstacles, min_clearance, target):
                    return self._forward(center, source)
        return self._recover(visible_left, visible_right, pose, obstacles, allow_reverse)
