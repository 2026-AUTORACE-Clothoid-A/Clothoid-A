"""Centre line and pure pursuit for the rubber cone course (2025 `rubber_cone_node.cpp`).

ROS-free. Pipeline unchanged from 2025: Delaunay triangulation of both boundaries -> midpoints of
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

try:
    from scipy.spatial import QhullError
except ImportError:  # scipy < 1.8 (older JetPack / apt images)
    from scipy.spatial.qhull import QhullError


@dataclass
class FollowerParams:
    speed: float = 0.3            # m/s (2025 launch value, node default was 0.4)
    reverse_speed: float = -0.2   # m/s, used when only one boundary is visible
    max_steer_deg: float = 21.0   # 2025 value (LIMO hardware limit is 28 deg inner wheel)
    vehicle_length: float = 0.25  # m, pure pursuit rear axle offset (2025 value)
    lfd: float = 0.3              # m, look-ahead distance
    centerline_ds: float = 0.05   # m, resampling step
    lsq_window_size: int = 9      # odd, >= 5
    # s, boundaries older than this are ignored. 2025 used 0.1, but the Tmini Plus scans at 10 Hz
    # and left/right arrive one after the other, so 0.1 would call the other side stale.
    boundary_timeout: float = 0.25

    def __post_init__(self):
        self.lsq_window_size = max(5, self.lsq_window_size)
        if self.lsq_window_size % 2 == 0:
            self.lsq_window_size += 1


@dataclass
class FollowResult:
    speed: float
    steering: float                # rad, + left
    reverse: bool = False
    center: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)), repr=False)


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


def compute_steering(center: np.ndarray, params: FollowerParams) -> float:
    """Pure pursuit steering (rad, + left) towards the look-ahead point of the centre line."""
    n = len(center)
    if n == 0:
        return 0.0
    closest = int(np.argmin(np.sum(center ** 2, axis=1)))

    target = None
    acc = 0.0
    for i in range(closest + 1, n):
        acc += float(np.linalg.norm(center[i] - center[i - 1]))
        if acc >= params.lfd:
            target = center[i]
            break
    if target is None:
        if n > closest + 1:
            target = center[-1]
        else:
            return 0.0

    theta = np.arctan2(target[1], target[0] + params.vehicle_length)
    return float(np.arctan2(2.0 * params.vehicle_length * np.sin(theta), params.lfd))


class ConeFollower:
    def __init__(self, params: FollowerParams):
        self.p = params

    def decide(self, left: np.ndarray, right: np.ndarray,
               left_fresh: bool, right_fresh: bool) -> Optional[FollowResult]:
        """None means "publish nothing" (no fresh boundary, or no centre line)."""
        p = self.p
        max_steer = np.deg2rad(p.max_steer_deg)
        if not left_fresh and not right_fresh:
            return None
        # one boundary only: back off with full lock away from the visible side (2025)
        if not left_fresh:
            return FollowResult(p.reverse_speed, -max_steer, reverse=True)
        if not right_fresh:
            return FollowResult(p.reverse_speed, +max_steer, reverse=True)

        center = build_center_line(left, right, p)
        if len(center) == 0:
            return None
        return FollowResult(p.speed, compute_steering(center, p), center=center)
