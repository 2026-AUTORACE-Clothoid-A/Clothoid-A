"""Short odometry-referenced continuation of the last observed cone path.

Never saves a path already reduced to a sideways/backward target. Continuation
is limited to one lookahead beyond the observed endpoint, along its observed
tangent. No map, waypoint table, reverse command or unbounded extrapolation.
"""
import math
import numpy as np


def transform(points, pose, inverse=False):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    return (points - (x, y)) @ rotation if inverse else points @ rotation.T + (x, y)


class ExitPathMemory:
    def __init__(self):
        self.clear()

    def clear(self):
        self.world = None
        self.observed_end = None
        self.end_direction = None
        self.origin = None
        self.travel_limit = 0.
        self.last_pose = None
        self.travelled = 0.
        self.saved_travel = 0.

    def update_pose(self, pose):
        if self.last_pose is not None:
            self.travelled += float(np.linalg.norm(np.array(pose[:2]) - self.last_pose))
        self.last_pose = np.array(pose[:2])

    def remember(self, path, pose, lookahead, spacing):
        self.update_pose(pose)
        if not np.isfinite(pose).all() or lookahead <= 0 or spacing <= 0:
            return False
        path = np.asarray(path, float).reshape(-1, 2)
        if len(path) < 3 or not np.isfinite(path).all():
            return False
        nearest = int(np.argmin(np.linalg.norm(path, axis=1)))
        path = path[nearest:]
        if len(path) < 3:
            return False
        lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
        length = float(lengths.sum())
        if length < lookahead or path[-1, 0] <= 0:
            return False
        # Use a finite tail, rather than a noisy last spline segment.
        arc = np.r_[0., np.cumsum(lengths)]
        tail = int(np.searchsorted(arc, max(0., length - lookahead / 2)))
        tail = min(tail, len(path) - 2)
        tangent = path[-1] - path[tail]
        norm = float(np.linalg.norm(tangent))
        if norm < 1e-6:
            return False
        tangent /= norm
        # Reject a terminal path that has already folded back toward the car.
        if np.dot(tangent, path[-1]) <= 0:
            return False
        extension = np.arange(spacing, lookahead + spacing / 2, spacing)
        continued = np.vstack((path, path[-1] + extension[:, None] * tangent))
        self.world = transform(continued, pose)
        self.observed_end = transform(path[-1:], pose)[0]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        self.end_direction = np.array([[c, -s], [s, c]]) @ tangent
        self.origin = np.array(pose[:2])
        self.travel_limit = length + lookahead
        self.saved_travel = self.travelled
        return True

    def front_axle_remaining(self, pose, rear_axle_x, wheelbase):
        """Signed progress to the observed endpoint's transverse plane, metres.

        Positive means the front axle has not reached it. This deliberately
        excludes the extrapolated tail and does not require hitting one exact
        point. Pose freshness is checked by the caller, as for path following.
        """
        if (self.observed_end is None or pose is None or not np.isfinite(pose).all()
                or not np.isfinite(rear_axle_x) or not np.isfinite(wheelbase) or wheelbase <= 0.):
            return None
        front = transform(np.array([[wheelbase-rear_axle_x, 0.]]), pose)[0]
        return float((self.observed_end-front) @ self.end_direction)

    def handover_ready(self, pose, rear_axle_x, wheelbase):
        """Gate lane readiness on progress when an observed endpoint is known.

        With no accepted path, retain the existing lane-only fallback rather
        than trapping the mission on a point that was never observed.
        """
        if self.observed_end is None:
            return True
        remaining = self.front_axle_remaining(pose, rear_axle_x, wheelbase)
        return remaining is not None and remaining <= 1e-9

    def remaining(self, pose, rear_axle_x):
        self.update_pose(pose)
        if self.world is None or self.travelled - self.saved_travel > self.travel_limit:
            return np.zeros((0, 2))
        path = transform(self.world, pose, inverse=True)
        nearest = int(np.argmin(np.linalg.norm(path - (-rear_axle_x, 0.), axis=1)))
        path = path[nearest:]
        # Reaching the end does not authorize another extension.
        if len(path) < 2 or path[-1, 0] <= 0:
            return np.zeros((0, 2))
        return path


def clearance(path, obstacles):
    """Minimum distance from observed obstacle centres to the remaining polyline."""
    path = np.asarray(path, float).reshape(-1, 2)
    obstacles = np.asarray(obstacles, float).reshape(-1, 2)
    if len(path) < 2:
        return 0.
    if not len(obstacles):
        return float('inf')
    if not np.isfinite(obstacles).all():
        return 0.
    a, delta = path[:-1], np.diff(path, axis=0)
    denom = np.maximum(np.sum(delta * delta, axis=1), 1e-12)
    diff = obstacles[:, None, :] - a[None, :, :]
    ratio = np.clip(np.sum(diff * delta[None, :, :], axis=2) / denom, 0., 1.)
    return float(np.linalg.norm(diff - ratio[:, :, None] * delta[None, :, :], axis=2).min())
