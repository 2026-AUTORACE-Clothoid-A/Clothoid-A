"""Measured corridor width and finite odometry-referenced cone path continuity.

Only paired observations train the width and refresh memory. Inferred paths
never become new observations or extend the lifetime of the saved geometry.
"""
from collections import deque
import numpy as np

from clothoid_lidar.cone_exit import transform, clearance


def width_samples(left, right):
    """Perpendicular distances on overlapping, locally parallel segments.

    Do not measure diagonals to clamped wall endpoints at a corner. A side's
    most local segment gets at most one vote in either direction.
    """
    samples = []
    for wall, other in ((left, right), (right, left)):
        for a, b in zip(wall[:3], wall[1:4]):
            step = b-a
            length = np.linalg.norm(step)
            if length < 1e-6:
                continue
            choices = []
            for c, d in zip(other, other[1:]):
                v = d-c
                norm = np.linalg.norm(v)
                if norm < 1e-6 or step@v < np.cos(np.deg2rad(20.))*length*norm:
                    continue
                u = (a-c)@v/(norm*norm)
                if 0. <= u <= 1.:
                    choices.append(float(np.linalg.norm(a-c-u*v)))
            if choices:
                samples.append(min(choices))
    return samples


def offset_center(wall, side, width, spacing):
    """Offset the observed polyline inward, stopping before an offset folds.

    The miter is the intersection of adjacent parallel-offset segments. Linear
    resampling cannot overshoot into the visible wall as a cubic fit can.
    """
    if len(wall) < 2:
        return np.zeros((0, 2))
    step = np.diff(wall, axis=0)
    lengths = np.linalg.norm(step, axis=1)
    if np.any(lengths < 1e-6):
        return np.zeros((0, 2))
    normal = np.column_stack((-step[:, 1], step[:, 0]))/lengths[:, None]
    normals = np.vstack((normal[:1], (normal[:-1]+normal[1:])/2, normal[-1:]))
    denom = np.einsum('ij,ij->i', normals, np.vstack((normal, normal[-1:])))
    if np.any(denom <= .5):
        return np.zeros((0, 2))
    center = wall-side*(width/2)*normals/denom[:, None]
    forward = np.einsum('ij,ij->i', np.diff(center, axis=0), step) > 0.
    if not np.all(forward):
        center = center[:int(np.flatnonzero(~forward)[0])+1]
    if len(center) < 2:
        return np.zeros((0, 2))
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(center, axis=0), axis=1))]
    if arc[-1] < spacing:
        return np.zeros((0, 2))
    sample = np.linspace(0., arc[-1], max(2, int(np.ceil(arc[-1]/spacing))+1))
    return np.column_stack([np.interp(sample, arc, center[:, k]) for k in (0, 1)])


class CorridorMemory:
    def __init__(self):
        self.clear()

    def clear(self):
        self.widths = deque(maxlen=20)
        self.left = self.right = self.path = None
        self.last_pose = None
        self.travelled = self.saved_travel = self.travel_limit = 0.

    @property
    def width(self):
        if not self.widths:
            return None
        values = np.asarray(self.widths)
        median = np.median(values)
        # Outliers from a bad pairing do not dominate the running mean.
        tolerance = max(.1*median, 3*np.median(np.abs(values-median)))
        return float(np.mean(values[np.abs(values-median) <= tolerance]))

    def update_pose(self, pose):
        if pose is None or not np.isfinite(pose).all():
            return False
        if self.last_pose is not None:
            distance = np.linalg.norm(np.asarray(pose[:2])-self.last_pose[:2])
            if distance > 1.:
                self.clear()
            else:
                self.travelled += float(distance)
        self.last_pose = np.asarray(pose)
        return True

    def observe(self, left, right, path, pose, lookahead):
        if not self.update_pose(pose) or len(path) < 2:
            return
        samples = width_samples(left, right)
        if samples:
            value = float(np.median(samples))
            if value <= 0. or (self.width is not None and not .7*self.width <= value <= 1.3*self.width):
                # A suspect pairing must not replace the retained walls/path.
                return
            self.widths.append(value)
        self.left, self.right = transform(left, pose), transform(right, pose)
        self.path = transform(path, pose)
        self.saved_travel = self.travelled
        self.travel_limit = (np.linalg.norm(np.diff(path, axis=0), axis=1).sum()
                             + (self.width or lookahead))

    def candidates(self, left, right, pose, spacing, build_center):
        """Yield finite alternatives, ordered from retained geometry to offsets."""
        if not self.update_pose(pose) or self.path is None:
            return
        if self.travelled-self.saved_travel > self.travel_limit:
            return
        for wall, saved, side in ((left, self.right, 1), (right, self.left, -1)):
            if not len(wall):
                continue
            if saved is not None and len(saved):
                retained = transform(saved, pose, inverse=True)
                # A remembered wall is useful only in the local opening still
                # supported by the current wall. No distant old course is joined.
                width = self.width
                if width is not None:
                    distance = np.linalg.norm(retained[:, None]-wall[None, :], axis=2).min(axis=1)
                    retained = retained[distance <= 1.4*width]
                if len(retained):
                    a, b = (wall, retained) if side == 1 else (retained, wall)
                    samples = width_samples(a, b)
                    if not samples or width is None or .7*width <= np.median(samples) <= 1.3*width:
                        yield build_center(a, b), 'retained_boundary'
        yield transform(self.path, pose, inverse=True), 'retained_path'
        width = self.width
        if width is not None:
            for wall, side in ((left, 1), (right, -1)):
                if len(wall) >= 2:
                    yield offset_center(wall, side, width, spacing), 'width_offset'


def usable_path(path, obstacles, min_clearance, target):
    return (len(path) >= 2 and np.isfinite(path).all() and target is not None
            and np.isfinite(target).all() and target[0] > 0.
            and obstacles is not None and clearance(path, obstacles) >= min_clearance)
