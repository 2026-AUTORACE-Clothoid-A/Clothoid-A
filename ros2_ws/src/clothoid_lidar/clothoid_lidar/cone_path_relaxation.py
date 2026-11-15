"""Bounded local bending reduction inside the currently observed corridor.

No corner classification or map: move a short reference polyline along its
normals, with fixed work and conservative observed-boundary clearance bounds.
"""
import numpy as np
from clothoid_lidar.cone_corner import _distance, _curvature
from clothoid_lidar.cone_exit import clearance, transform


class PathRelaxation:
    def __init__(self):
        self.clear()

    def clear(self):
        self.world = None
        self.pose = None

    def refine(self, center, left, right, obstacles, pose, spacing):
        original = np.asarray(center, float)
        if (len(original) < 5 or len(left) < 1 or len(right) < 1
                or len(left) > 64 or len(right) > 64
                or obstacles is None or pose is None
                or not np.isfinite(original).all() or not np.isfinite(pose).all()
                or not np.isfinite(left).all() or not np.isfinite(right).all()
                or len(obstacles) > 256 or not np.isfinite(obstacles).all()):
            self.clear()
            return original
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(original, axis=0), axis=1))]
        if arc[-1] < .4 or np.any(np.diff(arc) < 1e-6):
            self.clear()
            return original
        # Bound work without resampling artifacts. Long paths keep their reference.
        if len(original) > 64:
            self.clear()
            return original
        n = len(original)
        ref = original.copy()
        s = arc
        tangent = np.gradient(ref, axis=0)
        norm = np.linalg.norm(tangent, axis=1)
        if np.any(norm < 1e-6):
            self.clear()
            return original
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))/norm[:, None]
        # Same circular body envelope as the observed-corner builder, plus cone
        # radius and 20 mm margin. Boundary distances include connected segments.
        envelope = np.hypot(.161, .110)+.04+.02
        dl, dr = _distance(ref, left), _distance(ref, right)
        allowance = np.maximum(0., np.minimum(np.minimum(dl, dr)-envelope, .12*(dl+dr)))
        # Preserve both endpoint positions and tangents; never guess unseen exit.
        allowance[:2] = allowance[-2:] = 0.
        prior = np.zeros(n)
        if self.world is not None and np.linalg.norm(np.array(pose[:2])-self.pose[:2]) < .25:
            old = transform(self.world, pose, True)
            old_steps = np.diff(old, axis=0)
            denom = np.maximum(np.sum(old_steps**2, axis=1), 1e-12)
            fractions = np.clip(np.sum((ref[:, None]-old[:-1])*old_steps, axis=2)/denom, 0., 1.)
            feet = old[:-1]+fractions[:, :, None]*old_steps
            distance = np.linalg.norm(ref[:, None]-feet, axis=2)
            indices = distance.argmin(axis=1)
            alignment = np.sum(old_steps[indices]*tangent, axis=1)
            good = (distance[np.arange(n), indices] < .15) & (alignment > 0.)
            prior[good] = np.sum((feet[np.arange(n), indices][good]-ref[good])*normal[good], axis=1)
        prior = np.clip(prior, -allowance, allowance)
        offset = prior.copy()
        # Convex quadratic bending + centre attraction + temporal attraction.
        # D2^T D2 has norm <=16; this fixed step is below the gradient bound.
        for _ in range(32):
            path = ref+offset[:, None]*normal
            second = path[:-2]-2*path[1:-1]+path[2:]
            grad = np.zeros_like(path)
            grad[:-2] += second
            grad[1:-1] -= 2*second
            grad[2:] += second
            gradient = .04*offset+.10*(offset-prior)+np.sum(grad*normal, axis=1)
            offset = np.clip(offset-.06*gradient, -allowance, allowance)
        candidate = ref+offset[:, None]*normal
        # Interpolate displacement onto the ORIGINAL sampling so a zero update
        # preserves the existing centreline, rather than changing its geometry.
        delta = np.column_stack([np.interp(arc, s, (candidate-ref)[:, k]) for k in (0, 1)])
        candidate = original+delta
        # Validate current evidence after temporal smoothing. An old path never
        # overrides a newly observed obstacle. Reject the entire update on failure.
        checked = np.vstack((candidate, (candidate[:-1]+candidate[1:])/2))
        valid = (min(_distance(checked, left).min(), _distance(checked, right).min()) >= envelope
                 and clearance(candidate, obstacles) >= envelope
                 and np.all(np.sum(np.diff(candidate, axis=0)*np.diff(original, axis=0), axis=1) > 0.)
                 and np.max(_curvature(candidate)) <= max(np.max(_curvature(original)), 1/(.2/np.tan(np.deg2rad(28.))+.175/2))+1e-6)
        result = candidate if valid else original
        self.world = transform(result, pose)
        self.pose = np.asarray(pose)
        return result
