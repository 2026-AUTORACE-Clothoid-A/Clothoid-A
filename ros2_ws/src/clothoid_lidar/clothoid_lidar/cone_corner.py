"""Ordered midpoint path around an observed straight boundary endpoint.

Only uses the two current, sorted boundaries. A truncated or missing outer wall
does not invent a corner. Returning None keeps the existing centre-line method.
"""
import numpy as np


def _curvature(points):
    a, b = np.diff(points, axis=0)[:-1], np.diff(points, axis=0)[1:]
    denominator = np.linalg.norm(a, axis=1)*np.linalg.norm(b, axis=1)*np.linalg.norm(points[2:]-points[:-2], axis=1)
    return 2*np.abs(a[:, 0]*b[:, 1]-a[:, 1]*b[:, 0])/np.maximum(denominator, 1e-12)


def _distance(points, boundary):
    if len(boundary) == 1:
        return np.linalg.norm(points-boundary[0], axis=1)
    start = boundary[:-1]
    step = np.diff(boundary, axis=0)
    lengths = np.sum(step*step, axis=1)
    delta = points[:, None, :]-start
    along = np.clip(np.sum(delta*step, axis=2)/np.maximum(lengths, 1e-12), 0., 1.)
    return np.sqrt(np.min(np.sum((delta-along[..., None]*step)**2, axis=2), axis=1))

def _sample_clear_path(raw, inner, outer, spacing):
    s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(raw, axis=0), axis=1))]
    if np.any(np.diff(s) < 1e-6):
        return None
    sample = np.linspace(0., s[-1], max(2, int(np.ceil(s[-1]/max(.01, spacing)))+1))
    path = np.column_stack([np.interp(sample, s, raw[:, k]) for k in (0, 1)])
    # Circular body envelope (322 x 220 mm), cone radius 40 mm, margin 20 mm.
    clearance = np.hypot(.161, .110)+.04+.02
    if min(_distance(path, inner).min(), _distance(path, outer).min()) < clearance:
        return None
    return path


def _single_anchor_path(inner, outer, turn_sign, spacing, min_radius):
    """A measured inner cone and an ordered outer arc, without an invented wall."""
    offsets = outer-inner[0]
    widths = np.linalg.norm(offsets, axis=1)
    width = float(np.median(widths))
    if width < 1e-6 or np.any(widths < .65*width) or np.any(widths > 1.4*width):
        return None
    angles = np.unwrap(np.arctan2(offsets[:, 1], offsets[:, 0]))
    turns = np.diff(angles)*turn_sign
    # Ordered, connected outer wall turning around this particular anchor.
    if np.any(turns <= 0.) or np.any(turns > np.pi/2):
        return None
    if not np.deg2rad(25.) <= sum(turns) <= np.pi:
        return None
    raw = (outer+inner[0])/2
    if np.max(_curvature(raw)) > 1/min_radius:
        radius = max(min_radius, width/2)
        fitted = inner[0]+offsets/widths[:, None]*radius
        if np.max(np.linalg.norm(fitted-raw, axis=1)) > .08*width:
            return None
        if np.max(_curvature(fitted)) > 1/min_radius+1e-6:
            return None
        raw = fitted
    return _sample_clear_path(raw, inner, outer, spacing)


def endpoint_centerline(left, right, spacing, min_radius=.2/np.tan(np.deg2rad(28.))+.175/2):
    """Project the outer wall onto the inner straight, clamping at its end.

    Beyond the end, each midpoint is exactly between the endpoint and an outer
    wall point. Noisy midpoint radii may be fitted to a common turning radius,
    within the measured corridor tolerance. Linear resampling cannot overshoot.
    Width/straightness gates are relative to the observed opening, not bag poses.
    """
    for inner, outer, turn_sign in ((left, right, 1.), (right, left, -1.)):
        if len(inner) < 1 or len(outer) < 4:
            continue
        if not np.isfinite(inner).all() or not np.isfinite(outer).all():
            continue
        if len(inner) == 1:
            path = _single_anchor_path(inner, outer, turn_sign, spacing, min_radius)
            if path is not None:
                return path
            continue
        vector = inner[-1]-inner[0]
        length = np.linalg.norm(vector)
        if length < 1e-6:
            continue
        centre = np.mean(inner, axis=0)
        _, _, directions = np.linalg.svd(inner-centre, full_matrices=False)
        axis = directions[0]
        if axis@vector < 0:
            axis = -axis
        length = vector@axis
        origin = inner[-1]-length*axis
        along = (inner-origin)@axis
        if np.any(np.diff(along) <= 1e-6):
            continue
        outer_along = (outer-origin)@axis
        projected = origin+np.clip(outer_along, 0., length)[:, None]*axis
        offsets = outer-projected
        widths = np.linalg.norm(offsets, axis=1)
        overlap = (outer_along >= 0.) & (outer_along <= length)
        if np.count_nonzero(overlap) < 2:
            continue
        width = float(np.median(widths[overlap]))
        if width < 1e-6:
            continue
        residual = inner-(centre+((inner-centre)@axis)[:, None]*axis)
        if np.max(np.linalg.norm(residual, axis=1)) > .08*width:
            continue
        # Require a visible outer turn and at least two points beyond the inner end.
        past = outer_along > length+.05*width
        if np.count_nonzero(past) < 2:
            continue
        steps = np.diff(outer, axis=0)
        norms = np.linalg.norm(steps, axis=1)
        if np.any(norms < 1e-6):
            continue
        headings = np.unwrap(np.arctan2(steps[:, 1], steps[:, 0])-np.arctan2(axis[1], axis[0]))
        turns = headings*turn_sign
        if abs(turns[0]) > np.deg2rad(30.) or not np.deg2rad(25.) <= turns[-1] <= np.deg2rad(110.):
            continue
        if np.any(np.diff(turns) < -np.deg2rad(15.)):
            continue
        # Reject divergent walls and points crossing to the inner side.
        side = axis[0]*offsets[:, 1]-axis[1]*offsets[:, 0]
        if np.any(side[overlap]*turn_sign > .08*width):
            continue
        if np.any(widths < .65*width) or np.any(widths > 1.4*width):
            continue
        raw = (outer+projected)/2
        if np.max(_curvature(raw)) > 1/min_radius:
            # Fit midpoint distances around the SAME observed endpoint. Normalise
            # scan jitter instead of interpreting it as a vehicle steering turn.
            radius = max(min_radius, float(np.median(widths))/2)
            fitted = projected+offsets/widths[:, None]*radius
            if np.max(np.linalg.norm(fitted-raw, axis=1)) > .08*width:
                continue
            if np.max(_curvature(fitted)) > 1/min_radius+1e-6:
                continue
            raw = fitted
        path = _sample_clear_path(raw, inner, outer, spacing)
        if path is not None:
            return path
    return None
