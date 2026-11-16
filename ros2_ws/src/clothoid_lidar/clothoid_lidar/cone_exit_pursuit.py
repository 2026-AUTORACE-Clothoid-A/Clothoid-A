"""Rear-axle pursuit used only after the cone course enters EXIT.

Normal cone driving remains the unmodified main controller in cone_follower.py.
Exit geometry/speed have their own parameters so main tuning cannot change handover.
"""
from dataclasses import dataclass
from typing import Optional
import math
import numpy as np


@dataclass
class ExitPursuitParams:
    speed: float = .25
    lfd: float = .35
    wheelbase: float = .2
    rear_axle_x: float = .1
    centerline_ds: float = .05
    boundary_timeout: float = .25
    steering_gain: float = 1.

    def __post_init__(self):
        for name in ('speed', 'lfd', 'wheelbase', 'centerline_ds', 'boundary_timeout', 'steering_gain'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0.:
                raise ValueError(f'exit {name} must be finite and positive')
        if not math.isfinite(self.rear_axle_x) or self.rear_axle_x < 0.:
            raise ValueError('rear_axle_x must be finite and nonnegative')


def look_ahead_point(center: np.ndarray, params: ExitPursuitParams) -> Optional[np.ndarray]:
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


def compute_steering(center: np.ndarray, params: ExitPursuitParams) -> float:
    """Pure pursuit using the target's actual distance from the rear axle.

    The circle normally sets the target distance to lfd. Short-path and
    outside-circle fallbacks still use their actual rear-axle distance.
    tan(delta) = wheelbase * gain * 2*y/(x*x+y*y), in rear-axle coordinates.
    """
    return steering_from_target(look_ahead_point(center, params), params)


def steering_from_target(target: Optional[np.ndarray], params: ExitPursuitParams) -> float:
    """Use the already selected pursuit target for the command and its visualisation."""
    if target is None:
        return 0.0
    x, y = float(target[0]) + params.rear_axle_x, float(target[1])
    distance_squared = x*x + y*y
    if not np.isfinite(distance_squared) or distance_squared <= 1e-12:
        return 0.0
    return float(np.arctan2(2.0 * params.wheelbase * params.steering_gain * y, distance_squared))
