"""Feedback-confirmed connection from cone pursuit to a live lane controller.

No exit direction, waypoint, elapsed-time manoeuvre or straight-road assumption.
The lane can request either turn; limits describe the vehicle and the bounded
local manoeuvre, rather than a particular cone arrangement.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass
class ConnectionParams:
    speed: float = .2
    steer_rate_deg_s: float = 20.
    confirm_sec: float = .3
    max_distance: float = 1.
    max_duration: float = 6.

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'exit_connect_{name} must be finite and positive')


def central_limit(wheelbase):
    # Same LIMO inner-wheel limit and track as the driver / cone planner.
    if not math.isfinite(wheelbase) or wheelbase <= 0.:
        raise ValueError('wheelbase must be finite and positive')
    return math.atan(wheelbase / (wheelbase / math.tan(math.radians(28.)) + .175 / 2))


def forward_arc(steering, wheelbase, rear_axle_x, length, spacing):
    """Rear-axle swept centre path, base_link coordinates, for clearance checking."""
    s = np.linspace(0., length, max(2, int(math.ceil(length / spacing)) + 1))
    curvature = math.tan(steering) / wheelbase
    if abs(curvature) < 1e-8:
        return np.column_stack((s - rear_axle_x, np.zeros_like(s)))
    return np.column_stack((np.sin(curvature * s) / curvature - rear_axle_x,
                            (1. - np.cos(curvature * s)) / curvature))


class LaneConnection:
    def __init__(self, params=None):
        self.p = params or ConnectionParams()
        self.clear()

    def clear(self):
        self.started = self.last_time = self.last_pose = self.settled_since = None
        self.steering = self.travelled = 0.
        self.failed = False

    @property
    def active(self):
        return self.started is not None

    def start(self, now, pose, steering, wheelbase):
        if not all(math.isfinite(v) for v in (*pose, now, steering)):
            raise ValueError('connection start needs a finite pose, time and steering')
        self.clear()
        self.started = self.last_time = now
        self.last_pose = np.asarray(pose[:2], float)
        limit = central_limit(wheelbase)
        self.steering = float(np.clip(steering, -limit, limit))

    def pause(self, now):
        # Zero speed centres the LIMO wheels. Restart the slew from that angle;
        # never spend a sensor outage accumulating permission for a large jump.
        self.steering = 0.
        self.settled_since = None
        self.last_time = now

    def step(self, now, pose, speed, angular_z, wheelbase, max_speed,
             measured_speed, measured_yaw_rate, lane_ready, max_dt):
        """Return (speed, central steering, ready); confirmation requires motion feedback."""
        if not self.active:
            return 0., 0., False
        values = (*pose, now, speed, angular_z, measured_speed, measured_yaw_rate)
        if not all(math.isfinite(v) for v in values):
            self.pause(now)
            return 0., 0., False
        position = np.asarray(pose[:2], float)
        self.travelled += float(np.linalg.norm(position - self.last_pose))
        self.last_pose = position
        dt = now - self.last_time
        self.last_time = now
        if dt > max_dt:
            self.settled_since = None
        self.failed |= (dt < 0. or now - self.started > self.p.max_duration
                        or self.travelled > self.p.max_distance)
        if self.failed or speed <= 1e-3 or max_speed <= 0.:
            self.pause(now)
            return 0., 0., False
        limit = central_limit(wheelbase)
        requested = math.atan(wheelbase * angular_z / speed)
        desired = float(np.clip(requested, -limit, limit))
        max_step = math.radians(self.p.steer_rate_deg_s) * min(max(0., dt), max_dt)
        self.steering += float(np.clip(desired - self.steering, -max_step, max_step))
        commanded_speed = min(speed, max_speed, self.p.speed)
        # Compare equivalent central angles, not angular velocities at different
        # speeds. A persistent curve is just as valid as a straight lane.
        moving = measured_speed > .03
        measured = math.atan(wheelbase * measured_yaw_rate / measured_speed) if moving else 0.
        settled = (lane_ready and moving
                   and abs(requested - desired) <= math.radians(1.)
                   and abs(self.steering - desired) <= math.radians(1.)
                   and abs(measured - desired) <= math.radians(3.)
                   and abs(measured_speed - commanded_speed) <= max(.03, commanded_speed * .2))
        if not settled:
            self.settled_since = None
        elif self.settled_since is None:
            self.settled_since = now
        ready = self.settled_since is not None and now - self.settled_since >= self.p.confirm_sec
        return commanded_speed, self.steering, ready
