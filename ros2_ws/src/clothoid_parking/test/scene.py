"""Test scene: the 2026 parking grid as in simul/.../parking_map.yaml, a ray-cast lidar and a car.

Car model: bicycle at the rear axle, wheelbase 0.2, full lock 23.3 deg (rear-axle radius 0.46 m,
limo2 2026-11-08), steering slews at 1.5 rad/s (about 0.27 s to lock, as measured), zero speed
centres the steering (limo_base), speed follows the command at 1 m/s^2.
"""
import math
from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np

LIMO_L, LIMO_W = 0.322, 0.220
WHEELBASE, REAR = 0.2, 0.1
LASER_X = 0.103
MAX_STEER = math.radians(23.3)

# upper row (next to the lane) and inner row centres, cars standing along y
COLUMNS = [6.360085, 6.880126, 7.400167]
UPPER_Y, INNER_Y = 4.270171, 3.649864
LANE_Y = 4.84
BAY_W, BAY_D = 0.5, 0.6


def bay_rect(col: int, y: float = UPPER_Y):
    """xmin, xmax, ymin, ymax inside the lines."""
    cx = COLUMNS[col]
    return cx - BAY_W / 2, cx + BAY_W / 2, y - BAY_D / 2, y + BAY_D / 2


def rect_corners(x, y, yaw, length=LIMO_L, width=LIMO_W):
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for dl, dw in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
        out.append((x + dl * length / 2 * c - dw * width / 2 * s,
                    y + dl * length / 2 * s + dw * width / 2 * c))
    return out


SIGN = (6.20, 4.52)   # parking sign pole beside the first bay (simulator map)


def parked_cars(empty_col: int = 1, inner: bool = True) -> List[Tuple[float, float, float]]:
    cars = [(COLUMNS[i], UPPER_Y, math.pi / 2) for i in range(3) if i != empty_col]
    if inner:
        cars += [(cx, INNER_Y, math.pi / 2) for cx in COLUMNS]
    return cars


def segments(cars, sign=True):
    segs = []
    for car in cars:
        pts = rect_corners(*car)
        segs += [(pts[i], pts[(i + 1) % 4]) for i in range(4)]
    if sign:   # 4 cm square pole
        pts = rect_corners(SIGN[0], SIGN[1], 0.0, 0.04, 0.04)
        segs += [(pts[i], pts[(i + 1) % 4]) for i in range(4)]
    return np.array(segs)  # (N, 2, 2)


def scan(segs, x, y, yaw, fov_deg=110.0, step_deg=0.9, max_range=8.0, noise=0.0, rng=None,
         spikes=0.0, dropout=0.0):
    """LaserScan-like (ranges with inf for no return, angle_min, angle_increment) from base_link
    pose (x, y, yaw)."""
    lx, ly = x + LASER_X * math.cos(yaw), y + LASER_X * math.sin(yaw)
    rel = np.radians(np.arange(-fov_deg, fov_deg + 1e-9, step_deg))
    angles = yaw + rel
    d = np.stack([np.cos(angles), np.sin(angles)], 1)              # (R, 2)
    a, b = segs[:, 0], segs[:, 1]                                   # (S, 2)
    e = b - a
    # solve l + t d = a + s e for every ray/segment pair
    denom = d[:, None, 0] * e[None, :, 1] - d[:, None, 1] * e[None, :, 0]
    ax, ay = a[None, :, 0] - lx, a[None, :, 1] - ly
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (ax * e[None, :, 1] - ay * e[None, :, 0]) / denom
        s = (ax * d[:, None, 1] - ay * d[:, None, 0]) / denom
    hit = (np.abs(denom) > 1e-12) & (t > 0.02) & (s >= 0) & (s <= 1)
    t = np.where(hit, t, np.inf).min(axis=1)
    if noise and rng is not None:
        t = t + rng.normal(0, noise, t.shape)
    if spikes and rng is not None:   # limo2 Tmini Plus: ~1 % of beams jump, std ~0.3 m
        hit = np.isfinite(t) & (rng.random(t.shape) < spikes)
        t = np.where(hit, t + rng.normal(0, 0.3, t.shape), t)
    if dropout and rng is not None:  # 1.8 % read 0
        t = np.where(rng.random(t.shape) < dropout, 0.0, t)
    t[t > max_range] = np.inf
    return t, float(rel[0]), math.radians(step_deg)


@dataclass
class Car:
    x: float            # base_link
    y: float
    yaw: float
    v: float = 0.0
    steer: float = 0.0
    slew: float = 1.5   # rad/s
    accel: float = 1.0
    trace: list = field(default_factory=list)

    def step(self, dt, v_cmd, steer_cmd):
        dv = max(-self.accel * dt, min(self.accel * dt, v_cmd - self.v))
        self.v += dv
        target = 0.0 if abs(v_cmd) < 1e-3 else max(-MAX_STEER, min(MAX_STEER, steer_cmd))
        ds = max(-self.slew * dt, min(self.slew * dt, target - self.steer))
        self.steer += ds
        rx, ry = self.x - REAR * math.cos(self.yaw), self.y - REAR * math.sin(self.yaw)
        rx += self.v * dt * math.cos(self.yaw)
        ry += self.v * dt * math.sin(self.yaw)
        self.yaw += self.v * dt * math.tan(self.steer) / WHEELBASE
        self.x, self.y = rx + REAR * math.cos(self.yaw), ry + REAR * math.sin(self.yaw)
        self.trace.append((self.x, self.y, self.yaw))

    def corners(self):
        return rect_corners(self.x, self.y, self.yaw)


def corners_outside(car: Car, rect, line=0.0) -> int:
    xmin, xmax, ymin, ymax = rect
    return sum(not (xmin + line < cx < xmax - line and ymin + line < cy < ymax - line)
               for cx, cy in car.corners())


def collides(car: Car, cars) -> bool:
    """Separating axis test between the car body and every parked car."""
    mine = np.array(car.corners())
    for other in cars:
        theirs = np.array(rect_corners(*other))
        apart = False
        for poly in (mine, theirs):
            for i in range(4):
                edge = poly[(i + 1) % 4] - poly[i]
                axis = np.array([-edge[1], edge[0]])
                pa, pb = mine @ axis, theirs @ axis
                if pa.max() < pb.min() or pb.max() < pa.min():
                    apart = True
                    break
            if apart:
                break
        if not apart:
            return True
    return False
