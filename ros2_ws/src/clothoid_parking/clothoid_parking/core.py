"""Everything between the sensor callbacks and the command, without ROS (the node only wires it)."""
import math
from typing import Optional, Tuple

import numpy as np

from clothoid_parking.bay_finder import BayParams, find_bay
from clothoid_parking.planner import Phase, PlanParams, Planner
from clothoid_parking.tracker import Tracker


class ParkingCore:
    def __init__(self, plan: PlanParams = PlanParams(), bay: BayParams = BayParams(),
                 laser_x: float = 0.103, grid: float = 0.02, max_cells: int = 20000,
                 front_half_deg: float = 10.0, min_hits: int = 2):
        self.tracker = Tracker(rear_offset=plan.rear_offset, odom_scale=plan.odom_scale)
        self.planner = Planner(plan)
        self.bay_params = bay
        self.laser_x = laser_x
        self.grid = grid
        self.max_cells = max_cells
        self.front_half = math.radians(front_half_deg)
        self.cells = {}     # grid cell -> [x, y, hits]
        # a cell counts once it was hit this often: lidar spikes (~1 % of beams jump > 10 cm on the
        # Tmini Plus) land somewhere new every time, real surfaces come back
        self.min_hits = min_hits
        self.front_range: Optional[float] = None
        self.flag = False

    @property
    def phase(self) -> Phase:
        return self.planner.phase

    def imu(self, now: float, gyro_z: float):
        self.tracker.imu(now, gyro_z)

    def odom(self, now: float, speed: float):
        self.tracker.odom(now, speed)

    def set_flag(self, now: float, on: bool):
        if on and not self.flag and self.planner.phase == Phase.IDLE:
            self.cells.clear()
            self.planner.start(now, self.tracker.pose)
        self.flag = on

    def scan(self, now: float, ranges, angle_min: float, angle_increment: float,
             range_min: float = 0.02, range_max: float = 12.0):
        r = np.asarray(ranges, dtype=float)
        a = angle_min + np.arange(len(r)) * angle_increment
        ok = np.isfinite(r) & (r > max(range_min, 0.02)) & (r < range_max)
        front = ok & (np.abs(a) < self.front_half)
        # third closest: a single spike or stray beam must not count as an obstacle ahead
        fr = np.sort(r[front])
        self.front_range = float(fr[2]) if len(fr) >= 3 else None
        if self.planner.phase != Phase.SEARCH or not ok.any():
            return
        pose = self.tracker.pose
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        lx = r[ok] * np.cos(a[ok]) + self.laser_x
        ly = r[ok] * np.sin(a[ok])
        wx, wy = pose.x + lx * c - ly * s, pose.y + lx * s + ly * c
        for key, px, py in zip(zip(np.round(wx / self.grid).astype(int),
                                   np.round(wy / self.grid).astype(int)), wx, wy):
            cell = self.cells.get(key)
            if cell is not None:
                cell[2] += 1
            elif len(self.cells) < self.max_cells:
                self.cells[key] = [px, py, 1]
        points = self.points()
        start = self.planner._start
        bay = find_bay(points, pose.x, pose.y, start.yaw, self.bay_params)
        self.planner.offer_bay(bay, pose)

    def points(self) -> np.ndarray:
        pts = [(c[0], c[1]) for c in self.cells.values() if c[2] >= self.min_hits]
        return np.array(pts) if pts else np.zeros((0, 2))

    def command(self, now: float) -> Optional[Tuple[float, float]]:
        return self.planner.step(now, self.tracker.pose, self.front_range)
