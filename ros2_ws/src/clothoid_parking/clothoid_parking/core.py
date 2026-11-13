"""Everything between the sensor callbacks and the command, without ROS (the node only wires it)."""
import math
from dataclasses import replace
from collections import deque
from typing import Optional, Tuple

import numpy as np

from clothoid_parking.bay_finder import BayParams, find_bay, row_slope_angle
from clothoid_parking.planner import Phase, PlanParams, Planner
from clothoid_parking.tracker import Tracker, Pose2D
from clothoid_parking.layout import ParkingLayout


class ParkingCore:
    def __init__(self, plan: PlanParams = PlanParams(), bay: BayParams = BayParams(),
                 laser_x: float = 0.103, grid: float = 0.02, max_cells: int = 20000,
                 front_half_deg: float = 10.0, min_hits: int = 2, layout=None):
        self.tracker = Tracker(rear_offset=plan.rear_offset, odom_scale=plan.odom_scale)
        self.planner = Planner(replace(plan, strict_completion=True) if layout is not None and layout.enabled and layout.control_enabled else plan)
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
        self.layout = ParkingLayout(layout, min_hits=self.min_hits) if layout is not None and layout.enabled else None
        self.preparing = False
        self.approach_bay = None  # geometric candidate, not a declaration of free space
        self.pose_history = deque(maxlen=400)

    @property
    def phase(self) -> Phase:
        return self.planner.phase

    def imu(self, now: float, gyro_z: float):
        self.tracker.imu(now, gyro_z)

    def odom(self, now: float, speed: float):
        self.tracker.odom(now, speed)
        p = self.tracker.pose
        if not self.pose_history or now > self.pose_history[-1][0]:
            self.pose_history.append((now, Pose2D(p.x, p.y, p.yaw)))

    def prepare(self, on):
        if not on:
            self.approach_bay = None
        if on and not self.preparing and self.phase == Phase.IDLE and self.layout:
            self.layout.reset()
        self.preparing = on

    def pose_at(self, stamp):
        if not self.pose_history or stamp < self.pose_history[0][0] or stamp > self.pose_history[-1][0]+.10:
            return None
        if stamp >= self.pose_history[-1][0]:
            return self.pose_history[-1][1]
        for (a, p), (b, q) in zip(self.pose_history, list(self.pose_history)[1:]):
            if a <= stamp <= b:
                t = (stamp-a)/(b-a)
                dyaw = math.atan2(math.sin(q.yaw-p.yaw), math.cos(q.yaw-p.yaw))
                return Pose2D(p.x+t*(q.x-p.x), p.y+t*(q.y-p.y), p.yaw+t*dyaw)
        return None

    def observe_entry(self, stamp, origin, yaw):
        pose = self.pose_at(stamp)
        if self.layout is None or pose is None or not (self.preparing or self.flag):
            return False
        # Re-localization during a committed manoeuvre cannot move its target.
        if self.phase not in (Phase.IDLE, Phase.SEARCH):
            return False
        observed = self.layout.observe(stamp, origin, yaw, pose)
        if not observed:
            self.approach_bay = None
        return observed

    def set_flag(self, now: float, on: bool):
        if not on and self.layout is not None:
            self.layout.retain_free(now, False)
        if on and not self.flag and self.planner.phase == Phase.IDLE:
            self.cells.clear()
            self.planner.start(now, self.tracker.pose)
        self.flag = on

    def scan(self, now: float, ranges, angle_min: float, angle_increment: float,
             range_min: float = 0.02, range_max: float = 12.0):
        if self.layout is not None and (self.preparing or self.flag):
            pose = self.pose_at(now)
            if pose is not None:
                updated = self.layout.scan(now, ranges, angle_min, angle_increment,
                                           range_min, range_max, pose, self.laser_x)
                if updated and self.phase in (Phase.IDLE, Phase.SEARCH):
                    candidate = self.layout.choose(now, self.tracker.pose, self.planner.p)
                    if self.layout.p.control_enabled:
                        confirmed = (self.layout.origin is not None and
                                     self.layout.support_count >= self.layout.p.confirm_scans)
                        if confirmed and not self.layout.retaining:
                            self.layout.retain_free(now, True)
                        elif not confirmed:
                            self.layout.retain_free(now, False)
                        # A provisional target does not certify the bay as free.
                        # Preparation may advertise this geometry, but mission
                        # still requires its sign/entry checks; command() checks
                        # each near-term swept footprint after handover.
                        if (candidate is None and confirmed and
                                self.layout.states[self.layout.p.target_bay-1] == 'unknown'):
                            target = self.layout.bays()[self.layout.p.target_bay-1]
                            path = self.layout.path(target, self.tracker.pose, self.planner.p)
                            if path is not None and self.layout.coverage(path, now) != 'occupied':
                                candidate = target
                        self.approach_bay = candidate
                    if self.layout.p.control_enabled and self.phase == Phase.SEARCH:
                        # Revoke a candidate even inside the legacy freeze distance.
                        self.planner.bay = candidate
                        if self.layout.origin is not None:
                            self.planner.row_hint = self.layout.yaw
            if self.layout.p.control_enabled:
                return
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
        if len(points):
            # the row's direction is known before the gap is: drive along it, not along a tilted start
            angle = row_slope_angle(points, pose.x, pose.y, start.yaw, self.bay_params)
            if angle:
                self.planner.row_hint = start.yaw + self.bay_params.side * angle
        bay = find_bay(points, pose.x, pose.y, start.yaw, self.bay_params)
        self.planner.offer_bay(bay, pose)

    def points(self) -> np.ndarray:
        pts = [(c[0], c[1]) for c in self.cells.values() if c[2] >= self.min_hits]
        return np.array(pts) if pts else np.zeros((0, 2))

    def command(self, now: float) -> Optional[Tuple[float, float]]:
        previous_phase = self.phase
        if self.layout is not None and self.layout.p.control_enabled and self.phase not in (Phase.IDLE, Phase.DONE, Phase.ABORT):
            if now-self.layout.last_scan > .5:
                self.layout.retain_free(now, False)
                return 0., 0.
            if self.phase == Phase.SEARCH and (self.layout.origin is None or self.layout.support_count < self.layout.p.confirm_scans):
                return 0., 0.
            if self.phase == Phase.SEARCH and self.planner.bay is None:
                # Observe from a closer viewpoint, but never cross the turn
                # commitment point without a confirmed target footprint.
                target = self.layout.bays()[self.layout.p.target_bay-1]
                pose, p = self.tracker.pose, self.planner.p
                along = (target.face_x-pose.x)*math.cos(target.row_yaw)+(target.face_y-pose.y)*math.sin(target.row_yaw)
                remaining = along-p.turn_radius-p.turn_lead-self.layout.p.observation_stop_margin
                if now-self.layout.last_observation > self.layout.p.max_pose_age or remaining <= 0:
                    self.layout.reason = 'holding: designated bay not confirmed before turn'
                    return 0., 0.
                self.layout.reason = 'observing designated bay before turn'
                v = min(p.search_speed, max(p.min_speed, remaining))
                out = v, self.planner._hold(pose, target.row_yaw, v)
            else:
                out = self.planner.step(now, self.tracker.pose, self.front_range)
            if previous_phase == Phase.SEARCH and self.phase == Phase.TURN and not self.layout.retaining:
                self.layout.retain_free(now, True)
            elif self.phase not in (Phase.SEARCH, Phase.TURN, Phase.ENTER):
                self.layout.retain_free(now, False)
            if self.phase == Phase.PARKED and not self.layout.inside_bay(self.planner.bay, self.tracker.pose):
                self.planner._go(Phase.ABORT, now, 'vehicle corners outside designated bay')
                return 0., 0.
            if self.phase in (Phase.EXIT_BACK, Phase.EXIT_TURN):
                # Fixed parking obstacles: use the legacy IMU/odom reverse exit
                # along the entry route. The rear lidar sector is masked, so
                # requiring fresh free-space evidence here prevents all exits.
                # Scan freshness above and node IMU/odom checks still apply.
                self.layout.reason = 'exiting along entry route using IMU/odom'
                return out
            distance = .20
            if out is not None and self.phase == Phase.ENTER and self.planner.bay is not None:
                b, pose = self.planner.bay, self.tracker.pose
                remaining = abs((b.x-pose.x)*math.cos(b.entry_yaw)+(b.y-pose.y)*math.sin(b.entry_yaw))
                distance = min(distance, remaining)
            # The actuator clips full_lock to max_steer. Check the actual arc,
            # not an impossible tighter arc made from the unclipped command.
            if out is not None and not self.layout.motion_clear(
                    now, self.tracker.pose, out[0], max(-self.planner.p.max_steer,
                    min(self.planner.p.max_steer, out[1])), .2, distance):
                self.layout.reason = 'holding: '+self.layout.motion_reason
                return 0., 0.
            return out
        return self.planner.step(now, self.tracker.pose, self.front_range)
