"""Replay the legacy planner for visualization only; never publish a drive command."""
import math

from clothoid_parking.core import ParkingCore
from clothoid_parking.planner import Phase


def entry_path(pose, bay, params):
    """Nominal rear-axle straight/arc/entry reference, using the legacy turn geometry.

    This is a geometric reference, not a simulated driven trajectory: steering
    slew and the planner's early turn completion are not modelled here.
    """
    c, s = math.cos(bay.row_yaw), math.sin(bay.row_yaw)
    rx = pose.x-params.rear_offset*math.cos(pose.yaw)
    ry = pose.y-params.rear_offset*math.sin(pose.yaw)
    off = -(rx-bay.face_x)*s+(ry-bay.face_y)*c
    along = params.turn_radius+params.turn_lead
    tx = bay.face_x-along*c-off*s
    ty = bay.face_y-along*s+off*c
    cx = tx-params.side*params.turn_radius*s
    cy = ty+params.side*params.turn_radius*c
    start = math.atan2(ty-cy, tx-cx)
    points = [(rx, ry)]
    for k in range(25):
        angle = start+params.side*k*math.pi/48
        points.append((cx+params.turn_radius*math.cos(angle),
                       cy+params.turn_radius*math.sin(angle)))
    points.append((bay.x-params.rear_offset*math.cos(bay.entry_yaw),
                   bay.y-params.rear_offset*math.sin(bay.entry_yaw)))
    return points


class ParkingPreview:
    def __init__(self, plan, bay, laser_x):
        self.core = ParkingCore(plan, bay, laser_x=laser_x)
        self.path = []

    def step(self, now):
        # Save in the local world frame before the turn. Do not regenerate the
        # planned arc from the already-turning vehicle's changing lateral offset.
        planner = self.core.planner
        if planner.phase == Phase.SEARCH and planner.bay is not None:
            self.path = entry_path(self.core.tracker.pose, planner.bay, planner.p)
        self.core.command(now)  # only advances preview state; output is intentionally discarded
