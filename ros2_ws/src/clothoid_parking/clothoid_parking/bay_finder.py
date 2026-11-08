"""Find the empty bay between two parked LIMOs from lidar points.

The car drives along the bay row with the bays on one side (right by default). The parked LIMOs
stand in their bays nose or tail towards the lane, so the lidar sees their near ends as short
segments on one line (the row) and the empty bay as a gap of about one bay between two of them:
2026 grid 500 mm bays at 520 mm pitch, LIMO 220 mm wide, so 0.82 m between two cars around an
empty bay and 0.30 m between neighbours.

Points come in the local (odom) frame. They are taken into a frame along `ref_yaw` (the car's
heading when the search started, near the lane direction): u ahead, w to the left.
"""
import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np


@dataclass
class BayParams:
    side: float = -1.0            # -1: bays on the right, +1: on the left
    near: float = 0.15            # m, lateral band (from the car's path) where the parked cars are
    far: float = 0.80             # stops before the second row's cars (about 1.0 m away)
    behind: float = 0.6           # m, keep points this far behind the reference point
    ahead: float = 3.0
    cluster_gap: float = 0.15     # m along the row that splits two clusters (0.9 deg lidar steps
                                  # leave grazing faces sparse; neighbours stand 0.30 m apart)
    min_points: int = 2
    gap_min: float = 0.55         # m between two cars around an empty bay (nominal 0.82)
    gap_max: float = 1.20
    face_tolerance: float = 0.06  # m, the two near ends must line up this well
    face_window: float = 0.12     # m, near end measured only this close to the gap (a sign pole
                                  # beside the first car must not count as the car's end)
    max_row_angle_deg: float = 12.0
    face_to_centre: float = 0.161  # m from a parked LIMO's near end to its centre (322 mm long)


@dataclass
class Cluster:
    u: np.ndarray    # along the row, sorted
    lat: np.ndarray  # lateral distance, positive towards the bays

    @property
    def u_min(self) -> float:
        return float(self.u[0])

    @property
    def u_max(self) -> float:
        return float(self.u[-1])

    def face(self, from_u: float, to_u: float) -> float:
        """Near end (closest lateral distance) of the points between from_u and to_u."""
        m = (self.u >= from_u) & (self.u <= to_u)
        return float(np.percentile(self.lat[m], 10)) if m.any() else float('nan')


@dataclass
class Bay:
    x: float        # bay centre (where base_link should end), local frame
    y: float
    row_yaw: float  # direction of the row (driving direction past the bays)
    entry_yaw: float  # heading of a car parked nose-in
    width: float    # gap between the two flank cars
    face_x: float   # point on the line of the flank cars' near ends, centre of the gap
    face_y: float


def clusters_along_row(u: np.ndarray, lat: np.ndarray, p: BayParams) -> List[Cluster]:
    if len(u) == 0:
        return []
    order = np.argsort(u)
    u, lat = u[order], lat[order]
    breaks = np.flatnonzero(np.diff(u) > p.cluster_gap) + 1
    out = []
    for idx in np.split(np.arange(len(u)), breaks):
        if len(idx) >= p.min_points:
            out.append(Cluster(u[idx], lat[idx]))
    return out


def find_bay(points: np.ndarray, ref_x: float, ref_y: float, ref_yaw: float,
             p: BayParams = BayParams()) -> Optional[Bay]:
    """Nearest empty bay ahead of (ref_x, ref_y) along ref_yaw, or None."""
    if points is None or len(points) == 0:
        return None
    c, s = math.cos(ref_yaw), math.sin(ref_yaw)
    dx, dy = points[:, 0] - ref_x, points[:, 1] - ref_y
    u = dx * c + dy * s
    lat = (-dx * s + dy * c) * p.side          # positive towards the bays
    keep = (lat > p.near) & (lat < p.far) & (u > -p.behind) & (u < p.ahead)
    clusters = clusters_along_row(u[keep], lat[keep], p)
    for a, b in zip(clusters, clusters[1:]):
        gap = b.u_min - a.u_max
        if not (p.gap_min <= gap <= p.gap_max):
            continue
        fa = a.face(a.u_max - p.face_window, a.u_max)
        fb = b.face(b.u_min, b.u_min + p.face_window)
        if not abs(fa - fb) <= p.face_tolerance:
            continue
        # row direction from the two near ends; w = lat * side
        du = gap + p.face_window
        row = math.atan2((fb - fa) * p.side, du)
        if abs(row) > math.radians(p.max_row_angle_deg):
            continue
        uc = 0.5 * (a.u_max + b.u_min)
        w_face = 0.5 * (fa + fb) * p.side
        fx, fy = ref_x + uc * c - w_face * s, ref_y + uc * s + w_face * c
        row_yaw = ref_yaw + row
        entry_yaw = row_yaw + p.side * math.pi / 2
        return Bay(x=fx + p.face_to_centre * math.cos(entry_yaw),
                   y=fy + p.face_to_centre * math.sin(entry_yaw),
                   row_yaw=row_yaw, entry_yaw=entry_yaw, width=gap, face_x=fx, face_y=fy)
    return None
