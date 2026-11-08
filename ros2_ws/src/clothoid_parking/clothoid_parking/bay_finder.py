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
    # second row (behind the lane-side row, 620 mm further): used when the lane-side row has no
    # gap, through an empty lane-side bay in front of it
    inner_row: bool = True
    far2: float = 1.45            # m, band far..far2 holds the second row's near ends (~1.03 m)
    corridor_margin: float = 0.06  # m kept off the gap edges when checking the way in is empty
    min_flank: float = 0.12       # m along the row a flank must cover: a LIMO's end is 0.22 m wide,
                                  # the sign pole 0.04 m (it once made a skewed 'bay' with one car)


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

    def face_point(self):
        """(u, lat) of the near end: the 10 % nearest lateral distance and where along it lies."""
        near = float(np.percentile(self.lat, 10))
        sel = self.lat <= near + 0.02
        return float(np.median(self.u[sel])), near


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


def row_slope_angle(points: np.ndarray, ref_x: float, ref_y: float, ref_yaw: float,
                    p: BayParams) -> float:
    """Angle (rad, lat-towards-the-bays per metre along) of the line through the parked cars' near
    ends, 0 when it cannot be told. Theil-Sen over the cluster faces, then a fit without the faces
    off that line (the sign pole, a box)."""
    c, s = math.cos(ref_yaw), math.sin(ref_yaw)
    dx, dy = points[:, 0] - ref_x, points[:, 1] - ref_y
    u = dx * c + dy * s
    lat = (-dx * s + dy * c) * p.side
    # a wider lateral band than the search: tilted, the far cars drift out of near..far
    keep = (lat > p.near) & (lat < (p.far2 if p.inner_row else p.far + 0.3)) & (u > -p.behind) & (u < p.ahead)
    # both ends of every cluster: the sign pole merges with the first car, and its end then reads
    # the pole, while the far end of that cluster is still the car's face
    faces = []
    for cl in clusters_along_row(u[keep], lat[keep], p):
        if cl.u_max - cl.u_min < p.min_flank:
            continue
        w = p.face_window
        if cl.u_max - cl.u_min < 2 * w:
            faces.append(cl.face_point())
            continue
        for lo, hi in ((cl.u_min, cl.u_min + w), (cl.u_max - w, cl.u_max)):
            m = (cl.u >= lo) & (cl.u <= hi)
            near = float(np.percentile(cl.lat[m], 10))
            faces.append((float(np.median(cl.u[m & (cl.lat <= near + 0.02)])), near))
    if len(faces) < 2:
        return 0.0
    fu = np.array([f[0] for f in faces])
    fl = np.array([f[1] for f in faces])
    # pairs of near ends in one row only (the two rows are 0.62 m apart)
    slopes = [(fl[j] - fl[i]) / (fu[j] - fu[i]) for i in range(len(fu)) for j in range(i + 1, len(fu))
              if fu[j] - fu[i] > 0.3 and abs(fl[j] - fl[i]) < 0.3]
    if not slopes:
        return 0.0
    k = float(np.median(slopes))
    # offset of the row with most near ends on it (either row fits the same slope)
    res = fl - k * fu
    b = max((res[np.abs(res - r) < p.face_tolerance].size, r) for r in res)[1]
    on = np.abs(fl - (b + k * fu)) < p.face_tolerance
    if on.sum() >= 2 and np.ptp(fu[on]) > 0.3:
        k = float(np.polyfit(fu[on], fl[on], 1)[0])
    angle = math.atan(k)
    return angle if abs(angle) <= math.radians(p.max_row_angle_deg) else 0.0


def find_bay(points: np.ndarray, ref_x: float, ref_y: float, ref_yaw: float,
             p: BayParams = BayParams()) -> Optional[Bay]:
    """Nearest empty bay ahead of (ref_x, ref_y) along ref_yaw, or None."""
    if points is None or len(points) == 0:
        return None
    # The search frame starts along the car's heading. If the car is not parallel to the row, the
    # near ends of the parked cars look staggered in it (0.93 m x tan(4 deg) = 6.5 cm, past
    # face_tolerance), so first turn the frame onto the row the near ends actually lie on.
    ref_yaw = ref_yaw + p.side * row_slope_angle(points, ref_x, ref_y, ref_yaw, p)
    c, s = math.cos(ref_yaw), math.sin(ref_yaw)
    dx, dy = points[:, 0] - ref_x, points[:, 1] - ref_y
    u = dx * c + dy * s
    lat = (-dx * s + dy * c) * p.side          # positive towards the bays
    ahead = (u > -p.behind) & (u < p.ahead)
    row1 = ahead & (lat > p.near) & (lat < p.far)
    found = _gap(u[row1], lat[row1], p)
    if found is None and p.inner_row:
        row2 = ahead & (lat >= p.far) & (lat < p.far2)
        found = _gap(u[row2], lat[row2], p)
        if found is not None:
            # the lane-side bay in front must be empty: nothing in the first band across the gap
            lo, hi = found[0] - found[2] / 2 + p.corridor_margin, found[0] + found[2] / 2 - p.corridor_margin
            if np.any(row1 & (u > lo) & (u < hi)):
                found = None
    if found is None:
        return None
    uc, w_face, gap, row = found
    fx, fy = ref_x + uc * c - w_face * s, ref_y + uc * s + w_face * c
    row_yaw = ref_yaw + row
    entry_yaw = row_yaw + p.side * math.pi / 2
    return Bay(x=fx + p.face_to_centre * math.cos(entry_yaw),
               y=fy + p.face_to_centre * math.sin(entry_yaw),
               row_yaw=row_yaw, entry_yaw=entry_yaw, width=gap, face_x=fx, face_y=fy)


def _gap(u: np.ndarray, lat: np.ndarray, p: BayParams):
    """Nearest gap between two clusters of one row: (u of its centre, near-end w, width, row angle)."""
    clusters = [cl for cl in clusters_along_row(u, lat, p) if cl.u_max - cl.u_min >= p.min_flank]
    for a, b in zip(clusters, clusters[1:]):
        gap = b.u_min - a.u_max
        if not (p.gap_min <= gap <= p.gap_max):
            continue
        fa = a.face(a.u_max - p.face_window, a.u_max)
        fb = b.face(b.u_min, b.u_min + p.face_window)
        if not abs(fa - fb) <= p.face_tolerance:
            continue
        # row direction from the two near ends; w = lat * side
        row = math.atan2((fb - fa) * p.side, gap + p.face_window)
        if abs(row) > math.radians(p.max_row_angle_deg):
            continue
        return 0.5 * (a.u_max + b.u_min), 0.5 * (fa + fb) * p.side, gap, row
    return None
