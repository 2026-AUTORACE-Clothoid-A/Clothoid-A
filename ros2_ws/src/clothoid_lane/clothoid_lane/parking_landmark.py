"""Metric lane/entry-line registration, without ROS or a learned model.

A transverse line alone is NOT a parking landmark. This publishes a candidate;
the parking consumer must corroborate the layout with lidar before using it.
Camera calibration and drawing dimensions are priors, never ground truth.
"""
import math
from dataclasses import dataclass
import cv2
import numpy as np
from clothoid_common.parking_geometry import PARKING_GEOMETRY


@dataclass
class LandmarkParams:
    camera_height: float = .19
    camera_pitch: float = -.021
    camera_yaw: float = 0.
    camera_x: float = .15
    camera_y: float = -.01
    side: float = -1.
    # CAD: stop stripe centre x=5.629878, first bay boundary centre x=6.100085.
    entry_to_grid: float = PARKING_GEOMETRY.entry_to_grid
    near: float = .38
    far: float = 1.2


class ParkingLandmark:
    def __init__(self, params=None):
        self.p = params or LandmarkParams()
        self.lookup = None
        self.key = None

    def configure_camera(self, width, height, k, d):
        key = (width, height, tuple(k), tuple(d), self.p.camera_height,
               self.p.camera_pitch, self.p.camera_yaw, self.p.camera_x, self.p.camera_y)
        if self.key == key:
            return
        K = np.asarray(k, dtype=float).reshape(3, 3)
        if not np.isfinite(K).all() or not np.isfinite(d).all() or K[0, 0] <= 0 or K[1, 1] <= 0:
            raise ValueError('uncalibrated camera')
        v, u = np.mgrid[:height, :width]
        uv = np.stack((u.ravel(), v.ravel()), axis=1).astype(np.float32)
        norm = cv2.undistortPoints(uv.reshape(-1, 1, 2), K, np.asarray(d, dtype=float))
        norm = norm.reshape(height, width, 2)
        p = self.p
        below = np.arctan(norm[:, :, 1]) + p.camera_pitch
        fwd = p.camera_height / np.tan(np.maximum(below, .001))
        z = fwd * math.cos(p.camera_pitch) + p.camera_height * math.sin(p.camera_pitch)
        lateral = -norm[:, :, 0] * z
        c, s = math.cos(p.camera_yaw), math.sin(p.camera_yaw)
        gx = c*fwd-s*lateral+p.camera_x
        gy = s*fwd+c*lateral+p.camera_y
        roi = (below > math.radians(.5)) & (gx > .35) & (gx < 1.8) & (abs(gy) < .65)
        self.lookup = gx, gy, roi
        self.key = key

    def detect(self, bgr, lane_points=None):
        if self.lookup is None or bgr.shape[:2] != self.lookup[0].shape:
            return None
        gx, gy, roi = self.lookup
        # Full image decode is shared with lane. Projection is cached; only run at 5 Hz when armed.
        line = find_line(bgr, gx, gy, roi)
        if line is None or not self.p.near <= line['rho'] <= self.p.far:
            return None
        road = road_heading(line, lane_points, self.p.side)
        if road is None:
            return None
        angle, edge = road
        n = np.array([math.cos(angle), math.sin(angle)])
        origin = edge + self.p.entry_to_grid*n
        return dict(line, stripe_angle=line['angle'], angle=angle, origin=origin.tolist())


def road_heading(stripe, lane_points, side=-1.):
    """Two parallel road boundaries supply heading; the stripe supplies distance.

    A single edge or two conflicting lines cannot create a new parking frame.
    These are camera observations sharing one calibration, not independent GT.
    """
    segments = fit_boundary_segments([] if lane_points is None else lane_points)
    if len(segments) != 2:
        return None
    directions = [q[1]-q[0] for q in segments]
    angles = np.array([math.atan2(v[1], v[0]) for v in directions])
    # Same angular consistency scale as the registered parking layout (0.07 rad).
    if np.ptp(angles) > .07:
        return None
    angle = float(np.mean(angles))
    if abs(angle-stripe['angle']) > math.radians(12):
        return None
    n = np.array([math.cos(angle), math.sin(angle)])
    stripe_normal = np.array([math.cos(stripe['angle']), math.sin(stripe['angle'])])
    centres = [q.mean(axis=0) for q in segments]
    edges = [c+(stripe['rho']-c@stripe_normal)/(n@stripe_normal)*n for c in centres]
    lateral = np.array([-n[1], n[0]])
    edges.sort(key=lambda q: float(q@lateral))
    if abs(float((edges[1]-edges[0])@lateral)-PARKING_GEOMETRY.boundary_spacing) > .07:
        return None
    edge = edges[0 if side < 0 else 1]
    old_edge = np.asarray(stripe['corners'][0 if side < 0 else 1])
    if np.linalg.norm(edge-old_edge) > .07:
        return None
    return angle, edge


def find_line(im, gx, gy, roi):
    hsv = cv2.cvtColor(cv2.GaussianBlur(im, (5, 5), 0), cv2.COLOR_BGR2HSV)
    white = (hsv[:, :, 1] <= 60) & (hsv[:, :, 2] >= 150) & roi
    yellow = (hsv[:, :, 0] >= 18) & (hsv[:, :, 0] <= 35) & (hsv[:, :, 1] >= 60) & (hsv[:, :, 2] >= 90) & roi
    hp = cv2.HoughLinesP(white.astype(np.uint8) * 255, 1, np.pi / 180, 45, minLineLength=100, maxLineGap=8)
    if hp is None:
        return None
    wy, wx = np.nonzero(white)
    wp = np.column_stack((gx[wy, wx], gy[wy, wx]))
    yy, yx = np.nonzero(yellow)
    yp = np.column_stack((gx[yy, yx], gy[yy, yx]))
    out = []
    for (x1, y1, x2, y2), in hp:
        p1 = np.array((gx[y1, x1], gy[y1, x1]))
        p2 = np.array((gx[y2, x2], gy[y2, x2]))
        delta = p2 - p1
        length = np.linalg.norm(delta)
        if length < 0.25:
            continue
        tangent = delta / length
        normal = np.array((-tangent[1], tangent[0]))
        if normal[0] < 0:
            normal = -normal
        if abs(math.atan2(normal[1], normal[0])) > math.radians(30):
            continue
        rho = normal @ p1
        along = wp @ tangent
        lim = sorted((p1 @ tangent, p2 @ tangent))
        fit = (abs(wp @ normal - rho) < 0.02) & (along > lim[0] - 0.05) & (along < lim[1] + 0.05)
        pts = wp[fit]
        if len(pts) < 70:
            continue
        centre = np.mean(pts, axis=0)
        vals, vecs = np.linalg.eigh(np.cov(pts.T))
        n = vecs[:, 0]
        if n[0] < 0:
            n = -n
        angle = math.atan2(n[1], n[0])
        rho = n @ centre
        if abs(angle) > math.radians(30) or math.sqrt(vals[0]) > 0.015:
            continue
        tangent = np.array((-n[1], n[0]))
        support = yp[abs(yp @ n - rho) < 0.035]
        if len(support) < 8:
            continue
        coords = np.sort(support @ tangent)
        clusters = np.split(coords, np.where(np.diff(coords) > 0.06)[0] + 1)
        ends = [float(np.median(group)) for group in clusters if len(group) >= 4]
        pairs = []
        for a, b in zip(ends, ends[1:]):
            if 0.35 < b - a < 0.75 and a < 0.08 and (b > -0.08):
                pairs.append((a, b))
        if not pairs:
            continue
        a, b = min(pairs, key=lambda ab: abs(ab[1] - ab[0] - PARKING_GEOMETRY.boundary_spacing))
        corners = [rho * n + a * tangent, rho * n + b * tangent]
        extent = np.percentile(pts @ tangent, [2, 98])
        if extent[0] > a + 0.07 or extent[1] < b - 0.07:
            continue
        out.append({'rho': float(rho), 'angle': angle, 'width': b - a, 'corners': [q.tolist() for q in corners], 'fit_error': math.sqrt(vals[0]), 'raw_segment': [int(x1), int(y1), int(x2), int(y2)]})
    return min(out, key=lambda d: d['rho']) if out else None


def fit_boundary_segments(lane_points):
    """Compact straight-line observations from existing metric lane samples."""
    segments = []
    for points in lane_points:
        points = np.asarray(points, dtype=float)
        if len(points) < 4 or points.shape[1:] != (2,) or not np.isfinite(points).all():
            continue
        centre = np.mean(points, axis=0)
        values, vectors = np.linalg.eigh(np.cov(points.T))
        direction = vectors[:, -1]
        if direction[0] < 0:
            direction = -direction
        if abs(math.atan2(direction[1], direction[0])) > math.radians(30) or math.sqrt(max(0., values[0])) > .015:
            continue
        extents = np.percentile((points-centre)@direction, [5, 95])
        if extents[1]-extents[0] < .18:
            continue
        segments.append(centre+extents[:, None]*direction)
    return segments
