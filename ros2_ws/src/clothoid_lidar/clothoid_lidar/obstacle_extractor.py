"""2D lidar scan -> small round obstacles (cones, parked cars' corners).

ROS-free. Replaces the 2025 `obstacle_detector` package (`/raw_obstacles` circles), which has
no ROS2 release. It is a plain Euclidean clustering of consecutive beams, enough for cones;
the rubber-cone and static-obstacle missions both start from its output.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class ExtractorParams:
    range_min: float = 0.1       # m, ignore closer returns (own body)
    range_max: float = 2.0       # m
    angle_min_deg: float = -110.0  # YDLidar Tmini Plus on the LIMO sees +-110 deg
    angle_max_deg: float = 110.0
    cluster_gap: float = 0.08    # m, max distance between neighbouring beams of one object
    min_points: int = 2          # beams per object
    max_width: float = 0.15      # m, wider clusters (walls, cars) are not cones (retune)
    min_width: float = 0.0       # m, narrower clusters are dropped (0 keeps all; LIMO search skips cones)
    cone_radius: float = 0.04    # m, centre = surface centroid pushed back by this (retune)


def extract_obstacles(ranges, angle_min: float, angle_increment: float,
                      params: ExtractorParams) -> np.ndarray:
    """Return an (N, 2) array of obstacle centres in the scan frame (x forward, y left)."""
    ranges = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(ranges.size)
    lo, hi = np.deg2rad(params.angle_min_deg), np.deg2rad(params.angle_max_deg)
    valid = (np.isfinite(ranges) & (ranges >= params.range_min) & (ranges <= params.range_max)
             & (angles >= lo) & (angles <= hi))

    index = np.flatnonzero(valid)
    if index.size == 0:
        return np.zeros((0, 2))
    points = np.column_stack((ranges[index] * np.cos(angles[index]),
                              ranges[index] * np.sin(angles[index])))

    centres = []
    start = 0
    for i in range(1, index.size + 1):
        # a cluster ends at a missing beam or when neighbours are too far apart
        ends = (i == index.size
                or index[i] != index[i - 1] + 1
                or np.linalg.norm(points[i] - points[i - 1]) > params.cluster_gap)
        if not ends:
            continue
        cluster = points[start:i]
        start = i
        if len(cluster) < params.min_points:
            continue
        width = np.linalg.norm(cluster[-1] - cluster[0])
        if width > params.max_width or width < params.min_width:
            continue
        centroid = cluster.mean(axis=0)
        norm = np.linalg.norm(centroid)
        centres.append(centroid + centroid / norm * params.cone_radius if norm > 1e-6
                       else centroid)

    return np.array(centres) if centres else np.zeros((0, 2))


def transform_points(points: np.ndarray, tx: float, ty: float, yaw: float) -> np.ndarray:
    """Express scan-frame points in the target frame (frame origin at tx, ty rotated by yaw)."""
    if points.size == 0:
        return points
    c, s = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    return points @ rotation.T + np.array([tx, ty])
