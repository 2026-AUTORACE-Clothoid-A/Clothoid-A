"""Cluster-by-cluster view of `extract_obstacles` for tuning (ROS-free).

`classify_clusters` runs the same clustering as `obstacle_extractor.extract_obstacles` but keeps
every cluster with the reason it was kept or dropped, so rviz can show why a cone was missed
(merged with a neighbour -> too wide) or why a wall fragment became a "cone" (2 beams).
The accepted centres are identical to `extract_obstacles` (checked in test_lidar_debug.py).
"""
from dataclasses import dataclass
from typing import List

import numpy as np

from clothoid_lidar.obstacle_extractor import ExtractorParams

OK = 'ok'          # published as an obstacle centre
WIDE = 'wide'      # width > max_width
FEW = 'few'        # beams < min_points
NARROW = 'narrow'  # width < min_width (LIMO search drops cones)


@dataclass
class Cluster:
    points: np.ndarray   # (k, 2) scan-frame beam end points
    kind: str            # OK, WIDE, FEW or NARROW
    width: float         # m, first-to-last beam distance
    centre: np.ndarray   # (2,) published centre for OK, surface centroid otherwise


def classify_clusters(ranges, angle_min: float, angle_increment: float,
                      params: ExtractorParams) -> List[Cluster]:
    ranges = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(ranges.size)
    lo, hi = np.deg2rad(params.angle_min_deg), np.deg2rad(params.angle_max_deg)
    valid = (np.isfinite(ranges) & (ranges >= params.range_min) & (ranges <= params.range_max)
             & (angles >= lo) & (angles <= hi))
    index = np.flatnonzero(valid)
    if index.size == 0:
        return []
    points = np.column_stack((ranges[index] * np.cos(angles[index]),
                              ranges[index] * np.sin(angles[index])))

    out = []
    start = 0
    for i in range(1, index.size + 1):
        ends = (i == index.size
                or index[i] != index[i - 1] + 1
                or np.linalg.norm(points[i] - points[i - 1]) > params.cluster_gap)
        if not ends:
            continue
        cluster = points[start:i]
        start = i
        width = float(np.linalg.norm(cluster[-1] - cluster[0]))
        centroid = cluster.mean(axis=0)
        if len(cluster) < params.min_points:
            out.append(Cluster(cluster, FEW, width, centroid))
        elif width > params.max_width:
            out.append(Cluster(cluster, WIDE, width, centroid))
        elif width < params.min_width:
            out.append(Cluster(cluster, NARROW, width, centroid))
        else:
            norm = np.linalg.norm(centroid)
            centre = centroid + centroid / norm * params.cone_radius if norm > 1e-6 else centroid
            out.append(Cluster(cluster, OK, width, centre))
    return out


def accepted_centres(clusters: List[Cluster]) -> np.ndarray:
    c = [k.centre for k in clusters if k.kind == OK]
    return np.array(c) if c else np.zeros((0, 2))


def summary(clusters: List[Cluster]) -> dict:
    return {kind: sum(1 for k in clusters if k.kind == kind) for kind in (OK, WIDE, FEW, NARROW)}
