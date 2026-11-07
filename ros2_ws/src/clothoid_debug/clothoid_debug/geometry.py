"""Pure geometry for the replay view (ROS-free)."""
import math
from typing import List, Tuple

import numpy as np


class GroundProjection:
    """Pixels of the warped lane image -> floor points in base_link (x forward, y left).

    Assumes a flat floor and ignores lens distortion, which is small on the DaBai DC1. `warp` is the
    image -> warped matrix from `clothoid_lane.lane_detector.make_warp_matrix`, `k` is (fx, fy, cx, cy).
    """

    def __init__(self, warp: np.ndarray, k: Tuple[float, float, float, float], height: float,
                 pitch: float, camera_x: float, camera_y: float):
        self.inverse = np.linalg.inv(warp)
        self.k, self.height, self.pitch = k, height, pitch
        self.camera_x, self.camera_y = camera_x, camera_y

    def warp_to_image(self, u: float, v: float) -> Tuple[float, float]:
        p = self.inverse @ np.array([u, v, 1.0])
        return float(p[0] / p[2]), float(p[1] / p[2])

    def image_to_floor(self, px: float, py: float) -> Tuple[float, float]:
        fx, fy, cx, cy = self.k
        x, y = (px - cx) / fx, (py - cy) / fy   # optical ray: x right, y down, z = 1 forward
        c, s = math.cos(self.pitch), math.sin(self.pitch)
        forward, down = c - y * s, y * c + s
        if down <= 1e-6:
            raise ValueError('pixel is above the horizon')
        scale = self.height / down
        return self.camera_x + scale * forward, self.camera_y - scale * x

    def warp_to_floor(self, u: float, v: float) -> Tuple[float, float]:
        return self.image_to_floor(*self.warp_to_image(u, v))


def arc_points(curvature: float, start_x: float, length: float = 1.0,
               step: float = 0.05) -> List[Tuple[float, float]]:
    """Floor arc of constant curvature (1/m, + left) from the rear axle at (start_x, 0)."""
    points = []
    for i in range(int(length / step) + 1):
        s = i * step
        if abs(curvature) < 1e-6:
            points.append((start_x + s, 0.0))
        else:
            points.append((start_x + math.sin(curvature * s) / curvature,
                           (1.0 - math.cos(curvature * s)) / curvature))
    return points
