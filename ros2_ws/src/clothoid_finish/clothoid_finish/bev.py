"""Bird's-eye view of the floor in metres.

The warp is clothoid_lane's (camera_limo1/2.yaml warp_*), so the finish node sees the same picture
as the lane node. Every warped pixel is mapped back to the camera image and from there onto the
floor with a pinhole model (camera_info intrinsics, camera_height, camera_pitch + down as in
clothoid_debug), so distances do not depend on the warp being exactly metric.
"""
import math
from dataclasses import dataclass

import cv2
import numpy as np

from clothoid_lane.lane_detector import LaneParams, make_warp_matrix


@dataclass
class CameraModel:
    fx: float = 489.5            # limo2 /camera/color/camera_info, 640 x 480
    fy: float = 489.5
    cx: float = 319.9
    cy: float = 214.6
    height: float = 0.19         # lens above the floor (camera_limo2.yaml)
    pitch: float = -0.021        # rad, + = looking down (camera_limo2.yaml camera_pitch)
    x: float = 0.15              # lens ahead of base_link
    y: float = -0.01             # lens left of base_link


class Bev:
    def __init__(self, warp: LaneParams, cam: CameraModel):
        self.warp = warp
        self.cam = cam
        self.matrix = make_warp_matrix(warp)
        w, h = warp.warp_w, warp.warp_h
        bx, by = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
        pts = np.stack([bx.ravel(), by.ravel(), np.ones(bx.size)])
        img = np.linalg.inv(self.matrix) @ pts
        u, v = img[0] / img[2], img[1] / img[2]
        # pinhole onto the floor: angle below the horizon of each image row
        below = np.arctan((v - cam.cy) / cam.fy) + cam.pitch
        ok = below > math.radians(0.5)
        fwd = np.where(ok, cam.height / np.tan(np.where(ok, below, 1.0)), np.inf)
        # depth along the optical axis of that floor point (axis tilted down by pitch), then sideways
        z_c = fwd * math.cos(cam.pitch) + cam.height * math.sin(cam.pitch)
        left = -(u - cam.cx) / cam.fx * z_c
        # metres in base_link (x ahead, y left) of every warped pixel; inf where above the horizon
        self.gx = (fwd + cam.x).reshape(h, w)
        self.gy = (left + cam.y).reshape(h, w)
        self.valid = ok.reshape(h, w)

    def warp_image(self, bgr: np.ndarray) -> np.ndarray:
        return cv2.warpPerspective(cv2.GaussianBlur(bgr, (5, 5), 0), self.matrix,
                                   (self.warp.warp_w, self.warp.warp_h), flags=cv2.INTER_LINEAR)

    def near_far(self):
        g = self.gx[self.valid]
        return float(g.min()), float(g.max())
