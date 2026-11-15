"""Bounded continuation of a near-straight lane while a bump hides its markings."""
import math
from dataclasses import dataclass


@dataclass
class BumpGapParams:
    speed: float = 0.2
    max_distance: float = 0.6
    max_duration: float = 4.0
    max_steer_deg: float = 5.0
    max_yaw_error_deg: float = 8.0
    max_lateral_error: float = 0.1


class BumpGap:
    def __init__(self):
        self.reference = None
        self.active = False

    def clear(self):
        self.reference = None
        self.active = False

    def remember(self, now, steer, pose, imu_yaw, travel):
        self.reference = (now, steer, pose, imu_yaw, travel)
        self.active = False

    def steering(self, now, pose, imu_yaw, travel, wheelbase, params):
        if self.reference is None:
            return None
        stamp, steer, origin, yaw, start_travel = self.reference
        distance = travel - start_travel
        # A discontinuous camera stream must not initiate a blind continuation.
        if ((not self.active and not 0 <= now-stamp <= 0.15) or
                not 0 <= now-stamp < params.max_duration or
                not 0 <= distance < params.max_distance or
                abs(steer) > math.radians(params.max_steer_deg)):
            self.clear()
            return None
        curvature = math.tan(steer) / wheelbase
        expected_yaw = curvature * distance
        yaw_error = math.remainder(imu_yaw - yaw - expected_yaw, 2 * math.pi)
        dx, dy = pose[0] - origin[0], pose[1] - origin[1]
        lateral = -math.sin(origin[2])*dx + math.cos(origin[2])*dy
        expected_lateral = ((1 - math.cos(expected_yaw)) / curvature
                            if abs(curvature) > 1e-6 else 0.0)
        if (abs(yaw_error) > math.radians(params.max_yaw_error_deg) or
                abs(lateral - expected_lateral) > params.max_lateral_error):
            self.clear()
            return None
        self.active = True
        return steer
