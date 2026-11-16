"""Bounded continuation of a near-straight lane while a bump hides its markings."""
import math
from dataclasses import dataclass


@dataclass
class BumpGapParams:
    speed: float = 0.2
    max_distance: float = 0.8
    max_duration: float = 4.0
    max_steer_deg: float = 5.0
    max_yaw_error_deg: float = 8.0
    max_lateral_error: float = 0.1
    yaw_gain: float = 1.5


class BumpGap:
    def __init__(self):
        self.reference = None
        self.active = False
        self.recovery_target = self.recovery_stamp = None
        self.recovery_hits = 0

    def clear(self):
        self.reference = None
        self.active = False
        self.recovery_target = self.recovery_stamp = None
        self.recovery_hits = 0

    def remember(self, now, pose, imu_yaw, travel):
        self.clear()
        self.reference = (now, pose, imu_yaw, travel)

    def reacquired(self, stamp, target, usable, tolerance):
        """Do not steer toward a single fragment or a flickering recovered boundary."""
        if not usable:
            self.recovery_hits = 0
            self.recovery_target = self.recovery_stamp = None
            return False
        if stamp != self.recovery_stamp:
            if (self.recovery_stamp is None or not 0 < stamp-self.recovery_stamp <= .15 or
                    math.dist(target, self.recovery_target) > tolerance):
                self.recovery_hits = 0
            self.recovery_target, self.recovery_stamp = target, stamp
            self.recovery_hits += 1
        return self.recovery_hits >= 3

    def steering(self, now, pose, imu_yaw, travel, params):
        if self.reference is None:
            return None
        stamp, origin, yaw, start_travel = self.reference
        distance = travel - start_travel
        # A discontinuous camera stream must not initiate a blind continuation.
        if ((not self.active and not 0 <= now-stamp <= 0.15) or
                not 0 <= now-stamp < params.max_duration or
                not 0 <= distance < params.max_distance):
            self.clear()
            return None
        # The last steering correction may be recentring on a straight road. Holding
        # that wheel angle would keep turning through the missing line. Hold yaw instead.
        yaw_error = math.remainder(imu_yaw - yaw, 2 * math.pi)
        dx, dy = pose[0] - origin[0], pose[1] - origin[1]
        lateral = -math.sin(origin[2])*dx + math.cos(origin[2])*dy
        if (abs(yaw_error) > math.radians(params.max_yaw_error_deg) or
                abs(lateral) > params.max_lateral_error):
            self.clear()
            return None
        self.active = True
        limit = math.radians(params.max_steer_deg)
        return max(-limit, min(limit, -params.yaw_gain*yaw_error))
