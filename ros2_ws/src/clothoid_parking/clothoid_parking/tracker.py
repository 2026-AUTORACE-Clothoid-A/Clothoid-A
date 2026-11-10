"""Dead reckoning for the parking manoeuvre: yaw from the gyro, distance from /odom speed.

The 2026-11-08 floor sweep on limo2 found the gyro yaw within 1 % of lidar scan matching and the
/odom distance 1-2 % long, so this is enough for the few metres the manoeuvre covers. The pose is
base_link in a local frame that starts wherever the node started; only differences matter.

/odom speed is the rear-axle speed (limo_base, bicycle model), so the rear axle is what gets
integrated; base_link is `rear_offset` ahead of it. Integrating base_link with that speed instead
puts it 0.1 m off after a 90 deg turn.
"""
import math
from dataclasses import dataclass


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class Tracker:
    def __init__(self, still_speed: float = 0.005, still_sec: float = 0.5, bias_alpha: float = 0.02,
                 rear_offset: float = 0.1, odom_scale: float = 1.0):
        self.rear = Pose2D(-rear_offset, 0.0, 0.0)
        self.rear_offset = rear_offset
        self.odom_scale = odom_scale
        self.bias = 0.0
        self.speed = 0.0
        self.still_speed = still_speed
        self.still_sec = still_sec
        self.bias_alpha = bias_alpha
        self._t_imu = None
        self._t_odom = None
        self._still_since = None

    @property
    def pose(self) -> Pose2D:
        """base_link"""
        r = self.rear
        return Pose2D(r.x + self.rear_offset * math.cos(r.yaw),
                      r.y + self.rear_offset * math.sin(r.yaw), r.yaw)

    def still(self, now: float) -> bool:
        return self._still_since is not None and now - self._still_since >= self.still_sec

    def imu(self, now: float, gyro_z: float):
        if self.still(now):
            # gyro bias drifts slowly; learn it whenever the wheels have been still for a while
            self.bias += self.bias_alpha * (gyro_z - self.bias)
        elif self._t_imu is not None:
            dt = now - self._t_imu
            if 0.0 < dt < 0.2:
                self.rear.yaw += (gyro_z - self.bias) * dt
        self._t_imu = now

    def odom(self, now: float, speed: float):
        speed *= self.odom_scale
        if self._t_odom is not None:
            dt = now - self._t_odom
            if 0.0 < dt < 0.2:
                self.rear.x += speed * dt * math.cos(self.rear.yaw)
                self.rear.y += speed * dt * math.sin(self.rear.yaw)
        self._t_odom = now
        self.speed = speed
        if abs(speed) < self.still_speed:
            if self._still_since is None:
                self._still_since = now
        else:
            self._still_since = None
