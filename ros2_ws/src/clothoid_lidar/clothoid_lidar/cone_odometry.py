"""Cone-local dead reckoning: wheel speed at the rear axle, yaw from the IMU.

The chassis steering feedback clips before the real steering does. Its /odom
pose therefore cannot move retained cone geometry accurately through a turn.
Only /odom.twist.linear.x is used here. No global odometry or TF is replaced.
"""
import math


class ConeOdometry:
    def __init__(self, rear_offset=.1, timeout=.25):
        if not math.isfinite(rear_offset) or rear_offset < 0:
            raise ValueError('rear_offset must be finite and nonnegative')
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('pose timeout must be finite and positive')
        self.rear_offset, self.timeout = rear_offset, timeout
        self.bias = 0.
        self.generation = -1
        self.reset()

    def reset(self):
        """A new local frame cannot reuse geometry saved in the previous one."""
        self.rear_x, self.rear_y, self.yaw = -self.rear_offset, 0., 0.
        self.stamp = self.speed_stamp = self.imu_stamp = None
        self.speed = self.gyro = 0.
        self.quiet_since = None
        self.generation += 1

    def _receive(self, kind, stamp, value):
        if not all(math.isfinite(v) for v in (stamp, value)) or stamp < 0:
            self.reset()
            return False
        previous = self.speed_stamp if kind == 'speed' else self.imu_stamp
        if previous is not None and stamp <= previous:
            if previous-stamp > self.timeout:
                self.reset()  # ROS clock restarted; wait for both sensors in the new epoch.
            else:
                return False  # duplicate or late sample: never integrate an interval twice.
        if self.stamp is not None and stamp > self.stamp:
            have_both = self.speed_stamp is not None and self.imu_stamp is not None
            if have_both and (stamp-self.speed_stamp > self.timeout
                              or stamp-self.imu_stamp > self.timeout):
                self.reset()  # Unknown motion during a sensor gap cannot preserve an old path.
            elif have_both:
                dt = stamp-self.stamp
                # Interpolate the gyro on an IMU update; hold speed until its next sample.
                omega = ((self.gyro+value)/2 if kind == 'imu' else self.gyro)-self.bias
                angle = omega*dt
                distance = self.speed*dt
                # Exact constant-twist arc at the rear axle, including reverse motion.
                scale = math.sin(angle/2)/(angle/2) if abs(angle) > 1e-9 else 1.
                self.rear_x += distance*scale*math.cos(self.yaw+angle/2)
                self.rear_y += distance*scale*math.sin(self.yaw+angle/2)
                self.yaw += angle
        self.stamp = stamp if self.stamp is None else max(self.stamp, stamp)
        if kind == 'speed':
            self.speed, self.speed_stamp = value, stamp
        else:
            self.gyro, self.imu_stamp = value, stamp
            # Learn bias only with fresh stationary wheels AND a quiet gyro.
            # Carrying/turning a stopped car must not train its rotation as bias.
            quiet = (self.speed_stamp is not None and -.02 <= stamp-self.speed_stamp <= self.timeout
                     and abs(self.speed) < .005 and abs(value) < .03)
            if quiet:
                if self.quiet_since is None:
                    self.quiet_since = stamp
                if stamp-self.quiet_since >= .5:
                    self.bias += .02*(value-self.bias)
            else:
                self.quiet_since = None
        return True

    def odom(self, stamp, speed):
        return self._receive('speed', stamp, speed)

    def imu(self, stamp, gyro_z):
        return self._receive('imu', stamp, gyro_z)

    def pose(self, now):
        if not math.isfinite(now) or any(t is None or not -.02 <= now-t <= self.timeout
                                       for t in (self.stamp, self.speed_stamp, self.imu_stamp)):
            return None
        return (self.rear_x+self.rear_offset*math.cos(self.yaw),
                self.rear_y+self.rear_offset*math.sin(self.yaw), self.yaw)
