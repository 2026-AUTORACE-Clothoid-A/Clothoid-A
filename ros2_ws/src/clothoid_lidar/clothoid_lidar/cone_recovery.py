"""Short reverse sweep checked against a fresh full laser scan, not cone labels."""
import math
import numpy as np


def current_boundaries(left_stamp, right_stamp, left_fresh, right_fresh):
    """Never combine different scans or wait for the old side to time out."""
    if left_stamp is not None and right_stamp is not None and left_stamp != right_stamp:
        left_fresh &= left_stamp > right_stamp
        right_fresh &= right_stamp > left_stamp
    return left_fresh, right_fresh


def recovery_confirmed(left_stamp, right_stamp, previous_scan, left_fresh, right_fresh):
    if left_fresh and right_fresh:
        return True
    if previous_scan is None:
        return False
    return ((not left_fresh and (left_stamp is None or left_stamp < previous_scan))
            or (not right_fresh and (right_stamp is None or right_stamp < previous_scan)))


def reverse_sweep_clear(ranges, angle_min, angle_increment, range_min, range_max,
                        laser_pose, steering, wheelbase, rear_axle_x, distance):
    """Check a rectangular body swept backwards for at most one lookahead.

    Beam visibility is required outside the current body as well as clearance
    from measured surfaces. A missing/partial rear scan is not free space.
    """
    ranges = np.asarray(ranges, float)
    if len(ranges) < 2 or angle_increment <= 0 or distance <= 0 or not np.isfinite(laser_pose).all():
        return False
    lx, ly, yaw = laser_pose
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    angles = angle_min+np.arange(len(ranges))*angle_increment
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    points = np.column_stack((ranges[valid]*np.cos(angles[valid]),
                              ranges[valid]*np.sin(angles[valid])))@rotation.T+(lx, ly)
    # Ignore returns inside the known initial body (self reflections).
    outside = (np.abs(points[:, 0]) > .161) | (np.abs(points[:, 1]) > .110)
    points = points[outside]
    k = math.tan(steering)/wheelbase
    corners = np.array([(x, y) for x in (-.181, 0., .181) for y in (-.130, 0., .130)])
    for travel in np.linspace(0., -distance, max(3, int(math.ceil(distance/.025))+1)):
        angle = travel*k
        ca, sa = math.cos(angle), math.sin(angle)
        R = np.array([[ca, -sa], [sa, ca]])
        rear = np.array([travel, 0.]) if abs(k) < 1e-9 else np.array([math.sin(angle)/k, (1-math.cos(angle))/k])
        center = rear+np.array([-rear_axle_x, 0.])+R@np.array([rear_axle_x, 0.])
        if len(points):
            local = (points-center)@R
            if np.any((np.abs(local[:, 0]) <= .181) & (np.abs(local[:, 1]) <= .130)):
                return False
        footprint = corners@R.T+center
        footprint = footprint[(np.abs(footprint[:, 0]) > .181) | (np.abs(footprint[:, 1]) > .130)]
        if not len(footprint):
            continue
        polar = (footprint-(lx, ly))@rotation
        bearings = np.arctan2(polar[:, 1], polar[:, 0])
        indices = np.rint((bearings-angle_min)/angle_increment).astype(int)
        # A full revolution may have its seam at zero rather than -pi.
        if (len(ranges)-1)*angle_increment >= 2*math.pi-2*angle_increment:
            indices %= len(ranges)
        elif np.any((indices < 0) | (indices >= len(ranges))):
            return False
        measured = ranges[indices]
        known = ((np.isfinite(measured) & (measured >= range_min)) | np.isposinf(measured))
        if not np.all(known):
            return False
        available = np.minimum(measured, range_max)
        if np.any(np.linalg.norm(polar, axis=1)+.01 > available):
            return False
    return True
