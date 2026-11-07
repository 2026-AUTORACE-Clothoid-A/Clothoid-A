"""Steering law of lane_main, shared with clothoid_debug's lane_viewer (ROS-free)."""
import math

MAX_STEER_RAD = 0.48869   # LIMO inner wheel limit, 28 deg (lane_main clamps the central angle with it)
WHEELBASE = 0.2           # m


def steer_angle(target_x: float, target_y: float, warp_w: int, warp_h: int,
                gain: float = 1.0, max_steer: float = MAX_STEER_RAD) -> float:
    """Steering angle in rad, left positive, from a target point in the warped image.

    The 2025 law: angle = -atan(x / y) / (pi / 2), x and y in warped pixels from the bottom centre
    (the camera axis), times `gain`, clamped to +-max_steer.
    """
    x = target_x - warp_w / 2
    y = warp_h - target_y
    steer = -math.atan2(x, max(y, 1e-6)) / (math.pi / 2) * gain
    return max(-max_steer, min(max_steer, steer))


def angular_z(speed: float, steer: float, wheelbase: float = WHEELBASE) -> float:
    """Bicycle model: w = v * tan(delta) / wheelbase."""
    return speed * math.tan(steer) / wheelbase
