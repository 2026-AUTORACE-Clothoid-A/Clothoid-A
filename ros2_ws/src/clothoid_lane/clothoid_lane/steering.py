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


def speed_scaled_gain(gain: float, base_speed: float, speed: float, max_gain: float) -> float:
    """steer_gain for a slower speed: speed x gain kept about constant (lane.yaml tuning: 0.8 x 0.33,
    0.3 x 1.0), at most `max_gain`. At or above base_speed, or with nonsense input, `gain` as it is;
    never below `gain`."""
    if speed <= 0.0 or base_speed <= 0.0 or speed >= base_speed:
        return gain
    return max(gain, min(max_gain, gain * base_speed / speed))


def angular_z(speed: float, steer: float, wheelbase: float = WHEELBASE) -> float:
    """Bicycle model: w = v * tan(delta) / wheelbase."""
    return speed * math.tan(steer) / wheelbase


# after a loss longer than this the target starts again from the new detection (twist_mux lane timeout)
TARGET_RESET_SEC = 0.3


def limit_step(prev, new, max_step: float):
    """Move a target point from `prev` toward `new` by at most `max_step` (same units, warped px).

    A one- or two-frame jump of the target (a boundary that flickers in and out, a bad fit) then moves
    the target only a little, while a real change is followed within a few frames. Returns `new` when
    there is no previous point or max_step <= 0 (off). Only the detected target is moved: no line is
    invented, and a lost lane still publishes nothing.
    """
    if prev is None or max_step <= 0.0:
        return new
    dx, dy = new[0] - prev[0], new[1] - prev[1]
    dist = math.hypot(dx, dy)
    if dist <= max_step:
        return new
    k = max_step / dist
    return prev[0] + dx * k, prev[1] + dy * k
