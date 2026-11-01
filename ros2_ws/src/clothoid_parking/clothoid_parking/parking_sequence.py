"""Timed parking manoeuvre, ported from 2025 `parking_rect.py` (`parking_timer_callback`).

ROS-free. The 2025 car parked open loop: a fixed list of (speed, steering, duration) steps,
except one step that keeps turning until the lidar's closest point reaches a given angle /
distance. The step table now comes from yaml so the 2026 T-parking only needs new rows.
Steering is in rad, positive = left; the node converts it to a Twist.
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class Step:
    duration: float   # s (ignored for the sensor step, which lasts until its condition ends)
    speed: float      # m/s, negative = reverse
    steer: float      # rad


@dataclass
class SequenceParams:
    steps: List[Step]
    sensor_step: Optional[int] = None    # index of the step that waits for the lidar
    angle_limit: float = -2.0            # rad, keep going while closest angle is below this ...
    range_limit: float = 0.34            # m, ... or the closest point is nearer than this
    sensor_step_max_sec: float = 3.0     # NEW vs 2025: give up waiting (dead lidar -> endless turn)


def step_to_twist_values(step: Step, wheelbase: float) -> Tuple[float, float]:
    """(linear.x, angular.z) for a step: bicycle model w = v * tan(delta) / wheelbase.

    The sign of w while reversing still has to be confirmed on the LIMO driver.
    """
    return step.speed, step.speed * math.tan(step.steer) / wheelbase


def steps_from_lists(duration, speed, steer) -> List[Step]:
    if not (len(duration) == len(speed) == len(steer)):
        raise ValueError('step_duration, step_speed and step_steer must have the same length')
    return [Step(float(d), float(v), float(s)) for d, v, s in zip(duration, speed, steer)]


class ParkingSequence:
    def __init__(self, params: SequenceParams):
        if params.sensor_step is not None and not 0 <= params.sensor_step < len(params.steps):
            raise ValueError('sensor_step is outside the step table')
        self.p = params
        self.reset()

    def reset(self) -> None:
        self.start_time: Optional[float] = None
        self.sensor_done = False
        self.sensor_duration = 0.0
        self.done = False

    @property
    def started(self) -> bool:
        return self.start_time is not None

    def start(self, now: float) -> None:
        self.reset()
        self.start_time = now

    def update(self, now: float, closest_angle: Optional[float],
               closest_range: Optional[float]) -> Optional[Step]:
        """The step to drive right now, or None before start / after the last step.

        `closest_*` is the lidar's nearest point in the scan frame (None = no scan yet; the
        sensor step then keeps turning, as in 2025, until `sensor_step_max_sec`).
        """
        if not self.started or self.done:
            return None
        p = self.p
        dt = now - self.start_time
        end = 0.0  # time at which the steps handled so far end

        for index, step in enumerate(p.steps):
            if index == p.sensor_step:
                if not self.sensor_done:
                    waiting = (closest_angle is None or closest_range is None
                               or closest_angle < p.angle_limit or closest_range < p.range_limit)
                    if waiting and dt - end < p.sensor_step_max_sec:
                        return step
                    self.sensor_done = True
                    self.sensor_duration = dt - end
                end += self.sensor_duration
                continue
            end += step.duration
            if dt < end:
                return step

        self.done = True
        return None
