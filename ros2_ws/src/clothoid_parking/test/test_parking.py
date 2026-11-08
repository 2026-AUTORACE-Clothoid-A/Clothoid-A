"""Offline tests: bay finder, and the whole manoeuvre closed loop in the 2026 grid.

    pytest ros2_ws/src/clothoid_parking/test
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))

import scene  # noqa: E402
from clothoid_parking.bay_finder import BayParams, find_bay  # noqa: E402
from clothoid_parking.core import ParkingCore  # noqa: E402
from clothoid_parking.planner import Phase, PlanParams  # noqa: E402


def world_points(cars, x, y, yaw, sign=True):
    ranges, a0, inc = scene.scan(scene.segments(cars, sign), x, y, yaw)
    a = yaw + a0 + np.arange(len(ranges)) * inc
    ok = np.isfinite(ranges)
    lx, ly = x + scene.LASER_X * math.cos(yaw), y + scene.LASER_X * math.sin(yaw)
    return np.stack([lx + ranges[ok] * np.cos(a[ok]), ly + ranges[ok] * np.sin(a[ok])], 1)


@pytest.mark.parametrize('empty', [0, 1, 2])
def test_bay_finder_sees_the_gap_from_the_lane(empty):
    # the rule puts a LIMO on both sides of the empty bay; with the empty bay at the end of the
    # row, a car one bay further on stands in for that neighbour. The simulator's sign pole stands
    # in the first bay's mouth, so it is left out when that bay is the empty one.
    cars = scene.parked_cars(empty)
    if empty == 2:
        cars.append((scene.COLUMNS[2] + 0.52, scene.UPPER_Y, math.pi / 2))
    if empty == 0:
        cars.append((scene.COLUMNS[0] - 0.52, scene.UPPER_Y, math.pi / 2))
    x0 = scene.COLUMNS[0] - 1.0 - (0.52 if empty == 0 else 0)
    pts = np.vstack([world_points(cars, x, scene.LANE_Y, 0.0, sign=empty != 0)
                     for x in np.arange(x0, x0 + 0.6, 0.1)])
    bay = find_bay(pts, x0 + 0.5, scene.LANE_Y, 0.0)
    assert bay is not None
    assert bay.x == pytest.approx(scene.COLUMNS[empty], abs=0.02)
    assert bay.y == pytest.approx(scene.UPPER_Y, abs=0.02)
    assert bay.entry_yaw == pytest.approx(-math.pi / 2, abs=math.radians(1))


def test_bay_finder_without_a_gap_finds_nothing():
    cars = [(cx, scene.UPPER_Y, math.pi / 2) for cx in scene.COLUMNS]
    pts = world_points(cars, 5.5, scene.LANE_Y, 0.0)
    assert find_bay(pts, 5.5, scene.LANE_Y, 0.0) is None


# The scene's car is an ideal bicycle (no slip, rear axle 0.1 behind base_link, R 0.46), not limo2's
# fitted values in the PlanParams defaults; its odom reads long (odom_scale above) on purpose.
SCENE_PLAN = dict(rear_offset=0.1, odom_scale=1.0, turn_radius=0.46)


def drive(empty=1, lane_offset=0.0, heading_deg=0.0, start_x=5.5, gyro_bias=0.003,
          odom_scale=1.015, slew=1.5, seed=0, plan=None, extra_cars=(), sign=True,
          spikes=0.0, dropout=0.0, until=(Phase.PARKED, Phase.ABORT), seconds=25.0):
    rng = np.random.default_rng(seed)
    cars = (scene.parked_cars(empty) if empty is not None else []) + list(extra_cars)
    segs = scene.segments(cars, sign)
    car = scene.Car(start_x, scene.LANE_Y + lane_offset, math.radians(heading_deg), slew=slew)
    core = ParkingCore(plan or PlanParams(**SCENE_PLAN))
    dt, t = 0.005, 0.0
    v_cmd, steer_cmd = 0.0, 0.0
    # 1 s still (gyro bias learned), then the flag
    for i in range(int(seconds / dt)):
        t = i * dt
        if i % 2 == 0:   # imu 100 Hz
            rate = car.v * math.tan(car.steer) / scene.WHEELBASE
            core.imu(t, rate + gyro_bias + rng.normal(0, 0.002))
        if i % 4 == 0:   # odom 50 Hz
            core.odom(t, car.v * odom_scale)
        if i % 20 == 0:  # scan 10 Hz
            ranges, a0, inc = scene.scan(segs, car.x, car.y, car.yaw, noise=0.005, rng=rng,
                                         spikes=spikes, dropout=dropout)
            core.scan(t, ranges, a0, inc)
        if t >= 1.0:
            core.set_flag(t, True)
        if i % 10 == 0:  # node timer 20 Hz
            out = core.command(t)
            if out is not None:
                v_cmd, steer_cmd = out
            elif core.phase == Phase.DONE:
                v_cmd, steer_cmd = 0.0, 0.0     # node silent -> twist_mux timeout -> limo_base stops
        car.step(dt, v_cmd, steer_cmd)
        assert not scene.collides(car, cars), f'hit a parked car at t={t:.2f}, phase {core.phase}'
        if core.phase in until and abs(car.v) < 1e-3:
            break
    return core, car


def scenario(empty):
    """Both neighbours parked (rulebook): for an end bay a car one bay further stands in."""
    extra = []
    if empty == 2:
        extra = [(scene.COLUMNS[2] + 0.52, scene.UPPER_Y, math.pi / 2)]
    if empty == 0:
        extra = [(scene.COLUMNS[0] - 0.52, scene.UPPER_Y, math.pi / 2)]
    return dict(empty=empty, start_x=5.5 if empty else 5.0, extra_cars=extra, sign=empty != 0)


@pytest.mark.parametrize('empty', [0, 1, 2])
def test_parks_in_the_empty_bay(empty):
    core, car = drive(**scenario(empty))
    assert core.phase == Phase.PARKED, core.planner.reason
    assert scene.corners_outside(car, scene.bay_rect(empty), line=0.02) == 0
    assert car.yaw == pytest.approx(-math.pi / 2, abs=math.radians(4))


@pytest.mark.parametrize('lane_offset,heading_deg', [
    (0.05, 0.0), (-0.05, 0.0), (0.0, 3.0), (0.0, -3.0), (0.04, -2.0),
    # tilted at the start (limo2 2026-11-09 missed the bay from ~+4 deg): the row is fitted first
    (0.0, 6.0), (0.0, -6.0), (0.0, 8.0), (0.0, -8.0)])
def test_parks_from_an_imperfect_lane_position(lane_offset, heading_deg):
    core, car = drive(lane_offset=lane_offset, heading_deg=heading_deg)
    assert core.phase == Phase.PARKED, core.planner.reason
    assert scene.corners_outside(car, scene.bay_rect(1), line=0.02) == 0


@pytest.mark.parametrize('seed', range(5))
def test_parks_with_real_lidar_spikes_and_dropouts(seed):
    # rates measured from still periods in limo2 bags (2026-11-08)
    core, car = drive(seed=seed, spikes=0.01, dropout=0.018)
    assert core.phase == Phase.PARKED, core.planner.reason
    assert scene.corners_outside(car, scene.bay_rect(1), line=0.02) == 0
    assert car.x == pytest.approx(scene.COLUMNS[1], abs=0.04)


def test_slower_steering_still_parks():
    core, car = drive(slew=1.0)
    assert core.phase == Phase.PARKED, core.planner.reason
    assert scene.corners_outside(car, scene.bay_rect(1), line=0.02) == 0


def test_no_bay_aborts_and_stops():
    cars_full = [(scene.COLUMNS[1], scene.UPPER_Y, math.pi / 2)]
    core, car = drive(extra_cars=cars_full)
    assert core.phase == Phase.ABORT
    assert abs(car.v) < 1e-3


def test_parked_holds_zero_then_backs_out():
    core, car = drive()
    t = core.planner._t_phase
    assert core.command(t + 1.0) == (0.0, 0.0)
    assert core.command(t + 2.5) == (0.0, 0.0)
    v, steer = core.command(t + 3.1)     # park_hold_sec 3 s
    assert core.phase == Phase.EXIT_BACK and v < 0


@pytest.mark.parametrize('empty', [0, 1, 2])
def test_backs_out_into_the_lane(empty):
    # parked, then straight back and the arc in reverse; the drive loop checks every step for a
    # collision with the parked cars
    core, car = drive(**scenario(empty), until=(Phase.DONE, Phase.ABORT), seconds=45.0)
    assert core.phase == Phase.DONE, core.planner.reason
    assert core.command(1e6) is None                      # silent: lane takes over through twist_mux
    assert abs(math.degrees(car.yaw)) < 10.0               # facing along the lane again
    assert car.y == pytest.approx(scene.LANE_Y, abs=0.08)  # back in the lane (550 mm wide)
    assert car.x < scene.COLUMNS[empty]                    # still before the bay: lane drives on from here


def test_exit_can_be_switched_off():
    core, car = drive(plan=PlanParams(**dict(SCENE_PLAN, exit_enabled=False)),
                      until=(Phase.DONE, Phase.ABORT), seconds=35.0)
    assert core.phase == Phase.DONE
    assert scene.corners_outside(car, scene.bay_rect(1), line=0.02) == 0


def inner_scenario(upper_middle=False):
    """Lane-side row empty, second row with its middle bay empty (LIMOs on both sides)."""
    cars = [(scene.COLUMNS[0], scene.INNER_Y, math.pi / 2), (scene.COLUMNS[2], scene.INNER_Y, math.pi / 2)]
    if upper_middle:
        cars.append((scene.COLUMNS[1], scene.UPPER_Y, math.pi / 2))
    return cars


def test_parks_in_the_second_row_through_an_empty_lane_side_bay():
    core, car = drive(empty=None, extra_cars=inner_scenario())
    assert core.phase == Phase.PARKED, core.planner.reason
    assert scene.corners_outside(car, scene.bay_rect(1, scene.INNER_Y), line=0.02) == 0
    assert car.yaw == pytest.approx(-math.pi / 2, abs=math.radians(4))


def test_second_row_exit_backs_into_the_lane():
    core, car = drive(empty=None, extra_cars=inner_scenario(), until=(Phase.DONE, Phase.ABORT), seconds=50.0)
    assert core.phase == Phase.DONE, core.planner.reason
    assert abs(math.degrees(car.yaw)) < 10.0
    assert car.y == pytest.approx(scene.LANE_Y, abs=0.08)


def test_second_row_not_taken_when_the_bay_in_front_is_occupied():
    core, car = drive(empty=None, extra_cars=inner_scenario(upper_middle=True))
    assert core.phase == Phase.ABORT

