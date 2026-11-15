"""Connection invariants across exit directions, frames, rates and sensor failures."""
import math

import numpy as np
import pytest

from clothoid_lidar.cone_lane_connection import (ConnectionParams, LaneConnection,
                                               central_limit, forward_arc)
from clothoid_lidar.cone_exit import ExitPathMemory
from clothoid_lidar.cone_follower import FollowerParams, build_center_line, compute_steering


def simulate(initial, desired, hz=20, yaw=0., origin=(0., 0.), feedback=True):
    c = LaneConnection()
    pose = np.array([*origin, yaw], float)
    c.start(0., pose, initial, .2)
    last, rows = initial, []
    for i in range(1, int(5.9 * hz)):
        t, dt = i / hz, 1 / hz
        velocity = .2 if feedback else 0.
        omega = velocity * math.tan(last) / .2
        pose[:2] += velocity * dt * np.array([math.cos(pose[2]), math.sin(pose[2])])
        pose[2] += omega * dt
        v, angle, ready = c.step(t, pose, .3, .3 * math.tan(desired) / .2,
                                .2, .25, velocity, omega, True, .25)
        rows.append((t, v, angle, ready))
        if ready or c.failed:
            break
        last = angle
    return c, rows


@pytest.mark.parametrize('initial,desired', [(-20., 18.), (20., -18.),
    (0., 20.), (0., -20.), (20., 0.), (-20., 0.), (0., 0.), (15., 18.), (-15., -18.)])
@pytest.mark.parametrize('hz', [10, 20, 30])
def test_any_turn_direction_connects_without_a_steering_jump(initial, desired, hz):
    start, end = math.radians(initial), math.radians(desired)
    c, rows = simulate(start, end, hz)
    assert rows[-1][3], 'a curved lane must not need to become straight to accept control'
    assert rows[-1][2] == pytest.approx(end)
    assert all(0. < row[1] <= c.p.speed for row in rows)
    angles = [start] + [r[2] for r in rows]
    assert max(abs(b-a) for a,b in zip(angles, angles[1:])) <= math.radians(20.)/hz + 1e-12


def test_mirrored_exit_has_exactly_mirrored_response():
    _, left = simulate(math.radians(-17), math.radians(14))
    _, right = simulate(math.radians(17), math.radians(-14))
    assert np.array([r[:3] for r in left]) == pytest.approx(
        np.array([r[:3] for r in right]) * [1., 1., -1.])
    assert [r[3] for r in left] == [r[3] for r in right]


@pytest.mark.parametrize('yaw,origin', [(1.7, (3., -4.)), (-2.6, (-8., 2.)), (.3, (0., 0.))])
def test_map_translation_and_rotation_do_not_change_the_connection(yaw, origin):
    _, baseline = simulate(-.2, .2)
    _, transformed = simulate(-.2, .2, yaw=yaw, origin=origin)
    assert np.array([r[:3] for r in baseline]) == pytest.approx(
        np.array([r[:3] for r in transformed]))
    assert [r[3] for r in baseline] == [r[3] for r in transformed]


@pytest.mark.parametrize('radius', [.65, 1.2, 3.])
@pytest.mark.parametrize('spacing,width', [(.18, .5), (.3, .7), (.42, .9)])
@pytest.mark.parametrize('direction', [-1, 1])
def test_varied_cone_curves_and_spacings_share_the_same_exit_rule(radius, spacing, width, direction):
    p = FollowerParams(lfd=.35, corner_endpoint_enabled=True)
    theta = np.linspace(0., .9 / radius, max(4, round(.9 / spacing)))
    middle = np.column_stack((radius*np.sin(theta), direction*radius*(1.-np.cos(theta))))
    normals = np.column_stack((-direction*np.sin(theta), np.cos(theta)))
    path = build_center_line(middle+width/2*normals, middle-width/2*normals, p)
    memory = ExitPathMemory()
    assert memory.remember(path, (0., 0., 0.), p.lfd, p.centerline_ds)
    tangent = memory.end_direction
    end_heading = math.atan2(tangent[1], tangent[0])
    heading = end_heading + direction*.25
    rear_offset = p.rear_axle_x*np.array([math.cos(heading), math.sin(heading)])
    before = memory.observed_end - .01*tangent + rear_offset
    after = memory.observed_end + .01*tangent + rear_offset
    assert not memory.handover_ready((*before, heading), p.rear_axle_x)
    assert memory.handover_ready((*after, heading), p.rear_axle_x)
    initial = compute_steering(memory.remaining((*after, heading), p.rear_axle_x), p)
    initial = float(np.clip(initial, -central_limit(.2), central_limit(.2)))
    # The road may turn the same way or the other way after the cone corridor.
    for desired in [-.25, 0., .25]:
        c, rows = simulate(initial, desired, yaw=heading, origin=after)
        assert rows[-1][3] and not c.failed


def test_elapsed_time_with_no_vehicle_response_never_completes():
    c, rows = simulate(-.2, .2, feedback=False)
    assert not any(row[3] for row in rows)
    assert not c.failed


def test_unreachable_lane_curvature_cannot_be_confirmed_as_tracked():
    c, rows = simulate(0., math.radians(35.))
    assert not any(r[3] for r in rows)
    assert all(abs(r[2]) <= central_limit(.2) for r in rows)


def test_continuously_changing_lane_requests_cannot_release_early():
    c = LaneConnection();c.start(0., (0.,0.,0.), 0., .2)
    previous = 0.
    for i in range(1, 61):
        desired = .3 if (i//2)%2 else -.3
        _, angle, ready = c.step(i*.05,(i*.01,0.,0.),.3,.3*math.tan(desired)/.2,
                                .2,.25,.2,math.tan(previous),True,.25)
        assert not ready
        previous = angle


@pytest.mark.parametrize('kind', ['time', 'distance', 'rewind'])
def test_connection_bounds_stop_instead_of_releasing(kind):
    c = LaneConnection();c.start(10., (0., 0., 0.), 0., .2)
    t, pose = {'time': (17., (0., 0., 0.)), 'distance': (10.1, (1.1, 0., 0.)),
               'rewind': (9., (0., 0., 0.))}[kind]
    assert c.step(t, pose, .3, 0., .2, .25, .2, 0., True, .25) == (0., 0., False)
    assert c.failed
    assert c.step(17.1, pose, .3, 0., .2, .25, .2, 0., True, .25) == (0., 0., False)


def test_unstable_lane_feedback_and_sensor_gap_restart_confirmation():
    c = LaneConnection();c.start(0., (0., 0., 0.), 0., .2)
    def step(t, ready=True, rate=0.):
        return c.step(t, (.2*t, 0., 0.), .3, 0., .2, .25, .2, rate, ready, .25)
    for t in [.05, .1, .15, .2]:
        assert not step(t)[2]
    assert not step(.25, ready=False)[2]
    assert not step(.3)[2]
    assert not step(.35, rate=.3)[2]
    assert not step(.4)[2]
    assert not step(.9)[2], 'missing observations cannot complete the confirmation'
    for t in [.95, 1., 1.05, 1.1, 1.15]:
        assert not step(t)[2]
    assert step(1.25)[2]


def test_pause_does_not_accumulate_a_larger_resumption_step():
    c = LaneConnection();c.start(0., (0., 0., 0.), .2, .2)
    c.pause(2.)
    v, angle, ready = c.step(2.05, (0., 0., 0.), .3, .4, .2, .25, 0., 0., True, .25)
    assert abs(angle) <= math.radians(1.) + 1e-12
    assert not ready
    assert c.started == 0.


@pytest.mark.parametrize('speed,omega', [(0., 0.), (-.1, 0.), (float('nan'), 0.), (.3, float('inf'))])
def test_invalid_and_stopped_lane_commands_cannot_finish(speed, omega):
    c = LaneConnection();c.start(0., (0., 0., 0.), .1, .2)
    assert c.step(.05, (0., 0., 0.), speed, omega, .2, .25, .2, 0., True, .25) == (0., 0., False)


def test_clearance_arcs_are_mirrored_and_respect_the_rear_axle():
    left = forward_arc(.3, .2, .1, .35, .05)
    right = forward_arc(-.3, .2, .1, .35, .05)
    assert left[0] == pytest.approx([-.1, 0.])
    assert left == pytest.approx(right * [1., -1.])
    assert forward_arc(0., .2, .1, .35, .05)[-1] == pytest.approx([.25, 0.])
    assert math.degrees(central_limit(.2)) == pytest.approx(23.33, abs=.01)


@pytest.mark.parametrize('name', list(ConnectionParams.__dataclass_fields__))
@pytest.mark.parametrize('value', [0., -1., float('nan'), float('inf')])
def test_vehicle_and_abort_limits_must_be_positive_finite(name, value):
    with pytest.raises(ValueError):
        ConnectionParams(**{name: value})
