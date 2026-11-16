"""IMU/wheel odometry must use the same physical axle as cone pursuit."""
import math

import numpy as np
import pytest

from clothoid_lidar.cone_odometry import ConeOdometry


def drive(speed, gyro, seconds=2., offset=.1):
    tracker = ConeOdometry(offset)
    tracker.odom(0., speed)
    tracker.imu(0., gyro)
    for i in range(1, round(seconds*100)+1):
        t = i*.01
        if i % 2 == 0:
            tracker.odom(t, speed)
        tracker.imu(t, gyro)
    return tracker


@pytest.mark.parametrize('speed,gyro', [(.5, math.pi/4), (.5, -math.pi/4),
                                      (-.5, math.pi/4), (-.5, -math.pi/4)])
def test_quarter_circle_includes_base_link_offset_from_rear_axle(speed, gyro):
    tracker = drive(speed, gyro)
    angle, radius, offset = 2*gyro, speed/gyro, tracker.rear_offset
    expected = (radius*math.sin(angle)+offset*(math.cos(angle)-1),
                radius*(1-math.cos(angle))+offset*math.sin(angle), angle)
    np.testing.assert_allclose(tracker.pose(2.), expected, atol=1e-12)


@pytest.mark.parametrize('speed', [.25, -.25, 0.])
def test_straight_motion_and_reverse_use_wheel_speed(speed):
    np.testing.assert_allclose(drive(speed, 0.).pose(2.), (2*speed, 0., 0.), atol=1e-12)


@pytest.mark.parametrize('speed,gyro', [(.5, math.pi/4), (.5, -math.pi/4),
                                      (-.5, math.pi/4), (-.5, -math.pi/4)])
def test_25hz_sensor_pairs_preserve_axle_geometry_and_keep_exit_pose_fresh(speed, gyro):
    tracker = ConeOdometry()
    generation = tracker.generation
    for i in range(51):
        t = i*.04
        tracker.odom(t, speed)
        tracker.imu(t, gyro)
        assert tracker.pose(t+.02) is not None
        assert tracker.generation == generation
    angle, radius, offset = 2*gyro, speed/gyro, tracker.rear_offset
    expected = (radius*math.sin(angle)+offset*(math.cos(angle)-1),
                radius*(1-math.cos(angle))+offset*math.sin(angle), angle)
    np.testing.assert_allclose(tracker.pose(2.), expected, atol=1e-12)


@pytest.mark.parametrize('missing', ['imu', 'odom'])
def test_one_sensor_cannot_produce_a_pose(missing):
    tracker = ConeOdometry()
    for i in range(101):
        if missing == 'imu':
            tracker.odom(i*.01, .25)
        else:
            tracker.imu(i*.01, .5)
    assert tracker.pose(1.) is None


@pytest.mark.parametrize('missing', ['imu', 'odom'])
def test_sensor_gap_starts_new_frame_without_integrating_unknown_motion(missing):
    tracker = drive(.25, .5)
    generation = tracker.generation
    for i in range(201, 251):
        if missing == 'imu':
            tracker.odom(i*.01, .25)
        else:
            tracker.imu(i*.01, .5)
    assert tracker.pose(2.5) is None
    assert tracker.generation > generation
    tracker.odom(2.51, 0.)
    tracker.imu(2.51, 0.)
    np.testing.assert_allclose(tracker.pose(2.51), (0., 0., 0.))


def test_duplicate_and_late_packets_do_not_move_pose_twice():
    tracker = drive(.25, .5)
    before, generation = tracker.pose(2.), tracker.generation
    for stamp in (2., 1.99, 1.98):
        assert not tracker.odom(stamp, 10.)
        assert not tracker.imu(stamp, 10.)
    np.testing.assert_array_equal(tracker.pose(2.), before)
    assert tracker.generation == generation


def test_cross_stream_delay_cannot_rewind_integration_time():
    tracker = ConeOdometry()
    tracker.odom(0., .25)
    tracker.imu(0., .5)
    tracker.imu(.02, .5)
    before = tracker.pose(.02)
    tracker.odom(.01, .3)  # freshest wheel sample arrived after a newer IMU sample
    assert tracker.stamp == .02
    np.testing.assert_array_equal(tracker.pose(.02), before)


def test_clock_reset_requires_both_new_sensors_and_discards_old_frame():
    tracker = drive(.25, .5)
    generation = tracker.generation
    tracker.imu(.1, .5)
    assert tracker.generation > generation and tracker.pose(.1) is None
    tracker.odom(.1, 0.)
    np.testing.assert_allclose(tracker.pose(.1), (0., 0., 0.))


@pytest.mark.parametrize('kind,value', [('imu', float('nan')), ('imu', float('inf')),
                                      ('odom', float('nan'))])
def test_invalid_sensor_sample_invalidates_pose(kind, value):
    tracker = drive(.25, .5)
    getattr(tracker, kind)(2.01, value)
    assert tracker.pose(2.01) is None


def test_old_sensor_timestamps_are_not_freshened_by_querying_pose():
    tracker = drive(.25, .5)
    assert tracker.pose(2.3) is None
    assert tracker.pose(1.5) is None


def test_bias_is_learned_only_during_quiet_stationary_measurements():
    tracker = drive(0., .01, seconds=3.)
    assert .009 < tracker.bias < .011
    yaw = tracker.pose(3.)[2]
    for i in range(301, 501):
        t = i*.01
        tracker.odom(t, .25)
        tracker.imu(t, .51)
    # The moving gyro sample starts at 3.01; the first 10 ms still held the quiet gyro.
    assert tracker.pose(5.)[2]-yaw == pytest.approx(.995, abs=.003)
    # Turning the car by hand with stationary wheels is not a gyro bias sample.
    hand_turn = drive(0., .5)
    assert hand_turn.bias == 0.
    assert hand_turn.pose(2.)[2] == pytest.approx(1.)


def test_stationary_bias_can_learn_with_slightly_delayed_imu_callbacks():
    tracker = ConeOdometry()
    for i in range(301):
        tracker.odom(i*.01+.005, 0.)
        tracker.imu(i*.01, .01)
    assert .009 < tracker.bias < .011


@pytest.mark.parametrize('kwargs', [dict(rear_offset=-.1), dict(rear_offset=float('nan')),
                                   dict(timeout=0.), dict(timeout=float('inf'))])
def test_invalid_geometry_or_timeout_is_rejected(kwargs):
    with pytest.raises(ValueError):
        ConeOdometry(**kwargs)
