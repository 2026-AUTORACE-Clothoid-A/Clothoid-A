"""Broken markings and contest tile colours through the real lane command callback."""
import math
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Header

from clothoid_lane.lane_main import LaneMainNode


@pytest.fixture(params=['limo1', 'limo2'])
def lane(request):
    config = Path(__file__).parents[1] / 'config'
    rclpy.init(args=['--ros-args', '--params-file', str(config/'lane.yaml'),
                    '--params-file', str(config/f'camera_{request.param}.yaml')])
    node = LaneMainNode()
    clock = SimpleNamespace(t=100.)
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=round(clock.t*1e9)))
    commands = []
    node.cmd_pub = SimpleNamespace(publish=commands.append)
    node.cone_ready_pub = SimpleNamespace(publish=lambda msg: None)
    yield node, clock, commands
    node.destroy_node()
    rclpy.shutdown()


def camera(node, kind='clean'):
    bev = np.full((240, 320, 3), 45, np.uint8)
    if kind != 'lost':
        lo, hi = (110, 170) if kind == 'fragment' else (0, 239)
        for x in ((89, 259) if kind == 'offset' else (75, 245)):
            cv2.line(bev, (x, lo), (x, hi), (0, 200, 255), 7)
    if kind == 'tiles':
        for y in range(80, 240):
            for x in range(71, 250):
                bev[y, x] = ((56, 166, 225) if ((y-80)//57+(x-71)//60+1) % 2 == 0
                             else (40, 40, 40))
    return cv2.warpPerspective(bev, np.linalg.inv(node.detector._matrix), (640, 480))


def frame(lane, kind='clean', pitch=0.):
    node, clock, _ = lane
    clock.t += .04  # camera/sensor fixture at 25 Hz; no sensor-rate change in production
    stamp = node.get_clock().now().to_msg()
    imu = Imu()
    imu.header.stamp = stamp
    imu.orientation.y = math.sin(math.radians(pitch)/2)
    imu.orientation.w = math.cos(math.radians(pitch)/2)
    node.on_imu(imu)
    odom = Odometry()
    odom.header.stamp = stamp
    odom.pose.pose.orientation.w = 1.
    odom.pose.pose.position.x = (clock.t-100)*.2
    node.on_odom(odom)
    node.on_speed_limit(Float32(data=.2))
    return node.process_bgr(camera(node, kind), Header(stamp=stamp))


def approach(lane):
    node, _, _ = lane
    node.on_bump_approach(Bool(data=True))
    for _ in range(4):
        assert frame(lane).bump_usable
    return node.last_steer


@pytest.mark.parametrize('kind', ['lost', 'fragment', 'tiles'])
def test_broken_or_wide_marking_keeps_clean_steering_then_recovers(lane, kind):
    node, _, commands = lane
    steer = approach(lane)
    for _ in range(8):
        result = frame(lane, kind)
        assert not result.bump_usable
        if kind != 'lost':
            assert result.found  # a fragment/tile fit must not masquerade as recovery
        assert node.bump_gap.active
        assert abs(node.last_steer) < math.radians(1.)
        assert commands[-1].linear.x == pytest.approx(.2)
    for _ in range(2):
        frame(lane)
        assert node.bump_gap.active
    frame(lane)
    assert not node.bump_gap.active
    assert abs(node.last_steer) < math.radians(1.)


def test_existing_pitch_lock_takes_over_the_gap(lane):
    node, _, commands = lane
    steer = approach(lane)
    frame(lane, 'fragment')
    node.on_steering_lock(Bool(data=True))
    node.on_bump_approach(Bool(data=False))
    frame(lane, 'tiles', pitch=-4.)
    assert abs(node.last_steer) < math.radians(1.)
    assert commands[-1].linear.x == pytest.approx(.2)


def test_unarmed_lane_selection_is_unchanged(lane):
    node, _, commands = lane
    for _ in range(4):
        frame(lane)
    for _ in range(4):
        frame(lane, 'tiles')
    assert not node.bump_gap.active
    assert commands[-1].linear.x == pytest.approx(.2)
    assert abs(node.last_steer) > math.radians(1.)


def test_recentering_angle_is_not_frozen_through_a_gap_or_pitch_lock(lane):
    node, _, commands = lane
    node.on_bump_approach(Bool(data=True))
    for _ in range(4):
        result = frame(lane, 'offset')
    assert result.bump_usable and abs(node.last_steer) > math.radians(1.)
    frame(lane, 'lost')
    assert node.bump_gap.active and node.last_steer == pytest.approx(0.)
    node.on_steering_lock(Bool(data=True))
    node.on_bump_approach(Bool(data=False))
    frame(lane, 'tiles', pitch=-4.)
    assert node.last_steer == pytest.approx(0.)
    assert commands[-1].linear.x == pytest.approx(.2)


def test_recovery_slews_from_held_heading_not_the_old_recentering_target(lane):
    node, _, _ = lane
    node.on_bump_approach(Bool(data=True))
    for _ in range(4):
        frame(lane, 'offset')
    old_steer = node.last_steer
    frame(lane, 'lost')
    assert node.used_target[0] == pytest.approx(160.)
    for _ in range(2):
        frame(lane, 'offset')
        assert node.bump_gap.active
    prev = node.used_target
    frame(lane, 'offset')
    assert not node.bump_gap.active
    assert math.dist(prev, node.used_target) <= 8.000001
    assert abs(node.last_steer) < abs(old_steer)
