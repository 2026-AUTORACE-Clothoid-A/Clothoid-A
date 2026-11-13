"""Exercise the real lane callback when markings disappear before bump contact."""
import math
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Header

from clothoid_lane.lane_main import LaneMainNode


@pytest.fixture
def lane():
    rclpy.init()
    node = LaneMainNode()
    node.set_parameters([Parameter('base_speed', value=.8)])
    clock = SimpleNamespace(t=100.)
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=round(clock.t*1e9)))
    result = SimpleNamespace(found=True, target_x=160., target_y=185.,
                             debug_image=None, zigzag=False)
    node.detector = SimpleNamespace(p=SimpleNamespace(warp_w=320, warp_h=240),
                                    detect=lambda *args: result)
    commands = []
    node.cmd_pub = SimpleNamespace(publish=commands.append)
    yield node, clock, result, commands
    node.destroy_node()
    rclpy.shutdown()


def sensors(node, x=0., y=0., yaw=0., pitch=0.):
    stamp = node.get_clock().now().to_msg()
    odom = Odometry()
    odom.header.stamp = stamp
    odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
    odom.pose.pose.orientation.w = 1.
    node.on_odom(odom)
    imu = Imu()
    imu.header.stamp = stamp
    pitch = math.radians(pitch)
    imu.orientation.y = math.sin(pitch/2)*math.cos(yaw/2)
    imu.orientation.x = -math.sin(pitch/2)*math.sin(yaw/2)
    imu.orientation.z = math.cos(pitch/2)*math.sin(yaw/2)
    imu.orientation.w = math.cos(pitch/2)*math.cos(yaw/2)
    node.on_imu(imu)


def frame(node):
    node.process_bgr(np.zeros((480, 640, 3), np.uint8),
                     Header(stamp=node.get_clock().now().to_msg()))


def approach(lane):
    node, clock, result, commands = lane
    sensors(node)
    node.on_bump_approach(Bool(data=True))
    node.on_speed_limit(Float32(data=.2))
    frame(node)
    result.found = False
    commands.clear()


def test_gap_continues_until_pitch_lock_then_returns_to_measured_lane(lane):
    node, clock, result, commands = lane
    approach(lane)
    for i in range(1, 21):
        clock.t += .1
        sensors(node, x=i*.02)
        node.on_bump_approach(Bool(data=True))
        node.on_speed_limit(Float32(data=.2))
        frame(node)
    assert len(commands) == 20
    assert all(cmd.linear.x == pytest.approx(.2) for cmd in commands)
    sensors(node, x=.4, pitch=-4.)
    node.on_steering_lock(Bool(data=True))
    node.on_bump_approach(Bool(data=False))
    frame(node)
    assert commands[-1].linear.x == pytest.approx(.2)
    node.on_steering_lock(Bool(data=False))
    result.found, result.target_x = True, 170.
    frame(node)
    assert commands[-1].angular.z < 0


@pytest.mark.parametrize('failure', ['distance', 'duration', 'yaw', 'lateral', 'heartbeat',
                                    'imu', 'odom', 'camera', 'parking', 'cancel'])
def test_exhaustion_or_lost_inputs_end_the_gap_without_rearming(lane, failure):
    node, clock, result, commands = lane
    approach(lane)
    clock.t += .1
    sensors(node, x=.01)
    frame(node)
    assert len(commands) == 1
    if failure in ('distance', 'duration', 'heartbeat'):
        for i in range(1, 46):
            clock.t += .1
            sensors(node, x=.01 + i*.02 if failure == 'distance' else .01)
            if failure != 'heartbeat':
                node.on_bump_approach(Bool(data=True))
            frame(node)
    else:
        clock.t += .3 if failure in ('imu', 'odom', 'camera') else .1
        sensors(node, x=.01, y=.11 if failure == 'lateral' else 0.,
                yaw=math.radians(9) if failure == 'yaw' else 0.)
        if failure == 'imu':
            node.imu_stamp = Time(nanoseconds=100_000_000_000)
        elif failure == 'odom':
            node.odom_stamp = Time(nanoseconds=100_000_000_000)
        elif failure == 'camera':
            node.process_bgr(np.zeros((480, 640, 3), np.uint8),
                             Header(stamp=Time(nanoseconds=100_000_000_000).to_msg()))
        elif failure == 'parking':
            node.on_parking(Bool(data=True))
        elif failure == 'cancel':
            node.on_bump_approach(Bool(data=False))
        if failure != 'camera':
            frame(node)
    before = len(commands)
    # Recovery of sensor heartbeats alone cannot restore the lost visual reference.
    clock.t += .1
    sensors(node, x=.01)
    node.on_bump_approach(Bool(data=True))
    frame(node)
    assert len(commands) == before
    assert node.bump_gap.reference is None


@pytest.mark.parametrize('scenario', ['unarmed', 'turning', 'tilted', 'disabled'])
def test_gap_requires_a_level_near_straight_lane_and_explicit_approach(lane, scenario):
    node, clock, result, commands = lane
    sensors(node, pitch=-3. if scenario == 'tilted' else 0.)
    node.on_bump_approach(Bool(data=scenario != 'unarmed'))
    if scenario == 'turning':
        result.target_x = 240.
    if scenario == 'disabled':
        node.set_parameters([Parameter('bump_gap_speed', value=0.)])
    frame(node)
    commands.clear()
    result.found = False
    clock.t += .03
    sensors(node, x=.01)
    frame(node)
    assert not commands


def test_a_short_camera_loss_keeps_the_normal_lane_command_on_reacquisition(lane):
    node, clock, result, commands = lane
    approach(lane)
    clock.t += .03
    sensors(node, x=.006)
    frame(node)
    assert node.bump_gap.active
    result.found, result.target_x = True, 165.
    frame(node)
    assert not node.bump_gap.active
    assert commands[-1].angular.z < 0


def test_odometry_reset_discards_the_reference(lane):
    node, clock, result, commands = lane
    approach(lane)
    clock.t += .03
    sensors(node, x=10.)
    frame(node)
    assert not commands and node.bump_gap.reference is None


def test_relative_imu_heading_crosses_the_angle_wrap(lane):
    node, clock, result, commands = lane
    approach(lane)
    result.found = True
    sensors(node, yaw=math.radians(179.))
    frame(node)
    result.found = False
    commands.clear()
    clock.t += .03
    sensors(node, x=.006, yaw=math.radians(-179.))
    frame(node)
    assert len(commands) == 1


def test_gap_respects_a_lower_speed_limit(lane):
    node, clock, result, commands = lane
    approach(lane)
    clock.t += .03
    sensors(node, x=.003)
    node.on_speed_limit(Float32(data=.1))
    frame(node)
    assert commands[-1].linear.x == pytest.approx(.1)
