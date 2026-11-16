"""Contest tile colours through the actual camera warp, tracker and lane command path."""
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

from clothoid_lane.bump_heading import BumpHeading, BumpHeadingParams, HeadingReference
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


def camera(node, pattern=False):
    bev = np.full((240, 320, 3), 45, np.uint8)
    for x in (75, 245):
        cv2.line(bev, (x, 0), (x, 239), (0, 200, 255), 7)
    if pattern:
        # 20 mm high / 500 mm long archive shape, flattened here to isolate colour
        # confusion. This is NOT a dynamics or suspension simulation.
        for y in range(80, 240):
            for x in range(71, 250):
                bev[y, x] = ((56, 166, 225) if ((y-80)//57+(x-71)//60+1) % 2 == 0
                             else (40, 40, 40))
    return cv2.warpPerspective(bev, np.linalg.inv(node.detector._matrix), (640, 480))


def frame(lane, *, pattern=False, pitch=0., yaw=0., fresh=True, x=0.):
    node, clock, _ = lane
    clock.t += 1/30
    stamp = node.get_clock().now().to_msg()
    tick = math.floor((clock.t-100.)*25+1e-7)
    if fresh and tick != getattr(clock, 'sensor_tick', None):
        clock.sensor_tick = tick
        sensor_stamp = Time(nanoseconds=round((100.+tick/25)*1e9)).to_msg()
        imu = Imu()
        imu.header.stamp = sensor_stamp
        p, y = math.radians(pitch)/2, math.radians(yaw)/2
        imu.orientation.x = -math.sin(p)*math.sin(y)
        imu.orientation.y = math.sin(p)*math.cos(y)
        imu.orientation.z = math.cos(p)*math.sin(y)
        imu.orientation.w = math.cos(p)*math.cos(y)
        node.on_imu(imu)
        odom = Odometry()
        odom.header.stamp = sensor_stamp
        odom.pose.pose.orientation.z = math.sin(y)
        odom.pose.pose.orientation.w = math.cos(y)
        odom.pose.pose.position.x = x
        node.on_odom(odom)
    node.on_speed_limit(Float32(data=.2))
    return node.process_bgr(camera(node, pattern), Header(stamp=stamp))


def prepare(lane, yaw=0.):
    node, _, _ = lane
    for _ in range(5):
        assert not frame(lane, yaw=yaw).broad_yellow
    node.on_bump_prepare(Bool(data=True))
    assert node.bump_heading.reference is not None
    return node.bump_heading.reference


def test_tiles_cannot_overwrite_heading_even_when_lane_found(lane):
    node, _, commands = lane
    ref = prepare(lane)
    for _ in range(15):
        result = frame(lane, pattern=True)
        assert result.found and result.broad_yellow
        assert node.bump_heading.active
        assert node.bump_heading.reference is ref
        assert node.last_steer == pytest.approx(0.)
    assert commands[-1].linear.x == pytest.approx(.2)
    assert node.used_target == ref.target
    # The IMU contact lock must use the clean heading, not the tile's false target.
    node.on_steering_lock(Bool(data=True))
    frame(lane, pattern=True, pitch=-2.2)
    assert node.last_steer == pytest.approx(0.)
    assert node.bump_heading.reference is ref


@pytest.mark.parametrize('yaw', [-2., 2.])
def test_heading_feedback_corrects_both_directions(lane, yaw):
    node, _, _ = lane
    prepare(lane)
    frame(lane, pattern=True, yaw=yaw)
    assert math.degrees(node.last_steer) == pytest.approx(-1.5*yaw)


def test_heading_wrap_is_relative(lane):
    node, _, _ = lane
    prepare(lane, yaw=179.)
    frame(lane, pattern=True, yaw=-179.)
    assert math.degrees(node.last_steer) == pytest.approx(-3.)


def test_recovery_waits_for_clean_frames_after_pitch_unlock(lane):
    node, _, _ = lane
    prepare(lane)
    frame(lane, pattern=True)
    node.on_steering_lock(Bool(data=True))
    for _ in range(4):
        frame(lane)
    assert node.bump_heading.active
    node.on_bump_prepare(Bool(data=False))
    node.on_steering_lock(Bool(data=False))
    for _ in range(3):
        frame(lane, pattern=True)
    assert node.bump_heading.active
    for _ in range(2):
        frame(lane)
        assert node.bump_heading.active
    frame(lane)
    assert not node.bump_heading.active and node.bump_heading.reference is None
    assert abs(node.last_steer) < math.radians(1.)


def test_missing_clean_reference_stops_instead_of_saving_tiles(lane):
    node, _, commands = lane
    for _ in range(5):
        frame(lane, pattern=True)
    node.on_bump_prepare(Bool(data=True))
    assert node.bump_heading.reference is None
    frame(lane, pattern=True)
    assert node.bump_heading.failed
    assert commands[-1].linear.x == commands[-1].angular.z == 0.


def test_stale_25hz_inputs_stop_and_do_not_rearm_on_heartbeat(lane):
    node, clock, commands = lane
    prepare(lane)
    frame(lane, pattern=True)
    clock.t += .3
    frame(lane, pattern=True, fresh=False)
    assert commands[-1].linear.x == 0.
    node.on_bump_prepare(Bool(data=True))
    frame(lane, pattern=True)
    assert node.bump_heading.failed and commands[-1].linear.x == 0.


def test_normal_driving_and_25hz_topic_selection(lane):
    node, _, _ = lane
    for topic in ('imu', 'odom'):
        assert node.get_parameter(topic+'_topic').value == '/'+topic+'_25hz'
    for _ in range(10):
        frame(lane)
        assert not node.bump_heading.active


def test_odometry_reset_stops_frozen_heading(lane):
    node, _, commands = lane
    prepare(lane)
    frame(lane, pattern=True)
    for _ in range(2):
        frame(lane, pattern=True, x=10.)
    assert node.bump_heading.failed and commands[-1].linear.x == 0.


def test_500mm_20mm_gentle_profile_with_25hz_imu_and_odometry(lane):
    from clothoid_mission.mission_logic import (
        MissionLogic, MissionParams, Inputs, BUMP_ON, BUMP_COOLDOWN)

    node, clock, commands = lane
    prepare(lane)
    logic = MissionLogic(MissionParams())
    states, pitches = set(), []
    def height(x):
        return .01*(1+math.cos(2*math.pi*x/.5)) if abs(x) < .25 else 0.
    # Raised cosine is an approximate geometric profile, not the CAD's exact curve.
    # IMU and odometry are delivered on 40 ms source timestamps; camera runs at 30 Hz.
    for i in range(230):
        x = -.9+i*.2/30
        pitch = -math.degrees(math.atan2(height(x+.1)-height(x-.1), .2))
        pitches.append(pitch)
        out = logic.step(Inputs(now=clock.t, seen={'sign_bump'} if i == 0 else set(),
                                imu_pitch=pitch))
        states.add(logic.bump_state)
        node.on_bump_prepare(Bool(data=out.bump_prepare))
        node.on_steering_lock(Bool(data=out.steering_lock))
        frame(lane, pattern=-.8 < x < -.15, pitch=pitch, x=x+.9)
        assert commands[-1].linear.x == pytest.approx(.2)
        assert abs(node.last_steer) < math.radians(1.)
    assert max(map(abs, pitches)) > 2.
    assert {BUMP_ON, BUMP_COOLDOWN} <= states
    assert not node.bump_heading.active


@pytest.mark.parametrize('failure', ['distance', 'duration', 'yaw', 'lateral', 'sensor'])
def test_hold_bounds_are_not_refreshed_by_new_tiles(failure):
    p = BumpHeadingParams()
    h = BumpHeading()
    for t in (1., 1.04, 1.08):
        h.observe(HeadingReference(t, 0., (0., 0., 0.), 0., (160., 185.)), True, p)
    h.prepare(True, 1.1, p)
    def update(now=1.12, travel=0., yaw=0., pose=(0., 0., 0.), fresh=True):
        return h.update(now, travel, yaw, pose, frame_stamp=now, fresh=fresh, hazard=True,
                        clean=False, target=(190., 185.), locked=False, params=p)
    assert update() == (True, 0.)
    cases = {'distance': dict(travel=1.81), 'duration': dict(now=13.13),
             'yaw': dict(yaw=math.radians(9)), 'lateral': dict(pose=(0., .11, 0.)),
             'sensor': dict(fresh=False)}
    assert update(**cases[failure]) == (True, None)
    h.prepare(True, 1.15, p)
    assert update(now=1.16) == (True, None)
