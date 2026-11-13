"""Regression checks for the real lane command path during a bump lock."""
from pathlib import Path
from types import SimpleNamespace
import sys
import math

import numpy as np
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32, Header
from sensor_msgs.msg import Imu

for package in ('clothoid_lane', 'clothoid_common'):
    sys.path.insert(0, str(Path(__file__).parents[2] / package))

from clothoid_lane.lane_main import LaneMainNode
from clothoid_lane.steering import angular_z


@pytest.fixture
def lane():
    rclpy.init()
    node = LaneMainNode()
    node.set_parameters([Parameter('base_speed', value=.8), Parameter('steer_gain', value=.33),
                         Parameter('target_max_step_px', value=0.),
                         Parameter('steer_gain_speed_scaling', value=True)])
    result = SimpleNamespace(found=True, target_x=180., target_y=100.,
                             debug_image=None, zigzag=False)
    node.detector = SimpleNamespace(p=SimpleNamespace(warp_w=320, warp_h=240),
                                    detect=lambda *args: result)
    yield node, result
    node.destroy_node()
    rclpy.shutdown()


def frame(node):
    node.process_bgr(np.zeros((480, 640, 3), dtype=np.uint8), Header())


def test_lock_retains_the_actual_speed_scaled_angle_and_current_speed(lane):
    node, result = lane
    node.on_speed_limit(Float32(data=.2))
    frame(node)
    steer, cmd, target = node.last_steer, node.last_cmd, node.used_target
    assert abs(steer) > .05
    node.on_steering_lock(Bool(data=True))
    result.target_x = 100.       # tilt changes the projected lane target
    frame(node)
    assert node.last_cmd.angular.z == pytest.approx(cmd.angular.z)
    assert node.used_target == target
    node.on_speed_limit(Float32(data=.1))
    frame(node)
    assert node.last_cmd.linear.x == pytest.approx(.1)
    assert node.last_cmd.angular.z == pytest.approx(angular_z(.1, steer, .2))


def test_lock_survives_a_temporary_lane_loss_and_releases(lane):
    node, result = lane
    frame(node)
    before = node.last_cmd
    node.on_steering_lock(Bool(data=True))
    result.found = False
    frame(node)
    assert node.last_cmd is not before
    assert node.last_cmd.angular.z == pytest.approx(before.angular.z)
    node.on_steering_lock(Bool(data=False))
    result.found, result.target_x = True, 100.
    frame(node)
    assert node.last_cmd.angular.z * before.angular.z < 0


def test_stale_lock_expires_and_lock_without_prior_command_can_start(lane):
    node, result = lane
    node.on_steering_lock(Bool(data=True))
    frame(node)
    before = node.last_cmd
    result.target_x = 100.
    node.steering_lock_stamp = node.get_clock().now()-Duration(seconds=2.)
    frame(node)
    assert node.last_cmd.angular.z * before.angular.z < 0


def test_lock_uses_level_reference_before_camera_tilt(lane):
    node, result = lane
    imu = Imu()
    imu.orientation.w = 1.
    node.on_imu(imu)
    node.on_speed_limit(Float32(data=.2))
    frame(node)
    level_angle = node.last_steer
    imu.orientation.y = math.sin(math.radians(-3.)/2)
    imu.orientation.w = math.cos(math.radians(-3.)/2)
    node.on_imu(imu)
    result.target_x = 100.
    frame(node)
    assert node.last_steer * level_angle < 0
    node.on_steering_lock(Bool(data=True))
    frame(node)
    assert node.last_steer == pytest.approx(level_angle)


def test_stale_level_reference_is_not_reused(lane):
    node, result = lane
    imu = Imu()
    imu.orientation.w = 1.
    node.on_imu(imu)
    frame(node)
    node.level_steer_stamp = node.get_clock().now()-Duration(seconds=2.)
    imu.orientation.y = math.sin(math.radians(-3.)/2)
    imu.orientation.w = math.cos(math.radians(-3.)/2)
    node.on_imu(imu)
    result.target_x = 100.
    frame(node)
    assert node.level_steer * node.last_steer < 0
    before = node.last_steer
    node.on_steering_lock(Bool(data=True))
    frame(node)
    assert node.last_steer == before


def test_old_level_imu_cannot_label_a_new_tilted_camera_frame(lane):
    node, result = lane
    imu = Imu()
    imu.orientation.w = 1.
    node.on_imu(imu)
    frame(node)
    level_angle = node.level_steer
    imu.header.stamp = (node.get_clock().now()-Duration(seconds=.2)).to_msg()
    node.on_imu(imu)         # arrives now, but describes an older level pose
    result.target_x = 100.
    frame(node)
    assert node.last_steer * level_angle < 0
    node.on_steering_lock(Bool(data=True))
    frame(node)
    assert node.last_steer == level_angle
