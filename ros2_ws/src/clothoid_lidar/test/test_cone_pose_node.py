"""Exercise sensor callbacks and retained geometry without running the vehicle."""
import math
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.parameter import Parameter
from rclpy.time import Time

from clothoid_lidar.cone_follower_node import ConeFollowerNode
from clothoid_lidar.cone_follower import FollowResult


@pytest.fixture
def rig(monkeypatch):
    rclpy.init()
    node = ConeFollowerNode()
    seconds = [100.]
    clock = SimpleNamespace(now=lambda: Time(seconds=seconds[0]))
    monkeypatch.setattr(node, 'get_clock', lambda: clock)
    for name in ('cmd_pub', 'pose_pub', 'center_pub', 'target_pub', 'exit_done_pub'):
        setattr(node, name, Mock())
    node.pose_pub.get_subscription_count.return_value = 1
    node.center_pub.get_subscription_count.return_value = 0
    node.target_pub.get_subscription_count.return_value = 0

    def feed(t, speed=.25, gyro=.5):
        seconds[0] = 100.+t
        if speed is not None:
            odom = Odometry()
            odom.header.stamp = clock.now().to_msg()
            # Neither pose nor angular velocity from chassis feedback may drive cone memory.
            odom.pose.pose.position.x = 1e6
            odom.pose.pose.orientation.w = float('nan')
            odom.twist.twist.angular.z = -99.
            odom.twist.twist.linear.x = speed
            node.on_odom(odom)
        if gyro is not None:
            imu = Imu()
            imu.header.stamp = clock.now().to_msg()
            imu.angular_velocity.z = gyro
            node.on_imu(imu)

    yield node, seconds, feed
    node.destroy_node()
    rclpy.shutdown()


def seed_path(node):
    path = np.array([[0., 0.], [.5, 0.], [1., 0.]])
    assert node.exit_memory.remember(path, node.pose, .35, .05)
    node.follower.corridor.observe(path+[0., .4], path-[0., .4], path, node.pose, .35)
    return node.exit_memory.world.copy()


def test_callbacks_ignore_feedback_pose_and_use_gyro_with_rear_axle_geometry(rig):
    node, _, feed = rig
    feed(0.)
    for i in range(1, 101):
        feed(i*.01)
    angle, radius = .5, .5
    expected = (radius*math.sin(angle)+.1*(math.cos(angle)-1),
                radius*(1-math.cos(angle))+.1*math.sin(angle), angle)
    np.testing.assert_allclose(node.pose, expected, atol=1e-10)
    np.testing.assert_allclose(node.follower.corridor.last_pose, node.pose)
    diagnostic = node.pose_pub.publish.call_args.args[0]
    assert diagnostic.header.frame_id == 'cone_odom'
    assert diagnostic.child_frame_id == 'base_link'
    assert diagnostic.twist.twist.angular.z == pytest.approx(.5)
    assert diagnostic.twist.twist.linear.y == pytest.approx(.05)


def test_all_retained_geometry_and_control_use_same_local_pose(rig):
    node, _, feed = rig
    feed(0.)
    saved = seed_path(node)
    for i in range(1, 101):
        feed(i*.01)
    np.testing.assert_array_equal(node.exit_memory.world, saved)
    np.testing.assert_allclose(node.exit_memory.last_pose, node.pose[:2])
    np.testing.assert_allclose(node.follower.corridor.last_pose, node.pose)
    node.follower.decide = Mock(return_value=FollowResult(0., 0.))
    node.control()
    assert node.follower.decide.call_args.args[4] == node.pose


def test_missing_imu_cannot_complete_exit_even_when_lane_is_ready(rig):
    node, seconds, feed = rig
    feed(0., gyro=None)
    seconds[0] = 100.1
    node.active = node.exiting = node.lane_ready = True
    node.lane_ready_time = node.get_clock().now()
    node.exit_control()
    assert node.pose is None
    node.exit_done_pub.publish.assert_not_called()
    assert node.cmd_pub.publish.call_args.args[0].linear.x == 0.


@pytest.mark.parametrize('missing', ['gyro', 'speed'])
def test_sensor_gap_clears_both_memories_before_using_new_frame(rig, missing):
    node, _, feed = rig
    feed(0.)
    seed_path(node)
    for i in range(1, 40):
        feed(i*.01, **{missing: None})
    assert node.exit_memory.world is None
    assert node.follower.corridor.path is None
    assert node.pose is None
    feed(.4, speed=0., gyro=0.)
    np.testing.assert_allclose(node.pose, (0., 0., 0.))


def test_stale_received_message_cannot_freshen_pose(rig):
    node, seconds, feed = rig
    feed(0.)
    seconds[0] = 101.
    old = Imu()
    old.header.stamp = Time(seconds=100.).to_msg()
    node.on_imu(old)
    assert node.pose is None and node.pose_time is None


def test_rear_offset_change_invalidates_geometry_saved_in_old_frame(rig):
    node, _, feed = rig
    feed(0.)
    seed_path(node)
    result = node.set_parameters([Parameter('rear_axle_x', value=.12)])[0]
    assert result.successful
    assert node.pose is None
    assert node.exit_memory.world is None and node.follower.corridor.path is None
    assert node.odometry.rear_offset == .12
