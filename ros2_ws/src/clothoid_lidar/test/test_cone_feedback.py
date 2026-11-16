"""Feedback observers must not change cone commands or allocate unused paths."""
from unittest.mock import Mock

import numpy as np
import pytest
import rclpy
from std_msgs.msg import Bool

from clothoid_lidar import cone_follower_node
from clothoid_lidar.cone_follower import FollowResult


@pytest.fixture
def node():
    rclpy.init()
    instance = cone_follower_node.ConeFollowerNode()
    instance.cmd_pub = Mock()
    instance.center_pub = Mock()
    instance.target_pub = Mock()
    instance.center_pub.get_subscription_count.return_value = 0
    instance.target_pub.get_subscription_count.return_value = 0
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def forward():
    return FollowResult(.25, .2, center=np.array([[0., 0.], [.5, .1], [1., .3]]),
                        target=np.array([.3, .06]))


def test_unobserved_feedback_is_not_constructed_and_command_still_publishes(node, monkeypatch):
    def unexpected(*args, **kwargs):
        raise AssertionError('No feedback subscriber needs this message')

    monkeypatch.setattr(cone_follower_node, 'to_path', unexpected)
    monkeypatch.setattr(cone_follower_node, 'PointStamped', unexpected)
    node.publish_result(forward())
    command = node.cmd_pub.publish.call_args.args[0]
    assert command.linear.x == .25
    assert command.angular.z == pytest.approx(.25*np.tan(.2)/node.follower.p.wheelbase)
    node.center_pub.publish.assert_not_called()
    node.target_pub.publish.assert_not_called()


@pytest.mark.parametrize('center_count,target_count', [(1, 0), (0, 1), (1, 1)])
def test_rviz_or_bag_subscription_receives_current_feedback_without_changing_control(
        node, center_count, target_count):
    result = forward()
    node.publish_result(result)
    command_without_observer = node.cmd_pub.publish.call_args.args[0]
    node.center_pub.get_subscription_count.return_value = center_count
    node.target_pub.get_subscription_count.return_value = target_count
    node.publish_result(result)
    assert node.cmd_pub.publish.call_args.args[0] == command_without_observer
    assert node.center_pub.publish.call_count == center_count
    assert node.target_pub.publish.call_count == target_count
    if center_count:
        path = node.center_pub.publish.call_args.args[0]
        assert path.header.frame_id == 'base_link'
        np.testing.assert_array_equal(
            [[p.pose.position.x, p.pose.position.y] for p in path.poses], result.center)
    if target_count:
        target = node.target_pub.publish.call_args.args[0]
        assert target.header.frame_id == 'base_link'
        np.testing.assert_array_equal([target.point.x, target.point.y], result.target)


def test_unobserved_end_and_wait_paths_are_skipped_but_exit_still_stops(node, monkeypatch):
    def unexpected(*args, **kwargs):
        raise AssertionError('Unused path must not be constructed')

    monkeypatch.setattr(cone_follower_node, 'to_path', unexpected)
    node.active = True
    node.on_active(Bool(data=False))
    node.active = node.exiting = True
    node.exit_control()
    assert node.cmd_pub.publish.call_args.args[0].linear.x == 0.
    node.center_pub.publish.assert_not_called()


def test_observed_end_and_wait_paths_still_clear_the_display(node):
    node.center_pub.get_subscription_count.return_value = 1
    node.active = True
    node.on_active(Bool(data=False))
    assert not node.center_pub.publish.call_args.args[0].poses
    node.active = node.exiting = True
    node.exit_control()
    assert not node.center_pub.publish.call_args.args[0].poses
    assert node.center_pub.publish.call_count == 2
