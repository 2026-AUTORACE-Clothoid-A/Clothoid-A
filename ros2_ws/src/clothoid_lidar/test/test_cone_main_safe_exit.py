"""Main command fixtures plus the boundary between normal pursuit and safe exit."""
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Path as PathMessage
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from rclpy.time import Time

from clothoid_lidar.cone_follower_node import ConeFollowerNode


REFERENCE = json.loads((Path(__file__).parent/'data/main_cone_bce61a0.json').read_text())


@pytest.fixture
def node(monkeypatch):
    config = Path(__file__).parents[1]/'config/lidar.yaml'
    # Reproduce the operating point and exit controller used by the frozen reference.
    rclpy.init(args=['--ros-args', '--params-file', str(config),
                     '-p', 'speed:=0.4', '-p', 'exit_from_camera:=false'])
    n = ConeFollowerNode()
    monkeypatch.setattr(n, 'get_clock', lambda: SimpleNamespace(now=lambda: Time(seconds=100.)))
    for name in ('cmd_pub', 'center_pub', 'target_pub', 'exit_done_pub', 'connecting_pub'):
        setattr(n, name, Mock())
    n.center_pub.get_subscription_count.return_value = 1
    n.target_pub.get_subscription_count.return_value = 1
    n.active = True
    yield n
    n.destroy_node()
    rclpy.shutdown()


@pytest.mark.parametrize('case', REFERENCE['cases'], ids=lambda case: case['name'])
def test_normal_commands_and_paths_match_frozen_main_results_without_exit_sensors(node, case):
    node.left, node.right = np.array(case['left']), np.array(case['right'])
    node.left_time = Time(seconds=100. if case['left_fresh'] else 99.)
    node.right_time = Time(seconds=100. if case['right_fresh'] else 99.)
    node.control()
    expected = case['result']
    if expected is None:
        node.cmd_pub.publish.assert_not_called()
        return
    command = node.cmd_pub.publish.call_args.args[0]
    assert command.linear.x == pytest.approx(expected['speed'])
    assert command.angular.z == pytest.approx(expected['speed']*math.tan(expected['steering'])/.2)
    if expected['center']:
        path = node.center_pub.publish.call_args.args[0]
        np.testing.assert_allclose([[p.pose.position.x, p.pose.position.y] for p in path.poses],
                                   expected['center'], atol=1e-12)
    if expected['target'] is not None:
        point = node.target_pub.publish.call_args.args[0].point
        np.testing.assert_allclose([point.x, point.y], expected['target'], atol=1e-12)


def test_course_tuning_cannot_change_exit_speed_or_lookahead(node):
    assert node.follower.p.speed == .4 and node.follower.p.lfd == .4
    assert node.exit_params.speed == .25 and node.exit_params.lfd == .35
    assert all(r.successful for r in node.set_parameters([
        Parameter('speed', value=.6), Parameter('lfd', value=.7)]))
    assert node.follower.p.speed == .6 and node.follower.p.lfd == .7
    assert node.exit_params.speed == .25 and node.exit_params.lfd == .35
    node.exiting = True
    node.pose, node.pose_time = (.5, 0., 0.), node.get_clock().now()
    node.obstacles_time = node.get_clock().now()
    path = np.column_stack((np.arange(0., 1.01, .05), np.zeros(21)))
    assert node.exit_memory.remember(path, (0., 0., 0.), .35, .05)
    node.exit_control()
    assert node.cmd_pub.publish.call_args.args[0].linear.x == .25
    node.exit_done_pub.publish.assert_not_called()


def test_normal_boundary_callbacks_keep_main_order_instead_of_waiting_for_a_pair(node):
    def message(points):
        return PathMessage(poses=[PoseStamped() for _ in points])
    case = REFERENCE['cases'][0]
    def path(points):
        out = message(points)
        for pose, point in zip(out.poses, points):
            pose.pose.position.x, pose.pose.position.y = point
        return out
    node.on_left(path(case['left']))
    node.cmd_pub.publish.assert_not_called()
    node.on_right(path(case['right']))
    assert node.cmd_pub.publish.call_count == 1
    node.on_left(path(case['left']))
    assert node.cmd_pub.publish.call_count == 2


def test_exit_timer_stops_on_missing_pose_even_after_main_was_driving(node):
    case = REFERENCE['cases'][0]
    node.left, node.right = np.array(case['left']), np.array(case['right'])
    node.left_time = node.right_time = node.get_clock().now()
    node.control()
    assert node.cmd_pub.publish.call_args.args[0].linear.x == .4
    node.exiting = True
    node.exit_control()
    assert node.cmd_pub.publish.call_args.args[0].linear.x == 0.
    node.exit_done_pub.publish.assert_not_called()
