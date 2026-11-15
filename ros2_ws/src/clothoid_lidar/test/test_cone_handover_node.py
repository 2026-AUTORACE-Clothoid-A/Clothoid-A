"""The live timer must keep cone control until the same axle used by pursuit arrives."""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import rclpy
from rclpy.time import Time

from clothoid_lidar.cone_follower_node import ConeFollowerNode


@pytest.fixture
def node(monkeypatch):
    rclpy.init()
    n=ConeFollowerNode()
    clock=SimpleNamespace(now=lambda:Time(seconds=100.))
    monkeypatch.setattr(n,'get_clock',lambda:clock)
    n.cmd_pub=Mock();n.exit_done_pub=Mock();n.center_pub=Mock();n.target_pub=Mock()
    n.center_pub.get_subscription_count.return_value=0
    n.target_pub.get_subscription_count.return_value=0
    n.active=n.exiting=n.lane_ready=True
    n.pose_time=n.lane_ready_time=n.obstacles_time=clock.now()
    n.obstacles=np.zeros((0,2))
    n.pose=(.7,0.,0.)
    path=np.column_stack((np.arange(0.,.81,.05),np.zeros(17)))
    assert n.exit_memory.remember(path,(0.,0.,0.),.35,.05)
    yield n
    n.destroy_node();rclpy.shutdown()


def test_front_arrival_keeps_following_and_only_rear_arrival_acknowledges(node):
    node.exit_control()  # front at .8, rear at .6, saved endpoint at .8
    node.exit_done_pub.publish.assert_not_called()
    assert node.cmd_pub.publish.call_args.args[0].linear.x==node.follower.p.speed
    node.pose=(.89,0.,0.)
    node.exit_control()
    node.exit_done_pub.publish.assert_not_called()
    node.pose=(.90,0.,0.)
    node.exit_control()
    assert node.exit_done_pub.publish.call_count==1
    assert node.exit_done_pub.publish.call_args.args[0].data


@pytest.mark.parametrize('missing',['pose','lane'])
def test_rear_arrival_still_requires_fresh_pose_and_lane_readiness(node,missing):
    node.pose=(.91,0.,0.)
    if missing=='pose':node.pose_time=Time(seconds=99.)
    else:node.lane_ready_time=Time(seconds=99.)
    node.exit_control()
    node.exit_done_pub.publish.assert_not_called()


def test_missing_measured_endpoint_stops_instead_of_bypassing_arrival(node):
    node.exit_memory.clear()
    node.exit_control()
    node.exit_done_pub.publish.assert_not_called()
    assert node.cmd_pub.publish.call_args.args[0].linear.x==0.
