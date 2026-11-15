"""The live timer must keep cone control until the same axle used by pursuit arrives."""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import rclpy
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped

from clothoid_lidar.cone_follower_node import ConeFollowerNode


@pytest.fixture
def node(monkeypatch):
    rclpy.init()
    n=ConeFollowerNode()
    clock=SimpleNamespace(seconds=100.)
    clock.now=lambda:Time(seconds=clock.seconds)
    monkeypatch.setattr(n,'get_clock',lambda:clock)
    n.cmd_pub=Mock();n.exit_done_pub=Mock();n.center_pub=Mock();n.target_pub=Mock();n.connecting_pub=Mock()
    n._test_clock=clock
    n.center_pub.get_subscription_count.return_value=0
    n.target_pub.get_subscription_count.return_value=0
    n.active=n.exiting=n.lane_ready=True
    n.pose_time=n.lane_ready_time=n.obstacles_time=clock.now()
    n.obstacles=np.zeros((0,2))
    n.pose=(.7,0.,0.)
    path=np.column_stack((np.arange(0.,.81,.05),np.zeros(17)))
    assert n.exit_memory.remember(path,(0.,0.,0.),.35,.05)
    n.odometry.speed=.2;n.odometry.gyro=n.odometry.bias=0.
    candidate=TwistStamped();candidate.header.frame_id='base_link'
    candidate.header.stamp=clock.now().to_msg();candidate.twist.linear.x=.3
    n.on_lane_candidate(candidate)
    yield n
    n.destroy_node();rclpy.shutdown()


def tick(node, seconds):
    node._test_clock.seconds=seconds
    node.pose_time=node.lane_ready_time=node.obstacles_time=node._test_clock.now()
    candidate=TwistStamped();candidate.header.frame_id='base_link'
    candidate.header.stamp=node._test_clock.now().to_msg();candidate.twist.linear.x=.3
    node.on_lane_candidate(candidate)
    node.exit_control()


def test_front_arrival_keeps_following_and_rear_arrival_starts_connection(node):
    node.exit_control()  # front at .8, rear at .6, saved endpoint at .8
    node.exit_done_pub.publish.assert_not_called()
    assert node.cmd_pub.publish.call_args.args[0].linear.x==node.follower.p.speed
    node.pose=(.89,0.,0.)
    node.exit_control()
    node.exit_done_pub.publish.assert_not_called()
    node.pose=(.90,0.,0.)
    node.exit_control()
    node.exit_done_pub.publish.assert_not_called()
    assert node.connection.active
    assert node.cmd_pub.publish.call_args.args[0].linear.x==.2
    for i in range(1,9):
        tick(node,100.+i*.05)
        if node.exit_done_pub.publish.called:
            break
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


def test_rear_arrival_without_a_fresh_detected_lane_stops(node):
    node.pose=(.91,0.,0.);node.lane_candidate_time=Time(seconds=99.)
    node.exit_control()
    assert node.cmd_pub.publish.call_args.args[0].linear.x==0.
    node.exit_done_pub.publish.assert_not_called()


def test_turning_connection_checks_the_actual_arc_for_cones(node):
    node.pose=(.91,0.,0.);node.obstacles=np.array([[.2,0.]])
    node.exit_control()
    assert node.cmd_pub.publish.call_args.args[0].linear.x==0.
    node.exit_done_pub.publish.assert_not_called()


def test_stamped_candidate_cannot_be_refreshed_by_replay_or_delayed_delivery(node):
    original=node.lane_candidate
    for stamp,frame in [(99.,'base_link'),(100.,'base_link'),(100.,'camera'),(101.,'base_link')]:
        msg=TwistStamped();msg.header.stamp=Time(seconds=stamp).to_msg()
        msg.header.frame_id=frame;msg.twist.linear.x=.1
        node.on_lane_candidate(msg)
        assert node.lane_candidate is original
