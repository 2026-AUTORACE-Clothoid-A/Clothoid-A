"""lazy_sensors: finish subscribes /imu and /odom only from /parking_flag until STOP/ABORT."""
import os
import sys

import pytest

rclpy = pytest.importorskip('rclpy')
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from std_msgs.msg import Bool  # noqa: E402

from clothoid_finish.finish_node import FinishNode  # noqa: E402


@pytest.fixture
def node():
    rclpy.init()
    n = FinishNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def sensor_topics(n):
    return {s.topic_name for s in n.subscriptions} & {'/imu_25hz', '/odom_25hz'}


def test_no_imu_odom_before_parking_flag(node):
    assert sensor_topics(node) == set()


def test_parking_flag_subscribes_and_release_unsubscribes(node):
    node.on_arm(Bool(data=False))
    assert sensor_topics(node) == set()
    node.on_arm(Bool(data=True))
    assert sensor_topics(node) == {'/imu_25hz', '/odom_25hz'}
    node._sensors_on(False)
    assert sensor_topics(node) == set()


def test_always_on_when_lazy_off():
    rclpy.init(args=['--ros-args', '-p', 'lazy_sensors:=false'])
    n = FinishNode()
    try:
        assert sensor_topics(n) == {'/imu_25hz', '/odom_25hz'}
    finally:
        n.destroy_node()
        rclpy.shutdown()
