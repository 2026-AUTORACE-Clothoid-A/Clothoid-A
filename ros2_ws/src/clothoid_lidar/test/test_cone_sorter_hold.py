"""No cone course START in the parking area or after a finished course (1116 logs: the bays' cones
started a course in every finished run; parking / finish only masked it in the mux)."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import rclpy
from rclpy.time import Time
from std_msgs.msg import Bool

from clothoid_lidar.cone_sorter_node import ConeSorterNode


@pytest.fixture
def node(monkeypatch):
    rclpy.init()
    n = ConeSorterNode()
    clock = SimpleNamespace(seconds=100.)
    clock.now = lambda: Time(seconds=clock.seconds)
    monkeypatch.setattr(n, 'get_clock', lambda: clock)
    n._test_clock = clock
    n.left_pub, n.right_pub, n.active_pub, n.exit_pub = Mock(), Mock(), Mock(), Mock()
    n.started = 0

    def update(cones, stamp=None, may_start=True):          # the start test always passes
        if not n.mission.running and may_start:
            n.mission.running = True
            n.started += 1
        return None
    monkeypatch.setattr(n.mission, 'update', update)
    yield n
    n.destroy_node()
    rclpy.shutdown()


def scan(node, seconds):
    node._test_clock.seconds = seconds
    msg = SimpleNamespace(poses=[], header=None)
    node.on_obstacles(msg)


def end_by_handover(node):
    node.mission.exiting = True
    node.on_exit_complete(Bool(data=True))
    assert not node.mission.running


def test_parking_flag_holds_the_start_and_a_stale_flag_releases_it(node):
    node._test_clock.seconds = 100.
    node.on_parking(Bool(data=True))
    scan(node, 100.5)
    scan(node, 102.0)
    assert node.started == 0 and node.held == 'parking'
    scan(node, 102.2)                 # flag 2.2 s old (mission re-sends every 1 s): released
    assert node.started == 1


def test_no_second_course_after_the_lane_handover(node):
    scan(node, 100.)
    assert node.started == 1
    end_by_handover(node)
    for t in (101., 105., 150.):
        scan(node, t)
    assert node.started == 1 and node.held == 'course already done'


def test_single_course_false_allows_a_restart(node):
    node.set_parameters([rclpy.parameter.Parameter('single_course', value=False)])
    scan(node, 100.)
    end_by_handover(node)
    scan(node, 101.)
    assert node.started == 2


def test_a_running_course_is_not_stopped_by_the_parking_flag(node):
    scan(node, 100.)
    node._test_clock.seconds = 100.1
    node.on_parking(Bool(data=True))
    scan(node, 100.2)
    assert node.mission.running and node.started == 1
