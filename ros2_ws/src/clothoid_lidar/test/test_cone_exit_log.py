"""Exit status lines: at once on a change, then every exit_log_repeat s (1116: a 1 s throttle repeated
one waiting line for up to 190 s after the course)."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import rclpy
from rclpy.time import Time

from clothoid_lidar.cone_follower_node import ConeFollowerNode


@pytest.fixture
def node(monkeypatch):
    rclpy.init()
    n = ConeFollowerNode()
    clock = SimpleNamespace(seconds=100.)
    clock.now = lambda: Time(seconds=clock.seconds)
    logger = Mock()
    monkeypatch.setattr(n, 'get_clock', lambda: clock)
    monkeypatch.setattr(n, 'get_logger', lambda: logger)
    n._test_clock, n._test_logger = clock, logger
    yield n
    n.destroy_node()
    rclpy.shutdown()


def lines(node):
    return [c.args[0] for c in node._test_logger.info.call_args_list + node._test_logger.warn.call_args_list]


def test_same_status_repeats_every_ten_seconds_a_new_one_logs_at_once(node):
    for t in range(100, 131):                            # 30 s of one waiting status, 1 Hz calls
        node._test_clock.seconds = float(t)
        node._exit_log('wait', 'cone EXIT: no fresh clear forward path; waiting for lane', warn=True)
    assert len(node._test_logger.warn.call_args_list) == 4    # 100, 110, 120, 130
    node._test_clock.seconds = 130.5
    node._exit_log('lane ready', 'cone EXIT: lane ready; observed endpoint 1.306 m ahead of rear axle')
    node._test_clock.seconds = 131.
    node._exit_log('lane ready', 'cone EXIT: lane ready; observed endpoint 1.300 m ahead of rear axle')
    assert lines(node).count('cone EXIT: lane ready; observed endpoint 1.306 m ahead of rear axle') == 1
    assert len(lines(node)) == 5


def test_real_logger_survives_warn_then_info():
    """1118: 'waiting for lane' (warn) then 'lane ready' (info) raised ValueError and killed the node."""
    rclpy.init()
    n = ConeFollowerNode()
    clock = SimpleNamespace(seconds=100.)
    n.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=clock.seconds))
    try:
        n._exit_log('wait', 'cone EXIT: no fresh clear forward path; waiting for lane', warn=True)
        clock.seconds = 101.
        n._exit_log('lane ready', 'cone EXIT: lane ready; observed endpoint 1.306 m ahead of rear axle')
        clock.seconds = 102.
        n._exit_log('wait', 'cone EXIT: no fresh clear forward path; waiting for lane', warn=True)
    finally:
        n.destroy_node()
        rclpy.shutdown()
