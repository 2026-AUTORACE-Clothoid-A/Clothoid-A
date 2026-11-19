"""/speed_limit is held through a mission stall and released explicitly (1119 1050: mission stalled 1.5 s,
the 0.2 limit expired after 0.5 s and lane_main ran over the light's stop line at 0.8 m/s)."""
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.time import Time
from std_msgs.msg import Float32

from clothoid_lane.lane_main import LaneMainNode


@pytest.fixture
def lane():
    config = Path(__file__).parents[1] / 'config/lane.yaml'
    rclpy.init(args=['--ros-args', '--params-file', str(config)])
    node = LaneMainNode()
    clock = SimpleNamespace(t=100.)
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=round(clock.t*1e9)))
    yield node, clock
    node.destroy_node()
    rclpy.shutdown()


def test_limit_holds_through_a_stall_and_expires_later(lane):
    node, clock = lane
    base = node.get_parameter('base_speed').value
    node.on_speed_limit(Float32(data=.2))
    clock.t += 1.5                                  # mission silent as in 1050
    assert node.current_speed() == pytest.approx(.2)
    clock.t += 1.6                                  # 3.1 s: a dead mission no longer holds the car slow
    assert node.current_speed() == pytest.approx(base)


def test_nan_releases_at_once(lane):
    node, clock = lane
    node.on_speed_limit(Float32(data=.2))
    clock.t += .05
    node.on_speed_limit(Float32(data=math.nan))
    assert node.current_speed() == pytest.approx(node.get_parameter('base_speed').value)
