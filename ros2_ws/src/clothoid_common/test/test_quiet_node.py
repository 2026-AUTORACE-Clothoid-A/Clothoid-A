import os

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from clothoid_common.quiet_node import ENV, QuietEventsMixin


class Quiet(QuietEventsMixin, Node):
    def __init__(self, **kw):
        super().__init__('quiet_test', **kw)


@pytest.fixture
def ctx():
    rclpy.init(domain_id=191)
    yield
    rclpy.shutdown()


def n_events(entity):
    return len(entity.event_handlers)


def test_default_handlers_off(ctx):
    plain, quiet = Node('plain_test'), Quiet()
    for node in (plain, quiet):
        node.sub = node.create_subscription(Bool, '/quiet_t', lambda m: None, 1)
        node.pub = node.create_publisher(Bool, '/quiet_t', 1)
    assert n_events(plain.sub) > 0 and n_events(plain.pub) > 0
    assert n_events(quiet.sub) == 0 and n_events(quiet.pub) == 0
    # fewer waitables for the executor to walk on every wake-up
    assert len(list(quiet.waitables)) < len(list(plain.waitables))
    plain.destroy_node()
    quiet.destroy_node()


def test_param_brings_warnings_back(ctx):
    node = Quiet(parameter_overrides=[rclpy.parameter.Parameter('qos_event_warnings', value=True)])
    sub = node.create_subscription(Bool, '/quiet_t', lambda m: None, 1)
    assert n_events(sub) > 0
    node.destroy_node()


def test_env_brings_warnings_back(ctx, monkeypatch):
    monkeypatch.setenv(ENV, '1')
    node = Quiet()
    assert n_events(node.create_publisher(Bool, '/quiet_t', 1)) > 0
    node.destroy_node()
    assert os.environ[ENV] == '1'
