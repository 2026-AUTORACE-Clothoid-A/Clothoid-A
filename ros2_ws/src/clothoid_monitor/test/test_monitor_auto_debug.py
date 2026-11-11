import rclpy
from rclpy.task import Future
from rcl_interfaces.msg import SetParametersResult

from clothoid_monitor.monitor_node import (
    DEFAULT_PARAM_TARGET_NODES,
    MonitorNode,
)


def test_default_target_nodes_and_topics():
    assert 'lane_main' in DEFAULT_PARAM_TARGET_NODES
    assert 'yolo' in DEFAULT_PARAM_TARGET_NODES


def test_auto_enable_debug_tick_when_ready(monkeypatch):
    if not rclpy.ok():
        rclpy.init()
    try:
        node = MonitorNode()

        mock_set_client = type('MockClient', (), {})()
        mock_set_client.service_is_ready = lambda: True
        calls = []

        def fake_call_async(req):
            calls.append(req)
            fut = Future()
            res = type('Resp', (), {'results': [SetParametersResult(successful=True, reason='')]})()
            fut.set_result(res)
            return fut

        mock_set_client.call_async = fake_call_async
        node._param_clients['lane_main'] = {'set': mock_set_client}
        node._param_clients['yolo'] = {'set': mock_set_client}

        # Clear enabled nodes for test
        node._auto_enabled_nodes.clear()
        node._auto_debug_pending.clear()

        # Trigger check
        node._check_and_enable_debug_params()

        assert len(calls) == 2
        assert 'lane_main' in node._auto_enabled_nodes
        assert 'yolo' in node._auto_enabled_nodes

        # Calling again should not make duplicate calls
        calls.clear()
        node._check_and_enable_debug_params()
        assert len(calls) == 0

        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
