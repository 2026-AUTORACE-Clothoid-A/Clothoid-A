"""Nodes without rclpy's default QoS event handlers.

rclpy (Humble) gives every subscription and publisher a default event handler (it only logs an
"incompatible QoS" warning). The SingleThreadedExecutor rebuilds its wait set in Python every time
it wakes up and walks all of them, so the cost grows with (subscriptions + publishers) x (messages
per second). static_avoid spent ~70 % of its Python time there; switching the handlers off took its
process from 27 % to 24 % of a PC core (session B, 2026-11-15).

    from clothoid_common.quiet_node import QuietEventsMixin

    class LaneMainNode(QuietEventsMixin, Node):     # mixin first
        ...

The price: a QoS mismatch (e.g. a reliable subscriber on a best-effort topic) no longer logs a
warning; `ros2 topic info -v <topic>` still shows both sides' QoS. To get the warnings back:
`-p qos_event_warnings:=true` on one node, or CLOTHOID_QOS_WARNINGS=1 in the environment of the
launch (every node). An explicit `event_callbacks=` argument is always kept.
"""
import os

from rclpy.qos_event import PublisherEventCallbacks, SubscriptionEventCallbacks

PARAM = 'qos_event_warnings'
ENV = 'CLOTHOID_QOS_WARNINGS'


class QuietEventsMixin:
    """create_subscription / create_publisher without the default event handlers."""

    def _qos_warnings(self) -> bool:
        if os.environ.get(ENV, '') not in ('', '0', 'false'):
            return True
        if not hasattr(self, '_clock'):
            return False        # Node.__init__'s own publishers (/parameter_events): no parameters yet
        if not self.has_parameter(PARAM):
            self.declare_parameter(PARAM, False)
        return bool(self.get_parameter(PARAM).value)

    def create_subscription(self, *args, **kwargs):
        if 'event_callbacks' not in kwargs and not self._qos_warnings():
            kwargs['event_callbacks'] = SubscriptionEventCallbacks(use_default_callbacks=False)
        return super().create_subscription(*args, **kwargs)

    def create_publisher(self, *args, **kwargs):
        if 'event_callbacks' not in kwargs and not self._qos_warnings():
            kwargs['event_callbacks'] = PublisherEventCallbacks(use_default_callbacks=False)
        return super().create_publisher(*args, **kwargs)
