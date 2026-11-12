"""RViz-only snapshots of the running six-bay test. No vehicle command topics."""
import math
import time
from types import SimpleNamespace

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import Marker, MarkerArray
from clothoid_parking.preview import entry_path
from clothoid_parking.viz import layout_markers, observation_markers, preview_markers


class ReviewPublisher:
    def __init__(self, core, mode, target):
        rclpy.init(args=[])
        self.node = rclpy.create_node('parking_sim_review')
        self.clock = self.node.create_publisher(Clock, '/clock', 1)
        self.markers = self.node.create_publisher(MarkerArray, '/parking_sim_review/markers', 1)
        self.core, self.mode, self.target = core, mode, target
        self.last = -math.inf
        self.path = []

    def update(self, model, ranges=None):
        wall = time.monotonic()
        if wall-self.last < .2:
            return
        self.last = wall
        ns = round(model.time*1e9)
        stamp = Time(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)
        self.clock.publish(Clock(clock=stamp))
        core = self.core
        if core.planner.bay is not None and core.phase.value == 'search':
            self.path = entry_path(core.tracker.pose, core.planner.bay, core.planner.p)
        out = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        out.markers.extend(layout_markers(core, stamp).markers)
        out.markers.extend(observation_markers(core, stamp, model.time).markers)
        preview = preview_markers(SimpleNamespace(core=core, path=self.path), stamp)
        for marker in preview.markers:
            if marker.ns == 'legacy_preview_state':
                marker.text = (f'{self.mode.upper()} TEST | requested bay {self.target}\n'
                               f'{core.phase.value}: {core.planner.reason}\n'
                               'Supplied CAD anchor; camera NOT tested')
        out.markers.extend(preview.markers)
        raw = Marker()
        raw.header.frame_id, raw.header.stamp = 'base_link', stamp
        raw.ns, raw.id, raw.type = 'raw_scan', 0, Marker.POINTS
        raw.pose.orientation.w = 1.
        raw.scale.x = raw.scale.y = .025
        raw.color.r = raw.color.g = raw.color.b = raw.color.a = 1.
        for r, a in zip(model.scan() if ranges is None else ranges, model.angles):
            if math.isfinite(r) and r > .02:
                raw.points.append(Point(x=float(r*math.cos(a)+core.laser_x),
                                        y=float(r*math.sin(a)), z=.08))
        out.markers.append(raw)
        self.markers.publish(out)
        rclpy.spin_once(self.node, timeout_sec=0.)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
