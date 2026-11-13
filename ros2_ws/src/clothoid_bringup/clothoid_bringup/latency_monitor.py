#!/usr/bin/env python3
"""Latency / rate monitor for the whole stack. Read-only: it only subscribes.

Every `report_sec` it logs one line per topic:
  stamped topics (Image, LaserScan, PoseArray, DetectionArray, Path ...):
      rate, longest gap, age = receive time - header.stamp  (transport + driver delay, or
      processing delay for outputs that copy the input stamp)
  command topics (Twist, no header), given as `cmd_topic=source_topic`:
      rate, longest gap, age = receive time - header.stamp of the newest `source_topic` message
      = "sensor capture -> command" (lower bound: assumes the node used the newest frame)
  mux output (`mux_output_topic`): which input won, as counts per window, and every switch.

Message types are looked up from the ROS graph, so topics that do not exist yet are picked up later.
Works on a live robot and on `ros2 bag play --clock` with use_sim_time:=true.
Optional `csv_path` appends every report row (time, topic, kind, stats...) for plotting later.
"""
import csv
import math
import os
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

from clothoid_bringup.latency_stats import WindowStats, mux_winner, parse_pairs

TWIST = 'geometry_msgs/msg/Twist'


def stamp_sec(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class LatencyMonitor(Node):
    def __init__(self):
        super().__init__('latency_monitor')
        self.declare_parameter('stamped_topics', [
            '/camera/color/image_raw', '/scan', '/lidar/obstacles', '/detections'])
        self.declare_parameter('cmd_topics', [
            '/cmd_vel_lane=/camera/color/image_raw', '/cmd_vel_state=/camera/color/image_raw',
            '/cmd_vel_parking=/camera/color/image_raw', '/cmd_vel_cone=/scan', '/cmd_vel_avoid=/scan',
            '/cmd_vel_teleop='])
        self.declare_parameter('mux_output_topic', '/cmd_vel')
        self.declare_parameter('report_sec', 5.0)
        self.declare_parameter('winner_max_age', 0.5)   # s, inputs older than this cannot win
        self.declare_parameter('winner_grace', 0.1)     # s, wait for inputs that arrive after the output
        self.declare_parameter('csv_path', '')

        get = lambda n: self.get_parameter(n).value  # noqa: E731
        self.stamped = [t for t in get('stamped_topics') if t]
        self.cmd = parse_pairs(get('cmd_topics'))
        self.mux_out = get('mux_output_topic')
        self.report_sec = get('report_sec')

        self.stats = {}          # topic -> WindowStats
        self.newest_stamp = {}   # source topic -> header.stamp of the newest message
        self.history = {}        # cmd topic -> deque of ((lin.x, ang.z), arrival)
        self.pending = deque()   # mux outputs waiting for `winner_grace`: (values, arrival)
        self.winners = {}        # cmd topic -> count in this window
        self.current_winner = None
        self.subscribed = set()
        self.window_start = self.now()

        self.csv = None
        if get('csv_path'):
            path = os.path.expanduser(get('csv_path'))
            new = not os.path.exists(path)
            self.csv_file = open(path, 'a', newline='')
            self.csv = csv.writer(self.csv_file)
            if new:
                self.csv.writerow(['time', 'topic', 'kind', 'count', 'hz', 'max_gap',
                                   'age_p50', 'age_p95', 'age_max'])

        self.create_timer(1.0, self.discover)
        self.create_timer(self.report_sec, self.report)
        self.create_timer(0.05, self.judge_mux)
        self.get_logger().info('latency_monitor started')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- subscriptions ----------------------------------------------------------------------
    def discover(self):
        """Subscribe to every configured topic as soon as its type is known in the graph."""
        types = dict(self.get_topic_names_and_types())
        wanted = set(self.stamped) | {t for t, _ in self.cmd} | {s for _, s in self.cmd if s}
        if self.mux_out:
            wanted.add(self.mux_out)
        for topic in wanted - self.subscribed:
            if topic not in types:
                continue
            type_name = types[topic][0]
            try:
                cls = get_message(type_name)
            except (AttributeError, ModuleNotFoundError, ValueError) as exc:
                self.get_logger().warn(f'{topic}: cannot load {type_name}: {exc}')
                self.subscribed.add(topic)
                continue
            self.create_subscription(cls, topic, lambda msg, t=topic: self.on_msg(t, msg),
                                     qos_profile_sensor_data)
            self.subscribed.add(topic)
            self.get_logger().info(f'subscribed {topic} [{type_name}]')

    def on_msg(self, topic, msg):
        now = self.now()
        if hasattr(msg, 'header'):
            stamp = stamp_sec(msg)
            if stamp > 0.0:
                self.newest_stamp[topic] = stamp
            if topic in self.stamped:
                self.stats.setdefault(topic, WindowStats()).add(
                    now, now - stamp if stamp > 0.0 else None)
            return
        if not hasattr(msg, 'linear'):
            return
        values = (msg.linear.x, msg.angular.z)
        if topic == self.mux_out:
            self.stats.setdefault(topic, WindowStats()).add(now)
            self.pending.append((values, now))
            return
        self.history.setdefault(topic, deque(maxlen=50)).append((values, now))
        source = dict(self.cmd).get(topic, '')
        age = now - self.newest_stamp[source] if source in self.newest_stamp else None
        self.stats.setdefault(topic, WindowStats()).add(now, age)

    def judge_mux(self):
        """Attribute mux outputs older than `winner_grace` to the input that produced them."""
        now = self.now()
        grace = self.get_parameter('winner_grace').value
        max_age = self.get_parameter('winner_max_age').value
        while self.pending and now - self.pending[0][1] >= grace:
            values, t_out = self.pending.popleft()
            winner = mux_winner(values, t_out, self.history, max_age, grace)
            self.winners[winner] = self.winners.get(winner, 0) + 1
            if winner != self.current_winner:
                self.get_logger().info(f'mux: {self.current_winner} -> {winner}')
                self.current_winner = winner

    # --- report -----------------------------------------------------------------------------
    def report(self):
        now = self.now()
        window = max(now - self.window_start, 1e-6)
        self.window_start = now
        ms = lambda v: '   -' if math.isnan(v) else f'{v * 1000:4.0f}'  # noqa: E731
        lines = [f'--- {window:.1f} s window (ms)  topic: Hz | max gap | age p50 / p95 / max']
        for topic in list(self.stamped) + [t for t, _ in self.cmd] + [self.mux_out]:
            st = self.stats.get(topic)
            if st is None:
                if topic in self.subscribed:
                    lines.append(f'{topic:32s} silent')
                continue
            r = st.report(window, now)
            if topic == self.mux_out:
                kind = 'mux'
            else:
                kind = 'stamped' if topic in self.stamped else 'cmd'
            lines.append(f'{topic:32s} {r["hz"]:5.1f} | {ms(r["max_gap"])} | '
                         f'{ms(r["age_p50"])} / {ms(r["age_p95"])} / {ms(r["age_max"])}')
            if self.csv:
                self.csv.writerow([f'{now:.3f}', topic, kind, int(r['count']), f'{r["hz"]:.2f}',
                                   *(f'{r[k]:.4f}' for k in
                                     ('max_gap', 'age_p50', 'age_p95', 'age_max'))])
            st.reset()
        if self.winners:
            share = ', '.join(f'{k}={v}' for k, v in sorted(self.winners.items(),
                                                            key=lambda kv: -kv[1]))
            lines.append(f'mux winners: {share}')
            self.winners.clear()
        if self.csv:
            self.csv_file.flush()
        self.get_logger().info('\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = LatencyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
