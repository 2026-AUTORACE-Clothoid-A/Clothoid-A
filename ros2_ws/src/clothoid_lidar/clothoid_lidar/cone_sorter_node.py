#!/usr/bin/env python3
"""Cone course detection and left/right boundary publisher (2025 `cone_sorter_node`).

In:  obstacles (PoseArray, base_link), /avoid/active (Bool, static_avoid): no course START while the
     static obstacle mission runs (its LIMOs side by side pass the start test) and for
     avoid_release_hold s after it ends (an end among the LIMOs started it at once, 11-09 bags). A
     running course goes on; without the topic (no static_avoid, or older than avoid_active_timeout)
     it starts as before.
Out: left / right boundary (nav_msgs/Path) while the course runs, and a Bool "course active".
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool

from clothoid_common.quiet_node import QuietEventsMixin
from clothoid_lidar.cone_sorter import ConeMission, ConeSorter, SorterParams

_FIELDS = [
    ('trace_method', str),
    ('max_dist_between_cones', float), ('max_trace_length', int),
    ('threshold_directional_angle_deg', float), ('first_step_angle_deg', float),
    ('angle_from_left_first_cone', float),
    ('dist_from_left_first_cone', float), ('dist_from_right_first_cone', float),
    ('start_x_min', float), ('start_x_max', float), ('start_y_min', float),
    ('start_y_max', float), ('start_min_count', int),
    ('ing_x_min', float), ('ing_x_max', float), ('ing_y_min', float),
    ('ing_y_max', float), ('ing_min_count', int),
    ('max_n_neighbors', int), ('threshold_absolute_angle_deg', float),
    ('ellipse_major_radius_factor', float), ('ellipse_minor_radius_factor', float),
    ('len_override_factor', float), ('car_size', float), ('angle_cost_threshold_deg', float),
    ('residual_dist_threshold', float), ('wrong_dir_threshold_deg', float),
    ('w_angle_cost', float), ('w_residual_dist_cost', float), ('w_num_cones_cost', float),
    ('w_initial_dir_cost', float), ('w_change_dir_cost', float), ('w_wrong_dir_cost', float),
    ('max_search_nodes', int),
    ('boundary_timeout', float),
]


def to_path(points: np.ndarray, header) -> Path:
    path = Path()
    path.header = header
    for x, y in points:
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


class ConeSorterNode(QuietEventsMixin, Node):
    def __init__(self):
        super().__init__('cone_sorter')

        self.declare_parameter('obstacles_topic', '/lidar/obstacles')
        self.declare_parameter('left_boundary_topic', '/cone/left_boundary')
        self.declare_parameter('right_boundary_topic', '/cone/right_boundary')
        self.declare_parameter('mission_active_topic', '/cone/mission_active')
        self.declare_parameter('avoid_active_topic', '/avoid/active')
        self.declare_parameter('avoid_active_timeout', 0.5)   # s, an older /avoid/active is ignored
        self.declare_parameter('avoid_release_hold', 2.0)     # s after static_avoid ends

        defaults = SorterParams()
        for name, _ in _FIELDS:
            self.declare_parameter(name, getattr(defaults, name))

        self.mission = ConeMission(ConeSorter(self._build(lambda n: self.get_parameter(n).value)), hold_exit=True)

        self.left_pub = self.create_publisher(
            Path, self.get_parameter('left_boundary_topic').value, 1)
        self.right_pub = self.create_publisher(
            Path, self.get_parameter('right_boundary_topic').value, 1)
        self.active_pub = self.create_publisher(
            Bool, self.get_parameter('mission_active_topic').value, 1)
        self.create_subscription(
            PoseArray, self.get_parameter('obstacles_topic').value, self.on_obstacles, 1)
        self.avoid_active, self.avoid_seen, self.avoid_last = False, None, None
        self.held = False
        self.create_subscription(
            Bool, self.get_parameter('avoid_active_topic').value, self.on_avoid_active, 1)

        self.exit_pub = self.create_publisher(Bool, '/cone/exiting', 1)
        self.create_subscription(Bool, '/cone/exit_complete', self.on_exit_complete, 1)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.get_logger().info('cone_sorter started')

    @staticmethod
    def _build(value) -> SorterParams:
        return SorterParams(**{name: kind(value(name)) for name, kind in _FIELDS})

    def on_parameters(self, params):
        proposed = {p.name: p.value for p in params}
        try:
            self.mission.sorter.p = self._build(
                lambda n: proposed.get(n, self.get_parameter(n).value))
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def on_exit_complete(self, msg: Bool):
        if msg.data and self.mission.finish_exit():
            self.get_logger().info('cone course END: lane handover confirmed')
            self.exit_pub.publish(Bool(data=False))
            self.active_pub.publish(Bool(data=False))

    def on_avoid_active(self, msg: Bool):
        self.avoid_active, self.avoid_seen = msg.data, self.get_clock().now()

    def avoid_holds(self) -> bool:
        """static_avoid runs (fresh /avoid/active true) or ended less than avoid_release_hold ago."""
        now = self.get_clock().now()
        if self.avoid_active and self.avoid_seen is not None and (now - self.avoid_seen).nanoseconds \
                * 1e-9 <= self.get_parameter('avoid_active_timeout').value:
            self.avoid_last = now
            return True
        return self.avoid_last is not None and (now - self.avoid_last).nanoseconds * 1e-9 \
            < self.get_parameter('avoid_release_hold').value

    def on_obstacles(self, msg: PoseArray):
        cones = np.array([[p.position.x, p.position.y] for p in msg.poses]).reshape(-1, 2)
        was_running = self.mission.running
        was_exiting = self.mission.exiting
        held = not was_running and self.avoid_holds()
        if held != self.held:
            self.held = held
            self.get_logger().info('cone course start held: static obstacle mission running or just ended'
                                   if held else 'cone course start allowed')
        result = self.mission.update(cones, self.get_clock().now().nanoseconds * 1e-9,
                                     may_start=not held)
        if self.mission.running != was_running:
            self.get_logger().info(
                'cone course START' if self.mission.running else 'cone course END')

        if self.mission.exiting and not was_exiting:
            self.get_logger().info('cone course EXIT: retaining observed path until lane ready')
        self.exit_pub.publish(Bool(data=self.mission.exiting))
        if result is not None:
            header = msg.header
            if len(result.left):
                self.left_pub.publish(to_path(result.left, header))
            if len(result.right):
                self.right_pub.publish(to_path(result.right, header))
        self.active_pub.publish(Bool(data=bool(self.mission.running)))


def main(args=None):
    rclpy.init(args=args)
    node = ConeSorterNode()
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
