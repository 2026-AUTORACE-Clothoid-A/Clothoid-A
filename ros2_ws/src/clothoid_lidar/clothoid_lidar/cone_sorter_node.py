#!/usr/bin/env python3
"""Cone course detection and left/right boundary publisher (2025 `cone_sorter_node`).

In:  obstacles (PoseArray, base_link).
Out: left / right boundary (nav_msgs/Path) while the course runs, and a Bool "course active".
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool

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


class ConeSorterNode(Node):
    def __init__(self):
        super().__init__('cone_sorter')

        self.declare_parameter('obstacles_topic', '/lidar/obstacles')
        self.declare_parameter('left_boundary_topic', '/cone/left_boundary')
        self.declare_parameter('right_boundary_topic', '/cone/right_boundary')
        self.declare_parameter('mission_active_topic', '/cone/mission_active')

        defaults = SorterParams()
        for name, _ in _FIELDS:
            self.declare_parameter(name, getattr(defaults, name))

        self.mission = ConeMission(ConeSorter(self._build(lambda n: self.get_parameter(n).value)))

        self.left_pub = self.create_publisher(
            Path, self.get_parameter('left_boundary_topic').value, 1)
        self.right_pub = self.create_publisher(
            Path, self.get_parameter('right_boundary_topic').value, 1)
        self.active_pub = self.create_publisher(
            Bool, self.get_parameter('mission_active_topic').value, 1)
        self.create_subscription(
            PoseArray, self.get_parameter('obstacles_topic').value, self.on_obstacles, 1)

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

    def on_obstacles(self, msg: PoseArray):
        cones = np.array([[p.position.x, p.position.y] for p in msg.poses]).reshape(-1, 2)
        was_running = self.mission.running
        result = self.mission.update(cones, self.get_clock().now().nanoseconds * 1e-9)
        if self.mission.running != was_running:
            self.get_logger().info(
                'cone course START' if self.mission.running else 'cone course END')

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
