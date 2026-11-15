#!/usr/bin/env python3
"""LaserScan -> obstacle centres in base_link (stands in for 2025 `obstacle_detector`).

Publishes geometry_msgs/PoseArray on `obstacles_topic`; only positions are meaningful.
"""
import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from clothoid_common.quiet_node import QuietEventsMixin
from clothoid_lidar.obstacle_extractor import (ExtractorParams, extract_obstacles, transform_points)


class ObstacleExtractorNode(QuietEventsMixin, Node):
    def __init__(self):
        super().__init__('obstacle_extractor')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('obstacles_topic', '/lidar/obstacles')
        self.declare_parameter('target_frame', 'base_link')
        # use_tf false: take a fixed lidar pose instead (rosbag without /tf, bench tests)
        self.declare_parameter('use_tf', True)
        self.declare_parameter('lidar_x', 0.0)
        self.declare_parameter('lidar_y', 0.0)
        self.declare_parameter('lidar_yaw', 0.0)

        d = ExtractorParams()
        self.declare_parameter('range_min', d.range_min)
        self.declare_parameter('range_max', d.range_max)
        self.declare_parameter('angle_min_deg', d.angle_min_deg)
        self.declare_parameter('angle_max_deg', d.angle_max_deg)
        self.declare_parameter('cluster_gap', d.cluster_gap)
        self.declare_parameter('min_points', d.min_points)
        self.declare_parameter('max_width', d.max_width)
        self.declare_parameter('min_width', d.min_width)
        self.declare_parameter('cone_radius', d.cone_radius)

        self.params = self._build(lambda n: self.get_parameter(n).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(
            PoseArray, self.get_parameter('obstacles_topic').value, 1)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self.on_scan,
            qos_profile_sensor_data)

        self.add_on_set_parameters_callback(self.on_parameters)
        self.get_logger().info('obstacle_extractor started')

    @staticmethod
    def _build(value) -> ExtractorParams:
        return ExtractorParams(
            range_min=value('range_min'), range_max=value('range_max'),
            angle_min_deg=value('angle_min_deg'), angle_max_deg=value('angle_max_deg'),
            cluster_gap=value('cluster_gap'), min_points=value('min_points'),
            max_width=value('max_width'), min_width=value('min_width'), cone_radius=value('cone_radius'))

    def on_parameters(self, params):
        proposed = {p.name: p.value for p in params}
        try:
            self.params = self._build(lambda n: proposed.get(n, self.get_parameter(n).value))
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _lidar_pose(self, scan: LaserScan):
        """(x, y, yaw) of the scan frame in the target frame, or None if unknown."""
        if not self.get_parameter('use_tf').value:
            g = self.get_parameter
            return g('lidar_x').value, g('lidar_y').value, g('lidar_yaw').value
        try:
            tf = self.tf_buffer.lookup_transform(
                self.get_parameter('target_frame').value, scan.header.frame_id,
                rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(f'no TF to {scan.header.frame_id}: {exc}',
                                   throttle_duration_sec=2.0)
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return tf.transform.translation.x, tf.transform.translation.y, yaw

    def on_scan(self, scan: LaserScan):
        pose = self._lidar_pose(scan)
        if pose is None:
            return
        points = extract_obstacles(scan.ranges, scan.angle_min, scan.angle_increment,
                                   self.params)
        points = transform_points(points, *pose)

        out = PoseArray()
        out.header.stamp = scan.header.stamp
        out.header.frame_id = self.get_parameter('target_frame').value
        for x, y in points:
            p = Pose()
            p.position.x, p.position.y = float(x), float(y)
            p.orientation.w = 1.0
            out.poses.append(p)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleExtractorNode()
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
