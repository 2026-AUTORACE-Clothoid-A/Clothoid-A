#!/usr/bin/env python3
"""rviz view of the obstacle extractor's clustering, two parameter sets side by side.

Subscribes `scan_topic`, publishes visualization_msgs/MarkerArray on
  /lidar/debug/clusters_a   set A (normally the running lidar.yaml values)
  /lidar/debug/clusters_b   set B (candidate values to compare)
  /lidar/debug/zones        cone course start / continue boxes (base_link)
Cluster colours: green accepted (published centre = white dot), red width > max_width,
purple beams < min_points. Labels "width cm / beams". Clusters are drawn in the scan frame.
Set B can be changed live: `ros2 param set /lidar_debug b_cluster_gap 0.05`.
"""
import rclpy
from geometry_msgs.msg import Point
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from clothoid_lidar.lidar_debug import FEW, OK, WIDE, classify_clusters, summary
from clothoid_lidar.obstacle_extractor import ExtractorParams

FIELDS = ('cluster_gap', 'min_points', 'max_width', 'cone_radius')
COLOURS = {OK: (0.1, 0.85, 0.2), WIDE: (0.95, 0.15, 0.1), FEW: (0.7, 0.3, 0.9)}


def colour(rgb, a=1.0):
    return ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=a)


class LidarDebugNode(Node):
    def __init__(self):
        super().__init__('lidar_debug')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('label_range', 1.5)      # m, labels only for clusters this close
        d = ExtractorParams()
        self.declare_parameter('range_min', d.range_min)
        self.declare_parameter('range_max', d.range_max)
        self.declare_parameter('angle_min_deg', d.angle_min_deg)
        self.declare_parameter('angle_max_deg', d.angle_max_deg)
        for s, gap, minp in (('a', d.cluster_gap, d.min_points), ('b', 0.05, 3)):
            self.declare_parameter(f'{s}_cluster_gap', gap)
            self.declare_parameter(f'{s}_min_points', minp)
            self.declare_parameter(f'{s}_max_width', d.max_width)
            self.declare_parameter(f'{s}_cone_radius', d.cone_radius)
        # cone_sorter zones (base_link), from lidar.yaml cone_sorter
        for n, v in (('start_x_min', 0.0), ('start_x_max', 1.3), ('start_y_min', -0.6),
                     ('start_y_max', 0.6), ('ing_x_min', 0.0), ('ing_x_max', 1.0),
                     ('ing_y_min', -1.0), ('ing_y_max', 1.0)):
            self.declare_parameter(n, v)

        self.pubs = {s: self.create_publisher(MarkerArray, f'/lidar/debug/clusters_{s}', 1)
                     for s in ('a', 'b')}
        self.zone_pub = self.create_publisher(MarkerArray, '/lidar/debug/zones', 1)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.on_scan,
                                 qos_profile_sensor_data)
        self.create_timer(1.0, self.publish_zones)
        self.add_on_set_parameters_callback(lambda _: SetParametersResult(successful=True))
        self.get_logger().info('lidar_debug started: A=%s B=%s' % (self.params('a'), self.params('b')))

    def params(self, s) -> ExtractorParams:
        v = lambda n: self.get_parameter(n).value  # noqa: E731
        return ExtractorParams(range_min=v('range_min'), range_max=v('range_max'),
                               angle_min_deg=v('angle_min_deg'), angle_max_deg=v('angle_max_deg'),
                               **{f: v(f'{s}_{f}') for f in FIELDS})

    def on_scan(self, msg: LaserScan):
        for s, z in (('a', 0.0), ('b', 0.03)):
            p = self.params(s)
            clusters = classify_clusters(msg.ranges, msg.angle_min, msg.angle_increment, p)
            self.pubs[s].publish(self.markers(msg, clusters, p, s, z))

    def markers(self, msg, clusters, p, s, z):
        out = MarkerArray()
        out.markers.append(Marker(action=Marker.DELETEALL))
        life = Duration(seconds=0.5).to_msg()
        label_range = self.get_parameter('label_range').value

        def new(ns, mid, mtype, frame=msg.header.frame_id):
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = msg.header.stamp
            m.ns, m.id, m.type, m.action = f'{s}_{ns}', mid, mtype, Marker.ADD
            m.pose.orientation.w = 1.0
            m.lifetime = life
            return m

        centres = new('centres', 0, Marker.SPHERE_LIST)
        centres.scale.x = centres.scale.y = centres.scale.z = 0.03
        centres.color = colour((1.0, 1.0, 1.0))
        for i, c in enumerate(clusters):
            if len(c.points) == 1:
                m = new('clusters', i, Marker.SPHERE)
                m.pose.position = Point(x=float(c.points[0, 0]), y=float(c.points[0, 1]), z=z)
                m.scale.x = m.scale.y = m.scale.z = 0.02
            else:
                m = new('clusters', i, Marker.LINE_STRIP)
                m.scale.x = 0.012
                m.points = [Point(x=float(x), y=float(y), z=z) for x, y in c.points]
            m.color = colour(COLOURS[c.kind])
            out.markers.append(m)
            if c.kind == OK:
                centres.points.append(Point(x=float(c.centre[0]), y=float(c.centre[1]), z=z))
            near = float((c.centre ** 2).sum()) ** 0.5 < label_range
            if near and c.kind != FEW:
                t = new('labels', i, Marker.TEXT_VIEW_FACING)
                t.pose.position = Point(x=float(c.centre[0]) + 0.03, y=float(c.centre[1]) + 0.03,
                                        z=z + 0.05)
                t.scale.z = 0.035
                t.color = colour(COLOURS[c.kind])
                t.text = f'{c.width * 100:.0f}/{len(c.points)}'
                out.markers.append(t)
        out.markers.append(centres)

        n = summary(clusters)
        t = new('summary', 0, Marker.TEXT_VIEW_FACING, self.get_parameter('base_frame').value)
        # behind the car, A above B in the top-down view
        t.pose.position = Point(x=-0.35 if s == 'a' else -0.45, y=0.0, z=0.1)
        t.scale.z = 0.06
        t.color = colour((1.0, 1.0, 1.0) if s == 'a' else (0.4, 0.9, 1.0))
        t.text = (f'{s.upper()}: ok {n[OK]}  wide {n[WIDE]}  few {n[FEW]}   '
                  f'(gap {p.cluster_gap:.2f}, min {p.min_points}, width {p.max_width:.2f})')
        out.markers.append(t)
        return out

    def publish_zones(self):
        v = lambda n: self.get_parameter(n).value  # noqa: E731
        out = MarkerArray()
        for i, (name, rgb) in enumerate((('start', (1.0, 0.6, 0.1)), ('ing', (0.2, 0.8, 0.3)))):
            m = Marker()
            m.header.frame_id = self.get_parameter('base_frame').value
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type = 'cone_zones', i, Marker.LINE_STRIP
            m.pose.orientation.w = 1.0
            m.scale.x = 0.008
            m.color = colour(rgb, 0.8)
            x0, x1, y0, y1 = (v(f'{name}_x_min'), v(f'{name}_x_max'),
                              v(f'{name}_y_min'), v(f'{name}_y_max'))
            m.points = [Point(x=x, y=y, z=0.0) for x, y in
                        ((x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0))]
            out.markers.append(m)
            t = Marker()
            t.header = m.header
            t.ns, t.id, t.type = 'cone_zones_label', i, Marker.TEXT_VIEW_FACING
            t.pose.orientation.w = 1.0
            t.pose.position = Point(x=x1, y=y1, z=0.05)
            t.scale.z = 0.04
            t.color = m.color
            t.text = 'cone start zone' if name == 'start' else 'cone continue zone'
            out.markers.append(t)
        self.zone_pub.publish(out)


def main():
    rclpy.init()
    node = LidarDebugNode()
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
