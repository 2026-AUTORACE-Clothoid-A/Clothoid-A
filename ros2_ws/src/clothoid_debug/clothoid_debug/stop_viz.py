#!/usr/bin/env python3
"""The stop at a stop line (traffic light / pedestrian), for RViz (rviz/stop.rviz).

Listens only: camera, /detections (YOLO) and /mission/state (which carries the line stop's phase,
the line distance and the metres left; mission runs clothoid_finish's FinishCore). Publishes
  /viz/stop_overlay   camera image with the YOLO boxes and a banner: DRIVE, APPROACH (metres left,
                      speed limit), STOPPED (seconds)
  /viz/stop_markers   in base_link: the stop line edge as seen now (white bar), where the front
                      bumper stops (red bar, FinishCore's target), the front bumper (blue)
"""
import json
import time

import cv2
import rclpy
from clothoid_interfaces.msg import DetectionArray
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from clothoid_common.image_utils import bgr_to_image, image_to_bgr

STALE_SEC = 0.5
CLASS_COLORS = {   # BGR
    'traffic_light_red': (0, 0, 255), 'traffic_light_green': (0, 220, 0),
    'traffic_light_yellow': (0, 220, 255), 'sign_pedestrian': (255, 160, 0),
}


def rgba(r, g, b, a=1.0) -> ColorRGBA:
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


class StopViz(Node):
    def __init__(self):
        super().__init__('stop_viz')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('lane_width', 0.5)
        self.declare_parameter('frame_id', 'base_link')
        self.detections, self.det_time = [], None
        self.state, self.state_time = {}, None

        self.overlay_pub = self.create_publisher(Image, '/viz/stop_overlay', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/viz/stop_markers', 1)
        self.create_subscription(Image, self.get_parameter('image_topic').value, self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(DetectionArray, '/detections', self.on_detections, 5)
        self.create_subscription(String, '/mission/state', self.on_state, 5)
        self.create_timer(0.1, self.publish_markers)

    def fresh(self, stamp) -> bool:
        return stamp is not None and time.monotonic() - stamp <= STALE_SEC

    def on_detections(self, msg: DetectionArray):
        self.detections, self.det_time = list(msg.detections), time.monotonic()

    def on_state(self, msg: String):
        try:
            self.state, self.state_time = json.loads(msg.data), time.monotonic()
        except ValueError:
            pass

    def banner(self):
        """(text, BGR colour) of the mission state."""
        if not self.fresh(self.state_time):
            return 'NO MISSION', (128, 128, 128)
        s = self.state
        if s.get('state', 'drive') != 'drive':
            who = 'RED LIGHT' if s['state'] == 'stopped_light' else 'PEDESTRIAN'
            return f'STOPPED ({who}) {s.get("stopped_for") or 0.0:.1f} s', (0, 0, 230)
        if s.get('armed'):
            left = s.get('remaining')
            where = (s.get('line_phase') or 'no stop_line') if left is None \
                else f'{left * 100:.1f} cm to stop'
            limit = s.get('speed_limit')
            limit = f', limit {limit:.2f} m/s' if limit is not None else ''
            return f'APPROACH ({"+".join(s["armed"])}): {where}{limit}', (0, 170, 255)
        return 'DRIVE', (0, 180, 0)

    def on_image(self, msg: Image):
        try:
            bgr = image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return
        if self.fresh(self.det_time):
            for d in self.detections:
                color = CLASS_COLORS.get(d.class_name, (255, 255, 255))
                cv2.rectangle(bgr, (d.x_min, d.y_min), (d.x_max, d.y_max), color, 2)
                cv2.putText(bgr, f'{d.class_name} {d.confidence:.2f}', (d.x_min, max(12, d.y_min - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        text, color = self.banner()
        cv2.rectangle(bgr, (0, 0), (bgr.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(bgr, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
        seen = self.state.get('line') if self.fresh(self.state_time) else None
        line = f'stop line {seen:.3f} m ahead' if seen is not None else 'stop line: not seen'
        cv2.putText(bgr, line, (8, bgr.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 1, cv2.LINE_AA)
        self.overlay_pub.publish(bgr_to_image(bgr, msg.header))

    def bar(self, ident, x, color, width=0.03, ns='stop'):
        m = Marker()
        m.header.frame_id = self.get_parameter('frame_id').value
        m.ns, m.id, m.type, m.action = ns, ident, Marker.CUBE, Marker.ADD
        m.pose.position.x, m.pose.position.z = float(x), 0.005
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = width, float(self.get_parameter('lane_width').value), 0.01
        m.color = color
        m.lifetime.nanosec = 300_000_000
        return m

    def publish_markers(self):
        markers = MarkerArray()
        s = self.state if self.fresh(self.state_time) else {}
        axle = s.get('front', 0.161)
        markers.markers.append(self.bar(0, axle, rgba(0.2, 0.5, 1.0), 0.01, 'front'))
        if s.get('line') is not None:
            # bar drawn beyond the near edge, as the painted line lies
            markers.markers.append(self.bar(1, s['line'] + 0.015, rgba(1, 1, 1, 0.9)))
        if s.get('remaining') is not None:
            markers.markers.append(self.bar(2, axle + s['remaining'], rgba(1, 0.1, 0.1, 0.8), 0.01))
        text = Marker()
        text.header.frame_id = self.get_parameter('frame_id').value
        text.ns, text.id, text.type, text.action = 'banner', 3, Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose.position = Point(x=0.0, y=0.0, z=0.35)
        text.pose.orientation.w = 1.0
        text.scale.z = 0.08
        label, color = self.banner()
        text.color = rgba(color[2] / 255, color[1] / 255, color[0] / 255)
        text.text = label
        text.lifetime.nanosec = 300_000_000
        markers.markers.append(text)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = StopViz()
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
