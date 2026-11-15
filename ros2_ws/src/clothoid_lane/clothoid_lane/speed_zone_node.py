#!/usr/bin/env python3
"""Floor-mark slow zone node (ROS2 port of 2025 `red_road`).

Publishes `/speed_limit` (Float32) while a mark is under the car and for `hold_sec` after.
It never publishes a release value: lane_main drops the limit by itself when the heartbeat
stops, and a release from here would also cancel a limit set by another node (e.g. YOLO sign).
"""
import cv2
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from clothoid_common.image_utils import bgr_to_image, image_to_bgr
from clothoid_common.qos import IMAGE_QOS
from clothoid_common.quiet_node import QuietEventsMixin
from clothoid_lane.speed_zone import (SpeedZoneDetector, SpeedZoneHold, SpeedZoneParams)


class SpeedZoneNode(QuietEventsMixin, Node):
    def __init__(self):
        super().__init__('speed_zone')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('speed_limit_topic', '/speed_limit')
        self.declare_parameter('debug_image_topic', '/speed_zone/debug_image')

        d = SpeedZoneParams()
        self.declare_parameter('roi_x0', d.roi_x0)
        self.declare_parameter('roi_x1', d.roi_x1)
        self.declare_parameter('roi_y0', d.roi_y0)
        self.declare_parameter('roi_y1', d.roi_y1)
        self.declare_parameter('hsv_ranges', list(d.hsv_ranges))  # integers in yaml
        self.declare_parameter('min_area', d.min_area)
        self.declare_parameter('slow_speed', d.slow_speed)
        self.declare_parameter('hold_sec', d.hold_sec)
        self.declare_parameter('debug', False)

        params = self._build(lambda name: self.get_parameter(name).value)
        self.detector = SpeedZoneDetector(params)
        self.hold = SpeedZoneHold(params.hold_sec)

        self.limit_pub = self.create_publisher(
            Float32, self.get_parameter('speed_limit_topic').value, 1)
        self.debug_pub = self.create_publisher(
            Image, self.get_parameter('debug_image_topic').value, 1)
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.on_image, IMAGE_QOS)

        self.add_on_set_parameters_callback(self.on_parameters)
        self.get_logger().info('speed_zone started')

    @staticmethod
    def _build(value) -> SpeedZoneParams:
        return SpeedZoneParams(
            roi_x0=value('roi_x0'), roi_x1=value('roi_x1'),
            roi_y0=value('roi_y0'), roi_y1=value('roi_y1'),
            hsv_ranges=tuple(value('hsv_ranges')), min_area=value('min_area'),
            slow_speed=value('slow_speed'), hold_sec=value('hold_sec'))

    def on_parameters(self, params):
        proposed = {p.name: p.value for p in params}
        try:
            new = self._build(lambda n: proposed.get(n, self.get_parameter(n).value))
            self.detector.configure(new)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        self.hold.hold_sec = new.hold_sec
        return SetParametersResult(successful=True)

    def on_image(self, msg: Image):
        try:
            bgr = image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return

        debug = self.get_parameter('debug').value
        result = self.detector.detect(bgr, debug)
        if debug and result.debug_image is not None:
            self.debug_pub.publish(bgr_to_image(result.debug_image, msg.header))

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.hold.update(result.found, now):
            self.limit_pub.publish(Float32(data=float(self.detector.p.slow_speed)))


def main(args=None):
    rclpy.init(args=args)
    node = SpeedZoneNode()
    # one OpenCV thread per process: several camera nodes each spawning a pool fight over the
    # cores (lane_shared does the same with shared_opencv_threads); 0 keeps OpenCV's default
    threads = node.declare_parameter('opencv_threads', 1).value
    if threads:
        cv2.setNumThreads(threads)
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
