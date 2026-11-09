#!/usr/bin/env python3
"""Stop line node: one stop line right in front of the car (bottom of the bird's-eye view).

Publishes std_msgs/Bool on `near_topic` (/stopline_near) every frame: true once a single white
line across the lane has been seen in `near_frames` consecutive frames (stopline.NearLineDetector).
A stop line with crosswalk blocks right past it does not count.

Since 2026-11-09 this is all it does. The 2025 crosswalk stripe detector (stopline.StopLineDetector,
/stopline_detected) is no longer run: it cost about as much CPU again, and without /stopline_detected
mission's pedestrian / traffic light stops never trigger. It does not move the car.
"""
import cv2
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from clothoid_common.image_utils import bgr_to_image, image_to_bgr
from clothoid_lane.lane_detector import LaneParams
from clothoid_lane.stopline import NearLineDetector, NearLineParams


class StopLineNode(Node):
    def __init__(self):
        super().__init__('stopline')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('near_topic', '/stopline_near')
        self.declare_parameter('debug_image_topic', '/stopline/debug_image')
        self.declare_parameter('near_frames', 2)      # consecutive frames before it counts

        w = LaneParams()
        d = NearLineParams()
        self.declare_parameter('warp_x_h', w.x_h)
        self.declare_parameter('warp_x_l', w.x_l)
        self.declare_parameter('warp_y_h', w.y_h)
        self.declare_parameter('warp_y_l', w.y_l)
        self.declare_parameter('hsv_lower', list(d.hsv_lower))
        self.declare_parameter('hsv_upper', list(d.hsv_upper))
        self.declare_parameter('gray_low', d.gray_low)
        self.declare_parameter('debug', False)

        self.near = NearLineDetector(self._build(lambda n: self.get_parameter(n).value))
        self.near_count = 0
        self.near_pub = self.create_publisher(Bool, self.get_parameter('near_topic').value, 1)
        self.debug_pub = self.create_publisher(
            Image, self.get_parameter('debug_image_topic').value, 1)
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.on_image, qos_profile_sensor_data)

        self.add_on_set_parameters_callback(self.on_parameters)
        self.get_logger().info('stopline started (single stop line in front only)')

    @staticmethod
    def _build(value) -> NearLineParams:
        return NearLineParams(
            warp=LaneParams(x_h=value('warp_x_h'), x_l=value('warp_x_l'),
                            y_h=value('warp_y_h'), y_l=value('warp_y_l')),
            hsv_lower=tuple(value('hsv_lower')), hsv_upper=tuple(value('hsv_upper')),
            gray_low=value('gray_low'))

    def on_parameters(self, params):
        proposed = {p.name: p.value for p in params}
        try:
            self.near.configure(self._build(lambda n: proposed.get(n, self.get_parameter(n).value)))
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def on_image(self, msg: Image):
        try:
            bgr = image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return

        seen = self.near.detect(bgr)
        self.near_count = self.near_count + 1 if seen else 0
        self.near_pub.publish(Bool(data=self.near_count >= self.get_parameter('near_frames').value))
        if self.get_parameter('debug').value:
            # the window it judges (white mask), bottom = closest to the car
            mask = self.near.last_mask
            if mask is not None:
                img = cv2.cvtColor(mask.astype('uint8') * 255, cv2.COLOR_GRAY2BGR)
                p = self.near.p
                y = img.shape[0] - p.band_rows
                cv2.line(img, (0, y), (img.shape[1], y), (0, 0, 255) if not seen else (0, 255, 0), 1)
                self.debug_pub.publish(bgr_to_image(img, msg.header))


def main(args=None):
    rclpy.init(args=args)
    node = StopLineNode()
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
