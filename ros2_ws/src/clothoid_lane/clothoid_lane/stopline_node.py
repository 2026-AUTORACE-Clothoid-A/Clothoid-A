#!/usr/bin/env python3
"""Stop line node (ROS2 port of 2025 `crosswalk.py`).

Publishes `near_topic` (/stopline_near, std_msgs/Bool) every frame: one stop line right in front
with no crosswalk past it, the parking trigger. It does not move the car.

The 2025 crosswalk stripe flag (`stopline_topic`, /stopline_detected) is commented out (2026-11-09):
no 2026 mission needs crosswalks recognised (lights / pedestrians stop on the stop line, done in
clothoid_mission with finish's FinishCore). StopLineDetector stays because the parking trigger takes
its warp and white thresholds.
"""
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from clothoid_common.image_utils import bgr_to_image, image_to_bgr  # noqa: F401  (bgr_to_image: stripe debug, commented out)
from clothoid_lane.lane_detector import LaneParams
from clothoid_lane.stopline import NearLineDetector, NearLineParams, StopLineDetector, StopLineParams


class StopLineNode(Node):
    def __init__(self, subscribe_images=True):
        super().__init__('stopline')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('stopline_topic', '/stopline_detected')
        self.declare_parameter('debug_image_topic', '/stopline/debug_image')

        w = LaneParams()
        d = StopLineParams()
        self.declare_parameter('warp_x_h', w.x_h)
        self.declare_parameter('warp_x_l', w.x_l)
        self.declare_parameter('warp_y_h', w.y_h)
        self.declare_parameter('warp_y_l', w.y_l)
        self.declare_parameter('crop_x', d.crop_x)
        self.declare_parameter('crop_y', d.crop_y)
        self.declare_parameter('hsv_lower', list(d.hsv_lower))
        self.declare_parameter('hsv_upper', list(d.hsv_upper))
        self.declare_parameter('gray_low', d.gray_low)
        self.declare_parameter('gray_high', d.gray_high)
        self.declare_parameter('min_tall_w', d.min_tall_w)
        self.declare_parameter('min_wide_w', d.min_wide_w)
        self.declare_parameter('min_count', d.min_count)
        self.declare_parameter('min_white_ratio', d.min_white_ratio)
        self.declare_parameter('debug', False)
        # one stop line right in front (bottom of the warped image): the parking trigger
        self.declare_parameter('near_topic', '/stopline_near')
        self.declare_parameter('near_frames', 2)      # consecutive frames before it counts

        self.detector = StopLineDetector(self._build(lambda n: self.get_parameter(n).value))
        p = self.detector.p
        self.near = NearLineDetector(NearLineParams(warp=p.warp, hsv_lower=p.hsv_lower,
                                                    hsv_upper=p.hsv_upper, gray_low=p.gray_low))
        self.near_count = 0
        self.near_pub = self.create_publisher(Bool, self.get_parameter('near_topic').value, 1)

        # 2025 stripe flag, not used in 2026 (see the module docstring)
        # self.pub = self.create_publisher(Bool, self.get_parameter('stopline_topic').value, 1)
        self.debug_pub = self.create_publisher(
            Image, self.get_parameter('debug_image_topic').value, 1)
        if subscribe_images:
            self.create_subscription(
                Image, self.get_parameter('image_topic').value, self.on_image, qos_profile_sensor_data)

        self.add_on_set_parameters_callback(self.on_parameters)
        self.get_logger().info('stopline started')

    @staticmethod
    def _build(value) -> StopLineParams:
        return StopLineParams(
            warp=LaneParams(x_h=value('warp_x_h'), x_l=value('warp_x_l'),
                            y_h=value('warp_y_h'), y_l=value('warp_y_l')),
            crop_x=value('crop_x'), crop_y=value('crop_y'),
            hsv_lower=tuple(value('hsv_lower')), hsv_upper=tuple(value('hsv_upper')),
            gray_low=value('gray_low'), gray_high=value('gray_high'),
            min_tall_w=value('min_tall_w'), min_wide_w=value('min_wide_w'),
            min_count=value('min_count'), min_white_ratio=value('min_white_ratio'))

    def on_parameters(self, params):
        proposed = {p.name: p.value for p in params}
        try:
            self.detector.configure(
                self._build(lambda n: proposed.get(n, self.get_parameter(n).value)))
            p = self.detector.p
            self.near.configure(NearLineParams(warp=p.warp, hsv_lower=p.hsv_lower,
                                               hsv_upper=p.hsv_upper, gray_low=p.gray_low))
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def on_image(self, msg: Image):
        try:
            bgr = image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return

        self.on_bgr(bgr)

    def on_bgr(self, bgr):
        """The exact same detector, with either a standalone or shared image subscription."""
        # 2025 stripe flag, not used in 2026 (see the module docstring)
        # debug = self.get_parameter('debug').value
        # result = self.detector.detect(bgr, debug)
        # if debug and result.debug_image is not None:
        #     self.debug_pub.publish(bgr_to_image(result.debug_image, msg.header))
        # self.pub.publish(Bool(data=bool(result.found)))
        self.near_count = self.near_count + 1 if self.near.detect(bgr) else 0
        self.near_pub.publish(Bool(data=self.near_count >= self.get_parameter('near_frames').value))


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
