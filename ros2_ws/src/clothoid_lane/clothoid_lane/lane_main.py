#!/usr/bin/env python3
"""Lane following node (ROS2 port of 2025 `lane_main.py`).

The lane detector is picked with `detector`: `track2026` (default, lane_tracker.py: yellow outer
lines, white dividers between them, boundaries tracked between frames) or `sliding2025` (the 2025
detector, lane_detector.py). Both return a target point in the warped image; the steering below is
the 2025 one for both.

Publishes a Twist on `cmd_topic` (a twist_mux input). Steering is computed the same way
as in 2025 and converted to angular.z with the bicycle model: w = v * tan(delta) / wheelbase.
"""
import math

import rclpy
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int32

from clothoid_common.image_utils import bgr_to_image, image_to_bgr
from clothoid_lane.lane_detector import (SIDE_RIGHT, LaneDetector, LaneParams)
from clothoid_lane.lane_tracker import LaneTracker, TrackerParams

DETECTORS = ('track2026', 'sliding2025')
TRACKER_FIELDS = tuple(TrackerParams.__dataclass_fields__)
# parameters that need a new detector (rebuilding one also clears the tracked lines)
DETECTOR_PARAMS = {'detector', 'width', 'height', 'warp_w', 'warp_h', 'warp_x_h', 'warp_x_l',
                   'warp_y_h', 'warp_y_l', 'nwindows', 'margin', 'min_pixels', 'hsv_lower',
                   'hsv_upper', 'canny_low', 'canny_high', *TRACKER_FIELDS}


class LaneMainNode(Node):
    def __init__(self):
        super().__init__('lane_main')

        # topics (overridden by clothoid_bringup/config/topics.yaml)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('cmd_topic', '/cmd_vel_lane')
        self.declare_parameter('direction_topic', '/direction_flag')
        self.declare_parameter('parking_topic', '/parking_flag')
        self.declare_parameter('speed_limit_topic', '/speed_limit')
        self.declare_parameter('debug_image_topic', '/lane/debug_image')

        # driving
        self.declare_parameter('base_speed', 0.3)
        self.declare_parameter('wheelbase', 0.2)
        self.declare_parameter('max_steer_rad', 0.48869)  # LIMO inner wheel limit, 28 deg
        self.declare_parameter('speed_limit_timeout', 0.5)
        # keep the last command this long after the lane is lost (0 = 2025 behaviour, publish nothing)
        self.declare_parameter('lost_hold_sec', 0.0)

        # lane following side / width
        self.declare_parameter('initial_side', SIDE_RIGHT)  # 0 left, 1 right, 2 both
        self.declare_parameter('lane_width', 103)
        self.declare_parameter('parking_lane_width', 140)

        # detector: track2026 (yellow outer lines + white dividers, lane_tracker.py) or
        # sliding2025 (the 2025 port, lane_detector.py)
        self.declare_parameter('detector', 'track2026')
        for name, value in TrackerParams().__dict__.items():
            self.declare_parameter(name, value)
        defaults = LaneParams()
        self.declare_parameter('width', defaults.width)
        self.declare_parameter('height', defaults.height)
        self.declare_parameter('warp_w', defaults.warp_w)
        self.declare_parameter('warp_h', defaults.warp_h)
        self.declare_parameter('warp_x_h', defaults.x_h)
        self.declare_parameter('warp_x_l', defaults.x_l)
        self.declare_parameter('warp_y_h', defaults.y_h)
        self.declare_parameter('warp_y_l', defaults.y_l)
        self.declare_parameter('nwindows', defaults.nwindows)
        self.declare_parameter('margin', defaults.margin)
        self.declare_parameter('min_pixels', defaults.min_pixels)
        self.declare_parameter('hsv_lower', list(defaults.hsv_lower))
        self.declare_parameter('hsv_upper', list(defaults.hsv_upper))
        self.declare_parameter('canny_low', defaults.canny_low)
        self.declare_parameter('canny_high', defaults.canny_high)
        self.declare_parameter('debug', False)

        self.detector = self._make_detector(lambda name: self.get_parameter(name).value)
        self.side = self.get_parameter('initial_side').value
        self.lane_width = self.get_parameter('lane_width').value
        self.parking_started = False

        self.speed_limit = None
        self.speed_limit_stamp = None
        self.last_cmd = None
        self.last_found_stamp = None

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_topic').value, 1)
        self.debug_pub = self.create_publisher(
            Image, self.get_parameter('debug_image_topic').value, 1)
        # debug only, warped image pixels (clothoid_debug draws them): target z = 1 found, 0 lost
        self.target_pub = self.create_publisher(PointStamped, '/lane/target_warp', 1)
        self.left_pub = self.create_publisher(PolygonStamped, '/lane/left_warp', 1)
        self.right_pub = self.create_publisher(PolygonStamped, '/lane/right_warp', 1)
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.on_image, qos_profile_sensor_data)
        self.create_subscription(
            Int32, self.get_parameter('direction_topic').value, self.on_direction, 1)
        self.create_subscription(
            Bool, self.get_parameter('parking_topic').value, self.on_parking, 1)
        self.create_subscription(
            Float32, self.get_parameter('speed_limit_topic').value, self.on_speed_limit, 1)

        self.add_on_set_parameters_callback(self.on_parameters)
        self.get_logger().info('lane_main started')

    @staticmethod
    def _lane_params(value) -> LaneParams:
        return LaneParams(
            width=value('width'), height=value('height'),
            warp_w=value('warp_w'), warp_h=value('warp_h'),
            x_h=value('warp_x_h'), x_l=value('warp_x_l'), y_h=value('warp_y_h'), y_l=value('warp_y_l'),
            nwindows=value('nwindows'), margin=value('margin'), min_pixels=value('min_pixels'),
            hsv_lower=tuple(value('hsv_lower')), hsv_upper=tuple(value('hsv_upper')),
            canny_low=value('canny_low'), canny_high=value('canny_high'))

    def _make_detector(self, value):
        kind = value('detector')
        if kind not in DETECTORS:
            raise ValueError(f'detector must be one of {DETECTORS}, got {kind!r}')
        if kind == 'sliding2025':
            return LaneDetector(self._lane_params(value))
        return LaneTracker(self._lane_params(value),
                           TrackerParams(**{name: value(name) for name in TRACKER_FIELDS}))

    def on_parameters(self, params):
        # `ros2 param set` takes effect on the next frame. The new values are only applied
        # after this callback returns, so rebuild the detector from the proposed values.
        from rcl_interfaces.msg import SetParametersResult
        proposed = {p.name: p.value for p in params}
        if not DETECTOR_PARAMS & proposed.keys():
            return SetParametersResult(successful=True)

        def value(name):
            return proposed.get(name, self.get_parameter(name).value)

        try:
            self.detector = self._make_detector(value)
        except Exception as exc:  # invalid warp parameters, unknown detector etc.
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    # --- flags from other nodes -------------------------------------------------
    def on_direction(self, msg: Int32):
        # 2025: ignored once the parking mission started
        if not self.parking_started:
            self.side = msg.data

    def on_parking(self, msg: Bool):
        self.parking_started = msg.data
        if msg.data:
            self.side = SIDE_RIGHT
            self.lane_width = self.get_parameter('parking_lane_width').value
            self.get_logger().info('parking lane change')

    def on_speed_limit(self, msg: Float32):
        # <= 0 or NaN releases the limit
        if math.isnan(msg.data) or msg.data <= 0.0:
            self.speed_limit = None
        else:
            self.speed_limit = msg.data
            self.speed_limit_stamp = self.get_clock().now()

    def current_speed(self) -> float:
        speed = self.get_parameter('base_speed').value
        if self.speed_limit is not None:
            age = (self.get_clock().now() - self.speed_limit_stamp).nanoseconds * 1e-9
            if age <= self.get_parameter('speed_limit_timeout').value:
                speed = min(speed, self.speed_limit)
        return speed

    def _publish_debug_points(self, result, header):
        for pub, points in ((self.left_pub, result.left_points),
                            (self.right_pub, result.right_points)):
            poly = PolygonStamped(header=header)
            poly.polygon.points = [Point32(x=float(x), y=float(y)) for x, y in points]
            pub.publish(poly)
        target = PointStamped(header=header)
        target.point.x, target.point.y = result.target_x, result.target_y
        target.point.z = 1.0 if result.found else 0.0
        self.target_pub.publish(target)

    # --- main loop --------------------------------------------------------------
    def on_image(self, msg: Image):
        try:
            bgr = image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return

        debug = self.get_parameter('debug').value
        result = self.detector.detect(bgr, self.side, self.lane_width, debug)

        if debug and result.debug_image is not None:
            self.debug_pub.publish(bgr_to_image(result.debug_image, msg.header))
        if debug:
            self._publish_debug_points(result, msg.header)

        now = self.get_clock().now()
        if result.found:
            warp_w = self.detector.p.warp_w
            warp_h = self.detector.p.warp_h
            x = result.target_x - warp_w / 2
            y = warp_h - result.target_y
            # 2025: angle = -atan(x / y) / (pi / 2) * max_angle with max_angle = 1.0 rad
            steer = -math.atan2(x, max(y, 1e-6)) / (math.pi / 2)
            max_steer = self.get_parameter('max_steer_rad').value
            steer = max(-max_steer, min(max_steer, steer))

            speed = self.current_speed()
            cmd = Twist()
            cmd.linear.x = speed
            cmd.angular.z = speed * math.tan(steer) / self.get_parameter('wheelbase').value
            self.last_cmd = cmd
            self.last_found_stamp = now
            self.cmd_pub.publish(cmd)
            return

        hold = self.get_parameter('lost_hold_sec').value
        if hold > 0.0 and self.last_cmd is not None and self.last_found_stamp is not None:
            if (now - self.last_found_stamp).nanoseconds * 1e-9 <= hold:
                cmd = Twist()
                cmd.linear.x = min(self.last_cmd.linear.x, self.current_speed())
                cmd.angular.z = self.last_cmd.angular.z
                self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LaneMainNode()
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
