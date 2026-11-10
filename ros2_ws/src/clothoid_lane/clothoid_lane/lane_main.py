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

import cv2
import rclpy
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int32

from clothoid_common.image_utils import bgr_to_image, image_to_bgr
from clothoid_common.qos import IMAGE_QOS
from clothoid_lane.lane_detector import (SIDE_RIGHT, LaneDetector, LaneParams)
from clothoid_lane.lane_tracker import LaneTracker, TrackerParams
from clothoid_lane.steering import (MAX_STEER_RAD, TARGET_RESET_SEC, WHEELBASE, angular_z,
                                    limit_step, speed_scaled_gain, steer_angle)

DETECTORS = ('track2026', 'sliding2025')
TRACKER_FIELDS = tuple(TrackerParams.__dataclass_fields__)
# parameters that set the lane width in the warped image
WIDTH_PARAMS = {'lane_width_m', 'parking_lane_width_m', 'warp_px_per_m'}


def detector_defaults() -> dict:
    """Parameters that build the detector, with their defaults (also used by clothoid_debug's
    lane_viewer, which builds the same detector from the yaml files without ROS)."""
    d = LaneParams()
    values = {'detector': 'track2026',   # or sliding2025 (the 2025 port, lane_detector.py)
              'width': d.width, 'height': d.height, 'warp_w': d.warp_w, 'warp_h': d.warp_h,
              'warp_x_h': d.x_h, 'warp_x_l': d.x_l, 'warp_y_h': d.y_h, 'warp_y_l': d.y_l,
              'nwindows': d.nwindows, 'margin': d.margin, 'min_pixels': d.min_pixels,
              'hsv_lower': list(d.hsv_lower), 'hsv_upper': list(d.hsv_upper),
              'canny_low': d.canny_low, 'canny_high': d.canny_high}
    values.update(TrackerParams().__dict__)
    return values


# parameters that need a new detector (rebuilding one also clears the tracked lines)
DETECTOR_PARAMS = set(detector_defaults())


def lane_params(value) -> LaneParams:
    return LaneParams(
        width=value('width'), height=value('height'),
        warp_w=value('warp_w'), warp_h=value('warp_h'),
        x_h=value('warp_x_h'), x_l=value('warp_x_l'), y_h=value('warp_y_h'), y_l=value('warp_y_l'),
        nwindows=value('nwindows'), margin=value('margin'), min_pixels=value('min_pixels'),
        hsv_lower=tuple(value('hsv_lower')), hsv_upper=tuple(value('hsv_upper')),
        canny_low=value('canny_low'), canny_high=value('canny_high'))


def make_detector(value):
    """LaneTracker or LaneDetector from a `value(name)` lookup of the DETECTOR_PARAMS."""
    kind = value('detector')
    if kind not in DETECTORS:
        raise ValueError(f'detector must be one of {DETECTORS}, got {kind!r}')
    if kind == 'sliding2025':
        return LaneDetector(lane_params(value))
    return LaneTracker(lane_params(value),
                       TrackerParams(**{name: value(name) for name in TRACKER_FIELDS}))


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
        self.declare_parameter('wheelbase', WHEELBASE)
        self.declare_parameter('max_steer_rad', MAX_STEER_RAD)  # LIMO inner wheel limit, 28 deg
        # multiplies the 2025 steering law before the limit. 1.0 = 2025; limo2 turned too little in
        # the first right corner with 1.0 (2026-11-07 drives: 8.5 deg where about 13 is needed)
        self.declare_parameter('steer_gain', 1.0)
        # below base_speed (/speed_limit, zigzag) raise steer_gain so speed x gain stays the same: tuned
        # at 0.8 m/s with 0.33, the stop line approach at 0.2 turned a quarter of what a corner needs
        # and left the lane (limo2 2026-11-09). At most steer_gain_max (0.3 m/s drove with 1.0).
        self.declare_parameter('steer_gain_speed_scaling', True)
        self.declare_parameter('steer_gain_max', 1.0)
        self.declare_parameter('speed_limit_timeout', 0.5)
        # mission 6 (slow guide lines): speed while track2026 sees the zigzag lines, 0 = no change
        self.declare_parameter('zigzag_speed', 0.2)
        self.declare_parameter('zigzag_hold_sec', 0.5)   # keeps the slow speed over short misses
        # keep the last command this long after the lane is lost (0 = 2025 behaviour, publish nothing)
        self.declare_parameter('lost_hold_sec', 0.0)
        # the target point used for steering moves at most this many warped px per frame (0 = off).
        # 2026-11-08 bag 01:02: normal frames move it 0.3 px (median), corners 7 px (p95); the jumps
        # that flipped the steering by up to +-28 deg moved it 29 px (median). 8 px removed 23 of 30
        # jumps over 8 deg and changed the corner steering by 0.4 deg (p95) when replayed.
        self.declare_parameter('target_max_step_px', 8.0)

        # lane following side / width. Widths are real distances, line centre to centre in metres;
        # the warped image has warp_px_per_m pixels per metre across, set with the warp of each car
        # (config/camera_*.yaml), so the width in pixels is lane_width_m x warp_px_per_m.
        self.declare_parameter('initial_side', SIDE_RIGHT)  # 0 left, 1 right, 2 both
        self.declare_parameter('steering_lock_topic', '/steering_lock')
        self.declare_parameter('lane_width_m', 0.5)
        self.declare_parameter('parking_lane_width_m', 0.68)
        self.declare_parameter('warp_px_per_m', 340.0)

        # detector: track2026 (yellow outer lines + white dividers, lane_tracker.py) or
        # sliding2025 (the 2025 port, lane_detector.py), its warp, masks and windows
        for name, value in detector_defaults().items():
            self.declare_parameter(name, value)
        self.declare_parameter('debug', False)

        self.detector = self._make_detector(lambda name: self.get_parameter(name).value)
        self.side = self.get_parameter('initial_side').value
        self.parking_started = False
        self.parking_width = False   # set by the parking flag, kept afterwards (2025 behaviour)
        self.lane_width = self._lane_width_px(lambda name: self.get_parameter(name).value)

        self.speed_limit = None
        self.speed_limit_stamp = None
        self.steering_locked = False
        self.last_steer = 0.0
        self.last_cmd = None
        self.last_found_stamp = None
        self.used_target = None       # target point the last command steered to (after limit_step)
        self.last_zigzag_stamp = None
        self.image_observers = []  # optional in-process consumers of the same decoded frame

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_topic').value, 1)
        self.debug_pub = self.create_publisher(
            Image, self.get_parameter('debug_image_topic').value, 1)
        # debug only, warped image pixels (clothoid_debug draws them): target z = 1 found, 0 lost
        self.target_pub = self.create_publisher(PointStamped, '/lane/target_warp', 1)
        self.left_pub = self.create_publisher(PolygonStamped, '/lane/left_warp', 1)
        self.right_pub = self.create_publisher(PolygonStamped, '/lane/right_warp', 1)
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.on_image, IMAGE_QOS)
        self.create_subscription(
            Int32, self.get_parameter('direction_topic').value, self.on_direction, 1)
        self.create_subscription(
            Bool, self.get_parameter('parking_topic').value, self.on_parking, 1)
        self.create_subscription(
            Float32, self.get_parameter('speed_limit_topic').value, self.on_speed_limit, 1)
        self.create_subscription(
            Bool, self.get_parameter('steering_lock_topic').value, self.on_steering_lock, 1)

        self.add_on_set_parameters_callback(self.on_parameters)
        self.get_logger().info(f'lane_main started, lane width {self.lane_width:.0f} px')

    def _lane_width_px(self, value) -> float:
        name = 'parking_lane_width_m' if self.parking_width else 'lane_width_m'
        return value(name) * value('warp_px_per_m')

    @staticmethod
    def _make_detector(value):
        return make_detector(value)

    def on_parameters(self, params):
        # `ros2 param set` takes effect on the next frame. The new values are only applied
        # after this callback returns, so rebuild the detector from the proposed values.
        from rcl_interfaces.msg import SetParametersResult
        proposed = {p.name: p.value for p in params}

        def value(name):
            return proposed.get(name, self.get_parameter(name).value)

        width = WIDTH_PARAMS & proposed.keys()
        if width and any(value(name) <= 0.0 for name in WIDTH_PARAMS):
            return SetParametersResult(successful=False, reason='lane widths must be > 0')
        if DETECTOR_PARAMS & proposed.keys():
            try:
                self.detector = self._make_detector(value)
            except Exception as exc:  # invalid warp parameters, unknown detector etc.
                return SetParametersResult(successful=False, reason=str(exc))
        if width:
            self.lane_width = self._lane_width_px(value)
            self.get_logger().info(f'lane width {self.lane_width:.0f} px')
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
            self.parking_width = True
            self.lane_width = self._lane_width_px(lambda name: self.get_parameter(name).value)
            self.get_logger().info('parking lane change')

    def on_speed_limit(self, msg: Float32):
        # <= 0 or NaN releases the limit
        if math.isnan(msg.data) or msg.data <= 0.0:
            self.speed_limit = None
        else:
            self.speed_limit = msg.data
            self.speed_limit_stamp = self.get_clock().now()

    def on_steering_lock(self, msg: Bool):
        self.steering_locked = bool(msg.data)

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

        for observer in self.image_observers:
            observer(bgr)
        self.process_bgr(bgr, msg.header)

    def process_bgr(self, bgr, header):
        debug = self.get_parameter('debug').value
        result = self.detector.detect(bgr, self.side, self.lane_width, debug)

        now = self.get_clock().now()
        target = None
        if result.found:
            prev = self.used_target
            if self.last_found_stamp is None or \
                    (now - self.last_found_stamp).nanoseconds * 1e-9 > TARGET_RESET_SEC:
                prev = None
            target = limit_step(prev, (result.target_x, result.target_y),
                                self.get_parameter('target_max_step_px').value)
            self.used_target = target

        if debug and result.debug_image is not None:
            if target is not None:     # yellow ring: the point steered to (green dot: detected)
                cv2.circle(result.debug_image, (int(round(target[0])), int(round(target[1]))), 7,
                           (0, 255, 255), 2)
            self.debug_pub.publish(bgr_to_image(result.debug_image, header))
        if debug:
            self._publish_debug_points(result, header)

        if result.zigzag:
            self.last_zigzag_stamp = now
        if result.found:
            if self.steering_locked and self.last_cmd is not None:
                steer = self.last_steer
            else:
                steer = steer_angle(target[0], target[1],
                                    self.detector.p.warp_w, self.detector.p.warp_h,
                                    self.get_parameter('steer_gain').value,
                                    self.get_parameter('max_steer_rad').value)
                self.last_steer = steer
            speed = self.current_speed()
            zigzag_speed = self.get_parameter('zigzag_speed').value
            if zigzag_speed > 0.0 and self.last_zigzag_stamp is not None and (
                    now - self.last_zigzag_stamp).nanoseconds * 1e-9 <= \
                    self.get_parameter('zigzag_hold_sec').value:
                speed = min(speed, zigzag_speed)
            gain = self.get_parameter('steer_gain').value
            if self.get_parameter('steer_gain_speed_scaling').value:
                gain = speed_scaled_gain(gain, self.get_parameter('base_speed').value, speed,
                                         self.get_parameter('steer_gain_max').value)
            steer = steer_angle(target[0], target[1],
                                self.detector.p.warp_w, self.detector.p.warp_h,
                                gain, self.get_parameter('max_steer_rad').value)

            cmd = Twist()
            cmd.linear.x = speed
            cmd.angular.z = angular_z(speed, steer, self.get_parameter('wheelbase').value)
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
