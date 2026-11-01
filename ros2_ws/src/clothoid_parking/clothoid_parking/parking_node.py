#!/usr/bin/env python3
"""T-parking node (ROS2 port of 2025 `parking_rect.py`).

Waits for `/parking_flag`, starts the step sequence when the start line is seen, publishes a
Twist on `cmd_topic` (twist_mux input `parking`) at `rate_hz`, and publishes `/parking_done`
(true, once) after the last step. The step table lives in config/parking.yaml.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

from clothoid_common.image_utils import bgr_to_image, image_to_bgr
from clothoid_parking.parking_sequence import (
    ParkingSequence, SequenceParams, step_to_twist_values, steps_from_lists)
from clothoid_parking.start_line import StartLineDetector, StartLineParams

_LINE_FIELDS = ['x_h', 'x_l', 'y_h', 'y_l', 'crop_x', 'crop_y', 'gray_low', 'gray_high',
                'margin', 'shift', 'min_line_len', 'trigger_row']


class ParkingNode(Node):
    def __init__(self):
        super().__init__('parking')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('parking_topic', '/parking_flag')
        self.declare_parameter('cmd_topic', '/cmd_vel_parking')
        self.declare_parameter('parking_done_topic', '/parking_done')
        self.declare_parameter('marker_topic', '/closest_point_marker')
        self.declare_parameter('debug_image_topic', '/parking/debug_image')
        self.declare_parameter('wheelbase', 0.2)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('debug', False)

        s = SequenceParams(steps=[])
        self.declare_parameter('step_duration', [0.0])
        self.declare_parameter('step_speed', [0.0])
        self.declare_parameter('step_steer', [0.0])
        self.declare_parameter('sensor_step', -1)   # -1 = no lidar step
        self.declare_parameter('angle_limit', s.angle_limit)
        self.declare_parameter('range_limit', s.range_limit)
        self.declare_parameter('sensor_step_max_sec', s.sensor_step_max_sec)

        d = StartLineParams()
        for name in _LINE_FIELDS:
            self.declare_parameter(name, getattr(d, name))
        self.declare_parameter('hsv_lower', list(d.hsv_lower))
        self.declare_parameter('hsv_upper', list(d.hsv_upper))

        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self.sequence = ParkingSequence(self._sequence_params(get))
        self.detector = StartLineDetector(self._line_params(get))

        self.parking_flag = False
        self.closest_angle = None
        self.closest_range = None
        self.done_sent = False

        self.cmd_pub = self.create_publisher(Twist, get('cmd_topic'), 1)
        self.done_pub = self.create_publisher(Bool, get('parking_done_topic'), 1)
        self.marker_pub = self.create_publisher(Marker, get('marker_topic'), 10)
        self.debug_pub = self.create_publisher(Image, get('debug_image_topic'), 1)
        self.create_subscription(Image, get('image_topic'), self.on_image, qos_profile_sensor_data)
        self.create_subscription(LaserScan, get('scan_topic'), self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, get('parking_topic'), self.on_flag, 1)
        self.create_timer(1.0 / get('rate_hz'), self.on_timer)

        self.add_on_set_parameters_callback(self.on_parameters)
        self.get_logger().info('parking started')

    @staticmethod
    def _sequence_params(get) -> SequenceParams:
        sensor = get('sensor_step')
        return SequenceParams(
            steps=steps_from_lists(get('step_duration'), get('step_speed'), get('step_steer')),
            sensor_step=None if sensor < 0 else sensor,
            angle_limit=get('angle_limit'), range_limit=get('range_limit'),
            sensor_step_max_sec=get('sensor_step_max_sec'))

    @staticmethod
    def _line_params(get) -> StartLineParams:
        values = {name: get(name) for name in _LINE_FIELDS}
        return StartLineParams(hsv_lower=tuple(get('hsv_lower')),
                               hsv_upper=tuple(get('hsv_upper')), **values)

    def on_parameters(self, params):
        # table / detector changes apply on the next start; a manoeuvre in progress keeps its table
        proposed = {p.name: p.value for p in params}
        get = lambda n: proposed.get(n, self.get_parameter(n).value)  # noqa: E731
        try:
            sequence = self._sequence_params(get)
            ParkingSequence(sequence)  # validates sensor_step against the table
            line = self._line_params(get)
            self.detector.configure(line)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        if not self.sequence.started or self.sequence.done:
            self.sequence = ParkingSequence(sequence)
        return SetParametersResult(successful=True)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_flag(self, msg: Bool):
        self.parking_flag = msg.data

    def on_scan(self, scan: LaserScan):
        ranges = np.asarray(scan.ranges, dtype=float)
        valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
        if not valid.any():
            return
        index = int(np.flatnonzero(valid)[np.argmin(ranges[valid])])
        self.closest_range = float(ranges[index])
        self.closest_angle = scan.angle_min + index * scan.angle_increment
        if self.get_parameter('debug').value:
            self.publish_marker(scan.header.frame_id)

    def publish_marker(self, frame_id: str):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id = 'closest_point', 0
        marker.type, marker.action = Marker.SPHERE, Marker.ADD
        marker.pose.position.x = self.closest_range * math.cos(self.closest_angle)
        marker.pose.position.y = self.closest_range * math.sin(self.closest_angle)
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r, marker.color.a = 1.0, 1.0
        marker.lifetime.nanosec = 100_000_000
        self.marker_pub.publish(marker)

    def on_image(self, msg: Image):
        if self.sequence.started:
            return  # the start line is only needed to trigger
        try:
            bgr = image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return

        debug = self.get_parameter('debug').value
        result = self.detector.detect(bgr, debug)
        if debug and result.debug_image is not None:
            self.debug_pub.publish(bgr_to_image(result.debug_image, msg.header))
        if self.parking_flag and self.detector.should_start(result):
            self.get_logger().info(f'parking start line at row {result.center_y}: START')
            self.sequence.start(self.now_sec())
            self.done_sent = False

    def on_timer(self):
        step = self.sequence.update(self.now_sec(), self.closest_angle, self.closest_range)
        if step is not None:
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = step_to_twist_values(
                step, self.get_parameter('wheelbase').value)
            self.cmd_pub.publish(cmd)
        elif self.sequence.done and not self.done_sent:
            self.done_sent = True
            self.done_pub.publish(Bool(data=True))
            self.get_logger().info('parking sequence finished')


def main(args=None):
    rclpy.init(args=args)
    node = ParkingNode()
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
