#!/usr/bin/env python3
"""Rubber cone driving node (ROS2 port of 2025 `rubber_cone_node`).

Publishes a Twist on `cmd_topic` (twist_mux input `avoid`). Steering from pure pursuit is
converted with the bicycle model: w = v * tan(delta) / wheelbase. Reverse (one boundary only)
uses the same formula with a negative speed; the sign of angular.z while reversing has to be
confirmed on the car (see docs/INTERFACES.md).
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool, Header

from clothoid_lidar.cone_follower import ConeFollower, FollowerParams
from clothoid_lidar.cone_sorter_node import to_path


def path_to_array(msg: Path) -> np.ndarray:
    return np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses]).reshape(-1, 2)


class ConeFollowerNode(Node):
    def __init__(self):
        super().__init__('cone_follower')

        self.declare_parameter('cmd_topic', '/cmd_vel_avoid')
        self.declare_parameter('left_boundary_topic', '/cone/left_boundary')
        self.declare_parameter('right_boundary_topic', '/cone/right_boundary')
        self.declare_parameter('mission_active_topic', '/cone/mission_active')
        self.declare_parameter('centerline_topic', '/cone/centerline')

        d = FollowerParams()
        self.declare_parameter('wheelbase', d.wheelbase)
        self.declare_parameter('rear_axle_x', d.rear_axle_x)
        self.declare_parameter('speed', d.speed)
        self.declare_parameter('reverse_speed', d.reverse_speed)
        self.declare_parameter('reverse_steer_deg', d.reverse_steer_deg)
        self.declare_parameter('lfd', d.lfd)
        self.declare_parameter('centerline_ds', d.centerline_ds)
        self.declare_parameter('lsq_window_size', d.lsq_window_size)
        self.declare_parameter('boundary_timeout', d.boundary_timeout)

        self.follower = ConeFollower(self._build(lambda n: self.get_parameter(n).value))
        self.active = False
        self.left = self.right = np.zeros((0, 2))
        self.left_time = self.right_time = None

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_topic').value, 1)
        self.center_pub = self.create_publisher(
            Path, self.get_parameter('centerline_topic').value, 1)
        self.create_subscription(
            Path, self.get_parameter('left_boundary_topic').value, self.on_left, 1)
        self.create_subscription(
            Path, self.get_parameter('right_boundary_topic').value, self.on_right, 1)
        self.create_subscription(
            Bool, self.get_parameter('mission_active_topic').value, self.on_active, 1)

        self.add_on_set_parameters_callback(self.on_parameters)
        self.get_logger().info('cone_follower started')

    @staticmethod
    def _build(value) -> FollowerParams:
        return FollowerParams(
            speed=value('speed'), reverse_speed=value('reverse_speed'),
            reverse_steer_deg=value('reverse_steer_deg'), wheelbase=value('wheelbase'),
            rear_axle_x=value('rear_axle_x'), lfd=value('lfd'), centerline_ds=value('centerline_ds'),
            lsq_window_size=value('lsq_window_size'), boundary_timeout=value('boundary_timeout'))

    def on_parameters(self, params):
        proposed = {p.name: p.value for p in params}
        try:
            self.follower.p = self._build(lambda n: proposed.get(n, self.get_parameter(n).value))
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def on_left(self, msg: Path):
        self.left, self.left_time = path_to_array(msg), self.get_clock().now()
        if self.active:
            self.control()

    def on_right(self, msg: Path):
        self.right, self.right_time = path_to_array(msg), self.get_clock().now()
        if self.active:
            self.control()

    def on_active(self, msg: Bool):
        was_active, self.active = self.active, msg.data
        if self.active and not was_active:
            self.get_logger().info('cone mission START')
            if self.left_time is not None or self.right_time is not None:
                self.control()
        elif was_active and not self.active:
            self.get_logger().info('cone mission END')
            self.left = self.right = np.zeros((0, 2))
            self.left_time = self.right_time = None

    def _fresh(self, stamp) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return age <= self.follower.p.boundary_timeout

    def control(self):
        left_fresh, right_fresh = self._fresh(self.left_time), self._fresh(self.right_time)
        if not left_fresh and not right_fresh:
            self.get_logger().warn('all boundaries stale, no command', throttle_duration_sec=1.0)
            return
        result = self.follower.decide(self.left, self.right, left_fresh, right_fresh)
        if result is None:
            return
        if result.reverse:
            self.get_logger().warn('one boundary only, reversing', throttle_duration_sec=1.0)

        cmd = Twist()
        cmd.linear.x = float(result.speed)
        cmd.angular.z = float(
            result.speed * math.tan(result.steering) / self.follower.p.wheelbase)
        self.cmd_pub.publish(cmd)

        if len(result.center):
            header = Header(frame_id='base_link')
            header.stamp = self.get_clock().now().to_msg()
            self.center_pub.publish(to_path(result.center, header))


def main(args=None):
    rclpy.init(args=args)
    node = ConeFollowerNode()
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
