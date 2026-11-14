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
from geometry_msgs.msg import PointStamped, PoseArray, Twist
from nav_msgs.msg import Path, Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool, Header

from clothoid_lidar.cone_follower import (ConeFollower, FollowerParams, FollowResult,
                                           compute_steering, look_ahead_point)
from clothoid_lidar.cone_exit import ExitPathMemory, clearance
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
        self.declare_parameter('target_topic', '/cone/target')

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
        self.declare_parameter('corner_endpoint_enabled', d.corner_endpoint_enabled)
        self.declare_parameter('exit_clearance', 0.2)  # path to detected obstacle centre, metres

        self.follower = ConeFollower(self._build(lambda n: self.get_parameter(n).value))
        self.active = False
        self.exiting = False
        self.exit_memory = ExitPathMemory()
        self.pose = None
        self.pose_time = None
        self.obstacles = np.zeros((0, 2))
        self.obstacles_time = None
        self.lane_ready = False
        self.lane_ready_time = None
        self.left = self.right = np.zeros((0, 2))
        self.left_time = self.right_time = None
        self.left_scan_stamp = self.right_scan_stamp = None

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_topic').value, 1)
        self.center_pub = self.create_publisher(
            Path, self.get_parameter('centerline_topic').value, 1)
        self.target_pub = self.create_publisher(
            PointStamped, self.get_parameter('target_topic').value, 1)
        self.create_subscription(
            Path, self.get_parameter('left_boundary_topic').value, self.on_left, 1)
        self.create_subscription(
            Path, self.get_parameter('right_boundary_topic').value, self.on_right, 1)
        self.create_subscription(
            Bool, self.get_parameter('mission_active_topic').value, self.on_active, 1)

        self.exit_done_pub = self.create_publisher(Bool, '/cone/exit_complete', 1)
        self.create_subscription(Bool, '/cone/exiting', self.on_exiting, 1)
        self.create_subscription(Bool, '/lane/cone_exit_ready', self.on_lane_ready, 1)
        self.create_subscription(Odometry, '/odom', self.on_odom, 1)
        self.create_subscription(PoseArray, '/lidar/obstacles', self.on_obstacles, 1)
        self.create_timer(0.05, self.exit_control)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.get_logger().info('cone_follower started')

    @staticmethod
    def _build(value) -> FollowerParams:
        return FollowerParams(
            speed=value('speed'), reverse_speed=value('reverse_speed'),
            reverse_steer_deg=value('reverse_steer_deg'), wheelbase=value('wheelbase'),
            rear_axle_x=value('rear_axle_x'), lfd=value('lfd'), centerline_ds=value('centerline_ds'),
            lsq_window_size=value('lsq_window_size'), boundary_timeout=value('boundary_timeout'),
            corner_endpoint_enabled=value('corner_endpoint_enabled'))

    def on_parameters(self, params):
        proposed = {p.name: p.value for p in params}
        try:
            self.follower.p = self._build(lambda n: proposed.get(n, self.get_parameter(n).value))
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def on_left(self, msg: Path):
        self.left_scan_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.left, self.left_time = path_to_array(msg), self.get_clock().now()
        if self.active and not self.exiting:
            self.control()

    def on_right(self, msg: Path):
        self.right_scan_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.right, self.right_time = path_to_array(msg), self.get_clock().now()
        if self.active and not self.exiting:
            self.control()

    def on_active(self, msg: Bool):
        was_active, self.active = self.active, msg.data
        if self.active and not was_active:
            self.exit_memory.clear()
            self.follower.reset()
            self.get_logger().info('cone mission START')
            if self.left_time is not None or self.right_time is not None:
                self.control()
        elif was_active and not self.active:
            self.exiting = False
            self.exit_memory.clear()
            self.follower.reset()
            self.center_pub.publish(to_path(np.zeros((0, 2)), Header(frame_id='base_link')))
            self.get_logger().info('cone mission END')
            self.left = self.right = np.zeros((0, 2))
            self.left_time = self.right_time = None
            self.left_scan_stamp = self.right_scan_stamp = None

    def on_exiting(self, msg):
        if msg.data and not self.exiting:
            self.lane_ready = False
            self.lane_ready_time = None
        self.exiting = msg.data

    def on_lane_ready(self, msg):
        self.lane_ready = msg.data
        self.lane_ready_time = self.get_clock().now()

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        pose = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        if not all(math.isfinite(v) for v in pose):
            self.pose_time = None
            self.exit_memory.clear()
            self.follower.reset()
            return
        if self.pose is not None and math.hypot(pose[0]-self.pose[0], pose[1]-self.pose[1]) > 1.0:
            self.exit_memory.clear()
            self.follower.reset()
        self.exit_memory.update_pose(pose)
        self.follower.corridor.update_pose(pose)
        self.pose, self.pose_time = pose, self.get_clock().now()

    def on_obstacles(self, msg):
        self.obstacles = np.array([[p.position.x, p.position.y] for p in msg.poses]).reshape(-1, 2)
        self.obstacles_time = self.get_clock().now()

    def exit_control(self):
        if self.active and not self.exiting:
            if not self._fresh(self.left_time) or not self._fresh(self.right_time):
                self.control()
            return
        if not self.active or not self.exiting:
            return
        if self.lane_ready and self._fresh(self.lane_ready_time):
            self.exit_done_pub.publish(Bool(data=True))
            return
        p = self.follower.p
        center = (self.exit_memory.remaining(self.pose, p.rear_axle_x)
                  if self.pose is not None and self._fresh(self.pose_time) else np.zeros((0, 2)))
        target = look_ahead_point(center, p) if len(center) else None
        usable = (target is not None and target[0] > 0 and self._fresh(self.obstacles_time)
                  and clearance(center, self.obstacles) >= self.get_parameter('exit_clearance').value)
        if not usable:
            # Keep the cone MUX input stopped rather than fall through to an unready lane.
            self.cmd_pub.publish(Twist())
            self.center_pub.publish(to_path(center, Header(frame_id='base_link')))
            self.get_logger().warn('cone EXIT: no fresh clear forward path; waiting for lane',
                                   throttle_duration_sec=1.0)
            return
        self.publish_result(FollowResult(p.speed, compute_steering(center, p),
                                         center=center, target=target))

    def _fresh(self, stamp) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return 0 <= age <= self.follower.p.boundary_timeout

    def control(self):
        if self.exiting:
            return
        # The course starts with both boundaries on their way, one arrives first. A side not received
        # yet is not "lost": without this the first command of every course is a reverse.
        if self.left_time is None or self.right_time is None:
            return
        left_fresh, right_fresh = self._fresh(self.left_time), self._fresh(self.right_time)
        if left_fresh and right_fresh and self.left_scan_stamp != self.right_scan_stamp:
            # Both messages use the same scan header. Do not train width using
            # one boundary from before the vehicle moved and one from after.
            return
        pose = self.pose if self._fresh(self.pose_time) else None
        obstacles = self.obstacles if self._fresh(self.obstacles_time) else None
        result = self.follower.decide(self.left, self.right, left_fresh, right_fresh,
                                      pose, obstacles, self.get_parameter('exit_clearance').value)
        if result is None:
            self.cmd_pub.publish(Twist())
            return
        if result.reverse:
            self.get_logger().warn('one boundary only, reversing', throttle_duration_sec=1.0)

        if result.source != 'observed':
            self.get_logger().info('cone path continuity: '+result.source, throttle_duration_sec=1.0)
        if result.source == 'observed' and not result.reverse and len(result.center) and self.pose is not None and self._fresh(self.pose_time):
            self.exit_memory.remember(result.center, self.pose, self.follower.p.lfd,
                                      self.follower.p.centerline_ds)
        self.publish_result(result)

    def publish_result(self, result):
        cmd = Twist()
        cmd.linear.x = float(result.speed)
        cmd.angular.z = float(
            result.speed * math.tan(result.steering) / self.follower.p.wheelbase)
        self.cmd_pub.publish(cmd)

        if len(result.center):
            header = Header(frame_id='base_link')
            header.stamp = self.get_clock().now().to_msg()
            self.center_pub.publish(to_path(result.center, header))
        if result.target is not None:
            target = PointStamped()
            target.header.stamp = self.get_clock().now().to_msg()
            target.header.frame_id = 'base_link'
            target.point.x, target.point.y = float(result.target[0]), float(result.target[1])
            self.target_pub.publish(target)


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
