#!/usr/bin/env python3
"""Rubber cone driving node (ROS2 port of 2025 `rubber_cone_node`).

Publishes a Twist on `cmd_topic` (twist_mux input `cone`, 90). Steering from pure pursuit is
converted with the bicycle model: w = v * tan(delta) / wheelbase. A short reverse
recovery is allowed only when cones remain but no boundary path is usable, after
checking its body sweep against the fresh full laser scan.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseArray, Twist
from nav_msgs.msg import Path, Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Header
from tf2_ros import Buffer, TransformException, TransformListener

from clothoid_common.quiet_node import QuietEventsMixin
from clothoid_lidar.cone_follower import (ConeFollower, FollowerParams, FollowResult,
                                           steering_from_target, look_ahead_point)
from clothoid_lidar.cone_exit import ExitPathMemory, clearance
from clothoid_lidar.cone_sorter_node import to_path
from clothoid_lidar.cone_recovery import current_boundaries, recovery_confirmed, reverse_sweep_clear
from clothoid_lidar.cone_odometry import ConeOdometry


def path_to_array(msg: Path) -> np.ndarray:
    return np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses]).reshape(-1, 2)


class ConeFollowerNode(QuietEventsMixin, Node):
    def __init__(self):
        super().__init__('cone_follower')

        self.declare_parameter('cmd_topic', '/cmd_vel_cone')
        self.declare_parameter('left_boundary_topic', '/cone/left_boundary')
        self.declare_parameter('right_boundary_topic', '/cone/right_boundary')
        self.declare_parameter('mission_active_topic', '/cone/mission_active')
        self.declare_parameter('centerline_topic', '/cone/centerline')
        self.declare_parameter('target_topic', '/cone/target')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('pose_topic', '/cone/odom')

        d = FollowerParams()
        self.declare_parameter('wheelbase', d.wheelbase)
        self.declare_parameter('rear_axle_x', d.rear_axle_x)
        self.declare_parameter('speed', d.speed)
        self.declare_parameter('reverse_speed', d.reverse_speed)
        self.declare_parameter('reverse_steer_deg', d.reverse_steer_deg)
        self.declare_parameter('lfd', d.lfd)
        self.declare_parameter('steering_gain', d.steering_gain)
        self.declare_parameter('centerline_ds', d.centerline_ds)
        self.declare_parameter('lsq_window_size', d.lsq_window_size)
        self.declare_parameter('boundary_timeout', d.boundary_timeout)
        self.declare_parameter('corner_endpoint_enabled', d.corner_endpoint_enabled)
        self.declare_parameter('path_relaxation_enabled', d.path_relaxation_enabled)
        self.declare_parameter('exit_clearance', 0.2)  # path to detected obstacle centre, metres

        self.follower = ConeFollower(self._build(lambda n: self.get_parameter(n).value))
        self.odometry = ConeOdometry(self.follower.p.rear_axle_x, self.follower.p.boundary_timeout)
        self.pose_generation = self.odometry.generation
        self.active = False
        self.exiting = False
        self.exit_memory = ExitPathMemory()
        self.pose = None
        self.pose_time = None
        self.pose_header_age = None
        self.odom_rx_interval = None
        self.last_odom_rx = None
        self.obstacles = np.zeros((0, 2))
        self.obstacles_time = None
        self.obstacles_scan_stamp = self.previous_obstacles_stamp = self.start_scan_stamp = None
        self.scan = self.scan_time = self.laser_pose = None
        self.laser_frame = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lane_ready = False
        self.lane_ready_time = None
        self.left = self.right = np.zeros((0, 2))
        self.left_time = self.right_time = None
        self.left_scan_stamp = self.right_scan_stamp = None
        self.boundary_wait_since = None
        self.control_period = .05
        self.odom_received = 0

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_topic').value, 1)
        self.center_pub = self.create_publisher(
            Path, self.get_parameter('centerline_topic').value, 1)
        self.target_pub = self.create_publisher(
            PointStamped, self.get_parameter('target_topic').value, 1)
        self.pose_pub = self.create_publisher(Odometry, self.get_parameter('pose_topic').value, 1)
        self.create_subscription(
            Path, self.get_parameter('left_boundary_topic').value, self.on_left, 1)
        self.create_subscription(
            Path, self.get_parameter('right_boundary_topic').value, self.on_right, 1)
        self.create_subscription(
            Bool, self.get_parameter('mission_active_topic').value, self.on_active, 1)

        self.exit_done_pub = self.create_publisher(Bool, '/cone/exit_complete', 1)
        self.create_subscription(Bool, '/cone/exiting', self.on_exiting, 1)
        self.create_subscription(Bool, '/lane/cone_exit_ready', self.on_lane_ready, 1)
        # Pose is a high-rate sensor input. Prefer its latest sample rather
        # than waiting for retransmission of a lost reliable sample.
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.on_odom,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Imu, self.get_parameter('imu_topic').value, self.on_imu,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(PoseArray, '/lidar/obstacles', self.on_obstacles, 1)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(self.control_period, self.exit_control)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.get_logger().info('cone_follower started: local pose from IMU gyro + rear-axle wheel speed')

    @staticmethod
    def _build(value) -> FollowerParams:
        return FollowerParams(
            speed=value('speed'), reverse_speed=value('reverse_speed'),
            reverse_steer_deg=value('reverse_steer_deg'), wheelbase=value('wheelbase'),
            rear_axle_x=value('rear_axle_x'), lfd=value('lfd'), centerline_ds=value('centerline_ds'),
            steering_gain=value('steering_gain'),
            lsq_window_size=value('lsq_window_size'), boundary_timeout=value('boundary_timeout'),
            corner_endpoint_enabled=value('corner_endpoint_enabled'),
            path_relaxation_enabled=value('path_relaxation_enabled'))

    def on_parameters(self, params):
        proposed = {p.name: p.value for p in params}
        try:
            candidate = self._build(lambda n: proposed.get(n, self.get_parameter(n).value))
            estimator = ConeOdometry(candidate.rear_axle_x, candidate.boundary_timeout)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        if candidate.rear_axle_x != self.follower.p.rear_axle_x:
            self.odometry = estimator
            self._reset_pose_memory()
        else:
            self.odometry.timeout = candidate.boundary_timeout
        self.follower.p = candidate
        return SetParametersResult(successful=True)

    def on_left(self, msg: Path):
        self.left_scan_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.left, self.left_time = path_to_array(msg), self.get_clock().now()
        if self.active and not self.exiting:
            self.boundary_updated()

    def on_right(self, msg: Path):
        self.right_scan_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.right, self.right_time = path_to_array(msg), self.get_clock().now()
        if self.active and not self.exiting:
            self.boundary_updated()

    def boundary_updated(self):
        if self.left_scan_stamp is not None and self.left_scan_stamp == self.right_scan_stamp:
            self.boundary_wait_since = None
            self.control()
        elif self.boundary_wait_since is None:
            # The other message normally follows within milliseconds. Do not
            # publish a temporary zero/reverse from a half-updated scan.
            self.boundary_wait_since = self.get_clock().now()

    def on_active(self, msg: Bool):
        was_active, self.active = self.active, msg.data
        if self.active and not was_active:
            self.start_scan_stamp = self.obstacles_scan_stamp
            self.exit_memory.clear()
            self.follower.reset()
            self.boundary_wait_since = None
            p = self.follower.p
            self.get_logger().info(f'cone mission START (lfd={p.lfd:.2f}, speed={p.speed:.2f}, '
                                   f'gain={p.steering_gain:.2f}, boundary_timeout={p.boundary_timeout:.3f})')
            if self.left_time is not None or self.right_time is not None:
                self.boundary_updated()
        elif was_active and not self.active:
            self.exiting = False
            self.exit_memory.clear()
            self.follower.reset()
            if self.center_pub.get_subscription_count():
                self.center_pub.publish(to_path(np.zeros((0, 2)), Header(frame_id='base_link')))
            self.get_logger().info('cone mission END')
            self.left = self.right = np.zeros((0, 2))
            self.left_time = self.right_time = None
            self.left_scan_stamp = self.right_scan_stamp = None
            self.boundary_wait_since = None

    def on_exiting(self, msg):
        if msg.data and not self.exiting:
            self.lane_ready = False
            self.lane_ready_time = None
        self.exiting = msg.data

    def on_lane_ready(self, msg):
        self.lane_ready = msg.data
        self.lane_ready_time = self.get_clock().now()

    def on_odom(self, msg):
        self.odom_received += 1
        received = self.get_clock().now()
        self.odom_rx_interval = (None if self.last_odom_rx is None else
                                (received-self.last_odom_rx).nanoseconds*1e-9)
        self.last_odom_rx = received
        stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        self.pose_header_age = (received.nanoseconds-stamp)*1e-9 if stamp else None
        sensor_time = self._sensor_stamp(msg, received)
        if sensor_time is not None:
            self.odometry.odom(sensor_time, msg.twist.twist.linear.x)
        self._update_pose(received)
        if self.pose is not None and self.pose_pub.get_subscription_count():
            out = Odometry()
            out.header.stamp, out.header.frame_id = self.pose_time.to_msg(), 'cone_odom'
            out.child_frame_id = 'base_link'
            out.pose.pose.position.x, out.pose.pose.position.y = self.pose[:2]
            out.pose.pose.orientation.z = math.sin(self.pose[2]/2)
            out.pose.pose.orientation.w = math.cos(self.pose[2]/2)
            omega = self.odometry.gyro-self.odometry.bias
            out.twist.twist.linear.x = self.odometry.speed
            out.twist.twist.linear.y = self.odometry.rear_offset*omega
            out.twist.twist.angular.z = omega
            self.pose_pub.publish(out)

    def on_imu(self, msg):
        received = self.get_clock().now()
        sensor_time = self._sensor_stamp(msg, received)
        if sensor_time is not None:
            gyro = msg.angular_velocity.z if msg.angular_velocity_covariance[0] >= 0 else math.nan
            self.odometry.imu(sensor_time, gyro)
        self._update_pose(received)

    def _sensor_stamp(self, msg, received):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        stamp = stamp or received.nanoseconds*1e-9
        return stamp if -.02 <= received.nanoseconds*1e-9-stamp <= self.odometry.timeout else None

    def _reset_pose_memory(self):
        self.exit_memory.clear()
        self.follower.reset()
        self.pose = self.pose_time = None
        self.pose_generation = self.odometry.generation

    def _update_pose(self, received):
        if self.pose_generation != self.odometry.generation:
            self._reset_pose_memory()
            if self.active:
                self.get_logger().warn('cone local pose restarted after invalid or interrupted sensors; '
                                       'retained paths cleared', throttle_duration_sec=1.)
        self.pose = self.odometry.pose(received.nanoseconds*1e-9)
        if self.pose is None:
            self.pose_time = None
            return
        self.pose_time = rclpy.time.Time(nanoseconds=round(self.odometry.stamp*1e9),
                                       clock_type=received.clock_type)
        self.exit_memory.update_pose(self.pose)
        self.follower.corridor.update_pose(self.pose)

    def on_obstacles(self, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if self.active and self.start_scan_stamp is None:
            self.start_scan_stamp = stamp
        if stamp != self.obstacles_scan_stamp:
            self.previous_obstacles_stamp = self.obstacles_scan_stamp
            self.obstacles_scan_stamp = stamp
        self.obstacles = np.array([[p.position.x, p.position.y] for p in msg.poses]).reshape(-1, 2)
        self.obstacles_time = self.get_clock().now()

    def on_scan(self, msg):
        frame = msg.header.frame_id.lstrip('/')
        if frame != self.laser_frame or self.laser_pose is None:
            if frame in ('base_link', 'base_footprint'):
                self.laser_pose = (0., 0., 0.)
            else:
                try:
                    tr = self.tf_buffer.lookup_transform('base_link', frame, rclpy.time.Time()).transform
                except TransformException:
                    return
                q = tr.rotation
                self.laser_pose = (tr.translation.x, tr.translation.y,
                    math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            self.laser_frame = frame
        self.scan, self.scan_time = msg, self.get_clock().now()

    def exit_control(self):
        if self.active and not self.exiting:
            if (not self._fresh(self.left_time) or not self._fresh(self.right_time)
                    or self.left_scan_stamp != self.right_scan_stamp):
                if self.boundary_wait_since is not None:
                    waiting = (self.get_clock().now()-self.boundary_wait_since).nanoseconds*1e-9
                    if waiting < self.control_period/2:
                        return
                self.control()
            return
        if not self.active or not self.exiting:
            return
        p = self.follower.p
        pose = self.pose if self.pose is not None and self._fresh(self.pose_time) else None
        lane_ready = self.lane_ready and self._fresh(self.lane_ready_time)
        if pose is not None and lane_ready and self.exit_memory.handover_ready(
                pose, p.rear_axle_x):
            self.exit_done_pub.publish(Bool(data=True))
            return
        if lane_ready and pose is not None:
            remaining = self.exit_memory.rear_axle_remaining(pose, p.rear_axle_x)
            if remaining is not None and remaining > 0.:
                self.get_logger().info(f'cone EXIT: lane ready; observed endpoint {remaining:.3f} m ahead of rear axle',
                                       throttle_duration_sec=1.0)
        center = (self.exit_memory.remaining(pose, p.rear_axle_x)
                  if pose is not None else np.zeros((0, 2)))
        target = look_ahead_point(center, p) if len(center) else None
        usable = (target is not None and target[0] > 0 and self._fresh(self.obstacles_time)
                  and clearance(center, self.obstacles) >= self.get_parameter('exit_clearance').value)
        if not usable:
            # Keep the cone MUX input stopped rather than fall through to an unready lane.
            self.cmd_pub.publish(Twist())
            if self.center_pub.get_subscription_count():
                self.center_pub.publish(to_path(center, Header(frame_id='base_link')))
            message = ('cone EXIT: no observed endpoint; waiting for measured path'
                       if self.exit_memory.observed_end is None else
                       'cone EXIT: no fresh clear forward path; waiting for lane')
            self.get_logger().warn(message,
                                   throttle_duration_sec=1.0)
            return
        self.publish_result(FollowResult(p.speed, steering_from_target(target, p),
                                         center=center, target=target))

    def _fresh(self, stamp) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return 0 <= age <= self.follower.p.boundary_timeout

    def control(self):
        if self.exiting:
            return
        left_fresh, right_fresh = current_boundaries(
            self.left_scan_stamp, self.right_scan_stamp, self._fresh(self.left_time), self._fresh(self.right_time))
        confirmed = recovery_confirmed(self.left_scan_stamp, self.right_scan_stamp,
                                       self.previous_obstacles_stamp, left_fresh, right_fresh)
        # The start scan arms the mission without publishing boundaries. Give
        # that first complete scan a chance; this does not delay forward memory.
        started = (self.start_scan_stamp is not None and self.previous_obstacles_stamp is not None
                   and self.previous_obstacles_stamp > self.start_scan_stamp)
        pose = self.pose if self._fresh(self.pose_time) else None
        obstacles = self.obstacles if self._fresh(self.obstacles_time) else None
        result = self.follower.decide(self.left, self.right, left_fresh, right_fresh,
                                      pose, obstacles, self.get_parameter('exit_clearance').value,
                                      allow_reverse=confirmed and started)
        if result is None:
            self.cmd_pub.publish(Twist())
            return
        if result.reverse:
            remaining = self.follower.p.lfd-(self.follower.corridor.travelled-self.follower.reverse_start_travel)
            scan = self.scan
            clear = (scan is not None and self._fresh(self.scan_time) and self.laser_pose is not None
                     and reverse_sweep_clear(scan.ranges, scan.angle_min, scan.angle_increment,
                         scan.range_min, scan.range_max, self.laser_pose, result.steering,
                         self.follower.p.wheelbase, self.follower.p.rear_axle_x, remaining))
            if not clear:
                result = FollowResult(0., 0., source='recovery_blocked')
            else:
                self.get_logger().warn('cones detected but no usable boundary path: checked reverse recovery',
                                       throttle_duration_sec=1.0)
        elif result.source == 'no_boundary_path' and not confirmed and self.left_scan_stamp != self.right_scan_stamp:
            # The second message of a healthy scan is still on its way. Do not
            # send an intermediate stop/reverse between the two callbacks.
            return

        if result.source != 'observed':
            pose_age = None if self.pose_time is None else (self.get_clock().now()-self.pose_time).nanoseconds*1e-9
            self.get_logger().info('cone path continuity: '+result.source+
                                   f' (pose_age={pose_age}, odom_rx={self.odom_received}, '
                                   f'odom_interval={self.odom_rx_interval}, odom_header_age={self.pose_header_age}, '
                                   f'timeout={self.follower.p.boundary_timeout:.3f})', throttle_duration_sec=1.0)
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

        if (len(result.center) or result.speed <= 0.) and self.center_pub.get_subscription_count():
            header = Header(frame_id='base_link')
            header.stamp = self.get_clock().now().to_msg()
            self.center_pub.publish(to_path(result.center, header))
        if result.target is not None and self.target_pub.get_subscription_count():
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
