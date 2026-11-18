#!/usr/bin/env python3
"""Rubber cone driving node (ROS2 port of 2025 `rubber_cone_node`).

Publishes a Twist on `cmd_topic` (twist_mux input `cone`, 90). Steering from pure pursuit is
converted with the bicycle model: w = v * tan(delta) / wheelbase. Normal cone
sorting, pursuit and boundary-loss response match main. Exit uses separate
rear-axle pursuit parameters and the retained measured path.
With exit_from_camera, follow the live lane arc until LaneMain confirms reacquisition.
In the legacy mode, after rear-axle exit arrival, retain priority while connecting to freshly
detected lane steering; release only after the IMU / speed response agrees.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseArray, Twist, TwistStamped
from nav_msgs.msg import Path, Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Header

from clothoid_common.quiet_node import QuietEventsMixin
from clothoid_lidar.cone_follower import ConeFollower, FollowerParams, FollowResult
from clothoid_lidar.cone_exit_pursuit import ExitPursuitParams, steering_from_target, look_ahead_point
from clothoid_lidar.cone_exit import ExitPathMemory, clearance
from clothoid_lidar.cone_sorter_node import to_path
from clothoid_lidar.cone_odometry import ConeOdometry
from clothoid_lidar.cone_lane_connection import (ConnectionParams, LaneConnection,
                                               central_limit, forward_arc)


def path_to_array(msg: Path) -> np.ndarray:
    return np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses]).reshape(-1, 2)


class ConeFollowerNode(QuietEventsMixin, Node):
    def __init__(self):
        super().__init__('cone_follower')

        self.declare_parameter('cmd_topic', '/cmd_vel_cone')
        self.declare_parameter('exit_log_repeat', 10.0)   # s between repeats of one exit status line
        self.declare_parameter('left_boundary_topic', '/cone/left_boundary')
        self.declare_parameter('right_boundary_topic', '/cone/right_boundary')
        self.declare_parameter('mission_active_topic', '/cone/mission_active')
        self.declare_parameter('centerline_topic', '/cone/centerline')
        self.declare_parameter('target_topic', '/cone/target')
        self.declare_parameter('imu_topic', '/imu_25hz')
        self.declare_parameter('odom_topic', '/odom_25hz')
        self.declare_parameter('pose_topic', '/cone/odom')

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
        self.declare_parameter('exit_from_camera', False)
        self.declare_parameter('exit_speed', .25)
        self.declare_parameter('exit_lfd', .35)
        self.declare_parameter('exit_clearance', 0.2)  # path to detected obstacle centre, metres
        for name, value in vars(ConnectionParams()).items():
            self.declare_parameter('exit_connect_' + name, value)

        self.follower = ConeFollower(self._build(lambda n: self.get_parameter(n).value))
        self.exit_params = self._build_exit(lambda n: self.get_parameter(n).value)
        self.odometry = ConeOdometry(self.follower.p.rear_axle_x, self.follower.p.boundary_timeout)
        self.pose_generation = self.odometry.generation
        self.active = False
        self.exiting = False
        self.exit_logged = None
        self.exit_memory = ExitPathMemory()
        self.connection = LaneConnection(self._connection_params(lambda n: self.get_parameter(n).value))
        self.lane_candidate = self.lane_candidate_time = self.lane_candidate_stamp = None
        self.last_command_steering = 0.
        self.pose = None
        self.pose_time = None
        self.pose_header_age = None
        self.odom_rx_interval = None
        self.last_odom_rx = None
        self.obstacles = np.zeros((0, 2))
        self.obstacles_time = None
        self.lane_ready = False
        self.lane_ready_time = None
        self.left = self.right = np.zeros((0, 2))
        self.left_time = self.right_time = None
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
        self.connecting_pub = self.create_publisher(Bool, '/cone/connecting', 1)
        self.create_subscription(Bool, '/cone/exiting', self.on_exiting, 1)
        self.create_subscription(Bool, '/lane/cone_exit_ready', self.on_lane_ready, 1)
        self.create_subscription(TwistStamped, '/lane/cone_exit_candidate', self.on_lane_candidate, 1)
        # Pose is a high-rate sensor input. Prefer its latest sample rather
        # than waiting for retransmission of a lost reliable sample.
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.on_odom,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Imu, self.get_parameter('imu_topic').value, self.on_imu,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(PoseArray, '/lidar/obstacles', self.on_obstacles, 1)
        self.create_timer(self.control_period, self.exit_control)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.get_logger().info('cone_follower started: local pose from IMU gyro + rear-axle wheel speed')

    @staticmethod
    def _build(value) -> FollowerParams:
        return FollowerParams(
            speed=value('speed'), reverse_speed=value('reverse_speed'),
            reverse_steer_deg=value('reverse_steer_deg'), wheelbase=value('wheelbase'),
            rear_axle_x=value('rear_axle_x'), lfd=value('lfd'), centerline_ds=value('centerline_ds'),
            lsq_window_size=value('lsq_window_size'), boundary_timeout=value('boundary_timeout'))

    @staticmethod
    def _build_exit(value) -> ExitPursuitParams:
        params = ExitPursuitParams(speed=value('exit_speed'), lfd=value('exit_lfd'),
            wheelbase=value('wheelbase'), rear_axle_x=value('rear_axle_x'),
            centerline_ds=value('centerline_ds'), boundary_timeout=value('boundary_timeout'))
        central_limit(params.wheelbase)
        return params

    def on_parameters(self, params):
        proposed = {p.name: p.value for p in params}
        try:
            candidate = self._build(lambda n: proposed.get(n, self.get_parameter(n).value))
            exit_params = self._build_exit(lambda n: proposed.get(n, self.get_parameter(n).value))
            estimator = ConeOdometry(candidate.rear_axle_x, candidate.boundary_timeout)
            connection_params = self._connection_params(
                lambda n: proposed.get(n, self.get_parameter(n).value))
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        if candidate.rear_axle_x != self.follower.p.rear_axle_x:
            self.odometry = estimator
            self._reset_pose_memory()
        else:
            self.odometry.timeout = candidate.boundary_timeout
        if exit_params != self.exit_params:
            self.exit_memory.clear()
            self.reset_connection()
        self.follower.p = candidate
        self.exit_params = exit_params
        self.connection.p = connection_params
        return SetParametersResult(successful=True)

    @staticmethod
    def _connection_params(value):
        return ConnectionParams(**{n: value('exit_connect_' + n)
                                   for n in ConnectionParams.__dataclass_fields__})

    def reset_connection(self):
        self.connection.clear()
        self.lane_candidate = self.lane_candidate_time = self.lane_candidate_stamp = None
        self.connecting_pub.publish(Bool(data=False))

    def on_left(self, msg: Path):
        self.left, self.left_time = path_to_array(msg), self.get_clock().now()
        if self.active and not self.exiting:
            self.control()

    def on_right(self, msg: Path):
        self.right, self.right_time = path_to_array(msg), self.get_clock().now()
        if self.active and not self.exiting:
            self.control()

    def on_active(self, msg: Bool):
        was_active, self.active = self.active, msg.data
        if self.active and not was_active:
            self.reset_connection()
            self.exit_memory.clear()
            self.exit_logged = None
            p = self.follower.p
            self.get_logger().info(f'cone mission START (lfd={p.lfd:.2f}, speed={p.speed:.2f}, '
                                   f'boundary_timeout={p.boundary_timeout:.3f}); main pursuit, safe exit')
            if self.left_time is not None or self.right_time is not None:
                self.control()
        elif was_active and not self.active:
            self.reset_connection()
            self.exiting = False
            self.exit_memory.clear()
            if self.center_pub.get_subscription_count():
                self.center_pub.publish(to_path(np.zeros((0, 2)), Header(frame_id='base_link')))
            self.get_logger().info('cone mission END')
            self.left = self.right = np.zeros((0, 2))
            self.left_time = self.right_time = None

    def on_exiting(self, msg):
        if self.exiting and not msg.data:
            self.reset_connection()
        if msg.data and not self.exiting:
            self.lane_ready = False
            self.lane_ready_time = None
        self.exiting = msg.data

    def on_lane_ready(self, msg):
        self.lane_ready = msg.data
        self.lane_ready_time = self.get_clock().now()

    def on_lane_candidate(self, msg):
        now = self.get_clock().now()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = now.nanoseconds * 1e-9 - stamp
        if (msg.header.frame_id != 'base_link' or not 0. <= age <= self.follower.p.boundary_timeout
                or (self.lane_candidate_stamp is not None and stamp <= self.lane_candidate_stamp)
                or not all(math.isfinite(v) for v in (msg.twist.linear.x, msg.twist.angular.z))
                or msg.twist.linear.x < 0.):
            return
        self.lane_candidate = msg.twist
        self.lane_candidate_time, self.lane_candidate_stamp = now, stamp

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
        if not self.get_parameter('exit_from_camera').value:
            self.reset_connection()
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

    def on_obstacles(self, msg):
        self.obstacles = np.array([[p.position.x, p.position.y] for p in msg.poses]).reshape(-1, 2)
        self.obstacles_time = self.get_clock().now()

    def exit_control(self):
        if not self.active or not self.exiting:
            return
        p = self.exit_params
        pose = self.pose if self.pose is not None and self._fresh(self.pose_time) else None
        lane_ready = self.lane_ready and self._fresh(self.lane_ready_time)
        if self.get_parameter('exit_from_camera').value:
            self.follow_camera_exit(lane_ready)
            return
        arrived = pose is not None and self.exit_memory.handover_ready(pose, p.rear_axle_x)
        if self.connection.active or arrived:
            self.connect_lane(pose, lane_ready)
            return
        if lane_ready and pose is not None:
            remaining = self.exit_memory.rear_axle_remaining(pose, p.rear_axle_x)
            if remaining is not None and remaining > 0.:
                self._exit_log('lane ready', f'cone EXIT: lane ready; observed endpoint {remaining:.3f} m '
                               'ahead of rear axle')
        center = (self.exit_memory.remaining(pose, p.rear_axle_x)
                  if pose is not None else np.zeros((0, 2)))
        target = look_ahead_point(center, p) if len(center) else None
        usable = (target is not None and target[0] > 0 and self._fresh(self.obstacles_time)
                  and clearance(center, self.obstacles) >= self.get_parameter('exit_clearance').value)
        if not usable:
            # Keep the cone MUX input stopped rather than fall through to an unready lane.
            self.publish_stop()
            if self.center_pub.get_subscription_count():
                self.center_pub.publish(to_path(center, Header(frame_id='base_link')))
            message = ('cone EXIT: no observed endpoint; waiting for measured path'
                       if self.exit_memory.observed_end is None else
                       'cone EXIT: no fresh clear forward path; waiting for lane')
            self._exit_log(message, message, warn=True)
            return
        self.publish_result(FollowResult(p.speed, steering_from_target(target, p),
                                         center=center, target=target))

    def follow_camera_exit(self, lane_ready):
        # The rising edge resets LaneMain's boundary tracks once per exit attempt.
        self.connecting_pub.publish(Bool(data=True))
        p = self.exit_params
        now = self.get_clock().now().nanoseconds * 1e-9
        if (self.lane_candidate is None or not self._fresh(self.lane_candidate_time)
                or not 0. <= now - self.lane_candidate_stamp <= p.boundary_timeout
                or self.lane_candidate.linear.x <= 0.):
            self.publish_stop()
            self._exit_log('camera wait', 'cone EXIT: waiting for fresh camera lane')
            return
        candidate = self.lane_candidate
        limit = central_limit(p.wheelbase)
        steering = float(np.clip(math.atan2(p.wheelbase * candidate.angular.z,
                                            candidate.linear.x), -limit, limit))
        arc = forward_arc(steering, p.wheelbase, p.rear_axle_x, p.lfd, p.centerline_ds)
        if (not self._fresh(self.obstacles_time)
                or clearance(arc, self.obstacles) < self.get_parameter('exit_clearance').value):
            self.publish_stop()
            self._exit_log('camera blocked', 'cone EXIT: camera lane arc blocked or LiDAR stale')
            return
        speed = min(candidate.linear.x, p.speed, self.connection.p.speed)
        self.publish_result(FollowResult(speed, steering, center=arc))
        self._exit_log('camera lane', 'cone EXIT: following detected camera lane')
        if lane_ready:
            self.exit_done_pub.publish(Bool(data=True))

    def _exit_log(self, key, message, warn=False):
        """Exit-phase status: at once when it changes, then every exit_log_repeat s while it holds
        (1116: a 1 s throttle repeated one waiting line for up to 190 s after the course)."""
        now = self.get_clock().now().nanoseconds * 1e-9
        last = self.exit_logged
        if last is not None and last[0] == key and now - last[1] < self.get_parameter('exit_log_repeat').value - 1e-6:
            return
        self.exit_logged = (key, now)
        # one call site per severity: rclpy refuses a call site that changes severity (ValueError
        # 'Logger severity cannot be changed between calls'), which killed the node at every cone exit
        if warn:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def publish_stop(self):
        self.last_command_steering = 0.
        self.cmd_pub.publish(Twist())

    def connect_lane(self, pose, lane_ready):
        p = self.exit_params
        now = self.get_clock().now().nanoseconds * 1e-9
        candidate_fresh = (self.lane_candidate is not None and self._fresh(self.lane_candidate_time)
                           and 0. <= now - self.lane_candidate_stamp <= p.boundary_timeout)
        if pose is None or not candidate_fresh or not self._fresh(self.obstacles_time):
            if self.connection.active:
                self.connection.pause(now)
            self.publish_stop()
            return
        if not self.connection.active:
            self.connection.start(now, pose, self.last_command_steering, p.wheelbase)
            self.get_logger().info('cone EXIT: rear axle arrived; connecting to live lane at low speed')
        self.connecting_pub.publish(Bool(data=True))
        measured_speed = self.odometry.speed
        measured_yaw_rate = self.odometry.gyro - self.odometry.bias
        speed, steering, ready = self.connection.step(
            now, pose, self.lane_candidate.linear.x, self.lane_candidate.angular.z,
            p.wheelbase, p.speed, measured_speed, measured_yaw_rate,
            lane_ready, p.boundary_timeout)
        # Check both the requested turn and the measured turn while the steering
        # catches up. This uses the same fresh obstacle clearance as cone pursuit.
        measured = (math.atan(p.wheelbase * measured_yaw_rate / measured_speed)
                    if measured_speed > .03 else 0.)
        paths = [forward_arc(angle, p.wheelbase, p.rear_axle_x, p.lfd, p.centerline_ds)
                 for angle in (steering, measured)]
        clear = all(clearance(path, self.obstacles) >= self.get_parameter('exit_clearance').value
                    for path in paths)
        if speed <= 0. or not clear:
            self.connection.pause(now)
            self.publish_stop()
            self._exit_log('connection stopped',
                           'cone EXIT: lane connection stopped (stale, blocked or bounded out)', warn=True)
            return
        self.publish_result(FollowResult(speed, steering, center=paths[0]))
        if ready:
            # Keep publishing through the acknowledgement, so the last priority
            # command and the lane command describe the same steering arc.
            self.exit_done_pub.publish(Bool(data=True))

    def _fresh(self, stamp) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return 0 <= age <= self.follower.p.boundary_timeout

    def control(self):
        if self.exiting:
            return
        # Preserve main's normal callback and stale-boundary behaviour. Exit
        # observers record the resulting path without changing the command.
        if self.left_time is None or self.right_time is None:
            return
        now = self.get_clock().now()
        fresh = lambda stamp: (now - stamp).nanoseconds * 1e-9 <= self.follower.p.boundary_timeout
        left_fresh, right_fresh = fresh(self.left_time), fresh(self.right_time)
        if not left_fresh and not right_fresh:
            self.get_logger().warn('all boundaries stale, no command', throttle_duration_sec=1.0)
            return
        result = self.follower.decide(self.left, self.right, left_fresh, right_fresh)
        if result is None:
            return
        if result.reverse:
            self.get_logger().warn('one boundary only, reversing', throttle_duration_sec=1.0)
        if (not self.get_parameter('exit_from_camera').value and not result.reverse and len(result.center)
                and self.pose is not None and self._fresh(self.pose_time)):
            self.exit_memory.remember(result.center, self.pose, self.exit_params.lfd,
                                      self.exit_params.centerline_ds)
        self.publish_result(result)

    def publish_result(self, result):
        self.last_command_steering = (float(np.clip(result.steering,
            -central_limit(self.follower.p.wheelbase), central_limit(self.follower.p.wheelbase)))
            if result.speed > 0. else 0.)
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
