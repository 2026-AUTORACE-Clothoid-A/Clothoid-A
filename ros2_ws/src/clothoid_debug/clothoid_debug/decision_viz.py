#!/usr/bin/env python3
"""What the nodes saw and decided, for RViz (rviz/decision.rviz). Works live and on a replayed bag.

Only listens to the real nodes (`lane_main` with debug on, stopline, speed_zone, cone_*); it runs no
detection of its own and publishes no drive command. The winning input is worked out here with the
priorities and timeouts of `twist_mux.yaml`, because twist_mux is not running during a bag replay.

The pose in frame `track` starts at (0, 0) and integrates /imu yaw and the /odom wheel speed: no
localization, no correction, it drifts. /odom yaw is not used because it is computed from the steering
value and does not match the real rotation (2026-11-05 bags: about 1.9 x gyro in turns, 287-354 deg
against 450 deg for a whole run). This TF needs base_link to have no other parent, so do not use it
with `use_ekf:=true` or `pub_odom_tf:=true`. In the simulator `use_odom_pose` is true: it publishes
odom -> base_link and a true /odom pose, so the trail uses that and `track` is fixed to `odom`.
"""
import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, PolygonStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time as RosTime
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Bool, ColorRGBA, Float32
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from clothoid_common.image_utils import bgr_to_image, image_to_bgr
from clothoid_debug.geometry import GroundProjection, arc_points
from clothoid_lane.lane_detector import LaneParams, make_warp_matrix

# twist_mux.yaml: name, topic, timeout, highest priority first
MUX_INPUTS = (('state', '/cmd_vel_state', 0.3), ('parking', '/cmd_vel_parking', 0.3),
              ('avoid', '/cmd_vel_avoid', 0.3), ('lane', '/cmd_vel_lane', 0.3))

# trail colour: which input was deciding at that point (None = no node sent a command)
COLORS = {'lane': (0.1, 0.9, 0.2), 'avoid': (0.1, 0.8, 1.0), 'parking': (1.0, 0.8, 0.0),
          'state': (1.0, 0.1, 0.1), None: (0.75, 0.2, 1.0)}


def rgba(rgb, a=1.0) -> ColorRGBA:
    return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(a))


class DecisionViz(Node):
    def __init__(self):
        super().__init__('decision_viz')
        d = LaneParams()
        for name, value in (('warp_x_h', d.x_h), ('warp_x_l', d.x_l), ('warp_y_h', d.y_h),
                            ('warp_y_l', d.y_l)):
            self.declare_parameter(name, value)
        self.declare_parameter('use_odom_pose', False)
        self.declare_parameter('camera_height', 0.187)   # lens above the floor, measured on limo1
        # horizon of the warp trapezoid is at row 178, cy 217, fy 489 -> atan(39 / 489)
        self.declare_parameter('camera_pitch', 0.080)
        self.declare_parameter('camera_x', 0.23)
        self.declare_parameter('camera_y', 0.05)
        self.declare_parameter('wheelbase', 0.2)
        self.declare_parameter('rear_axle_x', -0.1)
        self.declare_parameter('fx', 489.14)
        self.declare_parameter('fy', 489.14)
        self.declare_parameter('cx', 322.0)
        self.declare_parameter('cy', 217.0)
        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self.wheelbase, self.rear_axle_x = get('wheelbase'), get('rear_axle_x')
        self.use_odom_pose = get('use_odom_pose')
        self.warp_params = LaneParams(x_h=get('warp_x_h'), x_l=get('warp_x_l'),
                                      y_h=get('warp_y_h'), y_l=get('warp_y_l'))
        self.projection = GroundProjection(
            make_warp_matrix(self.warp_params), (get('fx'), get('fy'), get('cx'), get('cy')),
            get('camera_height'), get('camera_pitch'), get('camera_x'), get('camera_y'))

        self.frames = {}          # image stamp -> latest camera frames, to match the lane result
        self.left, self.right = [], []
        self.target = None
        self.cone_target = None   # (x, y, time) from cone_follower, base_link
        self.cmd = {}             # input name -> (Twist, time)
        self.stopline = self.cone_active = False
        self.limit_time = None
        self.x = self.y = self.yaw = self.speed = 0.0
        self.last_imu = self.last_odom = None
        self.pose_stamp = RosTime()
        self.trail = []           # (x, y, deciding input)

        self.overlay_pub = self.create_publisher(Image, '/viz/overlay', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/viz/markers', 1)
        self.trail_pub = self.create_publisher(Marker, '/viz/trail', 1)   # its own RViz display
        self.tf_pub = TransformBroadcaster(self)

        self.create_subscription(Image, '/camera/color/image_raw', self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(PolygonStamped, '/lane/left_warp', self.on_left, 5)
        self.create_subscription(PolygonStamped, '/lane/right_warp', self.on_right, 5)
        self.create_subscription(PointStamped, '/lane/target_warp', self.on_target, 5)
        self.create_subscription(PointStamped, '/cone/target', self.on_cone_target, 5)
        self.create_subscription(Imu, '/imu', self.on_imu, 50)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(Bool, '/stopline_detected', self.on_stopline, 5)
        self.create_subscription(Bool, '/cone/mission_active', self.on_cone, 5)
        self.create_subscription(Float32, '/speed_limit', self.on_limit, 5)
        for name, topic, _ in MUX_INPUTS:
            self.create_subscription(Twist, topic, lambda m, n=name: self.on_cmd(n, m), 5)
        self.create_timer(0.1, self.publish_markers)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- inputs -----------------------------------------------------------------
    def on_image(self, msg):
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.frames[key] = msg
        while len(self.frames) > 5:
            del self.frames[next(iter(self.frames))]

    def on_left(self, msg):
        self.left = [(p.x, p.y) for p in msg.polygon.points]

    def on_right(self, msg):
        self.right = [(p.x, p.y) for p in msg.polygon.points]

    def on_target(self, msg):
        self.target = msg
        frame = self.frames.get((msg.header.stamp.sec, msg.header.stamp.nanosec))
        if frame is not None:
            self.publish_overlay(frame, msg)

    def on_cone_target(self, msg):
        self.cone_target = (msg.point.x, msg.point.y, self.now())

    def on_cmd(self, name, msg):
        self.cmd[name] = (msg, self.now())

    def on_stopline(self, msg):
        self.stopline = msg.data

    def on_cone(self, msg):
        self.cone_active = msg.data

    def on_limit(self, msg):
        self.limit_time = self.now()

    def on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_imu is not None and 0.0 < t - self.last_imu < 1.0:
            self.yaw += msg.angular_velocity.z * (t - self.last_imu)
        self.last_imu = t

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.speed = msg.twist.twist.linear.x
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        if self.use_odom_pose:
            self.x, self.y = msg.pose.pose.position.x, msg.pose.pose.position.y
            tf.header.frame_id, tf.child_frame_id = 'track', 'odom'
            tf.transform.rotation.w = 1.0
        else:
            if self.last_odom is not None and 0.0 < t - self.last_odom < 1.0:
                self.x += self.speed * math.cos(self.yaw) * (t - self.last_odom)
                self.y += self.speed * math.sin(self.yaw) * (t - self.last_odom)
            tf.header.frame_id, tf.child_frame_id = 'track', 'base_link'
            tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
            tf.transform.rotation.z = math.sin(self.yaw / 2.0)
            tf.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.last_odom = t
        self.tf_pub.sendTransform(tf)
        self.pose_stamp = msg.header.stamp

        if not self.trail or math.hypot(self.x - self.trail[-1][0], self.y - self.trail[-1][1]) > 0.03:
            self.trail.append((self.x, self.y, self.winner()[0]))

    # --- state ------------------------------------------------------------------
    def winner(self):
        """(input name, Twist) twist_mux would forward now, or (None, None)."""
        now = self.now()
        for name, _, timeout in MUX_INPUTS:
            if name in self.cmd and now - self.cmd[name][1] <= timeout:
                return name, self.cmd[name][0]
        return None, None

    def code_steer(self, cmd):
        if cmd is None or abs(cmd.linear.x) < 1e-3:
            return None
        return math.atan(self.wheelbase * cmd.angular.z / cmd.linear.x)

    def status(self):
        name, cmd = self.winner()
        steer = self.code_steer(cmd)
        if steer is None:
            decision = 'stop (speed 0)' if cmd is not None else 'no command'
        else:
            deg = math.degrees(steer)
            side = 'straight' if abs(deg) < 2.0 else 'left' if deg > 0.0 else 'right'
            decision = f'{cmd.linear.x:+.2f} m/s  steer {deg:+.0f} deg ({side})'
        lane = 'LOST' if self.target is not None and self.target.point.z < 0.5 else 'found'
        slow = self.limit_time is not None and self.now() - self.limit_time < 0.5
        return (f'{name or "none"}: {decision}\n'
                f'lane: {lane}   stopline: {"YES" if self.stopline else "no"}\n'
                f'slow: {"YES" if slow else "no"}   cone course: {"YES" if self.cone_active else "no"}')

    # --- camera overlay ---------------------------------------------------------
    def publish_overlay(self, frame, target):
        try:
            bgr = image_to_bgr(frame).copy()
        except ValueError:
            return
        p, project = self.warp_params, self.projection.warp_to_image
        h = p.warp_h
        corners = [project(0, 0), project(p.warp_w, 0), project(p.warp_w, h), project(0, h)]
        cv2.polylines(bgr, [np.array(corners, dtype=np.int32)], True, (255, 0, 255), 1)
        for points, color in ((self.left, (255, 150, 40)), (self.right, (0, 220, 255))):
            for u, v in points:
                x, y = project(u, v)
                cv2.circle(bgr, (int(x), int(y)), 4, color, -1)
        x, y = project(target.point.x, target.point.y)
        color = (0, 255, 0) if target.point.z > 0.5 else (0, 0, 255)
        cv2.drawMarker(bgr, (int(x), int(y)), color, cv2.MARKER_CROSS, 28, 3)
        for i, line in enumerate(self.status().split('\n')):
            cv2.putText(bgr, line, (6, 22 + i * 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
            cv2.putText(bgr, line, (6, 22 + i * 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        self.overlay_pub.publish(bgr_to_image(bgr, frame.header))

    # --- markers ----------------------------------------------------------------
    def marker(self, marker_id, kind, frame='base_link', scale=0.03, rgb=(1, 1, 1), a=1.0):
        m = Marker()
        m.header.frame_id = frame
        # the newest pose, which TF is known to hold whatever clock RViz runs on
        m.header.stamp = self.pose_stamp
        m.ns, m.id, m.type, m.action = 'decision', marker_id, kind, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color = rgba(rgb, a)
        return m

    def floor_points(self, warp_points):
        out = []
        for u, v in warp_points:
            try:
                x, y = self.projection.warp_to_floor(u, v)
            except ValueError:
                continue
            out.append(Point(x=x, y=y, z=0.01))
        return out

    def arc(self, marker_id, curvature, rgb):
        m = self.marker(marker_id, Marker.LINE_STRIP, scale=0.025, rgb=rgb)
        m.points = [Point(x=x, y=y, z=0.03) for x, y in arc_points(curvature, self.rear_axle_x)]
        return m

    def publish_markers(self):
        out = MarkerArray()

        left = self.marker(1, Marker.SPHERE_LIST, scale=0.035, rgb=(0.15, 0.6, 1.0))
        left.points = self.floor_points(self.left)
        right = self.marker(2, Marker.SPHERE_LIST, scale=0.035, rgb=(1.0, 0.85, 0.0))
        right.points = self.floor_points(self.right)
        out.markers += [left, right]

        # Big pink point: what the deciding node really follows. The lane target keeps being computed
        # during the cone course but steers nothing, so it shrinks to a small grey point.
        name = self.winner()[0]
        cone_fresh = self.cone_target is not None and self.now() - self.cone_target[2] <= 0.3
        follows_cones = name == 'avoid' and cone_fresh
        if self.target is not None:
            found = self.target.point.z > 0.5
            big = found and not follows_cones
            target = self.marker(3, Marker.SPHERE, scale=0.08 if big else 0.04,
                                 rgb=(1.0, 0.1, 0.9) if big else (0.4, 0.4, 0.4))
            points = self.floor_points([(self.target.point.x, self.target.point.y)])
            if points:
                target.pose.position = points[0]
                out.markers.append(target)
        cone = self.marker(8, Marker.SPHERE, scale=0.08, rgb=(1.0, 0.1, 0.9))
        if follows_cones:
            cone.pose.position.x, cone.pose.position.y = self.cone_target[:2]
            cone.pose.position.z = 0.03
        else:
            cone.action = Marker.DELETE
        out.markers.append(cone)

        _, cmd = self.winner()
        steer = self.code_steer(cmd)
        code_arc = self.arc(4, 0.0 if steer is None else math.tan(steer) / self.wheelbase,
                            (0.1, 1.0, 0.3))
        code_arc.action = Marker.DELETE if steer is None else Marker.ADD
        out.markers.append(code_arc)

        text = self.marker(6, Marker.TEXT_VIEW_FACING, scale=0.09)
        text.pose.position.x, text.pose.position.y, text.pose.position.z = -0.9, 0.0, 0.3
        text.text = self.status()
        out.markers.append(text)

        trail = self.marker(7, Marker.LINE_LIST, frame='track', scale=0.05)
        for a, b in zip(self.trail, self.trail[1:]):
            color = rgba(COLORS[b[2]])
            trail.points += [Point(x=a[0], y=a[1]), Point(x=b[0], y=b[1])]
            trail.colors += [color, color]
        self.trail_pub.publish(trail)
        self.marker_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DecisionViz()
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
