#!/usr/bin/env python3
"""Static obstacle mission node (3-lane section). Logic in static_avoid.py, design in
docs/STATIC_AVOID.md.

In:  limos_topic (PoseArray, base_link, from the `limo_extractor` instance), scan_topic, odom_topic
     (speed only), imu_topic (yaw), /cone/mission_active, /camera/red_ratio, parking_topic,
     /cmd_vel_state (mission stop commands), direction_topic,
     image_10hz_topic (default: the camera at 10 Hz from bringup use_image_10hz:=true, every frame) or,
     with image_source 'raw', image_topic (raw or .../compressed; every `image_every`-th frame), for the
     white entry line and the lane lines,
     island_topic (optional PointStamped in base_link: the island front from the lane tracker),
     /parameter_events (follows lane_main's white_s_max / white_v_min / hsv_lower / hsv_upper so
     venue tuning applies).
     Camera geometry: camera_height / camera_pitch / camera_x / camera_y from the lane package's
     per-car camera file (avoid.launch.py camera:=limo1|limo2).
Out: cmd_topic (/cmd_vel_avoid, 20 Hz while active, one Twist(0) when it ends),
     /avoid/active (Bool, every scan), /avoid/done (Bool, every scan: the section is behind us), /avoid/lane (Int32, 1..3 while active, 0 idle),
     debug: /avoid/path (Path), /avoid/cues; debug_heavy: /avoid/candidates (MarkerArray), /avoid/costmap (OccupancyGrid),
     /avoid/cues (MarkerArray: entry line, island and its source), all in base_link.
"""
import math
import time
from collections import deque
from dataclasses import fields

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseArray, PoseStamped, Twist
from rcl_interfaces.msg import ParameterEvent
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from clothoid_common.qos import IMAGE_QOS
from clothoid_common.quiet_node import QuietEventsMixin
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image, Imu, LaserScan
from std_msgs.msg import Bool, ColorRGBA, Float32, Header, Int32
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from clothoid_lidar.red_ratio_node import image_to_bgr
from clothoid_lidar.section_cues import (CameraModel, ColourParams, EntryLineTracker, WhiteParams,
                                         check_start_line, find_entry_line)
from clothoid_lidar.static_avoid import (ACTIVE, LETHAL, AvoidParams, Inhibit, LaneFitter,
                                         CourseMap, PoseTracker, StaticAvoid, course_start_line,
                                         section_to_base, yaw_from_quaternion)


class StaticAvoidNode(QuietEventsMixin, Node):
    def __init__(self):
        super().__init__('static_avoid')
        topics = {
            'limos_topic': '/lidar/limos', 'scan_topic': '/scan', 'odom_topic': '/odom',
            'imu_topic': '/imu', 'cmd_topic': '/cmd_vel_avoid',
            'cone_active_topic': '/cone/mission_active', 'red_ratio_topic': '/camera/red_ratio',
            'parking_topic': '/parking_flag', 'state_cmd_topic': '/cmd_vel_state',
            'direction_topic': '/direction_flag',
        }
        for name, value in topics.items():
            self.declare_parameter(name, value)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('use_red_ratio', False)
        self.declare_parameter('red_ratio_window', 0.5)   # s, max over this window
        self.declare_parameter('state_window', 0.5)       # s, a mission command this recent inhibits
        self.declare_parameter('scan_time_offset', 0.05)  # s, stamp is the start of the sweep
        self.declare_parameter('lidar_x', 0.103)          # used when TF is not available
        self.declare_parameter('lidar_y', 0.0)
        self.declare_parameter('debug', True)              # /avoid/path, /avoid/cues
        self.declare_parameter('debug_heavy', False)       # + /avoid/candidates, /avoid/costmap (rviz only)
        self.declare_parameter('load_log_period', 5.0)     # s, callback time log; 0 = off
        # camera cues
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('image_10hz_topic', '/camera/color/image_10hz')
        # '10hz': image_10hz_topic, every frame (the 30 Hz raw image costs a Python node 30~40 % of an
        # Orin core just to receive, docs/PIPELINE_LOAD_PLAN.md 0.1); 'raw': image_topic, every
        # image_every-th frame (bags, a car without the throttle)
        self.declare_parameter('image_source', '10hz')
        self.declare_parameter('image_every', 3)
        self.declare_parameter('ignore_cone', False)      # test track only: its cone wall is beside the section
        self.declare_parameter('island_topic', '')        # '' = no island signal from the lane side
        self.declare_parameter('lane_node', '/lane_main')  # whose white_* tuning to follow
        cam = CameraModel()
        for name, value in (('camera_height', cam.height), ('camera_pitch', cam.pitch),
                            ('camera_x', cam.x), ('camera_y', cam.y), ('fx', cam.fx),
                            ('fy', cam.fy), ('cx', cam.cx), ('cy', cam.cy)):
            self.declare_parameter(name, value)
        wp = WhiteParams()
        self.declare_parameter('white_s_max', wp.s_max)
        self.declare_parameter('white_v_min', wp.v_min)
        colours = ColourParams()
        self.declare_parameter('yellow_lower', list(colours.yellow_lower))
        self.declare_parameter('yellow_upper', list(colours.yellow_upper))
        d = AvoidParams()
        for f in fields(AvoidParams):
            value = getattr(d, f.name)
            self.declare_parameter(f.name, list(value) if isinstance(value, tuple) else value)
        self.p = AvoidParams(**{
            f.name: (tuple(self.get_parameter(f.name).value) if isinstance(getattr(d, f.name), tuple)
                     else self.get_parameter(f.name).value) for f in fields(AvoidParams)})

        self.avoid = StaticAvoid(self.p)
        self.tracker = PoseTracker()
        v = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cam = CameraModel(fx=v('fx'), fy=v('fy'), cx=v('cx'), cy=v('cy'),
                               height=v('camera_height'), pitch=v('camera_pitch'),
                               x=v('camera_x'), y=v('camera_y'))
        self.white = WhiteParams(s_max=int(v('white_s_max')), v_min=int(v('white_v_min')))
        self.colours = ColourParams(white_s_max=self.white.s_max, white_v_min=self.white.v_min)
        self.set_yellow(v('yellow_lower'), v('yellow_upper'))
        self.entry = EntryLineTracker()
        self.course = None
        self.snaps_logged = 0
        self.origin_logged = 'pending'
        self.line_start_odo = None          # odometer of the requested section start line
        self.blocker_fallback = False       # race start, the line start dropped: a blocker starts it
        if self.p.course_map:
            self.course = CourseMap(self.p)
            self.tracker.course = self.course
            self.entry.start_by_map = course_start_line(self.course, self.p)
        self.lane_fitter = LaneFitter(self.p, self.cam, self.colours)
        self.image_count = 0
        self.cone_active = False
        self.parking = False
        self.direction = None
        self.red = deque()          # (t, ratio)
        self.state_seen = None      # time of the last /cmd_vel_state message
        self.lidar_pose = None      # (x, y, yaw) of the scan frame in base_link
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cmd_pub = self.create_publisher(Twist, g('cmd_topic'), 1)
        self.active_pub = self.create_publisher(Bool, '/avoid/active', 1)
        self.lane_pub = self.create_publisher(Int32, '/avoid/lane', 1)
        self.done_pub = self.create_publisher(Bool, '/avoid/done', 1)
        self.path_pub = self.create_publisher(Path, '/avoid/path', 1)
        self.cand_pub = self.create_publisher(MarkerArray, '/avoid/candidates', 1)
        self.map_pub = self.create_publisher(OccupancyGrid, '/avoid/costmap', 1)
        self.cues_pub = self.create_publisher(MarkerArray, '/avoid/cues', 1)

        self.create_subscription(PoseArray, g('limos_topic'), self.timed('limos', self.on_limos), 1)
        self.create_subscription(LaserScan, g('scan_topic'), self.timed('scan', self.on_scan),
                                 qos_profile_sensor_data)
        # depth 10: while an image callback holds the executor, /odom and /imu pile up; depth 1 kept
        # only the last one (gaps of 0.2 s on the PC, 1 s on the car, so the pose at an image's time
        # was interpolated over the gap). Their callbacks only store (< 0.1 ms) and the commands come
        # from the timer, so a burst adds no delay. Keep it so: no control from these callbacks.
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 10)
        self.create_subscription(Imu, g('imu_topic'), self.on_imu, 10)
        self.create_subscription(Bool, g('cone_active_topic'), self.on_cone, 1)
        if g('use_red_ratio'):                # off since 11-07: no subscription (one less per wake-up)
            self.create_subscription(Float32, g('red_ratio_topic'), self.on_red, 5)
        self.create_subscription(Bool, g('parking_topic'), self.on_parking, 1)
        self.create_subscription(Twist, g('state_cmd_topic'), self.on_state_cmd, 5)
        self.create_subscription(Int32, g('direction_topic'), self.on_direction, 1)
        tenhz = g('image_source') == '10hz'
        image_topic = g('image_10hz_topic') if tenhz else g('image_topic')
        self.image_every = 1 if tenhz else max(1, int(g('image_every')))
        self.image_topic = image_topic
        self.last_image = None
        if image_topic.endswith('/compressed'):
            self.create_subscription(CompressedImage, image_topic, self.timed('image', self.on_image),
                                     IMAGE_QOS)
        elif image_topic:
            self.create_subscription(Image, image_topic, self.timed('image', self.on_image), IMAGE_QOS)
        if g('island_topic'):
            self.create_subscription(PointStamped, g('island_topic'), self.on_island, 1)
        self.create_subscription(ParameterEvent, '/parameter_events', self.on_param_event, 10)
        self.progress_from, self.no_progress = None, False   # (t, odometer) while commanding a speed
        self.load = {}                     # callback -> [count, total s, max s]
        self.odom_gap, self.last_odom_stamp = 0.0, None
        self.create_timer(0.05, self.timed('timer', self.on_timer))
        self.create_timer(0.5, self.timed('slow_debug', self.publish_costmap))
        self.create_timer(1.0, self.check_images)
        period = float(self.get_parameter('load_log_period').value)
        if period > 0.0:
            self.create_timer(period, self.log_load)
        self.get_logger().info(f'static_avoid started: lane width {self.p.lane_width:.2f} m, '
                               f'trigger {self.p.activate_range:.1f} m ahead, camera h '
                               f'{self.cam.height:.3f} pitch {self.cam.pitch:+.3f}, white S<='
                               f'{self.white.s_max} V>={self.white.v_min}')

    # ---- inputs -----------------------------------------------------------------------------
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def timed(self, name, callback):
        """callback wrapped to add its run time to self.load (see log_load)."""
        def run(*args):
            t = time.perf_counter()
            try:
                return callback(*args)
            finally:
                dt = time.perf_counter() - t
                c = self.load.setdefault(name, [0, 0.0, 0.0])
                c[0] += 1
                c[1] += dt
                c[2] = max(c[2], dt)
        return run

    def check_images(self):
        """No camera means no entry line and no lane fit: say so instead of failing silently."""
        t = self.now()
        if self.tracker.ready and (self.last_image is None or t - self.last_image > 3.0):
            self.get_logger().warn(f'no images on {self.image_topic} for 3 s (the 10 Hz topic needs '
                                   'bringup use_image_10hz:=true; else avoid.launch image:=raw)',
                                   throttle_duration_sec=10.0)

    def log_load(self):
        """One line per period: per callback count, mean and max run time, and the longest gap
        between the /odom stamps received (the node was busy if it is far above 0.02 s)."""
        period = float(self.get_parameter('load_log_period').value)
        parts = [f'{k} n{c[0]} avg {1000 * c[1] / max(c[0], 1):.0f}ms max {1000 * c[2]:.0f}ms'
                 for k, c in sorted(self.load.items())]
        self.get_logger().info(f'load {period:.0f}s ({self.avoid.state}): ' + ' | '.join(parts)
                               + f' | odom gap max {1000 * self.odom_gap:.0f}ms')
        self.load, self.odom_gap = {}, 0.0

    def stamp(self, msg) -> float:
        """Message time (the driver's stamp): bag playback's /clock ticks at 40 Hz, slower than
        /odom, so the node clock would merge odometry steps. Falls back to now() when unset."""
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        return t if t > 0.0 else self.now()

    def on_imu(self, msg: Imu):
        q = msg.orientation
        self.tracker.imu(self.stamp(msg), yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def on_odom(self, msg: Odometry):
        t = self.stamp(msg)
        if self.last_odom_stamp is not None:
            self.odom_gap = max(self.odom_gap, t - self.last_odom_stamp)
        self.last_odom_stamp = t
        p = msg.pose.pose.position
        self.tracker.odom(t, msg.twist.twist.linear.x, (p.x, p.y))
        self.maybe_reanchor()

    def maybe_reanchor(self):
        """Started before the entry line: once the base is on it, place the island from it. Only the
        distance along the road: the car is already steering, so its pose there is not the lane's."""
        if self.avoid.state != ACTIVE or self.avoid.island_source != 'estimate':
            return
        # the start line is confirmed up to 0.8 m after it (EntryLineTracker.is_start)
        c = self.entry.recent(self.tracker.distance, 1.0, 0.0, start_only=True)
        if c is not None:
            self.avoid.on_entry_crossing(self.tracker.at_distance(c.odometer))
            if self.avoid.island_source == 'entry line':
                self.get_logger().info(f'entry line crossed after the start: island '
                                       f'{self.avoid.island[0]:.2f} m')

    def on_cone(self, msg: Bool):
        self.cone_active = msg.data

    def on_parking(self, msg: Bool):
        self.parking = msg.data

    def on_direction(self, msg: Int32):
        self.direction = msg.data

    def on_red(self, msg: Float32):
        self.red.append((self.now(), float(msg.data)))

    def on_state_cmd(self, _msg: Twist):
        self.state_seen = self.now()

    def set_yellow(self, lower, upper):
        """lane_main's yellow, except main's 2025 value (S >= 215), which misses our tape."""
        if int(lower[1]) >= 200:
            self.get_logger().warn(f'yellow {list(lower)} is too strict for our tape: using '
                                   f'{ColourParams().yellow_lower}~{ColourParams().yellow_upper}')
            lower, upper = ColourParams().yellow_lower, ColourParams().yellow_upper
        self.colours.yellow_lower = tuple(int(x) for x in lower)
        self.colours.yellow_upper = tuple(int(x) for x in upper)

    def on_param_event(self, msg: ParameterEvent):
        """Follow the lane tracker's white and yellow thresholds when they are tuned at the venue."""
        if msg.node != self.get_parameter('lane_node').value:
            return
        for prm in list(msg.new_parameters) + list(msg.changed_parameters):
            if prm.name == 'white_s_max':
                self.white.s_max = self.colours.white_s_max = int(prm.value.integer_value)
            elif prm.name == 'white_v_min':
                self.white.v_min = self.colours.white_v_min = int(prm.value.integer_value)
            elif prm.name == 'hsv_lower':
                self.set_yellow(list(prm.value.integer_array_value), self.colours.yellow_upper)
            elif prm.name == 'hsv_upper':
                self.colours.yellow_upper = tuple(int(x) for x in prm.value.integer_array_value)
            else:
                continue
            self.get_logger().info(f'colours from {msg.node}: {self.colours}')

    def on_image(self, msg):
        self.image_count += 1
        self.last_image = self.now()
        if (self.image_count - 1) % self.image_every:
            return
        if not self.tracker.ready:
            return
        try:
            if isinstance(msg, CompressedImage):
                import cv2
                bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            else:
                bgr = image_to_bgr(msg)
        except Exception as exc:   # an odd encoding must not stop the mission
            self.get_logger().warn(f'image skipped: {exc}', throttle_duration_sec=5.0)
            return
        # where the car was when the frame was taken: a frame processed late (a busy node) must not be
        # placed at the present odometer (the entry line would look further on)
        t_img = self.stamp(msg)
        line = find_entry_line(bgr, self.cam, self.white)
        crossing = self.entry.update(t_img, self.tracker.distance_at(t_img), line)
        if crossing is not None:
            self.get_logger().info(f'white line: crossed at odometer {crossing.odometer:.2f} m')
        if self.course is not None:
            self.course.frozen = self.avoid.state == ACTIVE      # no heading snaps while avoiding
            for odo, deg in self.course.snaps[self.snaps_logged:]:
                if abs(deg) >= 1.0:
                    self.get_logger().info(f'course map: heading snapped by {deg:+.1f} deg on the '
                                           f'straight at odometer {odo:.1f} m')
            self.snaps_logged = len(self.course.snaps)
            why = self.course.should_fall_back(self.tracker.distance, self.entry)
            if why:
                self.entry.start_by_map = None
                self.get_logger().warn(f'course map: {why}; the dashed lane-line pair decides the '
                                       'section start line from now on')
            self.log_origin()
        for c in check_start_line(self.entry, bgr, self.cam, self.lane_fitter.colours,
                                  self.tracker.distance_at(t_img), self.p.lane_width):
            why = c.where or f'lane-line pair in {c.pairs} frames around it'
            self.get_logger().info(f'white line at odometer {c.odometer:.2f} m: '
                                   + ('SECTION START LINE' if c.start else 'not the section start')
                                   + f' ({why})')
            if c.start and self.course is not None and self.course.corner1 is not None:
                # race start: the start line starts the mission (on the next LIMO message)
                self.course.set_origin('race', 'the section start line where the map has it, after '
                                       f'the first right turn at map x {self.course.corner1[1]:.1f} m')
                self.log_origin()
                if self.course.origin == 'race':
                    self.avoid.request_line_start(self.tracker.line_pose(c.odometer))
                    self.line_start_odo = c.odometer
        had_event = self.avoid.event
        self.lane_fitter.on_frame(bgr, self.avoid, self.tracker, self.entry, t_img)
        if self.avoid.event != had_event and 'fit' in self.avoid.event:
            self.get_logger().info(f'{self.avoid.event}: island {self.avoid.island[0]:.2f} m')

    def log_origin(self):
        """The course map's judgement of where the node started, once when made."""
        if self.course is None or self.course.origin == self.origin_logged:
            return
        self.origin_logged = self.course.origin
        if self.course.origin == 'race':
            self.get_logger().info(f'course map: race start confirmed ({self.course.origin_why}); '
                                   'the mission starts on this line')
        else:
            self.entry.start_by_map = None
            self.get_logger().info(f'course map: not from the race start ({self.course.origin_why}); '
                                   'a blocker starts the mission, the dashed lane-line pair finds the '
                                   'section start line')

    def on_island(self, msg: PointStamped):
        if self.avoid.state != ACTIVE or not self.tracker.ready:
            return
        s, _, psi = self.avoid.lane_pose(self.tracker.latest())
        self.avoid.set_island_front(s + math.cos(psi) * msg.point.x - math.sin(psi) * msg.point.y,
                                    'lane tracker')

    def inhibit(self) -> Inhibit:
        t = self.now()
        while self.red and self.red[0][0] < t - self.get_parameter('red_ratio_window').value:
            self.red.popleft()
        red = max(r for _, r in self.red) if (self.red and self.get_parameter('use_red_ratio').value) \
            else None
        recent_state = (self.state_seen is not None
                        and t - self.state_seen <= self.get_parameter('state_window').value)
        cone = self.cone_active and not self.get_parameter('ignore_cone').value
        return Inhibit(cone_active=cone, red_ratio=red, parking=self.parking,
                       state_command=recent_state)

    def on_limos(self, msg: PoseArray):
        limos = (np.array([[p.position.x, p.position.y] for p in msg.poses])
                 if msg.poses else np.zeros((0, 2)))
        pose = self.tracker.latest() if self.tracker.ready else None
        c = self.entry.recent(self.tracker.distance, self.p.island_from_entry_max + 1.0, start_only=True)
        entry_back = self.tracker.distance - c.odometer if c is not None else None
        entry_pose = self.tracker.line_pose(c.odometer) if c is not None else None
        if self.avoid.line_start is not None and self.line_start_odo is not None:
            # the requested line itself, not looked up again (it may lie more than 1 m ahead)
            entry_back, entry_pose = self.tracker.distance - self.line_start_odo, self.avoid.line_start
        was_done = self.avoid.done
        had_line = self.avoid.line_start is not None
        # course map: a blocker starts the mission only away from the race start (see CourseMap); one
        # right after the launch is an avoid test launched in the section (nothing there at the race start)
        if (self.course is not None and self.course.origin == 'pending'
                and self.tracker.distance < self.p.course_test_start and self.avoid.blocked(limos)):
            self.course.set_origin('elsewhere', f'a LIMO in our lane {self.tracker.distance:.2f} m after '
                                   'the launch')
            self.log_origin()
        allowed = self.course is None or self.course.origin == 'elsewhere' or self.blocker_fallback
        started = self.avoid.on_limos(self.now(), limos, self.inhibit(), self.tracker.speed, pose,
                                      entry_back, entry_pose, allowed)
        if self.avoid.done and not was_done:
            self.get_logger().info(f'static obstacle DONE: {self.avoid.event}')
        if had_line and not started and self.avoid.line_start is None:
            self.get_logger().warn(f'static obstacle: {self.avoid.event}')
            # never left with no way to start (1114 23:39: the line start dropped, no blocker start)
            if not self.blocker_fallback and self.course is not None and self.course.origin == 'race':
                self.blocker_fallback = True
                self.get_logger().warn('course map: race start, but no start on the line; a blocker '
                                       'starts the mission from here')
        if started:
            how = ('section start line, at approach speed until a blocker'
                   if self.avoid.approach else 'blocked ahead')
            self.get_logger().info(f'static obstacle START ({how}) at odometer '
                                   f'{self.tracker.distance:.2f} m, island '
                                   f'{self.avoid.island[0]:.2f} m ({self.avoid.island_source})')

    def _lidar(self, scan: LaserScan):
        if self.lidar_pose is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.get_parameter('base_frame').value,
                                                     scan.header.frame_id, Time())
                q = tf.transform.rotation
                self.lidar_pose = (tf.transform.translation.x, tf.transform.translation.y,
                                   yaw_from_quaternion(q.x, q.y, q.z, q.w))
            except TransformException:
                return (self.get_parameter('lidar_x').value, self.get_parameter('lidar_y').value,
                        0.0)
        return self.lidar_pose

    def on_scan(self, scan: LaserScan):
        active = self.avoid.state == ACTIVE
        self.active_pub.publish(Bool(data=active))
        self.done_pub.publish(Bool(data=self.avoid.done))
        if not active or not self.tracker.ready:
            return
        lx, ly, lyaw = self._lidar(scan)
        r = np.asarray(scan.ranges, float)
        ang = scan.angle_min + scan.angle_increment * np.arange(r.size) + lyaw
        ok = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max)
        hits = np.column_stack((lx + r[ok] * np.cos(ang[ok]), ly + r[ok] * np.sin(ang[ok])))
        free = np.column_stack((np.cos(ang[~ok]), np.sin(ang[~ok])))
        t_scan = (Time.from_msg(scan.header.stamp).nanoseconds * 1e-9
                  + self.get_parameter('scan_time_offset').value)
        debug = self.get_parameter('debug').value
        heavy = self.get_parameter('debug_heavy').value
        self.avoid.on_scan(hits, np.array([lx, ly]), self.tracker.at(t_scan),
                           self.tracker.latest(), self.direction, keep_candidates=heavy,
                           free_dirs_base=free)
        if debug:
            self.publish_plan()

    # ---- control ----------------------------------------------------------------------------
    def on_timer(self):
        was_active = self.avoid.state == ACTIVE
        if not was_active or not self.tracker.ready:
            self.lane_pub.publish(Int32(data=0))
            return
        pose = self.tracker.latest()
        had_event = self.avoid.event
        cmd = self.avoid.control(pose, self.inhibit(), self.now())
        if self.avoid.event != had_event and self.avoid.event.startswith('sign look'):
            self.get_logger().info(self.avoid.event)
        elif self.avoid.event != had_event and self.avoid.event.startswith('stuck'):
            self.get_logger().warn(f'static obstacle {self.avoid.event}')
        if cmd is not None:
            self.watch_progress(cmd[0])
        if cmd is None:   # ended this cycle: one stop, then silence (lane_main / others take over)
            self.cmd_pub.publish(Twist())
            self.active_pub.publish(Bool(data=False))
            self.get_logger().info(f'static obstacle END: {self.avoid.event}')
            self.clear_debug()
            return
        out = Twist()
        out.linear.x, out.angular.z = float(cmd[0]), float(cmd[1])
        self.cmd_pub.publish(out)
        self.lane_pub.publish(Int32(data=self.avoid.lane(pose) or 0))

    def watch_progress(self, v: float):
        """Commanding a speed but not moving (a collision, wheels caught): one warning, until the car
        moves again. Mission commands only (lane_main's are not ours to judge)."""
        t, odo = self.now(), self.tracker.distance
        if abs(v) < 0.05:
            self.progress_from = None
            return
        if self.progress_from is None or abs(odo - self.progress_from[1]) >= 0.03:
            if self.no_progress:
                self.get_logger().info('static obstacle: moving again')
            self.progress_from, self.no_progress = (t, odo), False
            return
        if not self.no_progress and t - self.progress_from[0] >= 1.5:
            self.no_progress = True
            pose = self.avoid.lane_pose(self.tracker.latest())
            self.get_logger().warn(f'static obstacle: not moving (commanded {v:.2f} m/s, '
                                   f'{abs(odo - self.progress_from[1]):.2f} m in '
                                   f'{t - self.progress_from[0]:.1f} s; a collision?) at '
                                   + self.avoid.stuck_report(pose))

    # ---- debug ------------------------------------------------------------------------------
    def _header(self) -> Header:
        h = Header(frame_id=self.get_parameter('base_frame').value)
        h.stamp = self.get_clock().now().to_msg()
        return h

    def clear_debug(self):
        """RViz keeps drawing the last message: empty the path and markers when the mission ends."""
        self.path_pub.publish(Path(header=self._header()))
        for pub in (self.cand_pub, self.cues_pub):
            pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))

    def publish_plan(self):
        plan = self.avoid.plan
        if plan is None:
            return
        pose = self.avoid.lane_pose(self.tracker.latest())
        header = self._header()
        path = Path(header=header)
        for x, y in section_to_base(plan.path, pose):
            ps = PoseStamped(header=header)
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)
        if not self.get_parameter('debug_heavy').value:
            return

        out = MarkerArray()
        out.markers.append(Marker(action=Marker.DELETEALL))
        finite = [c for _, c in plan.candidates if math.isfinite(c)]
        lo, hi = (min(finite), max(finite)) if finite else (0.0, 1.0)
        for k, (cand, cost) in enumerate(plan.candidates):
            m = Marker(header=header, ns='candidates', id=k, type=Marker.LINE_STRIP)
            m.pose.orientation.w = 1.0
            m.scale.x = 0.006
            if math.isfinite(cost):
                f = (cost - lo) / max(hi - lo, 1e-6)
                m.color = ColorRGBA(r=float(f), g=float(1.0 - f), b=0.2, a=0.5)
            else:
                m.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.15)
            m.points = [Point(x=float(x), y=float(y), z=0.01)
                        for x, y in section_to_base(cand[::4], pose)]
            out.markers.append(m)
        self.cand_pub.publish(out)

    def publish_costmap(self):
        """2 Hz: the cues when debug, the costmap only when debug_heavy (45 000 cells as a Python
        list is slow on the Jetson)."""
        if self.avoid.state != ACTIVE or self.avoid.costmap is None or not self.tracker.ready:
            return
        if not self.get_parameter('debug_heavy').value:
            if self.get_parameter('debug').value:
                self.publish_cues(self.avoid.lane_pose(self.tracker.latest()))
            return
        cm = self.avoid.costmap
        s, d, psi = self.avoid.section_pose(self.tracker.latest())
        grid = OccupancyGrid(header=self._header())
        grid.info.resolution = float(self.p.resolution)
        grid.info.width, grid.info.height = cm.ns, cm.nd
        # cell (0, 0) of the section grid, expressed in base_link
        corner = section_to_base(np.array([[self.p.grid_s_min, -cm.d_max]]), (s, d, psi))[0]
        grid.info.origin.position.x, grid.info.origin.position.y = float(corner[0]), float(corner[1])
        grid.info.origin.orientation.z = math.sin(-psi / 2.0)
        grid.info.origin.orientation.w = math.cos(-psi / 2.0)
        cost = np.where(cm.cost >= LETHAL, 100, np.clip(cm.cost, 0, 99)).astype(np.int8)
        grid.data = cost.T.ravel().tolist()     # row-major, rows along d
        self.map_pub.publish(grid)
        self.publish_cues(self.avoid.lane_pose(self.tracker.latest()))

    def publish_cues(self, pose):
        """Entry line, lane lines and island (lane coordinates) drawn in base_link."""
        a, w = self.avoid, self.p.lane_width
        header = self._header()
        out = MarkerArray()
        out.markers.append(Marker(action=Marker.DELETEALL))
        s0, s1 = a.island
        tip = 0.25
        hexagon = np.array([(s0, 0.0), (s0 + tip, w / 2), (s1 - tip, w / 2), (s1, 0.0),
                            (s1 - tip, -w / 2), (s0 + tip, -w / 2), (s0, 0.0)])
        colour = {'lane tracker': (0.1, 0.8, 0.2), 'entry line': (0.95, 0.75, 0.1)}.get(
            a.island_source, (0.7, 0.3, 0.9))
        m = Marker(header=header, ns='island', id=0, type=Marker.LINE_STRIP)
        m.pose.orientation.w = 1.0
        m.scale.x = 0.02
        m.color = ColorRGBA(r=colour[0], g=colour[1], b=colour[2], a=0.9)
        m.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in section_to_base(hexagon, pose)]
        out.markers.append(m)
        t = Marker(header=header, ns='island_label', id=0, type=Marker.TEXT_VIEW_FACING)
        t.pose.orientation.w = 1.0
        x, y = section_to_base(np.array([[s0, 0.0]]), pose)[0]
        t.pose.position = Point(x=float(x), y=float(y), z=0.1)
        t.scale.z = 0.06
        t.color = m.color
        t.text = f'island ({a.island_source})'
        out.markers.append(t)
        for k, c in enumerate((-1.5, -0.5, 0.5, 1.5)):
            ln = Marker(header=header, ns='lane_model', id=k, type=Marker.LINE_STRIP)
            ln.pose.orientation.w = 1.0
            ln.scale.x = 0.01
            ln.color = ColorRGBA(r=0.1, g=0.9, b=0.9, a=0.8)
            ends = np.array([(pose[0] - 0.5, c * w), (pose[0] + 2.5, c * w)])
            ln.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in section_to_base(ends, pose)]
            out.markers.append(ln)
        if a.entry_s is not None:
            e = Marker(header=header, ns='entry_line', id=0, type=Marker.LINE_STRIP)
            e.pose.orientation.w = 1.0
            e.scale.x = 0.03
            e.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            ends = np.array([(a.entry_s, 1.5 * w), (a.entry_s, -1.5 * w)])
            e.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in section_to_base(ends, pose)]
            out.markers.append(e)
        self.cues_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = StaticAvoidNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
