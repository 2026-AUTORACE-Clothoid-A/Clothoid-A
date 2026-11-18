#!/usr/bin/env python3
"""Mission state node: wraps `MissionLogic` (see mission_logic.py for the rules it implements).

Publishes
  `cmd_topic` (Twist, twist_mux input `state`): zero velocity while the logic says stop
  `speed_limit_topic` (Float32): heartbeat while a slow sign or speed bump limit is active
  `direction_topic` (Int32), `parking_topic` (Bool), `steering_lock_topic` (Bool): on change and once a second
  `state_topic` (String, JSON): state, what armed it, the line stop (for clothoid_debug stop_viz)

Confirmed sign tracks prepare local observers. They do not transfer driving control.
Mission boundaries start slowing near front-wheel entry. The separate crosswalk
stop line retains the original bumper stop reference and final-stop processing.
Parking needs a separate entry line and fresh passive lidar bay evidence. Floor processing is
shared by those observers; parking-only preparation caps image decoding at 6 Hz.
Slow and bump signs prepare a start-line observer; measured speed sets the braking lead.
The bump IMU contact fallback also remains active while waiting for that line.
lane_main keeps steering during the stop-line approach; its STOP holds the car.
Its steering is not used (it follows the left yellow line, which the zigzag before the pedestrian
crosswalk turned 9 deg off in the sim), so its left-line-lost abort is off (`lost_hold_sec` 1e9).
The camera is subscribed only while armed, as in finish.
"""
import json
import math
import inspect

import cv2
from clothoid_common.color import COLOR_NAMES, ColorPipeline, color_defaults, color_params, validate_hsv
import numpy as np
import rclpy
from clothoid_common.qos import IMAGE_QOS
from clothoid_common.quiet_node import QuietEventsMixin
from clothoid_finish.bev import Bev, CameraModel
from clothoid_finish.finish_core import FinishParams, Phase
from clothoid_interfaces.msg import DetectionArray
from clothoid_lane.lane_detector import LaneParams
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu
from std_msgs.msg import Bool, Float32, Int32, String

from clothoid_mission.line_stop import LineStopCore
from clothoid_mission.mission_logic import (
    BUMP_APPROACHING, BUMP_ON, Inputs, MissionLogic, MissionParams, STOPPED_LIGHT, classes_seen,
    quaternion_to_pitch_deg)
from clothoid_mission.scene_evidence import MissionEntryEvidence, ParkingEntryEvidence
from clothoid_mission.sign_tracker import SignTracker

# FinishCore phase -> what mission_logic expects
FORK_SIGNS = ('sign_left', 'sign_right')
DIRECTION_NAMES = {0: 'left', 1: 'right', 2: 'both'}

LINE_PHASE = {Phase.IDLE: None, Phase.FOLLOW: 'search', Phase.COMMIT: 'commit',
              Phase.STOP: 'arrived', Phase.ABORT: 'abort'}


def sign_log_key(verdict: str):
    """Throttle key and repeat interval of a sign verdict: the stage word only, without the numbers
    ('IGNORED: invalid width or h 10 < 14' and '... h 9 < 14' are one line per interval). Far signs
    rejected for size / offset repeat every 5 s, the rest every 1 s; a new stage logs at once."""
    if verdict.startswith('ARMED'):
        return 'ARMED', 1.
    stage = verdict.split(':', 1)[-1].split()[0] if ':' in verdict else verdict
    return stage, 5. if stage in ('size', 'offset') else 1.


class MissionNode(QuietEventsMixin, Node):
    def __init__(self):
        super().__init__('mission')

        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('stopline_topic', '/stopline_detected')
        self.declare_parameter('parking_done_topic', '/parking_done')
        self.declare_parameter('state_topic', '/mission/state')   # JSON for clothoid_debug stop_viz
        # line stop (FinishCore): camera, intrinsics, gyro, wheel speed
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('compressed', False)
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('imu_topic', '/imu_25hz')
        self.declare_parameter('odom_topic', '/odom_25hz')
        self.declare_parameter('line_debug_topic', '/mission/line_debug_image')
        self.declare_parameter('debug', False)
        self.declare_parameter('white_h_min', 0)         # inclusive OpenCV hue range
        self.declare_parameter('white_h_max', 179)
        self.declare_parameter('white_s_max', 60)        # stop line white: S <= and V >=
        self.declare_parameter('white_v_min', 150)
        for name, default in color_defaults().items():
            self.declare_parameter(name, default)
        color_params(lambda name: self.get_parameter(name).value)
        self.declare_parameter('line_require_crosswalk', True)
        self.declare_parameter('front_axle_x', 0.10)
        self.declare_parameter('entry_reaction_sec', 0.10)
        self.declare_parameter('entry_deceleration', 2.5)
        self.declare_parameter('entry_confirm_frames', 2)
        self.declare_parameter('entry_min_span', 0.26)
        self.declare_parameter('entry_rate_hz', 10.0)
        for name, value in [('line_band', 0.22), ('line_min_run', 0.7),
                            ('line_min_thickness', 0.008), ('line_max_thickness', 0.12),
                            ('line_max_frame_gap', 0.25)]:
            self.declare_parameter(name, value)
        # warp and camera pose: clothoid_lane camera_limo*.yaml (as finish)
        for name, default in (('warp_x_h', 60), ('warp_x_l', 327), ('warp_y_h', 91), ('warp_y_l', 2)):
            self.declare_parameter(name, default)
        for name, default in (('camera_height', 0.19), ('camera_pitch', -0.021), ('camera_x', 0.15),
                              ('camera_y', -0.01)):
            self.declare_parameter(name, default)
        self.declare_parameter('speed_limit_topic', '/speed_limit')
        self.declare_parameter('direction_topic', '/direction_flag')
        self.declare_parameter('parking_topic', '/parking_flag')
        self.declare_parameter('steering_lock_topic', '/steering_lock')
        self.declare_parameter('yaw_hold_topic', '/lane/yaw_hold')
        self.declare_parameter('light_stopped_topic', '/mission/light_stopped')
        self.declare_parameter('bump_approach_topic', '/bump/approaching')
        # std_msgs/Bool stand-in for YOLO's sign_parking (parkgo publishes it while there is no model)
        self.declare_parameter('parking_sign_topic', '/parking_sign')
        # one stop line right in front (clothoid_lane stopline node): where parking starts
        self.declare_parameter('parking_line_topic', '/stopline_near')
        self.declare_parameter('cone_active_topic', '/cone/mission_active')
        self.declare_parameter('parking_prepare_topic', '/parking_prepare')
        self.declare_parameter('parking_scene_topic', '/parking/bay_candidate')
        self.declare_parameter('parking_scene_rate_hz', 6.0)
        self.declare_parameter('parking_entry_max_travel', 3.0)
        self.parking_entry_reaction = self.declare_parameter('parking_entry_reaction_sec', 0.20).value
        self.parking_entry_deceleration = self.declare_parameter('parking_entry_deceleration', 1.0).value
        if (not math.isfinite(self.parking_entry_reaction) or self.parking_entry_reaction < 0. or
                not math.isfinite(self.parking_entry_deceleration) or self.parking_entry_deceleration <= 0.):
            raise ValueError('invalid parking entry braking parameters')
        self.declare_parameter('avoid_active_topic', '/avoid/active')
        self.declare_parameter('cmd_topic', '/cmd_vel_state')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('input_timeout', 0.5)   # detections / stop line / imu older than this are dropped
        self.declare_parameter('refresh_sec', 1.0)
        for name, value in [('detection_confirm_hits', 3), ('detection_window', 5),
                            ('detection_max_age', 0.5), ('detection_confidence', 0.6),
                            ('sign_min_height', 14), ('light_min_height', 8),
                            ('detection_edge_margin', 2), ('fork_confirm_hits', 2),
                            ('fork_confirm_age', 1.0), ('fork_ignore_edge', True),
                            ('fork_confidence', 0.55), ('fork_any_x', True)]:
            self.declare_parameter(name, value)
        self.declare_parameter('scene_sign_candidates', False)
        self.declare_parameter('track_retention_sec', 2.0)
        self.declare_parameter('track_max_objects', 32)
        self.declare_parameter('track_match_gate', 1.0)
        self.declare_parameter('track_scale_ratio', 3.0)
        self.declare_parameter('light_max_offset_height_ratio', 3.5)
        self.declare_parameter('sign_max_offset_height_ratio', 5.0)
        self.declare_parameter('fork_max_offset_height_ratio', 13.0)
        self.declare_parameter('bump_max_offset_height_ratio', 0.0)
        for kind, area, height in [('bump', 0.0, 28.0), ('slow', 2500.0, 40.0),
                                   ('parking', 0.0, 28.0), ('pedestrian', 0.0, 28.0)]:
            self.declare_parameter(kind + '_min_area', area)
            self.declare_parameter(kind + '_min_height', height)

        d = MissionParams()
        self.declare_parameter('slow_speed', d.slow_speed)
        self.declare_parameter('slow_hold_sec', d.slow_hold_sec)
        self.declare_parameter('bump_pitch_thresh', d.bump_pitch_thresh)
        self.declare_parameter('bump_level_thresh', d.bump_level_thresh)
        self.declare_parameter('bump_level_duration', d.bump_level_duration_sec)
        self.declare_parameter('bump_arm_timeout', d.bump_arm_timeout_sec)
        self.declare_parameter('bump_cooldown', d.bump_cooldown_sec)
        self.declare_parameter('hill_slow_imu', d.hill_slow_imu)
        self.declare_parameter('hill_max_sec', d.hill_max_sec)
        self.declare_parameter('first_slow_yaw_hold', d.first_slow_yaw_hold)
        self.declare_parameter('yaw_hold_max_sec', d.yaw_hold_max_sec)
        self.declare_parameter('yaw_hold_after_m', d.yaw_hold_after_m)

        self.declare_parameter('pedestrian_wait_sec', d.pedestrian_wait_sec)
        self.declare_parameter('stop_on_yellow', d.stop_on_yellow)
        self.declare_parameter('arm_timeout_sec', d.arm_timeout_sec)
        self.declare_parameter('cooldown_sec', d.cooldown_sec)
        self.declare_parameter('use_stop_line', d.use_stop_line)
        self.declare_parameter('approach_speed', d.approach_speed)
        self.declare_parameter('fork_left_direction', d.fork_left_direction)
        self.declare_parameter('fork_right_direction', d.fork_right_direction)
        self.declare_parameter('parking_at_stopline', d.parking_at_stopline)
        self.declare_parameter('require_scene_confirmation', d.require_scene_confirmation)
        self.declare_parameter('parking_enabled', d.parking_enabled)
        self.declare_parameter('slow_arms_bump', d.slow_arms_bump)
        self.declare_parameter('slow_at_stopline', d.slow_at_stopline)
        self.declare_parameter('slow_until_pedestrian_stop', d.slow_until_pedestrian_stop)
        self.declare_parameter('bump_at_stopline', d.bump_at_stopline)
        self.declare_parameter('require_cone_for_parking', d.require_cone_for_parking)
        self.declare_parameter('block_bump_after_cone', d.block_bump_after_cone)
        self.declare_parameter('cone_confirm_sec', d.cone_confirm_sec)
        self.declare_parameter('avoid_release_hold', d.avoid_release_hold)

        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self.sign_tracker = SignTracker(
            hits=get('detection_confirm_hits'), window=get('detection_window'),
            max_age=get('detection_max_age'), confidence=get('detection_confidence'),
            sign_height=get('sign_min_height'), light_height=get('light_min_height'),
            edge_margin=get('detection_edge_margin'), retention=get('track_retention_sec'),
            max_tracks=get('track_max_objects'), match_gate=get('track_match_gate'),
            scale_ratio=get('track_scale_ratio'),
            class_hits={name: get('fork_confirm_hits') for name in FORK_SIGNS},
            class_max_age={name: get('fork_confirm_age') for name in FORK_SIGNS},
            edge_exempt=FORK_SIGNS if get('fork_ignore_edge') else (),
            class_confidence={name: get('fork_confidence') for name in FORK_SIGNS},
            any_x=FORK_SIGNS if get('fork_any_x') else ())
        self.logic = MissionLogic(MissionParams(
            slow_speed=get('slow_speed'),
            slow_hold_sec=get('slow_hold_sec'),
            bump_pitch_thresh=get('bump_pitch_thresh'),
            bump_level_thresh=get('bump_level_thresh'),
            bump_level_duration_sec=get('bump_level_duration'),
            bump_arm_timeout_sec=get('bump_arm_timeout'),
            bump_cooldown_sec=get('bump_cooldown'),
            hill_slow_imu=get('hill_slow_imu'),
            hill_max_sec=get('hill_max_sec'),
            first_slow_yaw_hold=get('first_slow_yaw_hold'),
            yaw_hold_max_sec=get('yaw_hold_max_sec'),
            yaw_hold_after_m=get('yaw_hold_after_m'),
            pedestrian_wait_sec=get('pedestrian_wait_sec'),
            stop_on_yellow=get('stop_on_yellow'),
            arm_timeout_sec=get('arm_timeout_sec'),
            cooldown_sec=get('cooldown_sec'),
            use_stop_line=get('use_stop_line'),
            approach_speed=get('approach_speed'),
            fork_left_direction=get('fork_left_direction'),
            fork_right_direction=get('fork_right_direction'),
            parking_at_stopline=get('parking_at_stopline'),
            require_scene_confirmation=get('require_scene_confirmation'),
            parking_enabled=get('parking_enabled'),
            slow_arms_bump=get('slow_arms_bump'),
            slow_at_stopline=get('slow_at_stopline'),
            slow_until_pedestrian_stop=get('slow_until_pedestrian_stop'),
            bump_at_stopline=get('bump_at_stopline'),
            require_cone_for_parking=get('require_cone_for_parking'),
            block_bump_after_cone=get('block_bump_after_cone'),
            cone_confirm_sec=get('cone_confirm_sec'),
            avoid_release_hold=get('avoid_release_hold')))

        self.yaw_hold_logged = False
        self.light_stopped_logged = False
        self.cone_active = False
        self.cone_entered_logged = False
        self.direction_logged = None
        self.bump_logged = None
        self.limit_logged = None
        self.line_reason = None
        self.sign_logged = {}
        self.seen, self.seen_time = set(), None
        self.light_target_id = None
        self.confirmed_tracks = []
        self.parking_scene_time = None
        self.parking_observing = self.parking_skipped = False
        self.parking_observing_time = self.parking_skipped_time = None
        self.travel, self._odom_stamp = 0., None
        self.latest_speed, self.latest_speed_time = None, None
        self.mission_entry_active = False
        self.entry_prepare_time = None
        self._scene_stamp = None
        self._entry_stamp = None
        self.stopline, self.stopline_time = False, None
        self.latest_pitch, self.latest_pitch_time = None, None
        self.parking_done = False
        self.avoid_active, self.avoid_time = False, None
        self.sign_time = None            # last /parking_sign true
        self.line, self.line_time = False, None
        self.parking_log = (False, False)
        self.state = self.logic.state
        self.sent = {}   # topic key -> (value, time) for change / refresh publishing

        self.cmd_pub = self.create_publisher(Twist, get('cmd_topic'), 1)
        self.limit_pub = self.create_publisher(Float32, get('speed_limit_topic'), 1)
        self.direction_pub = self.create_publisher(Int32, get('direction_topic'), 1)
        self.parking_pub = self.create_publisher(Bool, get('parking_topic'), 1)
        self.prepare_pub = self.create_publisher(Bool, get('parking_prepare_topic'), 1)
        self.entry_distance_pub = self.create_publisher(Float32, '/parking/entry_distance', 1)
        self.steering_lock_pub = self.create_publisher(Bool, get('steering_lock_topic'), 1)
        self.yaw_hold_pub = self.create_publisher(Bool, get('yaw_hold_topic'), 1)
        self.light_stopped_pub = self.create_publisher(Bool, get('light_stopped_topic'), 1)
        self.bump_approach_pub = self.create_publisher(Bool, get('bump_approach_topic'), 1)

        self.create_subscription(DetectionArray, get('detections_topic'), self.on_detections, 1)
        # 2025 stripe flag (clothoid_lane stopline), commented out there in 2026: the light /
        # pedestrian stop is the line stop below. `stopline` stays False.
        # self.create_subscription(Bool, get('stopline_topic'), self.on_stopline, 1)
        self.create_subscription(Bool, get('parking_done_topic'), self.on_parking_done, 1)
        self.mission_state_pub = self.create_publisher(String, get('state_topic'), 1)
        self.create_subscription(Bool, get('parking_sign_topic'), self.on_parking_sign, 1)
        self.create_subscription(Bool, get('parking_line_topic'), self.on_parking_line, 1)
        self.create_subscription(PoseStamped, get('parking_scene_topic'), self.on_parking_scene, 1)
        self.create_subscription(Bool, '/parking/observing', self.on_parking_observing, 1)
        self.create_subscription(Bool, '/parking/skipped', self.on_parking_skipped, 1)
        self.create_subscription(Bool, get('cone_active_topic'), self.on_cone_active, 1)
        self.create_subscription(Bool, get('avoid_active_topic'), self.on_avoid_active, 1)

        # Original crosswalk stop settings: FinishCore's calibrated bumper reference
        # and gap. Mission-entry tuning must not change the crosswalk stop target.
        self.parking_front = FinishParams().front
        self.finish_params = FinishParams(lost_hold_sec=1e9, band=get('line_band'),
            speed=get('approach_speed'))
        if (get('front_axle_x') < 0 or get('entry_reaction_sec') < 0 or
                get('entry_deceleration') <= 0 or get('entry_confirm_frames') < 2 or
                not 0 < get('entry_min_span') <= 2*get('line_band') or get('entry_rate_hz') <= 0):
            raise ValueError('invalid front axle or entry braking parameters')
        if get('parking_scene_rate_hz') <= 0 or get('parking_entry_max_travel') <= 0:
            raise ValueError('parking scene rate and entry travel must be positive')
        self.entry_evidence = ParkingEntryEvidence(
            confirm_frames=self.finish_params.confirm_frames, tolerance=self.finish_params.confirm_tolerance,
            max_frame_gap=get('line_max_frame_gap'), max_travel=get('parking_entry_max_travel'),
            crosswalk_margin=get('line_max_thickness'))
        self.mission_entry = MissionEntryEvidence(
            confirm_frames=get('entry_confirm_frames'), tolerance=self.finish_params.confirm_tolerance,
            max_frame_gap=get('line_max_frame_gap'), crosswalk_margin=get('line_max_thickness'))
        self.warp = LaneParams(x_h=get('warp_x_h'), x_l=get('warp_x_l'), y_h=get('warp_y_h'),
                               y_l=get('warp_y_l'))
        self.cam = CameraModel(height=get('camera_height'), pitch=get('camera_pitch'),
                               x=get('camera_x'), y=get('camera_y'))
        self.bev = None               # built once camera_info gives the intrinsics
        self.core = None
        self.core_phase = Phase.IDLE
        self.add_on_set_parameters_callback(self.on_hsv_parameters)
        self.line_speed = None        # FinishCore's speed last tick, the /speed_limit while it runs
        self.bridge = CvBridge()
        self.image_sub = None
        self.debug_pub = self.create_publisher(Image, get('line_debug_topic'), 1) \
            if get('debug') else None
        self.create_subscription(CameraInfo, get('camera_info_topic'), self.on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, get('imu_topic'), self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, get('odom_topic'), self.on_odom, qos_profile_sensor_data)

        self.create_timer(1.0 / get('rate_hz'), self.on_timer)
        self.get_logger().info('mission started (speed bump integrated)')
        self.get_logger().info(
            f'mission implementation: {inspect.getfile(MissionNode)}; '
            f'use_stop_line={get("use_stop_line")}; confirmation='
            f'{get("detection_confirm_hits")}/{get("detection_window")}')

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_detections(self, msg: DetectionArray):
        now = self.now_sec()
        stamp = self.stamp(msg, now)
        if not -.05 <= now - stamp <= self.get_parameter('input_timeout').value:
            return
        if self.sign_tracker._last_frame is not None and stamp <= self.sign_tracker._last_frame:
            return
        get = lambda name: self.get_parameter(name).value
        filters = dict(
            camera_center_x=self.cam.cx,
            light_max_offset_height_ratio=get('light_max_offset_height_ratio'),
            sign_max_offset_height_ratio=get('sign_max_offset_height_ratio'),
            fork_max_offset_height_ratio=get('fork_max_offset_height_ratio'),
            bump_max_offset_height_ratio=get('bump_max_offset_height_ratio'),
            min_sizes={'sign_' + kind: (get(kind + '_min_area'), get(kind + '_min_height'))
                       for kind in ('bump', 'slow', 'parking', 'pedestrian')})
        # Apply proximity gates to each box before collecting confirmation hits.
        # A valid box of one class must not admit a second distant box of that class.
        # With scene confirmation, parking/pedestrian tracks are observation
        # candidates, not immediate motion triggers. Keep their image quality
        # and temporal gates, but do not discard roadside signs by box proximity.
        scene_candidates = (get('scene_sign_candidates') and self.logic.p.require_scene_confirmation)
        detections = [d for d in msg.detections if
                      (scene_candidates and (d.class_name == 'sign_parking' or
                       (d.class_name == 'sign_pedestrian' and self.logic.p.use_stop_line and
                        get('line_require_crosswalk'))))
                      or classes_seen((d,), **filters)]
        tracks = self.sign_tracker.update(stamp, detections, self.warp.width, self.warp.height)
        self.confirmed_tracks = tracks
        # Start passive floor sensing on a quality candidate, before three-frame sign
        # confirmation. This grants no motion action; MissionLogic still requires confirmation.
        if any(t.last_seen == stamp and t.box.class_name in (
                'sign_bump', 'sign_slow', 'sign_pedestrian',
                'traffic_light_red', 'traffic_light_yellow', 'traffic_light_green')
               for t in self.sign_tracker.tracks.values()):
            self.entry_prepare_time = stamp
        if self.light_target_id is None:
            red = [t for t in tracks if t.box.class_name in ('traffic_light_red', 'traffic_light_yellow')]
            if red:
                self.light_target_id = max(red, key=lambda t: (t.hits, t.box.confidence)).id
                self.sign_tracker.pin(self.light_target_id)
        self.seen = {t.box.class_name for t in tracks if not t.box.class_name.startswith(
            'traffic_light_') or self.light_target_id is None or t.id == self.light_target_id}
        self.seen_time = stamp
        for d in msg.detections:
            if d.class_name in filters['min_sizes'] or d.class_name in FORK_SIGNS:
                self._log_sign(d, now, stamp, any(d is box for box in detections), filters['min_sizes'])

    def _log_sign(self, d, now, stamp, accepted, min_sizes):
        name = d.class_name
        w, h = max(0, d.x_max-d.x_min), max(0, d.y_max-d.y_min)
        area = w*h
        ratio = abs((d.x_min+d.x_max)/2-self.cam.cx)/h if h else math.inf
        tracker = self.sign_tracker
        track = next((t for t in tracker.tracks.values() if t.last_seen == stamp and
                      t.box.class_name == name and
                      (t.box.x_min, t.box.y_min, t.box.x_max, t.box.y_max) ==
                      (d.x_min, d.y_min, d.x_max, d.y_max)), None)
        minimum_area, minimum_height = min_sizes.get(name, (0., 0.))
        if not accepted:
            reason = 'size' if area < minimum_area or h < minimum_height else 'offset'
            verdict = f'IGNORED: {reason}'
        else:
            reason = tracker.box_reason(d, self.warp.width, self.warp.height)
            if reason:
                verdict = f'IGNORED: {reason}'
            elif track is None:
                verdict = 'IGNORED: association/overlap'
            else:
                verdict = 'ARMED' if track.confirmed else 'IGNORED: hits'
        key, interval = sign_log_key(verdict)
        last = self.sign_logged.get(name)
        if last is not None and now-last[0] < interval and last[1] == key:
            return
        self.sign_logged[name] = (now, key)
        hits = track.hits if track is not None else 0
        self.get_logger().info(
            f'[{name.upper().replace("SIGN_", "")}] {name} {verdict} '
            f'(hits {hits}/{tracker.required(name)}, conf {d.confidence:.2f}, '
            f'area {area:.0f}px^2, h {h}px, min_area {minimum_area:.0f}, min_h {minimum_height}, '
            f'offset {ratio:.1f} x h, box {d.x_min},{d.y_min},{d.x_max},{d.y_max})')

    def on_cone_active(self, msg: Bool):
        self.cone_active = msg.data

    def on_parking_observing(self, msg: Bool):
        self.parking_observing, self.parking_observing_time = msg.data, self.now_sec()

    def on_parking_skipped(self, msg: Bool):
        self.parking_skipped, self.parking_skipped_time = msg.data, self.now_sec()

    def on_parking_scene(self, msg: PoseStamped):
        now = self.now_sec()
        stamp = self.stamp(msg, now)
        p = msg.pose.position
        if (msg.header.frame_id == 'base_link' and self.logic.parking_armed and
                -.05 <= now - stamp <= self.get_parameter('input_timeout').value and
                math.isfinite(p.x) and math.isfinite(p.y) and p.x > 0):
            self.parking_scene_time = stamp

    def on_stopline(self, msg: Bool):
        self.stopline, self.stopline_time = msg.data, self.now_sec()

    def on_parking_line(self, msg: Bool):
        self.line, self.line_time = msg.data, self.now_sec()

    def on_parking_sign(self, msg: Bool):
        if msg.data:
            self.sign_time = self.now_sec()

    # ---- line stop (FinishCore) ------------------------------------------------------------------
    def on_hsv_parameters(self, params):
        """Update the existing colour mask without resetting the mission or its stop target."""
        from rcl_interfaces.msg import SetParametersResult
        hsv_names = ('white_h_min', 'white_h_max', 'white_s_max', 'white_v_min')
        names = hsv_names + COLOR_NAMES
        proposed = {p.name: p.value for p in params if p.name in names}
        if not proposed:
            return SetParametersResult(successful=True)
        values = {name: proposed.get(name, self.get_parameter(name).value) for name in names}
        try:
            validate_hsv((0, 0, 0), (179, 255, 255), *(values[name] for name in hsv_names))
            pipeline = ColorPipeline(color_params(values.__getitem__))
        except (ValueError, TypeError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        if self.core is not None:
            for name in hsv_names:
                setattr(self.core, name, values[name])
            self.core.color_pipeline = pipeline
        return SetParametersResult(successful=True)

    def new_core(self) -> LineStopCore:
        get = lambda n: self.get_parameter(n).value  # noqa: E731
        core = LineStopCore(self.bev, self.finish_params, get('white_s_max'), get('white_v_min'),
                            get('line_min_run'), white_h_min=get('white_h_min'),
                            white_h_max=get('white_h_max'), color=color_params(get))
        core.min_thickness = get('line_min_thickness')
        core.max_thickness = get('line_max_thickness')
        core.max_frame_gap = get('line_max_frame_gap')
        core.require_crosswalk = get('line_require_crosswalk')
        core.observe_scene = True
        core.entry_min_span = get('entry_min_span')
        core.debug = self.debug_pub is not None
        return core

    def on_info(self, msg: CameraInfo):
        if self.bev is not None or msg.k[0] <= 0:
            return
        self.cam.fx, self.cam.fy, self.cam.cx, self.cam.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        self.warp.width, self.warp.height = msg.width, msg.height
        self.bev = Bev(self.warp, self.cam)
        self.core = self.new_core()
        near, far = self.bev.near_far()
        self.get_logger().info(f'line stop: warp sees {near:.2f}~{far:.2f} m ahead of base_link')

    @staticmethod
    def stamp(msg, fallback) -> float:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return t if t > 0 else fallback

    def on_imu(self, msg: Imu):
        if self.core:
            self.core.imu(self.stamp(msg, self.now_sec()), msg.angular_velocity.z)
        q = msg.orientation
        self.latest_pitch = quaternion_to_pitch_deg(q.x, q.y, q.z, q.w)
        self.latest_pitch_time = self.now_sec()

    def on_odom(self, msg: Odometry):
        stamp = self.stamp(msg, self.now_sec())
        self.latest_speed = msg.twist.twist.linear.x
        self.latest_speed_time = stamp
        if self._odom_stamp is not None and 0 < stamp - self._odom_stamp < .2:
            self.travel += msg.twist.twist.linear.x * (stamp - self._odom_stamp) * self.finish_params.odom_scale
        self._odom_stamp = stamp
        if self.core:
            self.core.odom(stamp, msg.twist.twist.linear.x)

    def _image_on(self, on: bool):
        if on and self.image_sub is None:
            kind = CompressedImage if self.get_parameter('compressed').value else Image
            self.image_sub = self.create_subscription(
                kind, self.get_parameter('image_topic').value, self.on_image, IMAGE_QOS)
        elif not on and self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None

    def on_image(self, msg):
        if self.core is None:
            return
        stop_observer = self.core.phase in (Phase.FOLLOW, Phase.COMMIT)
        if not stop_observer and not self.mission_entry_active and not self.logic.parking_armed:
            return
        now = self.now_sec()
        stamp = self.stamp(msg, now)
        if not -.05 <= now - stamp <= self.get_parameter('input_timeout').value:
            return
        if self._scene_stamp is not None and stamp <= self._scene_stamp:
            return
        entry_rate = self.get_parameter('entry_rate_hz').value
        if not stop_observer and self._scene_stamp is not None:
            scene_rate = max(entry_rate if self.mission_entry_active else 0.,
                             self.get_parameter('parking_scene_rate_hz').value if self.logic.parking_armed else 0.)
            # One image per time slot avoids dropping every other 10 Hz source frame
            # due to tiny camera timestamp jitter around a strict 0.1 s interval.
            if int(stamp*scene_rate) <= int(self._scene_stamp*scene_rate):
                return
        if self._fresh(self.latest_pitch_time) and abs(self.latest_pitch) >= self.logic.p.bump_pitch_thresh:
            return                       # the fixed floor projection is invalid while tilted
        self._scene_stamp = stamp
        if self.get_parameter('compressed').value:
            bgr = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        else:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if bgr is None:
            return
        entry_due = self.mission_entry_active and (
            self._entry_stamp is None or int(stamp*entry_rate) > int(self._entry_stamp*entry_rate))
        self.core.observe_entry = entry_due
        self.core.image(stamp, bgr)
        if entry_due:
            self._entry_stamp = stamp
            self.mission_entry.observe(stamp, self.travel, self.core.observation)
        if self.logic.parking_armed:
            self.entry_evidence.observe(stamp, self.travel, self.core.observation)
        if self.debug_pub is not None:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(self.line_debug_image(), 'bgr8'))

    def line_debug_image(self) -> np.ndarray:
        core = self.core
        img = np.zeros(core.mask.shape + (3,), np.uint8)
        img[core.white] = (200, 200, 200)
        img[core.mask] = (0, 200, 255)
        gx, gy = core.bev.gx, core.bev.gy
        if core.left is not None:
            off, hd = core.left
            img[(np.abs(gy - (off + math.tan(hd) * gx)) < 0.006) & core.bev.valid] = (0, 255, 0)
        if core.cross_x is not None:
            img[(np.abs(gx - core.cross_x) < 0.004) & core.bev.valid] = (255, 0, 255)
        if self.mission_entry.entry_at is not None:
            entry_x = self.mission_entry.entry_at-self.travel
            img[(np.abs(gx-entry_x) < .004) & core.bev.valid] = (0, 180, 255)
        left = core.remaining()
        text = core.phase.value + ('' if left is None else f' {left * 100:.1f} cm')
        cv2.putText(img, text, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        return img

    def line_stop_step(self, now: float, arm: bool, require_crosswalk: bool = True):
        """Arm / disarm FinishCore; its speed for lane_main while it runs, else None."""
        if self.core is None:
            return None
        if arm:
            self.core.require_crosswalk = bool(
                self.get_parameter('line_require_crosswalk').value and require_crosswalk)
        if arm and self.core.phase == Phase.IDLE:
            self.core.start(now)
            self.line_reason = ('light' if self.logic.light_armed_at is not None else
                                'pedestrian' if self.logic.pedestrian_armed_at is not None else 'parking')
        elif not arm and self.core.phase != Phase.IDLE:
            self.core = self.new_core()          # fresh for the next stop (tracker restarts)
        if self.core.phase == Phase.IDLE:
            return None
        # FOLLOW observes geometry only. A distant sign neither slows lane_main nor
        # exhausts FinishCore's search-distance budget before its stop line appears.
        cmd = self.core.command(now) if self.core.phase == Phase.COMMIT else None
        if self.core.phase != self.core_phase:
            self.core_phase = self.core.phase
            reason = self.line_reason if self.core.phase == Phase.FOLLOW else self.core.reason
            self.get_logger().info(f'line stop {self.core.phase.value}: {reason}')
        if cmd is None or self.core.phase not in (Phase.FOLLOW, Phase.COMMIT):
            return None
        return float(cmd[0])                     # speed only: lane_main steers

    def on_parking_done(self, msg: Bool):
        self.parking_done = msg.data

    def on_avoid_active(self, msg: Bool):
        self.avoid_active, self.avoid_time = bool(msg.data), self.now_sec()

    def _fresh(self, stamp) -> bool:
        return stamp is not None and -.05 <= self.now_sec() - stamp <= self.get_parameter(
            'input_timeout').value

    def _publish_changed(self, key, value, publish):
        """Publish on change and refresh once per `refresh_sec` (lane_main logs every message)."""
        now = self.now_sec()
        last = self.sent.get(key)
        if (last is None or last[0] != value
                or now - last[1] >= self.get_parameter('refresh_sec').value):
            publish(value)
            self.sent[key] = (value, now)

    def on_timer(self):
        now = self.now_sec()
        core = self.core
        line_phase = LINE_PHASE[core.phase] if core is not None else None
        seen = set(self.seen) if self._fresh(self.seen_time) else set()
        if self._fresh(self.sign_time):
            seen.add('sign_parking')
        value = lambda name: self.get_parameter(name).value
        entry_slow = self.mission_entry_active and self.mission_entry.should_slow(
            self.travel, value('front_axle_x'),
            self.latest_speed if self._fresh(self.latest_speed_time) else None,
            min(self.logic.p.slow_speed, self.logic.p.approach_speed),
            value('entry_reaction_sec'), value('entry_deceleration'))
        entry_reached = self.mission_entry_active and self.mission_entry.reached(
            self.travel, value('front_axle_x'))
        parking_speed_limit = self.entry_evidence.approach_limit(
            self.travel, self.parking_front, self.logic.p.approach_speed,
            self.parking_entry_reaction, self.parking_entry_deceleration)
        out = self.logic.step(Inputs(
            now=self.now_sec(),
            seen=seen,
            stopline=self.stopline if self._fresh(self.stopline_time) else False,
            parking_line=(self.entry_evidence.reached(self.travel, self.parking_front)
                          if self.logic.p.require_scene_confirmation else
                          self.line if self._fresh(self.line_time) else False),
            parking_scene=self._fresh(self.parking_scene_time),
            parking_speed_limit=parking_speed_limit,
            parking_observing=self.parking_observing and self._fresh(self.parking_observing_time),
            parking_skipped=self.parking_skipped and self._fresh(self.parking_skipped_time),
            line_phase=line_phase,
            line_speed=self.line_speed,
            entry_slow=entry_slow, entry_reached=entry_reached,
            travel=self.travel if self._fresh(self.latest_speed_time) else None,
            parking_done=self.parking_done,
            imu_pitch=self.latest_pitch if self._fresh(self.latest_pitch_time) else None,
            cone_active=self.cone_active,
            avoid_active=self.avoid_active and self._fresh(self.avoid_time)))
        if self.logic.cone_entered and not self.cone_entered_logged:
            self.cone_entered_logged = True
            self.get_logger().info('cone course entered')
        self.line_speed = self.line_stop_step(now, out.stop_arm, out.stop_require_crosswalk)
        entry_observe = out.entry_arm or (self.logic.state == 'drive' and self._fresh(self.entry_prepare_time))
        if entry_observe and not self.mission_entry_active:
            self.mission_entry.clear()
            self._entry_stamp = None
        self.mission_entry_active = entry_observe
        if self.core is not None:
            self.core.observe_entry = entry_observe
        self._image_on(out.stop_arm or entry_observe or out.parking_prepare)
        self._publish_changed('parking_prepare', out.parking_prepare,
                              lambda v: self.prepare_pub.publish(Bool(data=bool(v))))
        entry = self.entry_evidence.entry_at
        distance = entry-self.travel if entry is not None else None
        entry_valid = (out.parking_prepare and distance is not None and
                       self.travel+self.parking_front-entry <= self.entry_evidence.max_travel)
        self.entry_distance_pub.publish(Float32(data=float(distance) if entry_valid else math.nan))
        if not out.parking_prepare:
            self.entry_evidence.clear()
            self.parking_scene_time = None
        if self.logic.light_armed_at is None and self.logic.state != STOPPED_LIGHT:
            self.light_target_id = None
            self.sign_tracker.pin(None)
        armed = [n for n, a in (('pedestrian', self.logic.pedestrian_armed_at),
                                ('light', self.logic.light_armed_at),
                                ('slow', self.logic.slow_armed_at),
                                ('bump_line', self.logic.bump_line_armed_at)) if a is not None]
        core = self.core
        left = core.remaining() if core is not None and core.phase != Phase.IDLE else None
        entry_braking = entry_slow and out.speed_limit is not None
        self.mission_state_pub.publish(String(data=json.dumps({
            'state': out.state, 'armed': armed,
            'cone_entered': self.logic.cone_entered,
            'seen': sorted(seen),
            'tracks': [{'id': t.id, 'class': t.box.class_name, 'hits': t.hits}
                       for t in self.confirmed_tracks] if self._fresh(self.seen_time) else [],
            'light_track': self.light_target_id,
            'parking_prepare': out.parking_prepare,
            'parking_scene': self._fresh(self.parking_scene_time),
            'parking_observing': self.parking_observing and self._fresh(self.parking_observing_time),
            'parking_skipped': self.parking_skipped and self._fresh(self.parking_skipped_time),
            'parking_entry': self.entry_evidence.entry_at,
            'entry_active': self.mission_entry_active,
            'mission_entry': self.mission_entry.entry_at,
            'entry_remaining': self.mission_entry.remaining(self.travel, value('front_axle_x')),
            'entry_slow': entry_slow, 'entry_reached': entry_reached,
            'entry_phase': (('entered' if entry_reached else 'braking' if entry_braking else
                             'confirmed' if self.mission_entry.entry_at is not None else 'search')
                            if self.mission_entry_active else None),
            'line_phase': None if core is None else LINE_PHASE[core.phase],
            'remaining': None if left is None else round(left, 4),
            'line': None if core is None or core.cross_x is None else round(core.cross_x, 4),
            'front': self.finish_params.front, 'entry_front': value('front_axle_x'),
            'speed_limit': out.speed_limit,
            'bump_state': self.logic.bump_state,
            'hill_imu': self.logic.hill_active,
            'imu_pitch': self.latest_pitch if self._fresh(self.latest_pitch_time) else None,
            'steering_lock': out.steering_lock,
            'stopped_for': (round(now - self.logic.stopped_at, 2) if out.stop else None)})))

        log = (self.logic.parking_armed, out.parking_flag)
        if log != self.parking_log:
            message = ('parking entry and bay confirmed: control on' if out.parking_flag else
                       'parking track confirmed: observing entry and bay' if out.parking_prepare else
                       'parking candidate expired or cleared')
            self.get_logger().info(message)
            self.parking_log = log
        if out.state != self.state:
            self.get_logger().info(f'{self.state} -> {out.state}')
            self.state = out.state

        if self.logic.bump_state != self.bump_logged:
            if self.bump_logged is not None:
                pitch = self.latest_pitch if self._fresh(self.latest_pitch_time) else None
                self.get_logger().info(f'bump: {self.bump_logged} -> {self.logic.bump_state} '
                                       f'(pitch {"none" if pitch is None else f"{pitch:+.1f} deg"})')
            self.bump_logged = self.logic.bump_state
        # why lane_main is slowed (sources as MissionLogic.step adds them); logged when the set
        # changes, not on every line stop ramp value
        limit = None
        if out.speed_limit is not None:
            limit = '+'.join(name for name, on in (
                ('slow sign', now < self.logic.slow_until),
                ('bump', self.logic.bump_state in (BUMP_APPROACHING, BUMP_ON)),
                ('hill imu', self.logic.hill_active),
                ('slow to crosswalk', self.logic.slow_for_pedestrian and now >= self.logic.slow_until),
                ('slow entry', self.logic.slow_armed_at is not None and now >= self.logic.slow_until),
                ('parking approach', out.parking_prepare and self.logic.p.require_scene_confirmation
                 and parking_speed_limit is not None),
                ('line stop', out.stop_arm)) if on) or 'other'
        if limit != self.limit_logged:
            self.get_logger().info('speed limit off' if limit is None else
                                   f'speed limit {out.speed_limit:.2f} ({limit})')
            self.limit_logged = limit

        if out.stop:
            self.cmd_pub.publish(Twist())   # zero command
        if out.speed_limit is not None:
            self.limit_pub.publish(Float32(data=float(out.speed_limit)))
        if out.direction is not None and out.direction != self.direction_logged:
            sign = next((n for n in FORK_SIGNS if n in self.seen), 'earlier sign')
            self.get_logger().info(f'fork: direction {out.direction} '
                                   f'({DIRECTION_NAMES.get(out.direction, "?")}) from {sign}')
            self.direction_logged = out.direction
        if out.direction is not None:
            self._publish_changed('direction', out.direction,
                                  lambda v: self.direction_pub.publish(Int32(data=int(v))))
        if out.parking_flag:
            self._publish_changed('parking', True,
                                  lambda v: self.parking_pub.publish(Bool(data=bool(v))))

        self._publish_changed('steering_lock', out.steering_lock,
                              lambda v: self.steering_lock_pub.publish(Bool(data=bool(v))))
        if out.yaw_hold != self.yaw_hold_logged:
            self.get_logger().info(f'yaw hold {"on (first slow zone)" if out.yaw_hold else "off"}')
            self.yaw_hold_logged = out.yaw_hold
        self._publish_changed('yaw_hold', out.yaw_hold,
                              lambda v: self.yaw_hold_pub.publish(Bool(data=bool(v))))
        if out.light_stopped and not self.light_stopped_logged:
            self.get_logger().info('traffic light stop: cone course START held from now on')
            self.light_stopped_logged = True
        self._publish_changed('light_stopped', out.light_stopped,
                              lambda v: self.light_stopped_pub.publish(Bool(data=bool(v))))
        self._publish_changed('bump_approaching', self.logic.bump_state == BUMP_APPROACHING,
                              lambda v: self.bump_approach_pub.publish(Bool(data=bool(v))))


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
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
