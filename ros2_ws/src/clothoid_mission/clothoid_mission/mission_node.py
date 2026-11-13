#!/usr/bin/env python3
"""Mission state node: wraps `MissionLogic` (see mission_logic.py for the rules it implements).

Publishes
  `cmd_topic` (Twist, twist_mux input `state`): zero velocity while the logic says stop
  `speed_limit_topic` (Float32): heartbeat while a slow sign or speed bump limit is active
  `direction_topic` (Int32), `parking_topic` (Bool), `steering_lock_topic` (Bool): on change and once a second
  `state_topic` (String, JSON): state, what armed it, the line stop (for clothoid_debug stop_viz)

Light / pedestrian stop line (`use_stop_line`): while armed (YOLO saw the pedestrian sign / red light)
this node runs clothoid_finish's FinishCore on the camera with the cross line in white
(line_stop.py). lane_main keeps steering the whole time: only FinishCore's speed (approach 0.2, then
its ramp min(0.2, 1.5 x remaining)) goes to lane_main as /speed_limit, and its STOP holds the car.
Its steering is not used (it follows the left yellow line, which the zigzag before the pedestrian
crosswalk turned 9 deg off in the sim), so its left-line-lost abort is off (`lost_hold_sec` 1e9).
The camera is subscribed only while armed, as in finish.
"""
import json
import math
import inspect

import cv2
import numpy as np
import rclpy
from clothoid_common.qos import IMAGE_QOS
from clothoid_finish.bev import Bev, CameraModel
from clothoid_finish.finish_core import FinishParams, Phase
from clothoid_interfaces.msg import DetectionArray
from clothoid_lane.lane_detector import LaneParams
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu
from std_msgs.msg import Bool, Float32, Int32, String

from clothoid_mission.line_stop import LineStopCore
from clothoid_mission.mission_logic import (
    Inputs, MissionLogic, MissionParams, DetectionGate, classes_seen, quaternion_to_pitch_deg)

# FinishCore phase -> what mission_logic expects
LINE_PHASE = {Phase.IDLE: None, Phase.FOLLOW: 'search', Phase.COMMIT: 'commit',
              Phase.STOP: 'arrived', Phase.ABORT: 'abort'}


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission')

        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('stopline_topic', '/stopline_detected')
        self.declare_parameter('parking_done_topic', '/parking_done')
        self.declare_parameter('state_topic', '/mission/state')   # JSON for clothoid_debug stop_viz
        # line stop (FinishCore): camera, intrinsics, gyro, wheel speed
        self.declare_parameter('image_topic', '/camera/color/image_raw/compressed')
        self.declare_parameter('compressed', True)
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('line_debug_topic', '/mission/line_debug_image')
        self.declare_parameter('debug', False)
        self.declare_parameter('white_s_max', 60)        # stop line white: S <= and V >=
        self.declare_parameter('white_v_min', 150)
        self.declare_parameter('line_require_crosswalk', True)
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
        # std_msgs/Bool stand-in for YOLO's sign_parking (parkgo publishes it while there is no model)
        self.declare_parameter('parking_sign_topic', '/parking_sign')
        # one stop line right in front (clothoid_lane stopline node): where parking starts
        self.declare_parameter('parking_line_topic', '/stopline_near')
        self.declare_parameter('cone_active_topic', '/cone/mission_active')
        self.declare_parameter('cmd_topic', '/cmd_vel_state')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('input_timeout', 0.5)   # detections / stop line / imu older than this are dropped
        self.declare_parameter('refresh_sec', 1.0)
        self.declare_parameter('light_max_offset_height_ratio', 3.5)
        self.declare_parameter('sign_max_offset_height_ratio', 5.0)
        self.declare_parameter('fork_max_offset_height_ratio', 10.0)
        for name, value in [('detection_confirm_hits', 3), ('detection_window', 5),
                            ('detection_max_age', 0.5), ('detection_confidence', 0.6),
                            ('sign_min_height', 14), ('light_min_height', 8),
                            ('detection_edge_margin', 2)]:
            self.declare_parameter(name, value)

        d = MissionParams()
        self.declare_parameter('slow_speed', d.slow_speed)
        self.declare_parameter('slow_hold_sec', d.slow_hold_sec)
        self.declare_parameter('bump_pitch_thresh', d.bump_pitch_thresh)
        self.declare_parameter('bump_level_thresh', d.bump_level_thresh)
        self.declare_parameter('bump_level_duration', d.bump_level_duration_sec)
        self.declare_parameter('bump_arm_timeout', d.bump_arm_timeout_sec)
        self.declare_parameter('bump_cooldown', d.bump_cooldown_sec)
        self.declare_parameter('bump_min_area', 2500.0)    # px^2, minimum bbox area to arm bump
        self.declare_parameter('bump_min_height', 40.0)    # px, minimum bbox height to arm bump
        self.declare_parameter('slow_min_area', 2500.0)    # px^2, minimum bbox area to trigger slow
        self.declare_parameter('slow_min_height', 40.0)    # px, minimum bbox height to trigger slow
        self.declare_parameter('parking_min_area', 2500.0) # px^2, minimum bbox area to latch parking
        self.declare_parameter('parking_min_height', 40.0) # px, minimum bbox height to latch parking
        self.declare_parameter('pedestrian_min_area', 2500.0) # px^2, minimum bbox area to arm pedestrian
        self.declare_parameter('pedestrian_min_height', 40.0) # px, minimum bbox height to arm pedestrian

        self.declare_parameter('pedestrian_wait_sec', d.pedestrian_wait_sec)
        self.declare_parameter('stop_on_yellow', d.stop_on_yellow)
        self.declare_parameter('arm_timeout_sec', d.arm_timeout_sec)
        self.declare_parameter('cooldown_sec', d.cooldown_sec)
        self.declare_parameter('use_stop_line', d.use_stop_line)
        self.declare_parameter('approach_speed', d.approach_speed)
        self.declare_parameter('fork_left_direction', d.fork_left_direction)
        self.declare_parameter('fork_right_direction', d.fork_right_direction)
        self.declare_parameter('parking_at_stopline', d.parking_at_stopline)
        self.declare_parameter('require_cone_for_parking', d.require_cone_for_parking)
        self.declare_parameter('block_bump_after_cone', d.block_bump_after_cone)

        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self.detection_gate = DetectionGate(
            hits=get('detection_confirm_hits'), window=get('detection_window'),
            max_age=get('detection_max_age'), confidence=get('detection_confidence'),
            sign_height=get('sign_min_height'), light_height=get('light_min_height'),
            edge_margin=get('detection_edge_margin'))
        self.logic = MissionLogic(MissionParams(
            slow_speed=get('slow_speed'),
            slow_hold_sec=get('slow_hold_sec'),
            bump_pitch_thresh=get('bump_pitch_thresh'),
            bump_level_thresh=get('bump_level_thresh'),
            bump_level_duration_sec=get('bump_level_duration'),
            bump_arm_timeout_sec=get('bump_arm_timeout'),
            bump_cooldown_sec=get('bump_cooldown'),
            pedestrian_wait_sec=get('pedestrian_wait_sec'),
            stop_on_yellow=get('stop_on_yellow'),
            arm_timeout_sec=get('arm_timeout_sec'),
            cooldown_sec=get('cooldown_sec'),
            use_stop_line=get('use_stop_line'),
            approach_speed=get('approach_speed'),
            fork_left_direction=get('fork_left_direction'),
            fork_right_direction=get('fork_right_direction'),
            parking_at_stopline=get('parking_at_stopline'),
            require_cone_for_parking=get('require_cone_for_parking'),
            block_bump_after_cone=get('block_bump_after_cone')))

        self.seen, self.seen_time = set(), None
        self.stopline, self.stopline_time = False, None
        self.latest_pitch, self.latest_pitch_time = None, None
        self.parking_done = False
        self.cone_active = False
        self.cone_entered_logged = False
        self.sign_time = None            # last /parking_sign true
        self.line, self.line_time = False, None
        self.parking_log = (False, False)
        self.state = self.logic.state
        self.sent = {}   # topic key -> (value, time) for change / refresh publishing

        self.cmd_pub = self.create_publisher(Twist, get('cmd_topic'), 1)
        self.limit_pub = self.create_publisher(Float32, get('speed_limit_topic'), 1)
        self.direction_pub = self.create_publisher(Int32, get('direction_topic'), 1)
        self.parking_pub = self.create_publisher(Bool, get('parking_topic'), 1)
        self.steering_lock_pub = self.create_publisher(Bool, get('steering_lock_topic'), 1)

        self.create_subscription(DetectionArray, get('detections_topic'), self.on_detections, 1)
        # 2025 stripe flag (clothoid_lane stopline), commented out there in 2026: the light /
        # pedestrian stop is the line stop below. `stopline` stays False.
        # self.create_subscription(Bool, get('stopline_topic'), self.on_stopline, 1)
        self.create_subscription(Bool, get('parking_done_topic'), self.on_parking_done, 1)
        self.mission_state_pub = self.create_publisher(String, get('state_topic'), 1)
        self.create_subscription(Bool, get('parking_sign_topic'), self.on_parking_sign, 1)
        self.create_subscription(Bool, get('parking_line_topic'), self.on_parking_line, 1)
        self.create_subscription(Bool, get('cone_active_topic'), self.on_cone_active, 1)

        # line stop: FinishCore defaults (front 0.161, stop_gap 0.03, ...) as finish; it does not
        # steer here, so losing the left yellow line must not abort the stop
        self.finish_params = FinishParams(lost_hold_sec=1e9, band=get('line_band'))
        self.warp = LaneParams(x_h=get('warp_x_h'), x_l=get('warp_x_l'), y_h=get('warp_y_h'),
                               y_l=get('warp_y_l'))
        self.cam = CameraModel(height=get('camera_height'), pitch=get('camera_pitch'),
                               x=get('camera_x'), y=get('camera_y'))
        self.bev = None               # built once camera_info gives the intrinsics
        self.core = None
        self.core_phase = Phase.IDLE
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
        min_sizes = {
            'sign_bump': (self.get_parameter('bump_min_area').value,
                          self.get_parameter('bump_min_height').value),
            'sign_slow': (self.get_parameter('slow_min_area').value,
                          self.get_parameter('slow_min_height').value),
            'sign_parking': (self.get_parameter('parking_min_area').value,
                             self.get_parameter('parking_min_height').value),
            'sign_pedestrian': (self.get_parameter('pedestrian_min_area').value,
                                self.get_parameter('pedestrian_min_height').value),
        }
        accepted = classes_seen(
            msg.detections, camera_center_x=self.cam.cx,
            light_max_offset_height_ratio=self.get_parameter(
                'light_max_offset_height_ratio').value,
            sign_max_offset_height_ratio=self.get_parameter('sign_max_offset_height_ratio').value,
            fork_max_offset_height_ratio=self.get_parameter('fork_max_offset_height_ratio').value,
            min_sizes=min_sizes)
        self.seen = self.detection_gate.update(
            self.now_sec(), msg.detections, accepted,
            width=self.warp.width, height=self.warp.height)
        self.seen_time = self.now_sec()

        # Log sign bbox size for easy on-track tuning
        for d in msg.detections:
            if d.class_name in min_sizes:
                min_area, min_h = min_sizes[d.class_name]
                w = max(0, d.x_max - d.x_min)
                h = max(0, d.y_max - d.y_min)
                area = float(w * h)
                tag = d.class_name.upper().replace('SIGN_', '')
                if d.class_name in self.seen:
                    self.get_logger().info(
                        f'[{tag}] {d.class_name} ARMED (area={area:.0f}px^2, h={h}px >= min_area={min_area:.0f}, min_h={min_h:.0f})',
                        throttle_duration_sec=1.0)
                else:
                    self.get_logger().info(
                        f'[{tag}] {d.class_name} IGNORED (area={area:.0f}px^2, h={h}px < min_area={min_area:.0f}, min_h={min_h:.0f})',
                        throttle_duration_sec=1.0)

    def on_stopline(self, msg: Bool):
        self.stopline, self.stopline_time = msg.data, self.now_sec()

    def on_parking_line(self, msg: Bool):
        self.line, self.line_time = msg.data, self.now_sec()

    def on_parking_sign(self, msg: Bool):
        if msg.data:
            self.sign_time = self.now_sec()

    # ---- line stop (FinishCore) ------------------------------------------------------------------
    def new_core(self) -> LineStopCore:
        get = lambda n: self.get_parameter(n).value  # noqa: E731
        core = LineStopCore(self.bev, self.finish_params, get('white_s_max'), get('white_v_min'),
                            get('line_min_run'))
        core.min_thickness = get('line_min_thickness')
        core.max_thickness = get('line_max_thickness')
        core.max_frame_gap = get('line_max_frame_gap')
        core.require_crosswalk = get('line_require_crosswalk')
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
        if self.core:
            self.core.odom(self.stamp(msg, self.now_sec()), msg.twist.twist.linear.x)

    def _image_on(self, on: bool):
        if on and self.image_sub is None:
            kind = CompressedImage if self.get_parameter('compressed').value else Image
            self.image_sub = self.create_subscription(
                kind, self.get_parameter('image_topic').value, self.on_image, IMAGE_QOS)
        elif not on and self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None

    def on_image(self, msg):
        if self.core is None or self.core.phase not in (Phase.FOLLOW, Phase.COMMIT):
            return
        if self.get_parameter('compressed').value:
            bgr = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        else:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if bgr is None:
            return
        self.core.image(self.now_sec(), bgr)
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
        left = core.remaining()
        text = core.phase.value + ('' if left is None else f' {left * 100:.1f} cm')
        cv2.putText(img, text, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        return img

    def line_stop_step(self, now: float, arm: bool):
        """Arm / disarm FinishCore; its speed for lane_main while it runs, else None."""
        if self.core is None:
            return None
        if arm and self.core.phase == Phase.IDLE:
            self.core.start(now)
            self._image_on(True)
        elif not arm and self.core.phase != Phase.IDLE:
            self.core = self.new_core()          # fresh for the next stop (tracker restarts)
            self._image_on(False)
        if self.core.phase == Phase.IDLE:
            return None
        cmd = self.core.command(now)
        if self.core.phase != self.core_phase:
            self.core_phase = self.core.phase
            self.get_logger().info(f'line stop {self.core.phase.value}: {self.core.reason}')
            if self.core.phase in (Phase.STOP, Phase.ABORT):
                self._image_on(False)
        if cmd is None or self.core.phase not in (Phase.FOLLOW, Phase.COMMIT):
            return None
        return float(cmd[0])                     # speed only: lane_main steers

    def on_parking_done(self, msg: Bool):
        self.parking_done = msg.data

    def on_cone_active(self, msg: Bool):
        self.cone_active = bool(msg.data)

    def _fresh(self, stamp) -> bool:
        return stamp is not None and self.now_sec() - stamp <= self.get_parameter(
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
        out = self.logic.step(Inputs(
            now=self.now_sec(),
            seen=seen,
            stopline=self.stopline if self._fresh(self.stopline_time) else False,
            parking_line=self.line if self._fresh(self.line_time) else False,
            line_phase=line_phase,
            line_speed=self.line_speed,
            parking_done=self.parking_done,
            imu_pitch=self.latest_pitch if self._fresh(self.latest_pitch_time) else None,
            cone_active=self.cone_active))
        if self.logic.cone_entered and not self.cone_entered_logged:
            self.cone_entered_logged = True
            self.get_logger().info('cone course entered: parking unlocked, bump blocked')
        self.line_speed = self.line_stop_step(now, out.stop_arm)
        armed = [n for n, a in (('pedestrian', self.logic.pedestrian_armed_at),
                                ('light', self.logic.light_armed_at)) if a is not None]
        core = self.core
        left = core.remaining() if core is not None and core.phase != Phase.IDLE else None
        self.mission_state_pub.publish(String(data=json.dumps({
            'state': out.state, 'armed': armed,
            'cone_entered': self.logic.cone_entered,
            'seen': sorted(seen),
            'line_phase': None if core is None else LINE_PHASE[core.phase],
            'remaining': None if left is None else round(left, 4),
            'line': None if core is None or core.cross_x is None else round(core.cross_x, 4),
            'front': self.finish_params.front, 'speed_limit': out.speed_limit,
            'stopped_for': (round(now - self.logic.stopped_at, 2) if out.stop else None)})))

        log = (self.logic.parking_armed, out.parking_flag)
        if log != self.parking_log:
            self.get_logger().info('parking sign seen: waiting for the stop line' if not log[1]
                                   else 'stop line: parking flag on')
            self.parking_log = log
        if out.state != self.state:
            self.get_logger().info(f'{self.state} -> {out.state}')
            self.state = out.state

        if out.stop:
            self.cmd_pub.publish(Twist())   # zero command
        if out.speed_limit is not None:
            self.limit_pub.publish(Float32(data=float(out.speed_limit)))
        if out.direction is not None:
            self._publish_changed('direction', out.direction,
                                  lambda v: self.direction_pub.publish(Int32(data=int(v))))
        if out.parking_flag:
            self._publish_changed('parking', True,
                                  lambda v: self.parking_pub.publish(Bool(data=bool(v))))

        self._publish_changed('steering_lock', out.steering_lock,
                              lambda v: self.steering_lock_pub.publish(Bool(data=bool(v))))


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
