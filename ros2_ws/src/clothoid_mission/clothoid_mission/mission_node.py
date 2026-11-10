#!/usr/bin/env python3
"""Mission state node: wraps `MissionLogic` (see mission_logic.py for the rules it implements).

Publishes
  `cmd_topic` (Twist, twist_mux input `state`): zero velocity while the logic says stop
  `speed_limit_topic` (Float32): heartbeat while a slow sign or speed bump limit is active
  `direction_topic` (Int32), `parking_topic` (Bool), `steering_lock_topic` (Bool): on change and once a second
"""
import rclpy
from clothoid_interfaces.msg import DetectionArray
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Int32

from clothoid_mission.mission_logic import (
    Inputs, MissionLogic, MissionParams, classes_seen, quaternion_to_pitch_deg)


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission')

        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('stopline_topic', '/stopline_detected')
        self.declare_parameter('parking_done_topic', '/parking_done')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('speed_limit_topic', '/speed_limit')
        self.declare_parameter('direction_topic', '/direction_flag')
        self.declare_parameter('parking_topic', '/parking_flag')
        self.declare_parameter('steering_lock_topic', '/steering_lock')
        self.declare_parameter('cmd_topic', '/cmd_vel_state')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('input_timeout', 0.5)   # detections / stop line / imu older than this are dropped
        self.declare_parameter('refresh_sec', 1.0)

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
        self.declare_parameter('fork_left_direction', d.fork_left_direction)
        self.declare_parameter('fork_right_direction', d.fork_right_direction)

        get = lambda name: self.get_parameter(name).value  # noqa: E731
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
            fork_left_direction=get('fork_left_direction'),
            fork_right_direction=get('fork_right_direction')))

        self.seen, self.seen_time = set(), None
        self.stopline, self.stopline_time = False, None
        self.latest_pitch, self.latest_pitch_time = None, None
        self.parking_done = False
        self.state = self.logic.state
        self.sent = {}   # topic key -> (value, time) for change / refresh publishing

        self.cmd_pub = self.create_publisher(Twist, get('cmd_topic'), 1)
        self.limit_pub = self.create_publisher(Float32, get('speed_limit_topic'), 1)
        self.direction_pub = self.create_publisher(Int32, get('direction_topic'), 1)
        self.parking_pub = self.create_publisher(Bool, get('parking_topic'), 1)
        self.steering_lock_pub = self.create_publisher(Bool, get('steering_lock_topic'), 1)

        self.create_subscription(DetectionArray, get('detections_topic'), self.on_detections, 1)
        self.create_subscription(Bool, get('stopline_topic'), self.on_stopline, 1)
        self.create_subscription(Bool, get('parking_done_topic'), self.on_parking_done, 1)
        self.create_subscription(Imu, get('imu_topic'), self.on_imu, 10)

        self.create_timer(1.0 / get('rate_hz'), self.on_timer)
        self.get_logger().info('mission started (speed bump integrated)')

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
        self.seen = classes_seen(msg.detections, min_sizes=min_sizes)
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

    def on_parking_done(self, msg: Bool):
        self.parking_done = msg.data

    def on_imu(self, msg: Imu):
        q = msg.orientation
        self.latest_pitch = quaternion_to_pitch_deg(q.x, q.y, q.z, q.w)
        self.latest_pitch_time = self.now_sec()

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
        out = self.logic.step(Inputs(
            now=self.now_sec(),
            seen=self.seen if self._fresh(self.seen_time) else set(),
            stopline=self.stopline if self._fresh(self.stopline_time) else False,
            parking_done=self.parking_done,
            imu_pitch=self.latest_pitch if self._fresh(self.latest_pitch_time) else None))

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
