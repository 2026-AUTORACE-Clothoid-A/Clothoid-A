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
    Inputs, MissionLogic, MissionParams, classes_seen)
from clothoid_mission.speed_bump_detector import (
    SpeedBumpParams, quaternion_to_pitch_deg)


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
        bp = d.bump_params
        self.declare_parameter('slow_speed', d.slow_speed)
        self.declare_parameter('slow_hold_sec', d.slow_hold_sec)
        self.declare_parameter('bump_pitch_thresh', bp.pitch_climb_thresh)
        self.declare_parameter('bump_level_thresh', bp.pitch_level_thresh)
        self.declare_parameter('bump_level_duration', bp.level_duration_sec)
        self.declare_parameter('bump_arm_timeout', bp.arm_timeout_sec)
        self.declare_parameter('bump_cooldown', bp.cooldown_sec)

        self.declare_parameter('pedestrian_wait_sec', d.pedestrian_wait_sec)
        self.declare_parameter('stop_on_yellow', d.stop_on_yellow)
        self.declare_parameter('arm_timeout_sec', d.arm_timeout_sec)
        self.declare_parameter('cooldown_sec', d.cooldown_sec)
        self.declare_parameter('fork_left_direction', d.fork_left_direction)
        self.declare_parameter('fork_right_direction', d.fork_right_direction)

        get = lambda name: self.get_parameter(name).value  # noqa: E731
        bump_params = SpeedBumpParams(
            slow_speed=get('slow_speed'),
            pitch_climb_thresh=get('bump_pitch_thresh'),
            pitch_level_thresh=get('bump_level_thresh'),
            level_duration_sec=get('bump_level_duration'),
            arm_timeout_sec=get('bump_arm_timeout'),
            cooldown_sec=get('bump_cooldown')
        )
        self.logic = MissionLogic(MissionParams(
            slow_speed=get('slow_speed'), slow_hold_sec=get('slow_hold_sec'),
            bump_params=bump_params,
            pedestrian_wait_sec=get('pedestrian_wait_sec'), stop_on_yellow=get('stop_on_yellow'),
            arm_timeout_sec=get('arm_timeout_sec'), cooldown_sec=get('cooldown_sec'),
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
        self.get_logger().info('mission started with SpeedBumpDetector enabled')

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_detections(self, msg: DetectionArray):
        self.seen, self.seen_time = classes_seen(msg.detections), self.now_sec()

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
