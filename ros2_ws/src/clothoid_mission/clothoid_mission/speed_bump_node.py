#!/usr/bin/env python3
"""Speed bump node (mission 5): slow from the bump sign until the IMU says the bump is passed.
Logic in speed_bump.py.

Subscribes `/detections` (YOLO) and `/imu` (pitch from the orientation).
Publishes
  `speed_limit_topic` (Float32): `slow_speed` at `rate_hz` while slowing. lane_main takes
      min(base_speed, limit) and drops the limit 0.5 s after the last message, so the car is back
      at base speed right after the bump. Never a release value (it would cancel other limits).
  `state_topic` (String): idle / slow / on_bump / passed, on change and once a second.

    ros2 launch clothoid_mission speed_bump.launch.py
"""
import rclpy
from clothoid_interfaces.msg import DetectionArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String

from clothoid_mission.speed_bump import SpeedBump, SpeedBumpParams, quaternion_to_pitch_deg

PARAMS = ('slow_speed', 'min_confidence', 'confirm_frames', 'up_deg', 'down_deg', 'level_deg',
          'level_sec', 'arm_timeout_sec', 'on_bump_max_sec', 'cooldown_sec', 'baseline_tau_sec')


class SpeedBumpNode(Node):
    def __init__(self):
        super().__init__('speed_bump')

        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('speed_limit_topic', '/speed_limit')
        self.declare_parameter('state_topic', '/speed_bump/state')
        self.declare_parameter('rate_hz', 20.0)
        d = SpeedBumpParams()
        for name in PARAMS:
            self.declare_parameter(name, getattr(d, name))

        self.logic = SpeedBump(SpeedBumpParams(
            **{name: self.get_parameter(name).value for name in PARAMS}))
        self.state = self.logic.state
        self.state_sent = -1.0

        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self.limit_pub = self.create_publisher(Float32, get('speed_limit_topic'), 1)
        self.state_pub = self.create_publisher(String, get('state_topic'), 1)
        self.create_subscription(DetectionArray, get('detections_topic'), self.on_detections, 1)
        self.create_subscription(Imu, get('imu_topic'), self.on_imu, qos_profile_sensor_data)
        self.create_timer(1.0 / get('rate_hz'), self.on_timer)
        self.get_logger().info('speed_bump started')

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_detections(self, msg: DetectionArray):
        self.logic.on_detections([(d.class_name, d.confidence) for d in msg.detections],
                                 self.now_sec())

    def on_imu(self, msg: Imu):
        q = msg.orientation
        self.logic.on_pitch(quaternion_to_pitch_deg(q.x, q.y, q.z, q.w), self.now_sec())

    def on_timer(self):
        now = self.now_sec()
        state = self.logic.update(now)
        if state != self.state:
            pitch = self.logic.pitch
            self.get_logger().info(f'{self.state} -> {state}'
                                   + (f' (pitch {pitch:+.1f} deg)' if pitch is not None else ''))
            self.state = state
            self.state_sent = -1.0
        if now - self.state_sent >= 1.0:
            self.state_pub.publish(String(data=state))
            self.state_sent = now
        limit = self.logic.limit
        if limit is not None:
            self.limit_pub.publish(Float32(data=float(limit)))


def main(args=None):
    rclpy.init(args=args)
    node = SpeedBumpNode()
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
