#!/usr/bin/env python3
"""Check the twist_mux arm lock (clothoid_bringup/config/twist_mux_arm.yaml) with fake inputs.

Run twist_mux first, on a private domain so nothing reaches a robot:
    export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
    CFG=ros2_ws/src/clothoid_bringup/config
    /opt/ros/humble/lib/twist_mux/twist_mux --ros-args --params-file $CFG/twist_mux.yaml \\
        --params-file $CFG/twist_mux_arm.yaml -r cmd_vel_out:=/cmd_vel &
    python3 tools/mux_arm_test.py
    kill %1      # start the binary directly: killing `ros2 run` leaves the node running

Phases (PHASE seconds each) and the expected /cmd_vel with the lock loaded:
  1 lane only, no /estop            -> nothing (the lock starts locked)
  2 lane + /estop false             -> lane (0.1)
  3 lane, /estop stopped            -> lane for ~0.3 s, then nothing
  4 lane + /estop true              -> nothing
  5 teleop + lane + /estop true     -> teleop (0.5), priority 200 > lock 150
Without twist_mux_arm.yaml, lane passes in every phase. Checked 2026-11-04 on Humble.
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from std_msgs.msg import Bool

PHASE = 2.0


class Tester(Node):
    def __init__(self):
        super().__init__('arm_lock_tester')
        self.lane = self.create_publisher(Twist, '/cmd_vel_lane', 1)
        self.teleop = self.create_publisher(Twist, '/cmd_vel_teleop', 1)
        self.estop = self.create_publisher(Bool, '/estop', 1)
        self.create_subscription(Twist, '/cmd_vel', self.on_out, 10)
        self.out = []          # (time since start, linear.x)
        self.t0 = time.monotonic()
        self.create_timer(0.1, self.tick)

    def t(self):
        return time.monotonic() - self.t0

    def phase(self):
        return int(self.t() // PHASE) + 1

    def tick(self):
        p = self.phase()
        if p > 5:
            return
        self.lane.publish(Twist(linear=Vector3(x=0.1)))
        if p == 2:
            self.estop.publish(Bool(data=False))
        if p in (4, 5):
            self.estop.publish(Bool(data=True))
        if p == 5:
            self.teleop.publish(Twist(linear=Vector3(x=0.5)))

    def on_out(self, msg):
        self.out.append((self.t(), msg.linear.x))


def main():
    rclpy.init()
    node = Tester()
    time.sleep(1.0)   # discovery
    node.t0 = time.monotonic()
    while node.t() < 5 * PHASE:
        rclpy.spin_once(node, timeout_sec=0.05)
    for p in range(1, 6):
        lo, hi = (p - 1) * PHASE, p * PHASE
        msgs = [(t, x) for t, x in node.out if lo <= t < hi]
        values = sorted({round(x, 2) for _, x in msgs})
        first = f'{msgs[0][0] - lo:.2f}s' if msgs else '-'
        last = f'{msgs[-1][0] - lo:.2f}s' if msgs else '-'
        print(f'phase {p}: {len(msgs):3d} msgs on /cmd_vel, linear.x {values}, first {first}, last {last}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
