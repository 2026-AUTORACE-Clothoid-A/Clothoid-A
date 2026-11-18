"""Replay LaneMain and the camera exit callback on one bag clock, in memory."""
from contextlib import contextmanager
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.bump_viewer.backend import capture_publishers, decode

import numpy as np
import cv2
import rclpy
from rclpy.clock import ClockType
from rclpy.time import Time
from std_msgs.msg import Bool
from clothoid_lane.lane_main import LaneMainNode
from clothoid_lidar.cone_follower import FollowResult
from clothoid_lidar.cone_follower_node import ConeFollowerNode


class CameraExitReplay:
    def __init__(self, lane, follower):
        self.lane, self.follower = lane, follower
        self.stamp = 0
        clock = SimpleNamespace(now=lambda: Time(nanoseconds=self.stamp, clock_type=ClockType.ROS_TIME))
        lane.get_clock = follower.get_clock = lambda: clock
        self.lp = capture_publishers(lane)
        self.fp = capture_publishers(follower)
        self.lp['cone_candidate_pub'].callback = follower.on_lane_candidate
        self.lp['cone_ready_pub'].callback = follower.on_lane_ready
        self.fp['connecting_pub'].callback = lane.on_cone_connecting
        self.completed = False
        self.last_image = None
        self.inputs = {lane.get_parameter(key+'_topic').value: callback for key, callback in [
            ('direction', lane.on_direction), ('parking', lane.on_parking),
            ('speed_limit', lane.on_speed_limit), ('steering_lock', lane.on_steering_lock),
            ('bump_approach', lane.on_bump_approach), ('yaw_hold', lane.on_yaw_hold)]}
        self.inputs['/parking/approach_speed_limit'] = lane.on_parking_approach

    def image(self, message):
        self.lp['cmd_pub'].last = None
        result = self.lane.process_bgr(decode(message), message.header)
        cmd = self.lp['cmd_pub'].last
        self.last_image = (result.found, None if cmd is None else cmd.linear.x)

    def flags(self, active, exiting):
        if active != self.follower.active:
            self.lane.on_cone_active(Bool(data=active))
            self.follower.on_active(Bool(data=active))
        if exiting != self.follower.exiting or exiting != self.lane.cone_handover.exit_requested:
            self.lane.cone_handover.request_exit(exiting)
            self.follower.on_exiting(Bool(data=exiting))

    def step(self, obstacles):
        f = self.follower
        f.obstacles, f.obstacles_time = obstacles, f.get_clock().now()
        self.fp['exit_done_pub'].last = None
        f.exit_control()
        cmd = self.fp['cmd_pub'].last
        angle = math.atan2(f.exit_params.wheelbase * cmd.angular.z, cmd.linear.x)
        path = self.fp['center_pub'].last
        center = np.array([(p.pose.position.x, p.pose.position.y) for p in path.poses]
                          if cmd.linear.x and path else []).reshape(-1, 2)
        result = FollowResult(cmd.linear.x, angle, center=center)
        result.source = f.exit_logged[0]
        self.completed = self.fp['exit_done_pub'].last is not None
        return result


@contextmanager
def camera_exit_replay():
    os.environ['ROS_DOMAIN_ID'] = '196'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    cv2.setNumThreads(1)
    args = ['--ros-args']
    for config in ('clothoid_bringup/config/topics.yaml', 'clothoid_lane/config/lane.yaml',
                   'clothoid_lane/config/camera_limo2.yaml', 'clothoid_lidar/config/lidar.yaml'):
        args.extend(['--params-file', str(ROOT/'ros2_ws/src'/config)])
    rclpy.init(args=args)
    nodes = []
    try:
        lane = LaneMainNode()
        nodes.append(lane)
        follower = ConeFollowerNode()
        nodes.append(follower)
        yield CameraExitReplay(lane, follower)
    finally:
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()
