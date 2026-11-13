#!/usr/bin/env python3
"""T-parking node (2026): find the empty bay with the lidar, one forward arc in, stop.

Idle until `/parking_flag` turns true, then drives on `cmd_topic` (twist_mux input `parking`, above
lane) through SEARCH -> TURN -> ENTER -> DONE (see planner.py). In DONE it keeps publishing zero
speed so lane cannot take the car back, and repeats `/parking_done` true once a second.
Phase changes go to the log and to `state_topic`. With `debug:=true` it also publishes RViz
markers on `marker_topic` (see viz.py, `ros2 launch clothoid_parking rviz.launch.py`).
Before that flag, /parking_prepare enables only a passive /scan subscription. It confirms a bay
at at most probe.rate_hz and publishes /parking/bay_candidate without sending a command.
With lazy_sensors, high-rate /imu and /odom still start only at the actual parking flag.
"""
import math
from dataclasses import fields

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped, PoseArray, Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String, Float32
from visualization_msgs.msg import MarkerArray

from clothoid_parking.bay_finder import BayParams
from clothoid_parking.core import ParkingCore
from clothoid_parking.approach import ApproachParams, ParkingApproach
from clothoid_parking.layout import LayoutParams
from clothoid_parking.preview import ParkingPreview
from clothoid_parking.planner import Phase, PlanParams
from clothoid_parking.scene_probe import ParkingSceneProbe, ProbeParams
from clothoid_parking.viz import markers, layout_markers, observation_markers, preview_markers

STALE_SEC = 0.5


class ParkingNode(Node):
    def __init__(self):
        super().__init__('parking')
        for name, default in (('scan_topic', '/scan'), ('imu_topic', '/imu'), ('odom_topic', '/odom'),
                              ('parking_topic', '/parking_flag'), ('cmd_topic', '/cmd_vel_parking'),
                              ('parking_prepare_topic', '/parking_prepare'),
                              ('parking_scene_topic', '/parking/bay_candidate'),
                              ('parking_done_topic', '/parking_done'),
                              ('state_topic', '/parking/state'),
                              ('marker_topic', '/parking/markers')):
            self.declare_parameter(name, default)
        self.declare_parameter('wheelbase', 0.2)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('laser_x', 0.103)
        self.declare_parameter('debug', False)
        self.inspection_only = self.declare_parameter('inspection_only', False).value
        # /imu 100 Hz + /odom 50 Hz + /scan cost ~20-25 % of an Orin core just to receive while the
        # car is nowhere near the parking; ParkingCore uses nothing from before the flag
        self.declare_parameter('lazy_sensors', True)
        self.declare_parameter('arm_wait_sec', 1.0)
        plan = self._declare_dataclass(PlanParams(), 'plan.')
        bay = self._declare_dataclass(BayParams(), 'bay.')
        probe = self._declare_dataclass(ProbeParams(), 'probe.')
        layout = self._declare_dataclass(LayoutParams(), 'layout.')
        self.approach = ParkingApproach(self._declare_dataclass(ApproachParams(), 'approach.'))
        if self.inspection_only:
            layout.enabled, layout.control_enabled = True, False
        get = lambda n: self.get_parameter(n).value  # noqa: E731

        self.core = ParkingCore(plan, bay, laser_x=get('laser_x'), layout=layout)
        self.preview = ParkingPreview(plan, bay, get('laser_x')) if self.inspection_only else None
        self.probe = ParkingSceneProbe(bay, probe, laser_x=get('laser_x'))
        self.probe_sub = None
        self.wheelbase = get('wheelbase')
        self.last = {'imu': None, 'odom': None, 'scan': None}
        self.phase = self.core.phase
        self.t_done_sent = 0.0
        self.sent_final_stop = False
        self.lazy = get('lazy_sensors')
        self.arm_wait = get('arm_wait_sec')
        self.entry_distance = self.entry_distance_stamp = None
        self.lane_steer, self.lane_command_stamp = 0., None
        self.t_armed = None           # flag seen, waiting for the sensors (lazy_sensors)
        self.sensor_subs = []

        self.cmd_pub = self.create_publisher(Twist, get('cmd_topic'), 1)
        self.done_pub = self.create_publisher(Bool, get('parking_done_topic'), 1)
        self.state_pub = self.create_publisher(String, get('state_topic'), 1)
        self.approach_pub = self.create_publisher(Float32, '/parking/approach_speed_limit', 1)
        self.observing_pub = self.create_publisher(Bool, '/parking/observing', 1)
        self.skipped_pub = self.create_publisher(Bool, '/parking/skipped', 1)
        self.approach_status_pub = self.create_publisher(String, '/parking/approach_status', 1)
        if self.preview is not None:
            self.preview_pub = self.create_publisher(MarkerArray, '/parking/preview_markers', 1)
            self.create_timer(.2, self.publish_preview)
        self.observation_pub = self.create_publisher(MarkerArray, '/parking/observation_markers', 1)
        self.layout_marker_pub = self.create_publisher(MarkerArray, '/parking/layout_markers', 1)
        self.layout_pub = self.create_publisher(PoseArray, '/parking/layout', 1)
        self.layout_status_pub = self.create_publisher(String, '/parking/layout_status', 1)
        self.create_subscription(PoseWithCovarianceStamped, '/parking/entry_candidate', self.on_entry, 1)
        self.create_subscription(Float32, '/parking/entry_distance', self.on_entry_distance, 1)
        self.create_subscription(Twist, '/cmd_vel_lane', self.on_lane_command, 1)
        self.create_timer(.5, self.publish_layout)
        self.candidate_pub = self.create_publisher(PoseStamped, get('parking_scene_topic'), 1)
        if not self.lazy:
            self._sensors_on(True)
        self.create_subscription(Bool, get('parking_topic'), self.on_flag, 1)
        self.create_subscription(Bool, get('parking_prepare_topic'), self.on_prepare, 1)
        self.create_timer(1.0 / get('rate_hz'), self.on_timer)
        if get('debug'):
            self.marker_pub = self.create_publisher(MarkerArray, get('marker_topic'), 1)
            self.create_timer(0.2, lambda: self.marker_pub.publish(
                markers(self.core, self.get_clock().now().to_msg())))
        self.get_logger().info(
            f'parking ready: R {plan.turn_radius} m, bays on the {"right" if plan.side < 0 else "left"}')

        if self.inspection_only:
            self.on_prepare(Bool(data=True))

    def _declare_dataclass(self, obj, prefix):
        for f in fields(obj):
            value = self.declare_parameter(prefix + f.name, getattr(obj, f.name)).value
            setattr(obj, f.name, type(getattr(obj, f.name))(value))
        return obj

    def now(self) -> float:
        # node clock: simulated time in the simulator (use_sim_time), wall time on the car
        return self.get_clock().now().nanoseconds * 1e-9

    def stamp(self, msg) -> float:
        """Message time for integration; the receive time can bunch up (the simulator's /clock
        ticks coarsely), which drops or doubles samples."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return t if t > 0 else self.now()

    def on_imu(self, msg: Imu):
        self.last['imu'] = self.now()
        self.core.imu(self.stamp(msg), msg.angular_velocity.z)
        if self.preview is not None:
            self.preview.core.imu(self.stamp(msg), msg.angular_velocity.z)

    def on_odom(self, msg: Odometry):
        self.last['odom'] = self.now()
        self.core.odom(self.stamp(msg), msg.twist.twist.linear.x)
        if self.preview is not None:
            self.preview.core.odom(self.stamp(msg), msg.twist.twist.linear.x)

    def on_scan(self, msg: LaserScan):
        self.last['scan'] = self.now()
        self.core.scan(self.stamp(msg), msg.ranges, msg.angle_min, msg.angle_increment,
                       msg.range_min, msg.range_max)
        if self.preview is not None:
            self.preview.core.scan(self.stamp(msg), msg.ranges, msg.angle_min, msg.angle_increment,
                                   msg.range_min, msg.range_max)
        if self.core.layout is not None and self.core.preparing:
            self.on_probe_scan(msg)

    def on_lane_command(self, msg):
        v, w = msg.linear.x, msg.angular.z
        if not math.isfinite(v) or not math.isfinite(w):
            return
        if abs(v) > .001:
            self.lane_steer = max(-self.core.planner.p.max_steer, min(self.core.planner.p.max_steer,
                                 math.atan(self.wheelbase*w/v)))
        self.lane_command_stamp = self.now()

    def on_entry_distance(self, msg):
        self.entry_distance = msg.data if math.isfinite(msg.data) else None
        self.entry_distance_stamp = self.now()

    def on_entry(self, msg):
        stamp = self.stamp(msg)
        if msg.header.frame_id != 'base_link' or not -.05 <= self.now()-stamp <= .5:
            return
        q = msg.pose.pose.orientation
        values = (q.x, q.y, q.z, q.w, msg.pose.pose.position.x, msg.pose.pose.position.y)
        if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[:4])-1) > .01:
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.core.observe_entry(stamp, (msg.pose.pose.position.x, msg.pose.pose.position.y), yaw)

    def publish_preview(self):
        if self.preview is not None:
            self.preview_pub.publish(preview_markers(self.preview, self.get_clock().now().to_msg()))

    def publish_layout(self):
        layout = self.core.layout
        if layout is None or self.core.phase in (Phase.DONE, Phase.ABORT) or not (self.core.preparing or self.core.flag):
            return
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        current = self.core.tracker.pose
        c, s = math.cos(current.yaw), math.sin(current.yaw)
        for bay in layout.bays():
            pose = Pose()
            dx, dy = bay.x-current.x, bay.y-current.y
            pose.position.x, pose.position.y = float(c*dx+s*dy), float(-s*dx+c*dy)
            yaw = bay.entry_yaw-current.yaw
            pose.orientation.z, pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            msg.poses.append(pose)
        self.layout_pub.publish(msg)
        self.layout_marker_pub.publish(layout_markers(self.core, msg.header.stamp))
        if self.observation_pub.get_subscription_count():
            self.observation_pub.publish(observation_markers(self.core, msg.header.stamp, self.now()))
        mode = 'control' if layout.p.control_enabled else 'observe'
        self.layout_status_pub.publish(String(data=mode+': '+layout.reason+'; '+', '.join(
            f'{i+1}:{state}' for i, state in enumerate(layout.states))))

    def on_prepare(self, msg: Bool):
        on = (msg.data or self.inspection_only) and self.core.phase == Phase.IDLE and self.t_armed is None
        if on != self.core.preparing:
            self.probe.reset()
        self.core.prepare(on)
        if self.core.layout is not None and on:
            self._sensors_on(True)
        elif self.core.layout is not None and not on and not msg.data and self.core.phase == Phase.IDLE and self.t_armed is None:
            self._sensors_on(False)
        if on and self.probe_sub is None and self.core.layout is None:
            self.probe.reset()
            self.probe_sub = self.create_subscription(
                LaserScan, self.get_parameter('scan_topic').value, self.on_probe_scan, qos_profile_sensor_data)
        elif not on and self.probe_sub is not None:
            self.destroy_subscription(self.probe_sub)
            self.probe_sub = None
            self.probe.reset()

    def on_probe_scan(self, msg: LaserScan):
        if self.core.phase != Phase.IDLE or self.t_armed is not None:
            return
        now, stamp = self.now(), self.stamp(msg)
        if not -.05 <= now - stamp <= STALE_SEC:
            return
        layout = self.core.layout
        if self.approach.skipped:
            return
        if layout is not None and layout.p.control_enabled:
            if layout.selected is None and self.core.approach_bay is None:
                return
            bay = (layout.bays()[layout.selected] if layout.selected is not None
                   else self.core.approach_bay)
            if layout.selected is None:
                # Handover to incremental observation is allowed only with a
                # fresh, observed-free immediate movement. It does not certify
                # the occluded final bay or bypass mission sign/entry evidence.
                if (layout.origin is None or layout.support_count < layout.p.confirm_scans or
                        now-layout.last_scan > STALE_SEC or self.stale() or
                        not layout.motion_clear(now, self.core.tracker.pose,
                                                self.core.planner.p.search_speed, 0., self.wheelbase)):
                    return
            pose = self.core.tracker.pose
            c, s = math.cos(pose.yaw), math.sin(pose.yaw)
            dx, dy = bay.face_x-pose.x, bay.face_y-pose.y
            face_x, face_y = c*dx+s*dy, -s*dx+c*dy
            row_yaw = bay.row_yaw-pose.yaw
        else:
            _, bay = self.probe.scan(stamp, msg.ranges, msg.angle_min, msg.angle_increment,
                                     msg.range_min, msg.range_max)
            if bay is None:
                return
            face_x, face_y, row_yaw = bay.face_x, bay.face_y, bay.row_yaw
        # Do not invite a handover after the full-lock turn point has passed.
        along = face_x * math.cos(row_yaw) + face_y * math.sin(row_yaw)
        if along < self.core.planner.p.turn_radius + self.core.planner.p.turn_lead:
            return
        candidate = PoseStamped()
        candidate.header.stamp = msg.header.stamp
        candidate.header.frame_id = 'base_link'
        candidate.pose.position.x, candidate.pose.position.y = float(face_x), float(face_y)
        candidate.pose.orientation.z = math.sin(row_yaw / 2)
        candidate.pose.orientation.w = math.cos(row_yaw / 2)
        self.candidate_pub.publish(candidate)

    def _sensors_on(self, on: bool):
        get = lambda n: self.get_parameter(n).value  # noqa: E731
        if on and not self.sensor_subs:
            self.sensor_subs = [
                self.create_subscription(Imu, get('imu_topic'), self.on_imu, qos_profile_sensor_data),
                self.create_subscription(Odometry, get('odom_topic'), self.on_odom, qos_profile_sensor_data),
                self.create_subscription(LaserScan, get('scan_topic'), self.on_scan, qos_profile_sensor_data)]
            if self.lazy:
                self.get_logger().info('imu/odom/scan subscribed')
        elif not on and self.sensor_subs and self.lazy:
            for sub in self.sensor_subs:
                self.destroy_subscription(sub)
            self.sensor_subs = []
            self.last = {k: None for k in self.last}
            self.get_logger().info('imu/odom/scan released')

    def on_flag(self, msg: Bool):
        if self.inspection_only:
            self.preview.core.set_flag(self.now(), msg.data)
            return
        if msg.data and not self.core.flag:
            if self.probe_sub is not None:
                self.destroy_subscription(self.probe_sub)
                self.probe_sub = None
                self.probe.reset()
            if self.t_armed is not None:
                return                  # already waiting for the sensors (on_timer starts the run)
            stale = self.stale()
            if stale and self.lazy and self.core.phase == Phase.IDLE:
                self._sensors_on(True)
                self.t_armed = self.now()
                return
            if stale:
                self.get_logger().error(f'parking flag but no {stale}: not starting')
                return
        if not msg.data:
            self.t_armed = None
        self.core.set_flag(self.now(), msg.data)

    def check_armed(self, now: float):
        """lazy_sensors: start the run once every sensor arrived after the flag subscribed them."""
        stale = self.stale()
        if not stale:
            self.t_armed = None
            self.get_logger().info('sensors in: parking starts')
            self.core.set_flag(now, True)
        elif now - self.t_armed > self.arm_wait:
            self.t_armed = None
            self.get_logger().error(
                f'parking flag but no {stale} after {self.arm_wait:.1f} s: not starting')

    def stale(self):
        now = self.now()
        return [k for k, t in self.last.items() if t is None or now - t > STALE_SEC]

    def send(self, v: float, steer: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(v * math.tan(steer) / self.wheelbase) if v else 0.0
        self.cmd_pub.publish(msg)

    def on_timer(self):
        now = self.now()
        if self.preview is not None:
            if not self.stale():
                self.preview.step(now)
            return  # inspection never sends commands or mission-completion flags
        cap = self.approach.update(now, self.core.preparing, self.core.phase != Phase.IDLE or self.t_armed is not None,
                                   self.core.layout, self.core.tracker.pose, self.core.planner.p,
                                   sensors_fresh=not self.stale() and self.lane_command_stamp is not None and
                                   0 <= now-self.lane_command_stamp <= STALE_SEC,
                                   speed=self.core.tracker.speed, steer=self.lane_steer, wheelbase=self.wheelbase,
                                   entry_distance=self.entry_distance if self.entry_distance_stamp is not None and
                                   0 <= now-self.entry_distance_stamp <= STALE_SEC else None)
        self.approach_pub.publish(Float32(data=-1. if cap is None else float(cap)))
        self.observing_pub.publish(Bool(data=self.approach.observing))
        self.skipped_pub.publish(Bool(data=self.approach.skipped))
        self.approach_status_pub.publish(String(data=self.approach.reason))
        if self.t_armed is not None:
            self.check_armed(now)
        out = self.core.command(now)
        if self.core.phase != self.phase:
            self.phase = self.core.phase
            text = f'{self.phase.value}: {self.core.planner.reason}'
            self.state_pub.publish(String(data=text))
            bay = self.core.planner.bay
            if bay is not None and self.phase == Phase.TURN:
                text += f' (bay gap {bay.width:.2f} m)'
            self.get_logger().info(text)
        if self.phase == Phase.DONE:
            # out of the bay: stop publishing so twist_mux hands the car back to lane after its
            # timeout (0.3 s); one zero first so the reverse does not carry on until then
            if not self.sent_final_stop:
                self.send(0.0, 0.0)
                self.sent_final_stop = True
                self._sensors_on(False)
            if now - self.t_done_sent >= 1.0:
                self.done_pub.publish(Bool(data=True))
                self.t_done_sent = now
            return
        if out is None:
            return
        moving = (Phase.SEARCH, Phase.TURN, Phase.ENTER, Phase.EXIT_BACK, Phase.EXIT_TURN)
        if self.phase in moving and self.stale():
            if self.core.layout is not None:
                self.core.layout.retain_free(now, False)
            self.send(0.0, 0.0)
            self.get_logger().warn(f'no {self.stale()}: holding', throttle_duration_sec=1.0)
            return
        self.send(*out)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingNode()
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
