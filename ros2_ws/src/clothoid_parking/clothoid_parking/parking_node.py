#!/usr/bin/env python3
"""T-parking node (2026): find the empty bay with the lidar, one forward arc in, stop.

Idle until `/parking_flag` turns true, then drives on `cmd_topic` (twist_mux input `parking`, above
lane) through SEARCH -> TURN -> ENTER -> DONE (see planner.py). In DONE it keeps publishing zero
speed so lane cannot take the car back, and repeats `/parking_done` true once a second.
Phase changes go to the log and to `state_topic`. With `debug:=true` it also publishes RViz
markers on `marker_topic` (see viz.py, `ros2 launch clothoid_parking rviz.launch.py`).
With `lazy_sensors` (default) /imu, /odom and /scan are subscribed only from the flag until DONE: the
flag subscribes them and the run starts once all three arrive (at most `arm_wait_sec`).
"""
import math
from dataclasses import fields

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray

from clothoid_common.quiet_node import QuietEventsMixin
from clothoid_parking.bay_finder import BayParams
from clothoid_parking.core import ParkingCore
from clothoid_parking.planner import Phase, PlanParams
from clothoid_parking.viz import markers

STALE_SEC = 0.5


class ParkingNode(QuietEventsMixin, Node):
    def __init__(self):
        super().__init__('parking')
        for name, default in (('scan_topic', '/scan'), ('imu_topic', '/imu'), ('odom_topic', '/odom'),
                              ('parking_topic', '/parking_flag'), ('cmd_topic', '/cmd_vel_parking'),
                              ('parking_done_topic', '/parking_done'),
                              ('state_topic', '/parking/state'),
                              ('marker_topic', '/parking/markers')):
            self.declare_parameter(name, default)
        self.declare_parameter('wheelbase', 0.2)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('laser_x', 0.103)
        self.declare_parameter('debug', False)
        # /imu 100 Hz + /odom 50 Hz + /scan cost ~20-25 % of an Orin core just to receive while the
        # car is nowhere near the parking; ParkingCore uses nothing from before the flag
        self.declare_parameter('lazy_sensors', True)
        self.declare_parameter('arm_wait_sec', 1.0)
        plan = self._declare_dataclass(PlanParams(), 'plan.')
        bay = self._declare_dataclass(BayParams(), 'bay.')
        get = lambda n: self.get_parameter(n).value  # noqa: E731

        self.core = ParkingCore(plan, bay, laser_x=get('laser_x'))
        self.wheelbase = get('wheelbase')
        self.last = {'imu': None, 'odom': None, 'scan': None}
        self.phase = self.core.phase
        self.t_done_sent = 0.0
        self.sent_final_stop = False
        self.lazy = get('lazy_sensors')
        self.arm_wait = get('arm_wait_sec')
        self.t_armed = None           # flag seen, waiting for the sensors (lazy_sensors)
        self.sensor_subs = []

        self.cmd_pub = self.create_publisher(Twist, get('cmd_topic'), 1)
        self.done_pub = self.create_publisher(Bool, get('parking_done_topic'), 1)
        self.state_pub = self.create_publisher(String, get('state_topic'), 1)
        if not self.lazy:
            self._sensors_on(True)
        self.create_subscription(Bool, get('parking_topic'), self.on_flag, 1)
        self.create_timer(1.0 / get('rate_hz'), self.on_timer)
        if get('debug'):
            self.marker_pub = self.create_publisher(MarkerArray, get('marker_topic'), 1)
            self.create_timer(0.2, lambda: self.marker_pub.publish(
                markers(self.core, self.get_clock().now().to_msg())))
        self.get_logger().info(
            f'parking ready: R {plan.turn_radius} m, bays on the {"right" if plan.side < 0 else "left"}')

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

    def on_odom(self, msg: Odometry):
        self.last['odom'] = self.now()
        self.core.odom(self.stamp(msg), msg.twist.twist.linear.x)

    def on_scan(self, msg: LaserScan):
        self.last['scan'] = self.now()
        self.core.scan(self.last['scan'], msg.ranges, msg.angle_min, msg.angle_increment,
                       msg.range_min, msg.range_max)

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
        if msg.data and not self.core.flag:
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
