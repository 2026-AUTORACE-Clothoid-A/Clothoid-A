"""ROS wiring for FinishCore: after /parking_done, yellow-line follow to the end line and stop.

Inputs: colour image (raw by default), camera_info (intrinsics), /imu, /odom, /parking_done.
The camera is subscribed only while following; with lazy_sensors (default) /imu and /odom only from
/parking_flag (parking starts, seconds before /parking_done) until STOP/ABORT.
Output: /cmd_vel_finish (twist_mux, same priority as parking), /finish_done, /finish/state,
/finish/debug_image with debug:=true. Warp and camera pose come from clothoid_lane's
camera_limo*.yaml ("/**" parameters), the yellow HSV from finish.yaml.
"""
import math
from dataclasses import fields

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu
from std_msgs.msg import Bool, String

from clothoid_common.qos import IMAGE_QOS
from clothoid_common.quiet_node import QuietEventsMixin
from clothoid_finish.bev import Bev, CameraModel
from clothoid_finish.finish_core import FinishCore, FinishParams, Phase
from clothoid_lane.lane_detector import LaneParams

STALE_SEC = 0.5


class FinishNode(QuietEventsMixin, Node):
    def __init__(self):
        super().__init__('finish')
        for name, default in (('image_topic', '/camera/color/image_raw'),
                              ('camera_info_topic', '/camera/color/camera_info'),
                              ('imu_topic', '/imu'), ('odom_topic', '/odom'),
                              ('start_topic', '/parking_done'), ('arm_topic', '/parking_flag'),
                              ('cmd_topic', '/cmd_vel_finish'),
                              ('done_topic', '/finish_done'), ('state_topic', '/finish/state'),
                              ('debug_topic', '/finish/debug_image')):
            self.declare_parameter(name, default)
        self.declare_parameter('compressed', False)
        # /imu (100 Hz) + /odom (50 Hz) cost ~20 % of an Orin core just to receive; finish only needs
        # them from the parking run on, so they are subscribed on arm_topic true (false: always)
        self.declare_parameter('lazy_sensors', True)
        self.declare_parameter('wheelbase', 0.2)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('debug', False)
        # warp and camera pose: clothoid_lane camera_limo*.yaml
        for name, default in (('warp_x_h', 60), ('warp_x_l', 327), ('warp_y_h', 91), ('warp_y_l', 2)):
            self.declare_parameter(name, default)
        for name, default in (('camera_height', 0.19), ('camera_pitch', -0.021), ('camera_x', 0.15),
                              ('camera_y', -0.01)):
            self.declare_parameter(name, default)
        for f in fields(FinishParams):
            value = getattr(FinishParams, f.name)
            self.declare_parameter(f.name, list(value) if isinstance(value, tuple) else value)
        get = lambda n: self.get_parameter(n).value  # noqa: E731
        self.params = FinishParams(**{f.name: (tuple(get(f.name)) if isinstance(getattr(FinishParams, f.name), tuple)
                                               else get(f.name)) for f in fields(FinishParams)})
        self.warp = LaneParams(x_h=get('warp_x_h'), x_l=get('warp_x_l'), y_h=get('warp_y_h'), y_l=get('warp_y_l'))
        self.cam = CameraModel(height=get('camera_height'), pitch=get('camera_pitch'),
                               x=get('camera_x'), y=get('camera_y'))
        self.core = None              # built once camera_info gives the intrinsics
        self.wheelbase = get('wheelbase')
        self.debug = get('debug')
        self.compressed = get('compressed')
        self.bridge = CvBridge()
        self.last = {'imu': None, 'odom': None, 'image': None}
        self.pending = []             # sensor calls before the core exists (only imu/odom matter)
        self.phase = Phase.IDLE
        self.t_done = 0.0

        self.cmd_pub = self.create_publisher(Twist, get('cmd_topic'), 1)
        self.done_pub = self.create_publisher(Bool, get('done_topic'), 1)
        self.state_pub = self.create_publisher(String, get('state_topic'), 1)
        self.debug_pub = self.create_publisher(Image, get('debug_topic'), 1) if self.debug else None
        # the camera is only subscribed while following (after /parking_done until STOP/ABORT):
        # receiving every frame while idle cost CPU for nothing, and a compressed subscriber makes
        # the camera driver JPEG-encode every frame
        self.image_type = CompressedImage if self.compressed else Image
        self.image_sub = None
        self.create_subscription(CameraInfo, get('camera_info_topic'), self.on_info, qos_profile_sensor_data)
        self.sensor_subs = []
        if get('lazy_sensors'):
            self.create_subscription(Bool, get('arm_topic'), self.on_arm, 1)
        else:
            self._sensors_on(True)
        self.create_subscription(Bool, get('start_topic'), self.on_start, 1)
        self.create_timer(1.0 / get('rate_hz'), self.on_timer)
        self.get_logger().info('finish ready: waiting for camera_info and the parking to end')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def stamp(self, msg) -> float:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return t if t > 0 else self.now()

    def on_info(self, msg: CameraInfo):
        if self.core is not None or msg.k[0] <= 0:
            return
        self.cam.fx, self.cam.fy, self.cam.cx, self.cam.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        self.warp.width, self.warp.height = msg.width, msg.height
        bev = Bev(self.warp, self.cam)
        self.core = FinishCore(bev, self.params)
        near, far = bev.near_far()
        self.get_logger().info(f'camera fx {self.cam.fx:.1f} cy {self.cam.cy:.1f}: '
                               f'warp sees {near:.2f}~{far:.2f} m ahead of base_link')

    def on_imu(self, msg: Imu):
        self.last['imu'] = self.now()
        if self.core:
            self.core.imu(self.stamp(msg), msg.angular_velocity.z)

    def on_odom(self, msg: Odometry):
        self.last['odom'] = self.now()
        if self.core:
            self.core.odom(self.stamp(msg), msg.twist.twist.linear.x)

    def on_arm(self, msg: Bool):
        if msg.data and self.phase == Phase.IDLE:
            self._sensors_on(True)

    def on_start(self, msg: Bool):
        if msg.data and self.core and self.core.phase == Phase.IDLE:
            self._sensors_on(True)      # no-op when /parking_flag armed them; stale() holds until data
            self.core.start(self.now())
            self._image_on(True)

    def _sensors_on(self, on: bool):
        get = lambda n: self.get_parameter(n).value  # noqa: E731
        if on and not self.sensor_subs:
            self.sensor_subs = [
                self.create_subscription(Imu, get('imu_topic'), self.on_imu, qos_profile_sensor_data),
                self.create_subscription(Odometry, get('odom_topic'), self.on_odom, qos_profile_sensor_data)]
            self.get_logger().info('imu/odom subscribed')
        elif not on and self.sensor_subs and get('lazy_sensors'):
            for sub in self.sensor_subs:
                self.destroy_subscription(sub)
            self.sensor_subs = []
            self.get_logger().info('imu/odom released')

    def _image_on(self, on: bool):
        if on and self.image_sub is None:
            self.image_sub = self.create_subscription(
                self.image_type, self.get_parameter('image_topic').value, self.on_image, IMAGE_QOS)
            self.get_logger().info('camera subscribed')
        elif not on and self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
            self.get_logger().info('camera released')

    def on_image(self, msg):
        if self.core is None or self.core.phase not in (Phase.FOLLOW, Phase.COMMIT):
            return
        self.last['image'] = self.now()
        if self.compressed:
            bgr = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        else:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if bgr is None:
            return
        self.core.image(self.now(), bgr)
        if self.debug_pub is not None:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(self.debug_image(), 'bgr8'))

    def debug_image(self) -> np.ndarray:
        core = self.core
        img = np.zeros(core.mask.shape + (3,), np.uint8)
        img[core.mask] = (0, 200, 255)
        gx, gy = core.bev.gx, core.bev.gy
        if core.left is not None:
            off, hd = core.left
            line = np.abs(gy - (off + math.tan(hd) * gx)) < 0.006
            img[line & core.bev.valid] = (0, 255, 0)
        if core.cross_x is not None:
            img[(np.abs(gx - core.cross_x) < 0.004) & core.bev.valid] = (0, 0, 255)
        cv2.putText(img, core.phase.value, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        return img

    def stale(self):
        t = self.now()
        for k in ('imu', 'odom', 'image'):
            if self.last[k] is None or t - self.last[k] > STALE_SEC:
                return k
        return None

    def send(self, v, steer):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(v * math.tan(steer) / self.wheelbase) if v else 0.0
        self.cmd_pub.publish(msg)

    def on_timer(self):
        if self.core is None:
            return
        now = self.now()
        out = self.core.command(now)
        if self.core.phase != self.phase:
            self.phase = self.core.phase
            text = f'{self.phase.value}: {self.core.reason}'
            self.state_pub.publish(String(data=text))
            self.get_logger().info(text)
            if self.phase in (Phase.STOP, Phase.ABORT):
                self._image_on(False)
                self._sensors_on(False)
        if out is None:
            return
        if self.phase in (Phase.FOLLOW, Phase.COMMIT) and self.stale():
            self.send(0.0, 0.0)
            self.get_logger().warn(f'no {self.stale()}: holding', throttle_duration_sec=1.0)
            return
        self.send(*out)
        if self.phase == Phase.STOP and now - self.t_done >= 1.0:
            self.done_pub.publish(Bool(data=True))
            self.t_done = now


def main(args=None):
    rclpy.init(args=args)
    node = FinishNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()
