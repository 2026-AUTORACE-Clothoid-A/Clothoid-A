"""Reuse LaneMain's image and fitted boundary points; publish only a compact candidate pose."""
import math
import cv2
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data
from clothoid_lane.parking_landmark import LandmarkParams, ParkingLandmark
from clothoid_lane.lane_detector import make_warp_matrix


class LandmarkPublisher:
    def __init__(self, node):
        self.node = node
        self.prepare = self.active = False
        self.last = -math.inf
        self.info = None
        self.always_observe = node.declare_parameter('parking_landmark.always_observe', False).value
        self.enabled = node.declare_parameter('parking_landmark.enabled', True).value
        self.hz = node.declare_parameter('parking_landmark.rate_hz', 5.).value
        if self.hz <= 0:
            raise ValueError('parking_landmark.rate_hz must be positive')
        p = LandmarkParams()
        for key in p.__dataclass_fields__:
            name = key if key.startswith('camera_') else 'parking_landmark.'+key
            if not node.has_parameter(name):
                node.declare_parameter(name, getattr(p, key))
            setattr(p, key, node.get_parameter(name).value)
        self.detector = ParkingLandmark(p)
        self.pub = node.create_publisher(PoseWithCovarianceStamped, '/parking/entry_candidate', 1)
        node.create_subscription(CameraInfo, '/camera/color/camera_info', self.on_info, qos_profile_sensor_data)
        node.create_subscription(Bool, '/parking_prepare', self.on_prepare, 1)
        node.create_subscription(Bool, '/parking_flag', self.on_flag, 1)
        node.create_subscription(Bool, '/parking_done', self.on_done, 1)

    def on_info(self, msg):
        # Do not apply rational/pinhole equations to equidistant cameras.
        self.info = msg if msg.distortion_model in ('plumb_bob', 'rational_polynomial', '') else None

    def on_prepare(self, msg):
        self.prepare = msg.data

    def on_flag(self, msg):
        self.active = msg.data

    def on_done(self, msg):
        if msg.data:
            self.active = self.prepare = False

    def process(self, bgr, header, result, warp):
        if not self.enabled or not (self.always_observe or self.prepare or self.active) or self.info is None:
            return
        t = header.stamp.sec+header.stamp.nanosec*1e-9
        now = self.node.get_clock().now().nanoseconds*1e-9
        if not -.05 <= now-t <= .5 or t-self.last < 1/self.hz:
            return
        self.last = t
        info = self.info
        try:
            self.detector.configure_camera(info.width, info.height, info.k, info.d)
        except (ValueError, cv2.error) as exc:
            self.node.get_logger().warning(f'parking landmark calibration unavailable: {exc}',
                                           throttle_duration_sec=5.)
            return
        inv = np.linalg.inv(make_warp_matrix(warp))
        gx, gy, _ = self.detector.lookup
        sides = []
        for side in (result.left_points, result.right_points):
            if not len(side):
                continue
            pixels = cv2.perspectiveTransform(np.asarray(side, np.float32).reshape(-1, 1, 2), inv).reshape(-1, 2)
            pixels = np.rint(pixels).astype(int)
            valid = (pixels[:, 0]>=0)&(pixels[:, 0]<info.width)&(pixels[:, 1]>=0)&(pixels[:, 1]<info.height)
            x, y = pixels[valid].T
            sides.append(np.column_stack((gx[y, x], gy[y, x])))
        obs = self.detector.detect(bgr, sides)
        if obs is None:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = header.stamp
        msg.header.frame_id = 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = obs['origin']
        msg.pose.pose.orientation.z = math.sin(obs['angle']/2)
        msg.pose.pose.orientation.w = math.cos(obs['angle']/2)
        # Conservative measurement uncertainty, not a claim of calibrated absolute accuracy.
        msg.pose.covariance[0] = msg.pose.covariance[7] = .07**2
        msg.pose.covariance[35] = math.radians(3)**2
        self.pub.publish(msg)
