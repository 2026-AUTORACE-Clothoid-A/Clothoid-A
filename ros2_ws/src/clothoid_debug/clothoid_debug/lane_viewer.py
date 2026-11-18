#!/usr/bin/env python3
"""Step through a bag and watch how lane_main finds the lines and builds its target point.

    ros2 run clothoid_debug lane_viewer <bag_dir>                     # window, starts paused
    ros2 run clothoid_debug lane_viewer <bag_dir> --start 10 -p look_row=120 -p steer_gain=1.5
    ros2 run clothoid_debug lane_viewer <bag_dir> --out lane.mp4      # no window, write a video
    ros2 run clothoid_debug lane_viewer <bag_dir> --tune              # + slider window

No ROS graph is needed: the bag is read directly and every camera frame goes through clothoid_lane's
detector and steering law, built from lane.yaml + config/camera_<camera>.yaml (`-p name=value`
overrides any of them). What you see is therefore what lane_main does with those parameters.
When the bag holds /lane/target_warp (the car ran with debug:=true) only the frames the car's
lane_main processed are used, so the tracker sees the same sequence, and the car's own target and
command are drawn next to the replayed ones.

Window: camera image with the warp area (magenta) and the target projected back (green; car's: cyan
cross; yellow ring: the point steered to after target_max_step_px) | warped image with masks,
sliding windows, look row and target (see LaneTracker._draw) |
numbers | steering over the last seconds (replay green, car's command cyan, measured from the gyro
orange, grey: the 23.4 deg the car can really steer).

Keys (with the viewer window focused): space play/pause, d or -> next frame, a or <- back (last 300
frames), w/s faster/slower, p save a png next to the bag, y print the current parameters (as yaml and
as -p), q or Esc quit.

--tune opens a second window with sliders (BEV far end, look_row, steer_gain, lane width, yellow /
white thresholds, window margin, minimum windows, side). A change is applied when the slider rests
for a moment: the tracker is re-run over the last second of frames so its state fits, the shown frame
is redrawn and later frames use the new values. The replay curve of the chart restarts there.
"""
import argparse
import collections
import math
import os
import struct
import sys

import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from clothoid_lane.lane_detector import SIDE_BOTH, SIDE_LEFT, SIDE_RIGHT, make_warp_matrix
from clothoid_lane.lane_main import detector_defaults, lane_params, make_detector
from clothoid_lane.steering import (MAX_STEER_RAD, TARGET_RESET_SEC, WHEELBASE, limit_step,
                                    steer_angle)

CAR_LIMIT_DEG = 23.4      # central angle the LIMO reaches when limo_base clamps the inner wheel at 28
IMAGE_TOPICS = ('/camera/color/image_raw/compressed', '/camera/color/image_raw')
SIDES = {SIDE_LEFT: 'left', SIDE_RIGHT: 'right', SIDE_BOTH: 'both'}
# warp the car used when the bag was recorded: (until unix time, (warp_x_h, warp_x_l, warp_y_h, warp_y_l)).
# The bag only holds the car's target in its own warped pixels, so it has to be mapped through the
# car's warp, not the replay's. limo2: 0.69 m warp until 2026-11-07 ~19:40 KST, the 1.0 m warp until
# ~20:56, then the 0.9 m warp of camera_limo2.yaml (later bags: the replay's own warp).
CAR_WARP_HISTORY = {'limo2': [(1794048000, (-16, 327, 120, 2)), (1794052590, (89, 327, 80, 2))]}

KEY_RIGHT, KEY_LEFT = (65363, 2555904), (65361, 2424832)
# keys typed while the keyboard is in Hangul mode: the jamo on the same key
HANGUL_KEYS = {'ㅇ': 'd', 'ㅁ': 'a', 'ㅂ': 'q', 'ㅈ': 'w', 'ㄴ': 's', 'ㅔ': 'p'}


def read_key(delay_ms: int) -> int:
    """waitKeyEx, normalised: GTK adds modifier bits (NumLock, Shift, ...) above the key code and
    sends Hangul jamo in Hangul input mode; returns -1, an arrow code or a lower-case ASCII code."""
    key = cv2.waitKeyEx(delay_ms)
    if key == -1:
        return -1
    if key in KEY_RIGHT or key in KEY_LEFT:
        return key
    code = key & 0xFFFF
    if code in (65363, 65361):
        return code
    char = chr(code) if code < 0x110000 else ''
    char = HANGUL_KEYS.get(char, char).lower()
    if len(char) == 1 and ord(char) < 128:
        return ord(char)
    if code == 0xFF1B:              # Escape keysym
        return 27
    print('key 0x%x not used (space, d/a or arrows, w/s, p, q)' % key)
    return 0


# --- parameters -------------------------------------------------------------------------------
def load_params(camera: str, overrides) -> dict:
    share = os.path.join(get_package_share_directory('clothoid_lane'), 'config')
    with open(os.path.join(share, 'lane.yaml')) as f:
        lane = yaml.safe_load(f)['lane_main']['ros__parameters']
    with open(os.path.join(share, f'camera_{camera}.yaml')) as f:
        profile = yaml.safe_load(f)
        cam = profile['/**']['ros__parameters']
        camera_lane = profile.get('lane_main', {}).get('ros__parameters', {})
    values = {'steer_gain': 1.0, 'max_steer_rad': MAX_STEER_RAD, 'wheelbase': WHEELBASE,
              'target_max_step_px': 8.0,
              'initial_side': SIDE_RIGHT, 'lane_width_m': 0.5, 'warp_px_per_m': 340.0,
              'base_speed': 0.3}
    values.update(detector_defaults())
    values.update(lane)
    values.update(cam)                 # the camera file wins, as in the launch files
    values.update(camera_lane)
    for item in overrides:
        name, _, text = item.partition('=')
        if name not in values:
            sys.exit(f'unknown parameter {name!r}')
        values[name] = yaml.safe_load(text)
    return values


# --- bag --------------------------------------------------------------------------------------
def open_reader(bag: str, topics):
    import rosbag2_py
    with open(os.path.join(bag, 'metadata.yaml')) as f:
        storage = yaml.safe_load(f)['rosbag2_bagfile_information'].get('storage_identifier', 'sqlite3')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    available = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in available]))
    return reader, available


def stamp(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def control_mode(data: bytes):
    """limo_msgs/LimoStatus control_mode without needing limo_msgs (3 = remote control)."""
    off = 4 + 8
    off += 4 + struct.unpack_from('<I', data, off)[0]
    return data[off + 1]


def read_small_topics(bag: str) -> dict:
    """Car's own lane output and motion, for comparison with the replay."""
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import PointStamped, Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from sensor_msgs.msg import CameraInfo
    reader, _ = open_reader(bag, ['/lane/target_warp', '/cmd_vel_lane', '/imu', '/odom',
                                  '/limo_status', '/camera/color/camera_info'])
    out = {'target': {}, 'cmd': [], 'imu': [], 'odom': [], 'mode': [], 'k': None}
    while reader.has_next():
        topic, data, t = reader.read_next()
        t *= 1e-9
        if topic == '/lane/target_warp':
            m = deserialize_message(data, PointStamped)
            out['target'][round(stamp(m.header), 4)] = (m.point.x, m.point.y, m.point.z > 0.5)
        elif topic == '/cmd_vel_lane':
            m = deserialize_message(data, Twist)
            out['cmd'].append((t, m.linear.x, m.angular.z))
        elif topic == '/imu':
            m = deserialize_message(data, Imu)
            out['imu'].append((stamp(m.header), m.angular_velocity.z))
        elif topic == '/odom':
            m = deserialize_message(data, Odometry)
            out['odom'].append((stamp(m.header), m.twist.twist.linear.x))
        elif topic == '/camera/color/camera_info' and out['k'] is None:
            out['k'] = list(deserialize_message(data, CameraInfo).k)
        elif topic == '/limo_status':
            try:
                out['mode'].append((t, control_mode(bytes(data))))
            except (struct.error, IndexError):
                pass
    for k in ('cmd', 'imu', 'odom', 'mode'):
        out[k] = np.array(out[k]) if out[k] else np.zeros((0, 3 if k == 'cmd' else 2))
    return out


def frames(bag: str, topic: str, only=None):
    """(header stamp, receive time, bgr, encoded image) of every camera frame, or of the stamps in
    `only`; the encoded image (jpeg/png bytes) lets the GUI decode the frame again later."""
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage, Image
    from clothoid_common.image_utils import image_to_bgr
    reader, _ = open_reader(bag, [topic])
    compressed = topic.endswith('compressed')
    while reader.has_next():
        _, data, t = reader.read_next()
        msg = deserialize_message(data, CompressedImage if compressed else Image)
        s = stamp(msg.header)
        if only and round(s, 4) not in only:
            continue
        if compressed:
            raw = bytes(msg.data)
            bgr = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        else:
            bgr = image_to_bgr(msg)
            raw = None                    # encoded on demand (png) by the GUI
        yield s, t * 1e-9, bgr, raw


def nearest(arr: np.ndarray, t: float, window: float):
    if len(arr) == 0:
        return None
    i = int(np.argmin(np.abs(arr[:, 0] - t)))
    return arr[i] if abs(arr[i, 0] - t) <= window else None


# --- drawing ----------------------------------------------------------------------------------
def text(img, s, org, scale=0.5, colour=(255, 255, 255), thick=1):
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, colour, thick, cv2.LINE_AA)


def cross(img, p, colour, size=7):
    x, y = int(round(p[0])), int(round(p[1]))
    cv2.line(img, (x - size, y - size), (x + size, y + size), colour, 2)
    cv2.line(img, (x - size, y + size), (x + size, y - size), colour, 2)


class Viewer:
    def __init__(self, values: dict, camera: str, car_warp=None, intrinsics=None):
        self.camera = camera
        # fy, cy of the colour camera (bag camera_info, else decision_viz's defaults)
        self.fy, self.cy = intrinsics or (489.14, 217.0)
        self.reconfigure(values)
        # car's warp, to put the car's own target (its warped pixels) on the camera image and on this BEV
        car_lp = self.lp if car_warp is None else lane_params(
            lambda n: dict(values, warp_x_h=car_warp[0], warp_x_l=car_warp[1], warp_y_h=car_warp[2],
                           warp_y_l=car_warp[3])[n])
        self.car_warp = (car_lp.x_h, car_lp.x_l, car_lp.y_h, car_lp.y_l)
        self.car_inverse = np.linalg.inv(make_warp_matrix(car_lp))
        self.history = {}          # frame index -> (t, replay steer deg, car command deg, gyro deg)

    def reconfigure(self, values: dict) -> None:
        """New parameters: a new detector (empty tracking state) and warp. Raises ValueError for
        a degenerate warp and then keeps the old configuration."""
        detector = make_detector(lambda name: values[name])
        lp = lane_params(lambda name: values[name])
        self.v, self.detector, self.lp = dict(values), detector, lp
        self.side = int(values['initial_side'])
        self.lane_width = values['lane_width_m'] * values['warp_px_per_m']
        self.matrix = make_warp_matrix(lp)
        self.inverse = np.linalg.inv(self.matrix)
        self.used_target, self.last_found_t = None, None   # lane_main's target_max_step_px state

    def reset_tracking(self) -> None:
        self.detector = make_detector(lambda name: self.v[name])
        self.used_target, self.last_found_t = None, None

    def ground_distance(self, warp_row: float):
        """Metres ahead of the camera of a warped row (camera_height / camera_pitch of the profile)."""
        v_img = self.to_image(self.lp.warp_w / 2, warp_row, 1.0)[1]
        below = math.atan((v_img - self.cy) / self.fy) + self.v.get('camera_pitch', 0.0)
        return self.v.get('camera_height', 0.19) / math.tan(below) if below > 1e-3 else float('inf')

    def to_image(self, x, y, scale, inverse=None):
        p = (self.inverse if inverse is None else inverse) @ np.array([x, y, 1.0])
        return p[0] / p[2] * scale, p[1] / p[2] * scale

    def car_to_warp(self, x, y):
        """Car's target (car's warped px) -> camera px -> this viewer's warped px."""
        ix, iy = self.to_image(x, y, 1.0, self.car_inverse)
        p = self.matrix @ np.array([ix, iy, 1.0])
        return p[0] / p[2], p[1] / p[2]

    def render(self, t, t_rel, index, total, bgr, small, recv, draw=True):
        """Run the detector on one frame; returns the window image, or None with draw=False
        (frames before --start still go through the tracker so its state matches)."""
        v, lp = self.v, self.lp
        result = self.detector.detect(bgr, self.side, self.lane_width, draw)
        steer, used = None, None
        if result.found:     # same target limit as lane_main
            prev = self.used_target
            if self.last_found_t is None or t - self.last_found_t > TARGET_RESET_SEC:
                prev = None
            used = limit_step(prev, (result.target_x, result.target_y), v['target_max_step_px'])
            self.used_target, self.last_found_t = used, t
            steer = math.degrees(steer_angle(used[0], used[1], lp.warp_w, lp.warp_h,
                                             v['steer_gain'], v['max_steer_rad']))
        car = small['target'].get(round(t, 4))
        cmd = nearest(small['cmd'], recv + 0.03, 0.08) if car is not None else None
        car_steer = None
        if cmd is not None and cmd[1] > 1e-3:
            car_steer = math.degrees(math.atan(cmd[2] * v['wheelbase'] / cmd[1]))
        imu, odom = nearest(small['imu'], t, 0.05), nearest(small['odom'], t, 0.05)
        gyro = None
        if imu is not None and odom is not None and odom[1] > 0.15:
            gyro = math.degrees(math.atan(imu[1] * v['wheelbase'] / odom[1]))
        mode = nearest(small['mode'], recv, 1.0)
        attempts = getattr(self.detector, '_attempts', None)    # track2026 only
        searches = '-' if attempts is None else '%d (%d lines)' % (
            len(attempts), sum(1 for _, ok in attempts if ok))
        self.history[index] = (t_rel, steer, car_steer, gyro)
        if not draw:
            return None

        # camera with the warp area and both targets projected back
        cam_scale = 640.0 / lp.width
        cam = cv2.resize(bgr, None, fx=cam_scale, fy=cam_scale)
        h, w = lp.height, lp.width
        src = np.float32([[lp.x_h, h // 2 + lp.y_h], [-lp.x_l, h - lp.y_l],
                          [w + lp.x_l, h - lp.y_l], [w - lp.x_h, h // 2 + lp.y_h]]) * cam_scale
        cv2.polylines(cam, [src.astype(np.int32)], True, (255, 0, 255), 2)
        row = self.to_image(0, v['look_row'], cam_scale)[1]
        cv2.line(cam, (0, int(row)), (cam.shape[1], int(row)), (160, 160, 160), 1)
        if result.found:
            cv2.circle(cam, tuple(int(c) for c in self.to_image(result.target_x, result.target_y,
                                                                cam_scale)), 6, (0, 255, 0), -1)
            cv2.circle(cam, tuple(int(c) for c in self.to_image(used[0], used[1], cam_scale)), 9,
                       (0, 255, 255), 2)
        if car is not None and car[2]:
            cross(cam, self.to_image(car[0], car[1], cam_scale, self.car_inverse), (255, 255, 0))
        text(cam, 'camera  t=%.2f s  (%d/%d)' % (t_rel, index + 1, total), (8, 20))

        # warped debug image, 2x
        sx, sy = 640.0 / lp.warp_w, cam.shape[0] / lp.warp_h
        warp = cv2.resize(result.debug_image, (640, cam.shape[0]), interpolation=cv2.INTER_NEAREST)
        if used is not None:      # yellow ring: the point steered to, after target_max_step_px
            cv2.circle(warp, (int(used[0] * sx), int(used[1] * sy)), 14, (0, 255, 255), 2)
        if car is not None and car[2]:
            cx, cy = self.car_to_warp(car[0], car[1])
            cross(warp, (cx * sx, cy * sy), (255, 255, 0))
        text(warp, 'warp: windows / target', (8, 20))
        top = np.hstack([cam, warp])

        # numbers
        info = np.zeros((230, 600, 3), np.uint8)
        fmt = lambda a: '---' if a is None else '%+.1f' % a  # noqa: E731
        lines = [
            ('lane %s   reference %s   zigzag %s' % (
                'FOUND' if result.found else 'LOST (nothing published)',
                result.reference or '-', 'yes' if result.zigzag else 'no'),
             (0, 255, 0) if result.found else (0, 0, 255)),
            ('target x %.1f y %.1f, steered to %s  windows %s' % (
                result.target_x, result.target_y,
                '-' if used is None else '%.1f %.1f' % used, searches), None),
            ('car target        ' + ('x %.1f  y %.1f (car warp %s)' % (car[0], car[1], ','.join(
                map(str, self.car_warp))) if car and car[2] else ('lost' if car else 'not in bag')),
             (255, 255, 0)),
            ('steer  replay %s   car cmd %s   gyro %s deg' % (fmt(steer), fmt(car_steer), fmt(gyro)),
             None),
            ('BEV %.2f-%.2f m, target row %d = %.2f m ahead of the camera' % (
                self.ground_distance(lp.warp_h - 1), self.ground_distance(0), v['look_row'],
                self.ground_distance(v['look_row'])), (180, 180, 180)),
            ('steer_gain %.2f  lane %.0f px (%.2f m)  side %s' % (
                v['steer_gain'], self.lane_width, v['lane_width_m'],
                SIDES.get(self.side, self.side)), (180, 180, 180)),
            ('camera %s  control mode %s' % (
                self.camera, '---' if mode is None else {1: 'auto', 3: 'REMOTE'}.get(int(mode[1]),
                                                                                 int(mode[1]))),
             (180, 180, 180)),
            ('space play  d/-> next  a/<- back  w/s speed  p png  y params  q quit', (130, 130, 130)),
        ]
        for i, (s, colour) in enumerate(lines):
            text(info, s, (8, 22 + i * 26), 0.5, colour or (255, 255, 255))

        # steering strip chart, last 8 s
        chart = np.zeros((230, top.shape[1] - info.shape[1], 3), np.uint8)
        ch, cw = chart.shape[:2]
        span, lim = 8.0, 30.0
        y_of = lambda d: int(ch / 2 - d / lim * (ch / 2 - 10))  # noqa: E731
        for d, colour in ((0, (70, 70, 70)), (CAR_LIMIT_DEG, (90, 90, 90)), (-CAR_LIMIT_DEG, (90, 90, 90))):
            cv2.line(chart, (0, y_of(d)), (cw, y_of(d)), colour, 1)
        for k, colour in ((3, (0, 140, 255)), (2, (255, 255, 0)), (1, (0, 255, 0))):
            pts = [(int(cw - (t_rel - r[0]) / span * cw), y_of(max(-lim, min(lim, r[k]))))
                   for _, r in sorted(self.history.items())
                   if r[k] is not None and 0 <= t_rel - r[0] <= span]
            for a, b in zip(pts, pts[1:]):
                if abs(a[0] - b[0]) < cw / span * 0.5:      # no line across a gap
                    cv2.line(chart, a, b, colour, 2)
        text(chart, 'steer deg (left +), last 8 s: replay / car cmd / gyro  grey +-%.1f car limit'
             % CAR_LIMIT_DEG, (8, 18), 0.45, (200, 200, 200))
        return np.vstack([top, np.hstack([info, chart])])


# --- tuning sliders ---------------------------------------------------------------------------
class Tuner:
    """Slider window. Each slider maps to lane_main parameters; `changed()` reports a new parameter
    set once the sliders have rested for a moment."""
    WINDOW = 'lane_params'
    SETTLE = 0.3          # s without slider movement before applying

    def __init__(self, values: dict):
        self.base = dict(values)
        b = self.base
        # the BEV far end slides along the warp's own side lines, so straight lanes stay parallel
        bottom = (-b['warp_x_l'], b['height'] - b['warp_y_l'])
        top = (b['warp_x_h'], b['height'] // 2 + b['warp_y_h'])
        slope = (top[0] - bottom[0]) / (top[1] - bottom[1])

        def far_set(v, r):
            v['warp_y_h'] = int(r - v['height'] // 2)
            v['warp_x_h'] = int(round(bottom[0] + slope * (r - bottom[1])))

        def hsv(key, i):
            def set_(v, x):
                v[key] = list(v[key])
                v[key][i] = int(x)
            return (lambda v: int(v[key][i])), set_
        self.sliders = [   # name, min, max, get(values) -> int, set(values, int)
            ('BEV far end (image row)', 250, 440, lambda v: v['height'] // 2 + v['warp_y_h'], far_set),
            ('look_row (warp row)', 1, 239, lambda v: int(v['look_row']),
             lambda v, x: v.__setitem__('look_row', int(x))),
            ('steer_gain x100', 0, 300, lambda v: int(round(v['steer_gain'] * 100)),
             lambda v, x: v.__setitem__('steer_gain', x / 100.0)),
            ('lane width cm', 30, 80, lambda v: int(round(v['lane_width_m'] * 100)),
             lambda v, x: v.__setitem__('lane_width_m', x / 100.0)),
            ('yellow H min', 0, 179, *hsv('hsv_lower', 0)),
            ('yellow H max', 0, 179, *hsv('hsv_upper', 0)),
            ('yellow S min', 0, 255, *hsv('hsv_lower', 1)),
            ('yellow V min', 0, 255, *hsv('hsv_lower', 2)),
            ('white S max', 0, 255, lambda v: int(v['white_s_max']),
             lambda v, x: v.__setitem__('white_s_max', int(x))),
            ('white V min', 0, 255, lambda v: int(v['white_v_min']),
             lambda v, x: v.__setitem__('white_v_min', int(x))),
            ('white dash min px', 1, 80, lambda v: int(round(v['white_min_length'])),
             lambda v, x: v.__setitem__('white_min_length', float(x))),
            ('target max step px (0 off)', 0, 40, lambda v: int(round(v['target_max_step_px'])),
             lambda v, x: v.__setitem__('target_max_step_px', float(x))),
            ('window margin px', 5, 80, lambda v: int(v['margin']),
             lambda v, x: v.__setitem__('margin', int(x))),
            ('min windows', 1, 20, lambda v: int(v['min_line_windows']),
             lambda v, x: v.__setitem__('min_line_windows', int(x))),
            ('side 0 left 1 right 2 both', 0, 2, lambda v: int(v['initial_side']),
             lambda v, x: v.__setitem__('initial_side', int(x))),
        ]
        cv2.namedWindow(self.WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.WINDOW, 520, 40 * len(self.sliders) + 20)
        for name, lo, hi, get, _ in self.sliders:
            cv2.createTrackbar(name, self.WINDOW, min(hi, max(lo, get(b))), hi, lambda _x: None)
            cv2.setTrackbarMin(name, self.WINDOW, lo)
        self.applied = self.positions()
        self.pending, self.moved_at = None, 0.0

    def positions(self):
        return tuple(cv2.getTrackbarPos(name, self.WINDOW) for name, *_ in self.sliders)

    def values(self, positions) -> dict:
        v = dict(self.base)
        for (name, lo, hi, get, set_), x in zip(self.sliders, positions):
            if x != min(hi, max(lo, get(self.base))):     # only touched sliders change a value
                set_(v, x)
        return v

    def changed(self, now: float):
        pos = self.positions()
        if pos != (self.pending or self.applied):
            self.pending, self.moved_at = pos, now
            return None
        if self.pending is not None and now - self.moved_at >= self.SETTLE:
            self.applied, self.pending = self.pending, None
            return self.values(self.applied)
        return None


def describe(values: dict, base: dict) -> str:
    keys = [k for k in values if values[k] != base.get(k)]
    if not keys:
        return 'no parameter differs from the yaml files'
    yaml_lines = '\n'.join('    %s: %s' % (k, values[k]) for k in keys)
    args = ' '.join('-p %s=%s' % (k, str(values[k]).replace(' ', '')) for k in keys)
    return 'changed parameters (yaml):\n%s\nas viewer arguments: %s' % (yaml_lines, args)


def main(argv=None):
    import time
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag', help='bag directory')
    ap.add_argument('--camera', default='limo2', help='camera profile, config/camera_<name>.yaml')
    ap.add_argument('-p', '--param', action='append', default=[], metavar='NAME=VALUE',
                    help='override a lane_main parameter, e.g. -p look_row=120')
    ap.add_argument('--start', type=float, default=0.0, help='seconds from the first frame')
    ap.add_argument('--car-warp', metavar='X_H,X_L,Y_H,Y_L',
                    help="warp the car ran with, to place its recorded target (default: from the bag "
                         "date, CAR_WARP_HISTORY, else the replay's)")
    ap.add_argument('--all-frames', action='store_true',
                    help="use every camera frame, not only those the car's lane_main processed")
    ap.add_argument('--out', help='write a video (mp4) instead of opening a window')
    ap.add_argument('--tune', action='store_true', help='slider window to change parameters live')
    args = ap.parse_args(argv)
    bag = os.path.abspath(args.bag.rstrip('/'))

    file_values = load_params(args.camera, [])
    values = load_params(args.camera, args.param)
    car_warp = None
    if args.car_warp:
        car_warp = tuple(int(x) for x in args.car_warp.split(','))
    else:
        with open(os.path.join(bag, 'metadata.yaml')) as f:
            start = yaml.safe_load(f)['rosbag2_bagfile_information']['starting_time'][
                'nanoseconds_since_epoch'] * 1e-9
        for until, warp in CAR_WARP_HISTORY.get(args.camera, []):
            if start < until:
                car_warp = warp
                break
    small = read_small_topics(bag)
    intrinsics = (small['k'][4], small['k'][5]) if small['k'] else None
    viewer = Viewer(values, args.camera, car_warp, intrinsics)
    if car_warp:
        print('car warp (for its recorded target): %s' % (car_warp,))
    _, available = open_reader(bag, [])
    topic = next((t for t in IMAGE_TOPICS if t in available), None)
    if topic is None:
        sys.exit('no camera image in the bag (%s)' % ', '.join(IMAGE_TOPICS))
    only = None if args.all_frames or not small['target'] else set(small['target'])
    with open(os.path.join(bag, 'metadata.yaml')) as f:
        counts = {t['topic_metadata']['name']: t['message_count'] for t in
                  yaml.safe_load(f)['rosbag2_bagfile_information']['topics_with_message_count']}
    total = len(only) if only else counts.get(topic, 0)
    print('%s: %s, %d frames%s' % (os.path.basename(bag), topic, total,
                                   " (the car's lane_main frames)" if only else ''))

    gen = enumerate(frames(bag, topic, only))
    t0 = None

    if args.out:
        writer, n = None, 0
        fps = 30.0 if only is None else max(5.0, len(only) / max(1e-3, max(only) - min(only)))
        for index, (s_, recv, bgr, _) in gen:
            t0 = s_ if t0 is None else t0
            draw = s_ - t0 >= args.start
            img = viewer.render(s_, s_ - t0, index, total, bgr, small, recv, draw)
            if not draw:
                continue
            if writer is None:
                writer = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*'mp4v'), fps,
                                         (img.shape[1], img.shape[0]))
            writer.write(img)
            n += 1
            if n % 200 == 0:
                print('  %d frames' % n)
        if writer is not None:
            writer.release()
        print('wrote %s (%d frames)' % (args.out, n))
        return

    # --- interactive -------------------------------------------------------------------------
    WARM = 30             # frames re-run before a redrawn frame so the tracker state fits
    window = 'lane_viewer'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    tuner = Tuner(values) if args.tune else None

    def closed():
        # OpenCV with GTK (4.5.4 in Humble) answers -1, "not supported": then only q / Esc quit
        return cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) == 0

    # each entry: index, stamp, receive time, t_rel, encoded camera image, drawn jpeg, version
    cache = collections.deque(maxlen=300)
    state = {'version': 0, 'at': None}      # parameter version; frame index the tracker last saw

    def decode(e):
        return cv2.imdecode(np.frombuffer(e['raw'], np.uint8), cv2.IMREAD_COLOR)

    def process(e, bgr=None, draw=True):
        img = viewer.render(e['s'], e['t'], e['index'], total, decode(e) if bgr is None else bgr,
                            small, e['recv'], draw)
        state['at'] = e['index']
        if draw:
            e['img'] = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 92])[1]
            e['version'] = state['version']

    def warm_up(upto):
        """Fresh tracker run over the cached frames just before index `upto`."""
        viewer.reset_tracking()
        for e in cache:
            if upto - WARM <= e['index'] < upto:
                process(e, draw=False)

    def ensure(pos):
        e = cache[pos]
        if e['img'] is not None and e['version'] == state['version']:
            return
        if state['at'] != e['index'] - 1:
            warm_up(e['index'])
        process(e)

    def next_frame():
        """Next frame at or after --start into the cache; False at the end of the bag."""
        nonlocal t0
        if cache and state['at'] != cache[-1]['index']:
            warm_up(cache[-1]['index'] + 1)
        for index, (s_, recv, bgr, raw) in gen:
            t0 = s_ if t0 is None else t0
            if raw is None:
                raw = cv2.imencode('.png', bgr)[1].tobytes()
            e = {'index': index, 's': s_, 'recv': recv, 't': s_ - t0, 'raw': raw, 'img': None,
                 'version': -1}
            if e['t'] < args.start:
                process(e, bgr, draw=False)
                continue
            process(e, bgr)
            cache.append(e)
            return True
        return False

    pos, playing, rate, want_next, ended = -1, False, 1.0, True, False
    while True:
        if want_next:
            want_next = False
            if pos + 1 < len(cache):
                pos += 1
            elif not ended:
                if next_frame():
                    pos = len(cache) - 1
                else:
                    ended, playing = True, False
                    print('end of bag')
            else:
                playing = False
        if pos < 0:
            sys.exit('no frame at or after --start')
        ensure(pos)
        e = cache[pos]
        cv2.imshow(window, cv2.imdecode(e['img'], cv2.IMREAD_COLOR))
        if playing:
            nxt = cache[pos + 1]['t'] if pos + 1 < len(cache) else e['t'] + 0.033
            key = read_key(max(1, int((nxt - e['t']) / rate * 1000)))
        else:
            key = -1
            while key == -1 and not closed():
                key = read_key(50)
                if tuner and tuner.pending is not None or tuner and tuner.positions() != tuner.applied:
                    break                     # a slider moved: go and check it
        if tuner:
            new = tuner.changed(time.monotonic())
            while new is None and tuner.pending is not None and key == -1 and not playing:
                key = read_key(50)
                new = tuner.changed(time.monotonic())
            if new is not None:
                try:
                    viewer.reconfigure(new)
                except ValueError as exc:
                    print('parameters not applied:', exc)
                else:
                    state['version'] += 1
                    state['at'] = None
                    for k, r in viewer.history.items():   # the replay curve restarts
                        viewer.history[k] = (r[0], None, r[2], r[3])
                    print('applied:', describe(new, file_values).split('\n', 1)[-1].replace('\n', ' | '))
        if key in (ord('q'), 27) or closed():
            break
        if key == ord(' '):
            playing = not playing
            want_next = playing
        elif key in (ord('d'),) + KEY_RIGHT:
            playing, want_next = False, True
        elif key in (ord('a'),) + KEY_LEFT:
            playing, pos = False, max(0, pos - 1)
        elif key == ord('w'):
            rate = min(8.0, rate * 2)
            print('speed x%g' % rate)
        elif key == ord('s'):
            rate = max(0.125, rate / 2)
            print('speed x%g' % rate)
        elif key == ord('p'):
            path = os.path.join(os.path.dirname(bag), '%s_%.2f.png' % (os.path.basename(bag), e['t']))
            cv2.imwrite(path, cv2.imdecode(e['img'], cv2.IMREAD_COLOR))
            print('saved', path)
        elif key == ord('y'):
            print(describe(viewer.v, file_values))
        elif key == -1 and playing:
            want_next = True
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
