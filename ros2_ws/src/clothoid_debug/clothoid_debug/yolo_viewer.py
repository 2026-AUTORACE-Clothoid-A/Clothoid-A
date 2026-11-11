#!/usr/bin/env python3
"""Step through a bag and watch which YOLO boxes become mission triggers, and why the others do not.

    ros2 run clothoid_debug yolo_viewer <bag_dir>                         # window, starts paused
    ros2 run clothoid_debug yolo_viewer <bag_dir> --tune                  # + slider window
    ros2 run clothoid_debug yolo_viewer <bag_dir> --model best.pt --yolo-hz 15 -p sign_min_height=20
    ros2 run clothoid_debug yolo_viewer <bag_dir> --out yolo.mp4          # no window, write a video

No ROS graph is needed. The camera frames are read from the bag and run through the YOLO model once
(the boxes are cached next to the bag, `yolo_cache_<model>.json`); if the bag holds the car's own
/detections those are used instead (`--source car`). Every box then goes through the same code the
car runs, with the values of yolo.yaml + mission.yaml (`-p name=value` overrides any of them):

  yolo_node      min_confidence
  mission_node   classes_seen()   per class box size (min area / height), side offset / box height
                 DetectionGate    detection_confidence, sign / light min height, image edge,
                                  then `hits` matching boxes within `max_age` s in the last `window`
                                  YOLO frames
  mission_logic  MissionLogic     what a confirmed class does (slow limit, arm a stop, bump, fork,
                                  parking), stepped at rate_hz like the node, IMU pitch from /imu

The car's YOLO rate decides how many frames the gate sees: `--yolo-hz` (and the slider) keeps only
that many frames per second, as the car's node would get from a throttled topic. 0 = every frame.

Not replayed: the stop line (FinishCore). A pedestrian sign or red light therefore only *arms* the
stop here (and the arm expires after arm_timeout_sec); the car stops when it then reaches the line.

Window: camera image with the boxes of the latest YOLO frame | the numbers and the mission state |
timeline of the whole bag (one row per class, then the mission outputs). Click the timeline to jump.

Box colours: grey  below the YOLO node's min_confidence (never published)
             red   dropped by classes_seen (size / side offset) or the gate (conf / height / edge)
             yellow passed the filters, waiting for hits (k/N shown)
             green  confirmed: the class is a trigger now

Keys (with the viewer window focused): space play/pause, d or -> next frame, a or <- back,
n / b next / previous change of the triggers, w/s faster/slower, p save a png next to the bag,
y print the changed parameters, q or Esc quit.
"""
import argparse
import bisect
import json
import os
import sys
import time

import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from clothoid_mission.mission_logic import (
    BUMP_APPROACHING, BUMP_COOLDOWN, BUMP_ON, DetectionGate, Inputs, MissionLogic, MissionParams,
    classes_seen, quaternion_to_pitch_deg)
from clothoid_debug.lane_viewer import IMAGE_TOPICS, open_reader, stamp, text

CLASSES = ('traffic_light_red', 'traffic_light_green', 'sign_pedestrian', 'sign_slow', 'sign_bump',
           'sign_parking', 'sign_left', 'sign_right')
SIZE_CLASSES = {'sign_bump': 'bump', 'sign_slow': 'slow', 'sign_parking': 'parking',
                'sign_pedestrian': 'pedestrian'}
INFER_CONF = 0.05          # model threshold for the cache; the replay applies model_conf / min_confidence
YOLO_KEYS = ('model_conf', 'min_confidence')
# mission_node parameter -> MissionParams field
MISSION_FIELDS = {
    'slow_speed': 'slow_speed', 'slow_hold_sec': 'slow_hold_sec',
    'bump_pitch_thresh': 'bump_pitch_thresh', 'bump_level_thresh': 'bump_level_thresh',
    'bump_level_duration': 'bump_level_duration_sec', 'bump_arm_timeout': 'bump_arm_timeout_sec',
    'bump_cooldown': 'bump_cooldown_sec', 'pedestrian_wait_sec': 'pedestrian_wait_sec',
    'stop_on_yellow': 'stop_on_yellow', 'arm_timeout_sec': 'arm_timeout_sec',
    'cooldown_sec': 'cooldown_sec', 'use_stop_line': 'use_stop_line',
    'approach_speed': 'approach_speed', 'fork_left_direction': 'fork_left_direction',
    'fork_right_direction': 'fork_right_direction', 'parking_at_stopline': 'parking_at_stopline'}

GREY, RED, YELLOW, GREEN = (150, 150, 150), (60, 60, 235), (0, 215, 255), (60, 220, 60)
CAM_W, CAM_H, PANEL_W = 640, 480, 420
ROW_H, LABEL_W = 15, 150

KEY_RIGHT, KEY_LEFT = (65363, 2555904), (65361, 2424832)
HANGUL_KEYS = {'ㅇ': 'd', 'ㅁ': 'a', 'ㅂ': 'q', 'ㅈ': 'w', 'ㄴ': 's', 'ㅔ': 'p', 'ㅛ': 'y',
               'ㅜ': 'n', 'ㅠ': 'b'}


def read_key(delay_ms: int) -> int:
    """waitKeyEx without modifier bits and Hangul jamo: -1, an arrow code or a lower-case ASCII code."""
    key = cv2.waitKeyEx(delay_ms)
    if key == -1 or key in KEY_RIGHT or key in KEY_LEFT:
        return key
    code = key & 0xFFFF
    if code in (65363, 65361):
        return code
    if code == 0xFF1B:
        return 27
    char = HANGUL_KEYS.get(chr(code), chr(code)).lower() if code < 0x110000 else ''
    return ord(char) if len(char) == 1 and ord(char) < 128 else 0


# --- parameters -------------------------------------------------------------------------------
def load_params(overrides) -> dict:
    """mission_node defaults, then mission.yaml, then yolo.yaml (model_conf, min_confidence)."""
    d = MissionParams()
    values = {
        'rate_hz': 20.0, 'input_timeout': 0.5,
        'light_max_offset_height_ratio': 3.5, 'sign_max_offset_height_ratio': 5.0,
        'fork_max_offset_height_ratio': 10.0,
        'detection_confirm_hits': 3, 'detection_window': 5, 'detection_max_age': 0.5,
        'detection_confidence': 0.6, 'sign_min_height': 14, 'light_min_height': 8,
        'detection_edge_margin': 2,
        'model_conf': 0.25, 'min_confidence': 0.5, 'yolo_hz': 0.0,
    }
    for name in SIZE_CLASSES.values():
        values[f'{name}_min_area'] = 2500.0
        values[f'{name}_min_height'] = 40.0
    for name, field in MISSION_FIELDS.items():
        values[name] = getattr(d, field)
    with open(os.path.join(get_package_share_directory('clothoid_mission'), 'config',
                           'mission.yaml')) as f:
        values.update(yaml.safe_load(f)['mission']['ros__parameters'])
    with open(os.path.join(get_package_share_directory('clothoid_yolo'), 'config',
                           'yolo.yaml')) as f:
        yolo = yaml.safe_load(f)['yolo']['ros__parameters']
    values.update({k: yolo[k] for k in YOLO_KEYS if k in yolo})
    for item in overrides:
        name, _, value = item.partition('=')
        if name not in values:
            sys.exit(f'unknown parameter {name!r}')
        values[name] = yaml.safe_load(value)
    return values


def describe(values: dict, base: dict) -> str:
    keys = [k for k in values if values[k] != base.get(k)]
    if not keys:
        return 'no parameter differs from the yaml files'
    lines = []
    for title, group in (('yolo.yaml', [k for k in keys if k in YOLO_KEYS]),
                         ('mission.yaml', [k for k in keys if k not in YOLO_KEYS and k != 'yolo_hz'])):
        if group:
            lines.append(f'  {title}:')
            lines += ['    %s: %s' % (k, values[k]) for k in group]
    if 'yolo_hz' in keys:
        lines.append('  YOLO rate (not a parameter, the topic the node reads): %g Hz' % values['yolo_hz'])
    args = ' '.join('-p %s=%s' % (k, values[k]) for k in keys)
    return 'changed parameters:\n%s\nas viewer arguments: %s' % ('\n'.join(lines), args)


# --- bag and model ----------------------------------------------------------------------------
class Box:
    __slots__ = ('class_name', 'confidence', 'x_min', 'y_min', 'x_max', 'y_max')

    def __init__(self, class_name, confidence, x_min, y_min, x_max, y_max):
        self.class_name, self.confidence = class_name, float(confidence)
        self.x_min, self.y_min, self.x_max, self.y_max = int(x_min), int(y_min), int(x_max), int(y_max)

    def as_list(self):
        return [self.class_name, round(self.confidence, 4), self.x_min, self.y_min, self.x_max, self.y_max]


def read_bag(bag: str):
    """Camera frames (stamp, jpeg/png bytes), the car's detections if recorded, IMU pitch, intrinsics."""
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu
    from clothoid_common.image_utils import image_to_bgr
    _, available = open_reader(bag, [])
    topic = next((t for t in IMAGE_TOPICS if t in available), None)
    if topic is None:
        sys.exit('no camera image in the bag (%s)' % ', '.join(IMAGE_TOPICS))
    topics = [topic, '/imu', '/camera/color/camera_info']
    det_type = None
    if '/detections' in available:
        from clothoid_interfaces.msg import DetectionArray as det_type
        topics.append('/detections')
    reader, _ = open_reader(bag, topics)
    frames, car, pitch, info = [], [], [], None
    compressed = topic.endswith('compressed')
    while reader.has_next():
        name, data, t = reader.read_next()
        if name == topic:
            msg = deserialize_message(data, CompressedImage if compressed else Image)
            raw = bytes(msg.data) if compressed else cv2.imencode('.png', image_to_bgr(msg))[1].tobytes()
            frames.append((stamp(msg.header) or t * 1e-9, raw))
        elif name == '/imu':
            m = deserialize_message(data, Imu)
            q = m.orientation
            pitch.append((stamp(m.header) or t * 1e-9, quaternion_to_pitch_deg(q.x, q.y, q.z, q.w)))
        elif name == '/camera/color/camera_info' and info is None:
            m = deserialize_message(data, CameraInfo)
            info = (m.k[2], m.width, m.height)
        elif name == '/detections':
            m = deserialize_message(data, det_type)
            car.append((stamp(m.header) or t * 1e-9,
                        [Box(d.class_name, d.confidence, d.x_min, d.y_min, d.x_max, d.y_max)
                         for d in m.detections]))
    frames.sort(key=lambda f: f[0])
    car.sort(key=lambda f: f[0])
    pitch = np.array(sorted(pitch)) if pitch else np.zeros((0, 2))
    return topic, frames, car, pitch, info


def find_model(path: str) -> str:
    if path:
        return os.path.abspath(os.path.expanduser(path))
    from clothoid_yolo.yolo_node import DEFAULT_MODEL_PATH, resolve_model_path
    for name in ('best.pt', DEFAULT_MODEL_PATH):
        found = resolve_model_path(name)
        if os.path.isfile(found) and not found.endswith('.engine'):
            return found
    return ''


def infer_all(bag: str, frames, model: str, imgsz: int, device: str):
    """YOLO boxes of every frame (threshold INFER_CONF), cached next to the bag per model file."""
    st = os.stat(model)
    key = {'model': os.path.basename(model), 'size': st.st_size, 'mtime': int(st.st_mtime),
           'imgsz': imgsz, 'conf': INFER_CONF, 'frames': len(frames)}
    cache = os.path.join(bag, 'yolo_cache_%s.json' % os.path.splitext(os.path.basename(model))[0])
    if os.path.isfile(cache):
        try:
            with open(cache) as f:
                data = json.load(f)
            if data.get('key') == key:
                print('YOLO boxes from cache', cache)
                return [[Box(*b) for b in boxes] for boxes in data['boxes']]
        except (OSError, ValueError, TypeError):
            pass
    from clothoid_yolo.yolo_detector import UltralyticsBackend
    backend = UltralyticsBackend(model, imgsz, device or None, INFER_CONF)
    print('running %s on %d frames (device %s) ...' % (os.path.basename(model), len(frames),
                                                        device or 'auto'))
    out, t_start = [], time.monotonic()
    for i, (_, raw) in enumerate(frames):
        bgr = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        out.append([Box(b.class_name, b.confidence, b.x_min, b.y_min, b.x_max, b.y_max)
                    for b in backend.infer(bgr)])
        if (i + 1) % 100 == 0:
            rate = (i + 1) / (time.monotonic() - t_start)
            print('  %d/%d frames (%.0f frames/s)' % (i + 1, len(frames), rate))
    try:
        with open(cache, 'w') as f:
            json.dump({'key': key, 'boxes': [[b.as_list() for b in boxes] for boxes in out]}, f)
        print('cached in', cache)
    except OSError as exc:
        print('cache not written:', exc)
    return out


# --- replay -----------------------------------------------------------------------------------
class Replay:
    """The YOLO frames the node would get, every box's fate, and the mission outputs over time."""

    def __init__(self, stamps, boxes, pitch, cx, width, height):
        self.stamps, self.boxes, self.pitch = stamps, boxes, pitch
        self.cx, self.width, self.height = cx, width, height

    def run(self, v: dict) -> None:
        # frames the node gets at yolo_hz (a throttle passes a frame once the period has elapsed)
        period = 1.0 / v['yolo_hz'] if v['yolo_hz'] > 0 else 0.0
        used, last = [], None
        for i, s in enumerate(self.stamps):
            if last is None or s - last >= period - 1e-3:
                used.append(i)
                last = s
        min_sizes = {cls: (v[f'{n}_min_area'], v[f'{n}_min_height']) for cls, n in SIZE_CLASSES.items()}
        gate = DetectionGate(
            hits=int(v['detection_confirm_hits']), window=int(v['detection_window']),
            max_age=v['detection_max_age'], confidence=v['detection_confidence'],
            sign_height=v['sign_min_height'], light_height=v['light_min_height'],
            edge_margin=v['detection_edge_margin'])
        ratios = dict(light_max_offset_height_ratio=v['light_max_offset_height_ratio'],
                      sign_max_offset_height_ratio=v['sign_max_offset_height_ratio'],
                      fork_max_offset_height_ratio=v['fork_max_offset_height_ratio'])
        threshold = max(v['model_conf'], v['min_confidence'])
        self.used, self.used_stamps, self.results, self.seen = used, [self.stamps[i] for i in used], [], []
        for i in used:
            now = self.stamps[i]
            published = [b for b in self.boxes[i] if b.confidence >= threshold]
            accepted = classes_seen(published, camera_center_x=self.cx, min_sizes=min_sizes, **ratios)
            seen = gate.update(now, published, accepted, width=self.width, height=self.height)
            fates = []
            for b in self.boxes[i]:
                fates.append((b,) + self._fate(b, threshold, accepted, gate, now, min_sizes, ratios, seen))
            self.results.append(fates)
            self.seen.append(seen)
        self._mission(v)

    def _fate(self, b, threshold, accepted, gate, now, min_sizes, ratios, seen):
        """(status, reason) of one box, in the order the car's code drops it."""
        w, h = b.x_max - b.x_min, b.y_max - b.y_min
        if b.confidence < threshold:
            return 'model', 'conf %.2f < %.2f' % (b.confidence, threshold)
        if b.class_name not in CLASSES:
            return 'drop', 'class not in Detection.msg'
        if b.class_name not in accepted:
            if b.class_name in min_sizes:
                area_min, h_min = min_sizes[b.class_name]
                if w * h < area_min or h < h_min:
                    return 'drop', 'size %dx%d, need area %.0f h %.0f' % (w, h, area_min, h_min)
            ratio = abs((b.x_min + b.x_max) / 2.0 - self.cx) / max(h, 1)
            return 'drop', 'side offset %.1f x height' % ratio
        if b.confidence < gate.confidence:
            return 'drop', 'gate conf %.2f < %.2f' % (b.confidence, gate.confidence)
        minimum = gate.light_height if b.class_name.startswith('traffic_light_') else gate.sign_height
        if h < minimum:
            return 'drop', 'gate height %d < %d' % (h, minimum)
        m = gate.edge_margin
        if b.x_min < m or b.y_min < m or b.x_max > self.width - m or b.y_max > self.height - m:
            return 'drop', 'touches the image edge'
        hits = sum(any(gate.matches(b, old) for old in frame)
                   for stamp_, frame in gate.history if 0 <= now - stamp_ <= gate.max_age)
        if b.class_name in seen and hits >= gate.hits:
            return 'ok', 'hits %d/%d' % (hits, gate.hits)
        return 'wait', 'hits %d/%d' % (hits, gate.hits)

    def _mission(self, v: dict) -> None:
        """MissionLogic on the node's timer; seen counts while fresher than input_timeout."""
        logic = MissionLogic(MissionParams(**{f: v[n] for n, f in MISSION_FIELDS.items()}))
        self.ticks, self.outputs = [], []
        if not self.stamps:
            return
        t, end, dt = self.stamps[0], self.stamps[-1], 1.0 / v['rate_hz']
        j = -1
        while t <= end + 1e-9:
            while j + 1 < len(self.used_stamps) and self.used_stamps[j + 1] <= t:
                j += 1
            seen = set(self.seen[j]) if j >= 0 and t - self.used_stamps[j] <= v['input_timeout'] else set()
            pitch = None
            if len(self.pitch):
                k = int(np.searchsorted(self.pitch[:, 0], t, side='right')) - 1
                if k >= 0 and t - self.pitch[k, 0] <= v['input_timeout']:
                    pitch = float(self.pitch[k, 1])
            out = logic.step(Inputs(now=t, seen=seen, imu_pitch=pitch))
            armed = [n for n, a in (('pedestrian', logic.pedestrian_armed_at),
                                    ('light', logic.light_armed_at)) if a is not None]
            self.ticks.append(t)
            self.outputs.append({'seen': sorted(seen), 'state': out.state, 'speed_limit': out.speed_limit,
                                 'slow': t < logic.slow_until, 'direction': out.direction,
                                 'parking_flag': out.parking_flag, 'bump': logic.bump_state,
                                 'armed': armed, 'pitch': pitch})
            t += dt

    def at(self, t: float):
        """Latest YOLO frame (index into used) and mission output at or before t."""
        j = bisect.bisect_right(self.used_stamps, t + 1e-6) - 1
        k = bisect.bisect_right(self.ticks, t + 1e-6) - 1
        return j, (self.outputs[k] if k >= 0 else None)

    def events(self):
        """Stamps where the confirmed classes or the mission outputs change."""
        out, last = [], None
        for t, o in zip(self.ticks, self.outputs):
            sig = (tuple(o['seen']), o['state'], o['slow'], o['direction'], o['parking_flag'],
                   o['bump'], tuple(o['armed']))
            if sig != last:
                if last is not None:
                    out.append(t)
                last = sig
        return out


# --- drawing ----------------------------------------------------------------------------------
STATUS_COLOUR = {'model': GREY, 'drop': RED, 'wait': YELLOW, 'ok': GREEN}
STATUS_RANK = {'model': 0, 'drop': 1, 'wait': 2, 'ok': 3}
MISSION_ROWS = ('slow limit', 'pedestrian armed', 'light armed', 'bump', 'fork direction',
                'parking flag')


class Drawer:
    def __init__(self, replay: Replay, t0: float, t1: float, source: str):
        self.r, self.t0, self.t1, self.source = replay, t0, max(t1, t0 + 1e-3), source
        self.width = CAM_W + PANEL_W
        self.tl_top = CAM_H
        self.tl_h = (len(CLASSES) + len(MISSION_ROWS)) * ROW_H + 28
        self.timeline = None

    def x_of(self, t: float) -> int:
        return int(LABEL_W + (t - self.t0) / (self.t1 - self.t0) * (self.width - LABEL_W - 10))

    def t_of(self, x: int) -> float:
        return self.t0 + (x - LABEL_W) / (self.width - LABEL_W - 10) * (self.t1 - self.t0)

    def build_timeline(self) -> None:
        img = np.full((self.tl_h, self.width, 3), 30, np.uint8)
        rows = list(CLASSES) + list(MISSION_ROWS)
        for i, name in enumerate(rows):
            y = 6 + i * ROW_H
            if i == len(CLASSES):
                cv2.line(img, (0, y - 2), (self.width, y - 2), (90, 90, 90), 1)
            text(img, name.replace('traffic_light_', 'light_'), (6, y + 11), 0.4, (220, 220, 220))
        # classes: best fate per YOLO frame
        for t, fates in zip(self.r.used_stamps, self.r.results):
            best = {}
            for b, status, _ in fates:
                if b.class_name in CLASSES and STATUS_RANK[status] >= STATUS_RANK.get(best.get(b.class_name), -1):
                    best[b.class_name] = status
            x = self.x_of(t)
            for name, status in best.items():
                y = 6 + CLASSES.index(name) * ROW_H
                inset = 4 if status == 'model' else 1
                cv2.line(img, (x, y + inset), (x, y + ROW_H - 2 - inset), STATUS_COLOUR[status], 2)
        # mission outputs on the timer
        base = len(CLASSES)
        bump_colour = {BUMP_APPROACHING: YELLOW, BUMP_ON: RED, BUMP_COOLDOWN: GREY}
        for t, o in zip(self.r.ticks, self.r.outputs):
            x = self.x_of(t)
            marks = [
                (0, (255, 160, 60) if o['slow'] else None),
                (1, YELLOW if 'pedestrian' in o['armed'] else None),
                (2, YELLOW if 'light' in o['armed'] else None),
                (3, bump_colour.get(o['bump'])),
                (4, None if o['direction'] is None else ((255, 120, 0) if o['direction'] == 0 else
                                                         (0, 140, 255) if o['direction'] == 1 else GREY)),
                (5, GREEN if o['parking_flag'] else None)]
            for row, colour in marks:
                if colour is not None:
                    y = 6 + (base + row) * ROW_H
                    cv2.line(img, (x, y + 1), (x, y + ROW_H - 3), colour, 3)
        # seconds axis
        y = self.tl_h - 8
        for s in range(0, int(self.t1 - self.t0) + 1, 5 if self.t1 - self.t0 > 30 else 1):
            x = self.x_of(self.t0 + s)
            cv2.line(img, (x, y - 6), (x, y - 2), (120, 120, 120), 1)
            if s % 5 == 0:
                text(img, '%ds' % s, (x - 6, y + 6), 0.35, (160, 160, 160))
        self.timeline = img

    def render(self, t: float, index: int, total: int, raw: bytes, v: dict) -> np.ndarray:
        bgr = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        if bgr.shape[:2] != (CAM_H, CAM_W):
            bgr = cv2.resize(bgr, (CAM_W, CAM_H))
        canvas = np.full((CAM_H + self.tl_h, self.width, 3), 20, np.uint8)
        j, out = self.r.at(t)
        fates = self.r.results[j] if j >= 0 else []
        age = t - self.r.used_stamps[j] if j >= 0 else None
        cx = int(round(self.r.cx))
        cv2.line(bgr, (cx, 0), (cx, CAM_H), (90, 90, 90), 1)
        order = sorted(fates, key=lambda f: STATUS_RANK[f[1]])
        for b, status, reason in order:
            colour = STATUS_COLOUR[status]
            thick = 3 if status == 'ok' else 1 if status == 'model' else 2
            cv2.rectangle(bgr, (b.x_min, b.y_min), (b.x_max, b.y_max), colour, thick)
            if status != 'model':
                label = '%s %.2f' % (b.class_name.replace('traffic_light_', 'light_').replace('sign_', ''),
                                     b.confidence)
                text(bgr, label, (b.x_min, max(12, b.y_min - 16)), 0.45, colour)
                text(bgr, reason, (b.x_min, max(24, b.y_min - 3)), 0.4, colour)
        canvas[:CAM_H, :CAM_W] = bgr

        # panel
        x0, y = CAM_W + 10, 22

        def line(s, colour=(230, 230, 230), scale=0.48, gap=19):
            nonlocal y
            text(canvas, s, (x0, y), scale, colour)
            y += gap
        line('t %.2f s   frame %d/%d' % (t - self.t0, index + 1, total), scale=0.55)
        line('boxes: %s, YOLO at %s' % (self.source, 'every frame' if v['yolo_hz'] <= 0
                                         else '%g Hz' % v['yolo_hz']), (180, 180, 180), 0.42)
        line('YOLO frame %s' % ('-' if age is None else '%.0f ms ago' % (age * 1000)),
             (180, 180, 180), 0.42)
        y += 4
        line('MISSION (mission_logic)', (255, 200, 120))
        if out is not None:
            line('triggers: %s' % (', '.join(s.replace('traffic_light_', 'light_') for s in out['seen'])
                                   or '-'), GREEN if out['seen'] else (230, 230, 230))
            limit = out['speed_limit']
            line('speed limit: %s%s' % ('-' if limit is None else '%.2f m/s' % limit,
                                        ' (slow sign)' if out['slow'] else ''))
            line('armed stop: %s' % (', '.join(out['armed']) or '-'),
                 YELLOW if out['armed'] else (230, 230, 230))
            line('bump: %s   pitch %s' % (out['bump'], '-' if out['pitch'] is None
                                           else '%+.1f deg' % out['pitch']))
            d = out['direction']
            line('fork direction: %s' % ({None: '-', 0: 'left lane', 1: 'right lane', 2: 'both'}
                                         .get(d, d)))
            line('parking flag: %s' % ('ON' if out['parking_flag'] else '-'),
                 GREEN if out['parking_flag'] else (230, 230, 230))
            line('(stop line not replayed: armed = stops at the next line)', (150, 150, 150), 0.38)
        y += 6
        line('BOXES in this YOLO frame', (255, 200, 120))
        shown = [f for f in order[::-1] if f[1] != 'model'][:8]
        if not shown:
            line('none above min_confidence %.2f' % max(v['model_conf'], v['min_confidence']),
                 (150, 150, 150), 0.42)
        for b, status, reason in shown:
            h = b.y_max - b.y_min
            ratio = abs((b.x_min + b.x_max) / 2.0 - self.r.cx) / max(h, 1)
            line('%s %.2f h%d side%.1f' % (b.class_name.replace('traffic_light_', 'light_'),
                                           b.confidence, h, ratio), STATUS_COLOUR[status], 0.42, 15)
            line('   ' + reason, STATUS_COLOUR[status], 0.38, 16)
        hidden = sum(1 for f in order if f[1] == 'model')
        if hidden:
            line('+%d grey box(es) below min_confidence' % hidden, (150, 150, 150), 0.38)
        y = CAM_H - 30
        line('grey: not published   red: dropped', (180, 180, 180), 0.38, 15)
        line('yellow: counting hits   green: trigger', (180, 180, 180), 0.38, 15)

        # timeline
        canvas[self.tl_top:, :] = self.timeline
        x = self.x_of(t)
        cv2.line(canvas, (x, self.tl_top), (x, self.tl_top + self.tl_h - 14), (255, 255, 255), 1)
        return canvas


# --- tuning -----------------------------------------------------------------------------------
class Tuner:
    """Slider window. A change is applied once the sliders rest for a moment; the whole bag is
    replayed again (no YOLO inference, only the filters), so the timeline updates at once."""
    WINDOW = 'yolo_params'
    LIST_WINDOW = 'yolo_param_list'   # own window: under the 20 sliders it gets squeezed away
    SETTLE = 0.3

    def __init__(self, values: dict):
        self.base = dict(values)

        def scaled(key, k, cast=float):
            return (lambda v: int(round(v[key] * k)), lambda v, x: v.__setitem__(key, cast(x / k)))

        def plain(key, cast=int):
            return (lambda v: int(round(v[key])), lambda v, x: v.__setitem__(key, cast(x)))
        self.sliders = [   # name, min, max, get(values) -> int, set(values, int)
            ('YOLO Hz (0 every frame)', 0, 30, *plain('yolo_hz', float)),
            ('yolo min_confidence x100', 5, 100, *scaled('min_confidence', 100)),
            ('gate confidence x100', 0, 100, *scaled('detection_confidence', 100)),
            ('gate hits', 1, 10, *plain('detection_confirm_hits')),
            ('gate window (frames)', 1, 15, *plain('detection_window')),
            ('gate max_age x100 s', 5, 300, *scaled('detection_max_age', 100)),
            ('sign min height px', 0, 200, *plain('sign_min_height')),
            ('light min height px', 0, 200, *plain('light_min_height')),
            ('edge margin px', 0, 50, *plain('detection_edge_margin')),
            ('light side ratio x10', 0, 200, *scaled('light_max_offset_height_ratio', 10)),
            ('sign side ratio x10', 0, 200, *scaled('sign_max_offset_height_ratio', 10)),
            ('fork side ratio x10', 0, 200, *scaled('fork_max_offset_height_ratio', 10)),
        ]
        for name in SIZE_CLASSES.values():
            self.sliders.append(('%s min height px' % name, 0, 200, *plain(f'{name}_min_height', float)))
            self.sliders.append(('%s min area /100 px2' % name, 0, 300,
                                 *scaled(f'{name}_min_area', 0.01)))
        # numbered names: some OpenCV builds (Qt) cut or hide the trackbar labels, so the list
        # drawn under the sliders repeats them in the same order with the real value
        self.sliders = [('%d %s' % (i + 1, s[0]),) + s[1:] for i, s in enumerate(self.sliders)]
        self.keys = [key for key in (
            'yolo_hz', 'min_confidence', 'detection_confidence', 'detection_confirm_hits',
            'detection_window', 'detection_max_age', 'sign_min_height', 'light_min_height',
            'detection_edge_margin', 'light_max_offset_height_ratio',
            'sign_max_offset_height_ratio', 'fork_max_offset_height_ratio')]
        for name in SIZE_CLASSES.values():
            self.keys += [f'{name}_min_height', f'{name}_min_area']
        cv2.namedWindow(self.WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.WINDOW, 560, 32 * len(self.sliders) + 20)
        cv2.namedWindow(self.LIST_WINDOW, cv2.WINDOW_AUTOSIZE)
        for name, lo, hi, get, _ in self.sliders:
            cv2.createTrackbar(name, self.WINDOW, min(hi, max(lo, get(self.base))), hi, lambda _x: None)
            cv2.setTrackbarMin(name, self.WINDOW, lo)
        self.applied = self.positions()
        self.pending, self.moved_at = None, 0.0
        self.drawn = None
        self.refresh()

    def refresh(self) -> None:
        """Redraw the list of slider names and the values they stand for, if a slider moved."""
        pos = self.positions()
        if pos == self.drawn:
            return
        self.drawn = pos
        v = self.values(pos)
        img = np.full((24 * len(self.sliders) + 34, 560, 3), 30, np.uint8)
        text(img, 'slider (top to bottom) = value used   [parameter name]', (8, 20), 0.45, (255, 200, 120))
        for i, ((name, *_), key) in enumerate(zip(self.sliders, self.keys)):
            changed = v[key] != self.base[key]
            value = ('%g' % v[key]) if isinstance(v[key], (int, float)) else str(v[key])
            text(img, '%-28s = %s' % (name, value), (8, 44 + 24 * i), 0.45,
                 YELLOW if changed else (230, 230, 230))
            text(img, '[%s]' % key, (330, 44 + 24 * i), 0.38, (150, 150, 150))
        cv2.imshow(self.LIST_WINDOW, img)

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


# --- main -------------------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag', help='bag directory')
    ap.add_argument('--model', default='', help='YOLO weights (.pt/.onnx); default: clothoid_yolo/models/best.pt')
    ap.add_argument('--source', choices=('auto', 'model', 'car'), default='auto',
                    help="boxes from the model, or the car's recorded /detections (auto: car if recorded)")
    ap.add_argument('--yolo-hz', type=float, default=10.0,
                    help='YOLO frames per second the node gets (0: every camera frame), default 10')
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--device', default='', help="'' auto, 'cpu', 'cuda:0'")
    ap.add_argument('-p', '--param', action='append', default=[], metavar='NAME=VALUE',
                    help='override a yolo.yaml / mission.yaml parameter, e.g. -p sign_min_height=20')
    ap.add_argument('--start', type=float, default=0.0, help='seconds from the first frame')
    ap.add_argument('--out', help='write a video (.mp4), or one frame at --start (.png), instead of a window')
    ap.add_argument('--tune', action='store_true', help='slider window to change parameters live')
    args = ap.parse_args(argv)
    bag = os.path.abspath(args.bag.rstrip('/'))

    file_values = load_params([])
    file_values['yolo_hz'] = args.yolo_hz
    values = load_params(args.param)
    if not any(item.startswith('yolo_hz=') for item in args.param):
        values['yolo_hz'] = args.yolo_hz

    topic, frames, car, pitch, info = read_bag(bag)
    if not frames:
        sys.exit('no camera frame in the bag')
    cx, width, height = info if info else (319.9, 640, 480)
    source = args.source if args.source != 'auto' else ('car' if car else 'model')
    if source == 'car':
        if not car:
            sys.exit('the bag has no /detections (run with --source model)')
        stamps, boxes = [s for s, _ in car], [b for _, b in car]
        values['yolo_hz'] = file_values['yolo_hz'] = 0.0      # the car's frames as they came
        label = "car's /detections"
    else:
        model = find_model(args.model)
        if not model or not os.path.isfile(model):
            sys.exit('no YOLO model: pass --model /path/best.pt '
                     '(or copy it to ros2_ws/src/clothoid_yolo/models/best.pt)')
        stamps, boxes = [s for s, _ in frames], infer_all(bag, frames, model, args.imgsz, args.device)
        label = os.path.basename(model)
    print('%s: %s, %d frames, %d YOLO frames (%s)' % (os.path.basename(bag), topic, len(frames),
                                                       len(stamps), label))

    replay = Replay(stamps, boxes, pitch, cx, width, height)
    replay.run(values)
    t0 = frames[0][0]
    drawer = Drawer(replay, t0, frames[-1][0], label)
    drawer.build_timeline()
    total = len(frames)
    start = next((i for i, (s, _) in enumerate(frames) if s - t0 >= args.start), total - 1)

    if args.out:
        if args.out.endswith('.png'):
            s, raw = frames[start]
            cv2.imwrite(args.out, drawer.render(s, start, total, raw, values))
            print('wrote', args.out)
            return
        writer = None
        for i in range(start, total):
            s, raw = frames[i]
            img = drawer.render(s, i, total, raw, values)
            if writer is None:
                writer = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*'mp4v'), 30.0,
                                         (img.shape[1], img.shape[0]))
            writer.write(img)
        writer.release()
        print('wrote %s (%d frames)' % (args.out, total - start))
        return

    window = 'yolo_viewer'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, drawer.width, CAM_H + drawer.tl_h)
    tuner = Tuner(values) if args.tune else None
    stamps_all = [s for s, _ in frames]
    clicked = {'t': None}

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN and y >= drawer.tl_top and x >= LABEL_W:
            clicked['t'] = drawer.t_of(x)
    cv2.setMouseCallback(window, on_mouse)

    def closed():
        return cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) == 0

    def seek(t):
        return min(total - 1, max(0, bisect.bisect_left(stamps_all, t)))

    events = replay.events()
    print('%d trigger changes; keys: space play, d/a step, n/b next/prev change, w/s speed, '
          'p png, y params, q quit; click the timeline to jump' % len(events))
    pos, playing, rate = start, False, 1.0
    while True:
        s, raw = frames[pos]
        cv2.imshow(window, drawer.render(s, pos, total, raw, values))
        if playing:
            nxt = stamps_all[pos + 1] if pos + 1 < total else s + 0.033
            key = read_key(max(1, int((nxt - s) / rate * 1000)))
        else:
            key = -1
            while key == -1 and not closed() and clicked['t'] is None:
                key = read_key(50)
                if tuner and (tuner.pending is not None or tuner.positions() != tuner.applied):
                    tuner.refresh()
                    break
        if tuner:
            tuner.refresh()
            new = tuner.changed(time.monotonic())
            while new is None and tuner.pending is not None and key == -1 and not playing:
                key = read_key(50)
                tuner.refresh()
                new = tuner.changed(time.monotonic())
            if new is not None:
                if source == 'car':
                    new['yolo_hz'] = 0.0
                values = new
                replay.run(values)
                drawer.build_timeline()
                events = replay.events()
                print('applied:', describe(values, file_values).split('\n', 1)[-1].replace('\n', ' | '))
        if clicked['t'] is not None:
            pos, playing, clicked['t'] = seek(clicked['t']), False, None
            continue
        if key in (ord('q'), 27) or closed():
            break
        if key == ord(' '):
            playing = not playing
        elif key in (ord('d'),) + KEY_RIGHT:
            playing, pos = False, min(total - 1, pos + 1)
        elif key in (ord('a'),) + KEY_LEFT:
            playing, pos = False, max(0, pos - 1)
        elif key == ord('n'):
            later = [e for e in events if e > s + 1e-3]
            playing, pos = False, seek(later[0]) if later else pos
        elif key == ord('b'):
            earlier = [e for e in events if e < s - 1e-3]
            playing, pos = False, seek(earlier[-1]) if earlier else pos
        elif key == ord('w'):
            rate = min(8.0, rate * 2)
            print('speed x%g' % rate)
        elif key == ord('s'):
            rate = max(0.125, rate / 2)
            print('speed x%g' % rate)
        elif key == ord('p'):
            path = os.path.join(os.path.dirname(bag), '%s_yolo_%.2f.png' % (os.path.basename(bag), s - t0))
            cv2.imwrite(path, drawer.render(s, pos, total, raw, values))
            print('saved', path)
        elif key == ord('y'):
            print(describe(values, file_values))
        elif key == -1 and playing:
            if pos + 1 < total:
                pos += 1
            else:
                playing = False
                print('end of bag')
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
