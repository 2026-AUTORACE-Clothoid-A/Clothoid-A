"""Read-only bag replay using the current mission and lane node callbacks.

No executor is spun. All application publishers are replaced with local captures
before any bag callback runs; generated commands never leave this process.
"""
from dataclasses import asdict
from functools import lru_cache
from pathlib import Path
from types import SimpleNamespace
import hashlib
import heapq
import importlib
import json
import os
import sqlite3
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for package in ('clothoid_common', 'clothoid_lane', 'clothoid_parking',
                'clothoid_finish', 'clothoid_mission'):
    sys.path.insert(0, str(ROOT/'ros2_ws/src'/package))


class Capture:
    def __init__(self, callback=None):
        self.last = None
        self.callback = callback

    def publish(self, message):
        self.last = message
        if self.callback:
            self.callback(message)

    def get_subscription_count(self):
        return 1


def capture_publishers(node):
    captures = {}
    for name, value in list(vars(node).items()):
        if name.endswith('_pub') and hasattr(value, 'publish'):
            captures[name] = Capture()
            setattr(node, name, captures[name])
    return captures


def modules():
    names = (
        'clothoid_common.image_utils', 'clothoid_lane.lane_detector',
        'clothoid_lane.lane_tracker', 'clothoid_lane.steering', 'clothoid_lane.bump_gap',
        'clothoid_mission.scene_evidence', 'clothoid_mission.sign_tracker',
        'clothoid_parking.tracker', 'clothoid_finish.bev', 'clothoid_finish.finish_core',
        'clothoid_mission.line_stop', 'clothoid_mission.mission_logic',
        'clothoid_lane.lane_main', 'clothoid_mission.mission_node',
    )
    return {name: importlib.reload(importlib.import_module(name)) for name in names}


def bag_files(path):
    path = Path(path).resolve()
    files = sorted(path.glob('*.db3')) if path.is_dir() else [path]
    if not files or any(not p.is_file() or p.suffix != '.db3' for p in files):
        raise ValueError('SQLite3 백 폴더 또는 .db3 파일을 선택하세요.')
    return files


def inventory(path):
    files = bag_files(path)
    topics, starts, ends, count = {}, [], [], {}
    for file in files:
        with sqlite3.connect(f'file:{file}?mode=ro', uri=True) as db:
            topics.update(dict(db.execute('select name,type from topics')))
            a, b = db.execute('select min(timestamp),max(timestamp) from messages').fetchone()
            if a is not None:
                starts.append(a); ends.append(b)
            for name, n in db.execute('select t.name,count(*) from messages m join topics t '
                                      'on t.id=m.topic_id group by t.name'):
                count[name] = count.get(name, 0)+n
    if not starts:
        raise ValueError('백에 메시지가 없습니다.')
    return files, topics, min(starts), max(ends), count


def messages(files, wanted):
    connections = []
    streams = []
    try:
        for file in files:
            db = sqlite3.connect(f'file:{file}?mode=ro', uri=True)
            connections.append(db)
            table = {i: (n, t) for i, n, t in db.execute('select id,name,type from topics') if n in wanted}
            if not table:
                continue
            query = ('select timestamp,id,topic_id,data from messages where topic_id in ('+
                     ','.join('?' for _ in table)+') order by timestamp,id')
            cursor = db.execute(query, list(table))
            def stream(cursor=cursor, file=file, table=table):
                for stamp, row, topic, blob in cursor:
                    yield stamp, str(file), row, *table[topic], blob
            streams.append(stream())
        yield from heapq.merge(*streams, key=lambda item: (item[0], item[1], item[2]))
    finally:
        for db in connections:
            db.close()


def decode(message):
    if hasattr(message, 'format'):
        frame = cv2.imdecode(np.frombuffer(bytes(message.data), np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError('카메라 JPEG/PNG를 읽을 수 없습니다.')
        return frame
    from clothoid_common.image_utils import image_to_bgr
    return image_to_bgr(message)


@lru_cache(maxsize=8)
def read_image(reference):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    file, row, kind = reference
    with sqlite3.connect(f'file:{file}?mode=ro', uri=True) as db:
        blob = db.execute('select data from messages where id=?', (row,)).fetchone()[0]
    return decode(deserialize_message(blob, get_message(kind)))


def detection_rows(node, detections):
    """Explanations read the active gate parameters; acceptance comes from the real node."""
    from clothoid_mission.mission_logic import classes_seen
    value = lambda name: node.get_parameter(name).value
    sizes = {f'sign_{name}': (value(name+'_min_area'), value(name+'_min_height'))
             for name in ('bump', 'slow', 'parking', 'pedestrian')}
    rows = []
    for d in detections:
        w, h = d.x_max-d.x_min, d.y_max-d.y_min
        accepted = classes_seen([d], camera_center_x=node.cam.cx, min_sizes=sizes,
            light_max_offset_height_ratio=value('light_max_offset_height_ratio'),
            sign_max_offset_height_ratio=value('sign_max_offset_height_ratio'),
            fork_max_offset_height_ratio=value('fork_max_offset_height_ratio'),
            bump_max_offset_height_ratio=value('bump_max_offset_height_ratio'))
        scene_candidate = (value('scene_sign_candidates') and node.logic.p.require_scene_confirmation and
            (d.class_name == 'sign_parking' or (d.class_name == 'sign_pedestrian' and
             node.logic.p.use_stop_line and value('line_require_crosswalk'))))
        gate = node.sign_tracker
        track = next((t for t in gate.tracks.values() if t.last_seen == node.seen_time and
                      t.box.class_name == d.class_name and
                      (t.box.x_min, t.box.y_min, t.box.x_max, t.box.y_max) ==
                      (d.x_min, d.y_min, d.x_max, d.y_max)), None)
        hits = track.hits if track is not None else 0
        reason = gate.box_reason(d, node.warp.width, node.warp.height)
        if not scene_candidate and d.class_name not in accepted:
            minimum_size = sizes.get(d.class_name, (0, 0))
            status = '크기 탈락' if w*h < minimum_size[0] or h < minimum_size[1] else '가로 위치 탈락'
        elif reason:
            status = '품질 탈락: '+reason
        elif track is None:
            status = '중복/색상 겹침 · 초록 우선'
        else:
            status = '확정' if track.confirmed else f'확인 {hits}/{gate.required(d.class_name)}'
        rows.append(dict(name=d.class_name, confidence=float(d.confidence),
                         box=(d.x_min, d.y_min, d.x_max, d.y_max), status=status,
                         hits=hits, required=gate.required(d.class_name)))
    return rows


def analyse(path, camera='limo2', progress=lambda value: None):
    import rclpy
    from rclpy.time import Time
    from rclpy.clock import ClockType
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    files, topics, start, end, counts = inventory(path)
    cameras = [n for n, t in topics.items() if t in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage')
               and not any(word in n for word in ('debug', 'depth', 'warp', 'viz/'))]
    if not cameras:
        raise ValueError('원본 카메라 토픽이 없는 백입니다.')
    image_topic = min(cameras, key=lambda n: ('CompressedImage' not in topics[n], 'color' not in n, n))
    loaded = modules()
    mission_module = loaded['clothoid_mission.mission_node']
    lane_module = loaded['clothoid_lane.lane_main']
    cfg = ROOT/'ros2_ws/src'
    configs = [cfg/'clothoid_bringup/config/topics.yaml', cfg/'clothoid_lane/config/lane.yaml',
               cfg/'clothoid_lane/config/stopline.yaml', cfg/f'clothoid_lane/config/camera_{camera}.yaml',
               cfg/'clothoid_mission/config/mission.yaml']
    args = ['--ros-args']
    for file in configs:
        args += ['--params-file', str(file)]
    args += ['-p', 'debug:=true']
    # This process owns a separate, localhost-only domain, including any node created on reload.
    os.environ['ROS_DOMAIN_ID'] = '229'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    cv2.setNumThreads(1)
    nodes = []
    rclpy.init(args=args)
    try:
        mission = mission_module.MissionNode(); nodes.append(mission)
        lane = lane_module.LaneMainNode(); nodes.append(lane)
        mp, lp = capture_publishers(mission), capture_publishers(lane)
        mission._image_on = lambda on: None
        clock = SimpleNamespace(ns=start)
        virtual = SimpleNamespace(now=lambda: Time(nanoseconds=clock.ns, clock_type=ClockType.ROS_TIME))
        for node in nodes:
            node.get_clock = lambda: virtual
            node.get_logger = lambda: SimpleNamespace(info=lambda *a, **k: None,
                warn=lambda *a, **k: None, warning=lambda *a, **k: None,
                debug=lambda *a, **k: None, error=lambda *a, **k: None)
        for key, callback in (('limit_pub', lane.on_speed_limit), ('direction_pub', lane.on_direction),
                              ('parking_pub', lane.on_parking), ('steering_lock_pub', lane.on_steering_lock),
                              ('bump_approach_pub', lane.on_bump_approach)):
            if key in mp:
                mp[key].callback = callback
        warnings = []
        if topics[image_topic] == 'sensor_msgs/msg/CompressedImage':
            warnings.append('압축 영상 재계산: 실제 raw 입력과 픽셀 차이 가능')
        if ('CompressedImage' in topics[image_topic]) != mission.get_parameter('compressed').value:
            warnings.append('미션 영상 입력 형식을 변환하여 재생')
        def sensor(node, parameter, fallback):
            preferred = node.get_parameter(parameter).value
            if preferred in topics:
                return preferred
            if fallback in topics:
                warnings.append(f'{parameter}: {preferred} 미기록 → {fallback} 재생')
                return fallback
            warnings.append(f'{parameter} 미기록')
            return None
        mi, li = sensor(mission, 'imu_topic', '/imu'), sensor(lane, 'imu_topic', '/imu')
        mo, lo = sensor(mission, 'odom_topic', '/odom'), sensor(lane, 'odom_topic', '/odom')
        info = mission.get_parameter('camera_info_topic').value
        if info not in topics:
            warnings.append('CameraInfo 미기록: 정지선 거리 재계산 불가')
        if '/detections' not in topics:
            warnings.append('YOLO 검출 미기록: 표지판·미션 재계산 제한')
        wanted = {image_topic, info, mi, li, mo, lo, '/detections', '/mission/state',
                  '/cmd_vel', '/cmd_vel_lane', '/speed_limit', '/steering_lock',
                  '/bump/approaching', '/parking_done', '/parking_sign', '/stopline_near',
                  '/cone/mission_active'}-{None}
        frames, events = [], []
        recorded = {}; dets = []; det_stamp = None
        current_result = None
        actual_detect = lane.detector.detect
        def detect(*args, **kwargs):
            nonlocal current_result
            current_result = actual_detect(*args, **kwargs)
            return current_result
        lane.detector.detect = detect
        last_state = None; last_lane = None; last_confirmed = None; last_recorded = None
        next_tick = start
        period = round(1e9/mission.get_parameter('rate_hz').value)
        value = lambda n: mission.get_parameter(n).value
        def tick(stamp):
            nonlocal last_state, last_confirmed
            clock.ns = stamp
            mission.on_timer()
            state = json.loads(mp['mission_state_pub'].last.data)
            key = (state.get('bump_state'), state.get('line_phase'), state.get('steering_lock'),
                   state.get('entry_phase'))
            if key != last_state:
                events.append(((stamp-start)*1e-9,
                    f'현재 · 범프 {key[0]} / 시작선 {key[3] or "대기"} / 정지선 {key[1] or "대기"} / 잠금 {key[2]}'))
                last_state = key
            confirmed = 'sign_bump' in state.get('seen', [])
            if confirmed != last_confirmed:
                if confirmed:
                    events.append(((stamp-start)*1e-9, '현재 · 방지턱 표지판 확정'))
                last_confirmed = confirmed

        types = {}; images = 0
        for stamp, file, row, name, kind, blob in messages(files, wanted):
            while next_tick < stamp:
                tick(next_tick); next_tick += period
            clock.ns = stamp
            if kind not in types:
                types[kind] = get_message(kind)
            message = deserialize_message(blob, types[kind])
            if name == info:
                mission.on_info(message)
            if name == mi:
                mission.on_imu(message)
            if name == li:
                lane.on_imu(message)
            if name == mo:
                mission.on_odom(message)
            if name == lo:
                lane.on_odom(message)
            if name == '/detections':
                mission.on_detections(message)
                dets = detection_rows(mission, message.detections); det_stamp = stamp
            for topic, callback in (('/parking_done', mission.on_parking_done),
                    ('/parking_sign', mission.on_parking_sign), ('/stopline_near', mission.on_parking_line),
                    ('/cone/mission_active', mission.on_cone_active)):
                if name == topic:
                    callback(message)
            if name == '/mission/state':
                try:
                    original = json.loads(message.data)
                    recorded['state'] = (stamp, original)
                    key = (original.get('bump_state'), original.get('line_phase'), original.get('steering_lock'))
                    if key != last_recorded:
                        events.append(((stamp-start)*1e-9,
                            f'기록 · 범프 {key[0] or "미기록"} / 정지선 {key[1] or "대기"} / 잠금 {key[2]}'))
                        last_recorded = key
                except (ValueError, TypeError):
                    pass
            elif name in ('/cmd_vel', '/cmd_vel_lane'):
                recorded[name] = (stamp, (message.linear.x, message.angular.z))
            elif name in ('/speed_limit', '/steering_lock', '/bump/approaching'):
                recorded[name] = (stamp, message.data)
            if name != image_topic:
                continue
            bgr = decode(message)
            expected_compressed = mission.get_parameter('compressed').value
            mission_image = message
            if hasattr(message, 'format') != expected_compressed:
                if expected_compressed:
                    from sensor_msgs.msg import CompressedImage
                    ok, encoded = cv2.imencode('.jpg', bgr)
                    if not ok:
                        raise ValueError('미션 입력 영상 변환 실패')
                    mission_image = CompressedImage(header=message.header, format='jpeg', data=encoded.tobytes())
                else:
                    from clothoid_common.image_utils import bgr_to_image
                    mission_image = bgr_to_image(bgr, message.header)
            mission.on_image(mission_image)
            lp['cmd_pub'].last = None
            from clothoid_common.image_utils import bgr_to_image
            lane.on_image(bgr_to_image(bgr, message.header))
            lane_jpeg = None
            if current_result is not None and current_result.debug_image is not None:
                ok, encoded = cv2.imencode('.jpg', current_result.debug_image,
                                           [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    lane_jpeg = encoded.tobytes()
            found = current_result is not None and current_result.found
            if found != last_lane:
                events.append(((stamp-start)*1e-9, '현재 · 차선 검출' if found else '현재 · 차선 유실'))
                last_lane = found
            state = json.loads(mp['mission_state_pub'].last.data) if mp['mission_state_pub'].last else {}
            command = lp['cmd_pub'].last
            age = None if det_stamp is None else (stamp-det_stamp)*1e-9
            def recorded_value(key):
                item = recorded.get(key)
                return item[1] if item and 0 <= (stamp-item[0])*1e-9 <= value('input_timeout') else None
            frames.append(dict(t=(stamp-start)*1e-9, reference=(file, row, kind),
                state=state, recorded_state=recorded_value('state'),
                recorded_command=recorded_value('/cmd_vel'), recorded_limit=recorded_value('/speed_limit'),
                detections=dets, detection_age=age, lane_found=bool(found), lane_jpeg=lane_jpeg,
                command=None if command is None else (command.linear.x, command.angular.z),
                steering_deg=float(np.degrees(lane.last_steer)), bump_gap=lane.bump_gap.active,
                raw_pitch=mission.latest_pitch if mission._fresh(mission.latest_pitch_time) else None))
            images += 1
            if images % 10 == 0:
                progress(min(99, round(images/max(1, counts[image_topic])*100)))
        if not frames:
            raise ValueError('카메라 프레임을 읽지 못했습니다.')
        parameters = {n: mission.get_parameter(n).value for n in (
            'white_s_max', 'white_v_min', 'line_min_run', 'line_band', 'line_min_thickness',
            'line_max_thickness', 'line_require_crosswalk', 'input_timeout', 'bump_pitch_thresh',
            'bump_level_thresh', 'bump_level_duration', 'bump_arm_timeout', 'bump_cooldown',
            'bump_min_area', 'bump_min_height', 'detection_confidence', 'detection_confirm_hits',
            'detection_window', 'block_bump_after_cone', 'front_axle_x', 'stop_line_gap',
            'entry_confirm_frames', 'entry_min_span', 'entry_reaction_sec', 'entry_deceleration', 'entry_rate_hz')}
        digest = hashlib.sha256()
        for module in loaded.values():
            digest.update(Path(module.__file__).read_bytes())
        for file in configs:
            digest.update(file.read_bytes())
        digest.update(Path(__file__).read_bytes())
        result = dict(path=str(Path(path)), frames=frames, events=events, warnings=sorted(set(warnings)),
            params=parameters, warp=lane.detector.p, bev=mission.bev, camera=camera,
            detector_values={n: lane.get_parameter(n).value for n in lane_module.DETECTOR_PARAMS},
            cam=asdict(mission.cam), finish_params=mission.finish_params,
            image_topic=image_topic,
            digest=digest.hexdigest()[:12], duration=(end-start)*1e-9,
            yellow_lower=tuple(lane.detector.p.hsv_lower), yellow_upper=tuple(lane.detector.p.hsv_upper),
            replay='Production MissionNode + LaneMainNode callbacks; 20 Hz mission timer; recorded detections')
        progress(100)
        return result
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


def discover():
    found = {p.parent for p in (ROOT/'bag').rglob('metadata.yaml')}
    found.update(p.parent for p in (ROOT/'bag').glob('*/*.db3'))
    return sorted(found, key=lambda p: p.stat().st_mtime, reverse=True)
