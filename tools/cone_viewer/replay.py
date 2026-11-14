"""Read a ROS bag and replay the current cone modules without publishing ROS topics."""
from pathlib import Path
import hashlib
import importlib
import math
import sqlite3

import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = Path(__file__).resolve().parents[2]


def analyse(path, progress=lambda value: None):
    # A reload always follows source edits, even when the viewer is already open.
    from clothoid_lidar import cone_trace, cone_corner, cone_sorter, cone_follower, obstacle_extractor, cone_corridor, cone_exit, cone_recovery
    for module in (cone_trace, cone_corner, cone_exit, cone_corridor, cone_recovery, cone_sorter, cone_follower, obstacle_extractor):
        importlib.reload(module)
    config_path = ROOT/'ros2_ws/src/clothoid_lidar/config/lidar.yaml'
    config = yaml.safe_load(config_path.read_text())
    def params(cls, name):
        return cls(**{k: v for k, v in config[name]['ros__parameters'].items()
                      if k in cls.__dataclass_fields__})
    ep = params(obstacle_extractor.ExtractorParams, 'obstacle_extractor')
    sp = params(cone_sorter.SorterParams, 'cone_sorter')
    fp = params(cone_follower.FollowerParams, 'cone_follower')
    mission = cone_sorter.ConeMission(cone_sorter.ConeSorter(sp))
    follower = cone_follower.ConeFollower(fp)
    files = sorted(Path(path).glob('*.db3')) if Path(path).is_dir() else [Path(path)]
    if not files:
        raise ValueError('bag에 db3 파일이 없습니다.')
    sources = []; start_stamps = []
    for file in files:
        with sqlite3.connect(f'file:{file}?mode=ro', uri=True) as db:
            minimum = db.execute('select min(timestamp) from messages').fetchone()[0]
            if minimum is not None:start_stamps.append(minimum)
            topics = {i: (n, t) for i, n, t in db.execute('select id,name,type from topics')}
            wanted = {i: (n, t) for i, (n, t) in topics.items()
                      if t == 'sensor_msgs/msg/LaserScan' or n in ('/tf_static', '/tf', '/cmd_vel', '/cone/mission_active', '/odom')
                      or (t in ('sensor_msgs/msg/CompressedImage', 'sensor_msgs/msg/Image')
                          and 'debug' not in n and 'depth' not in n and 'warp' not in n)}
            camera = sorted([(i, n, t) for i, (n, t) in wanted.items() if 'Image' in t],
                            key=lambda r: ('CompressedImage' not in r[2], r[1]))
            chosen = camera[0][0] if camera else None
            wanted = {i: nt for i, nt in wanted.items() if 'Image' not in nt[1] or i == chosen}
            query = 'select topic_id,timestamp,data from messages where topic_id in ('+','.join('?' for _ in wanted)+') order by timestamp'
            for i, stamp, data in db.execute(query, list(wanted)):
                sources.append((stamp, *wanted[i], data))
    if not sources:
        raise ValueError('재생 가능한 센서 토픽이 없습니다.')
    sources.sort(key=lambda r: r[0])
    t0 = min(start_stamps)
    frames, cameras, recorded, flags = [], [], [], []
    transforms = {}; types = {}
    # TF may have been recorded after the first scan. Read fixed sensor transforms first.
    for _, name, ty, blob in sources:
        if name not in ('/tf_static', '/tf'):
            continue
        kind = types.setdefault(ty, get_message(ty))
        for tr in deserialize_message(blob, kind).transforms:
            if tr.header.frame_id.lstrip('/') in ('base_link', 'base_footprint'):
                q = tr.transform.rotation
                transforms[tr.child_frame_id.lstrip('/')] = (tr.transform.translation.x, tr.transform.translation.y,
                    math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
    left = right = np.zeros((0, 2)); lt = rt = None; active = False
    pose = None; pose_time = None
    previous_scan = start_scan = None
    last_event = None; events = []; command_rows = []
    for idx, (stamp, name, ty, blob) in enumerate(sources):
        t = (stamp-t0)*1e-9
        if idx % 100 == 0:
            progress(int(idx/len(sources)*100))
        if name in ('/tf', '/tf_static'):
            continue
        kind = types.setdefault(ty, get_message(ty)); msg = deserialize_message(blob, kind)
        if 'Image' in ty:
            if ty.endswith('CompressedImage'):
                cameras.append((t, bytes(msg.data)))
            else:
                from PyQt5.QtGui import QImage
                channels = 3 if msg.encoding in ('bgr8', 'rgb8') else 1
                if channels == 3:
                    image = QImage(bytes(msg.data), msg.width, msg.height, msg.step, QImage.Format_RGB888)
                    if msg.encoding == 'bgr8':image = image.rgbSwapped()
                    cameras.append((t, image.copy()))
            continue
        if name == '/cmd_vel':
            recorded.append((t, msg.linear.x, msg.angular.z)); continue
        if name == '/cone/mission_active':
            flags.append((t, bool(msg.data))); continue
        if name == '/odom':
            q = msg.pose.pose.orientation; pos = msg.pose.pose.position
            pose = (pos.x, pos.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            pose_time = t; follower.corridor.update_pose(pose)
            continue
        if ty != 'sensor_msgs/msg/LaserScan':
            continue
        frame = msg.header.frame_id.lstrip('/')
        transform = (0., 0., 0.) if frame in ('base_link', 'base_footprint') else transforms.get(frame)
        if transform is None:
            raise ValueError(f'{frame} → base_link TF가 없습니다. 추정 변환으로 그리지 않습니다.')
        points = obstacle_extractor.transform_points(obstacle_extractor.extract_obstacles(
            msg.ranges, msg.angle_min, msg.angle_increment, ep), *transform)
        ranges = np.asarray(msg.ranges); angles = msg.angle_min+np.arange(len(ranges))*msg.angle_increment
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= min(msg.range_max, 3.))
        scan = obstacle_extractor.transform_points(np.column_stack((ranges[valid]*np.cos(angles[valid]), ranges[valid]*np.sin(angles[valid]))), *transform)
        result = mission.update(points, t)
        command = None; callback_results = []
        def control():
            lf, rf = cone_recovery.current_boundaries(lt, rt,
                lt is not None and t-lt <= fp.boundary_timeout, rt is not None and t-rt <= fp.boundary_timeout)
            confirmed = cone_recovery.recovery_confirmed(lt, rt, previous_scan, lf, rf)
            started = start_scan is not None and previous_scan is not None and previous_scan > start_scan
            current_pose = pose if pose_time is not None and 0 <= t-pose_time <= fp.boundary_timeout else None
            value = follower.decide(left, right, lf, rf, current_pose, points,
                                    config['cone_follower']['ros__parameters'].get('exit_clearance', .2),
                                    allow_reverse=confirmed and started)
            if value is not None and value.reverse:
                remaining = fp.lfd-(follower.corridor.travelled-follower.reverse_start_travel)
                if not cone_recovery.reverse_sweep_clear(msg.ranges,msg.angle_min,msg.angle_increment,
                        msg.range_min,msg.range_max,transform,value.steering,fp.wheelbase,fp.rear_axle_x,remaining):
                    value = cone_follower.FollowResult(0.,0.,source='recovery_blocked')
            elif value is not None and value.source == 'no_boundary_path' and not confirmed and lt != rt:
                return None
            if value is not None:
                command_rows.append((t, value.speed, value.steering))
                callback_results.append(value)
            return value
        if result is not None:
            if len(result.left):
                left, lt = result.left, t
            if len(result.right):
                right, rt = result.right, t
            # Evaluate one completed scan, not an artificial intermediate
            # combination of its new left wall and the previous right wall.
            if active:command = control()
        command = callback_results[-1] if callback_results else None
        if active != mission.running:
            follower.reset()
            if mission.running:start_scan = t
            events.append((t, '콘 시작' if mission.running else '콘 종료'))
        active = mission.running
        if not active:
            left = right = np.zeros((0, 2)); lt = rt = None
        lf, rf = cone_recovery.current_boundaries(lt,rt,
            lt is not None and t-lt <= fp.boundary_timeout,rt is not None and t-rt <= fp.boundary_timeout)
        state = '콘 비활성' if not active else ('경계 수신 대기' if lt is None or rt is None else ('후진 명령' if command and command.reverse else
                ('보완 경로 주행' if command and command.speed > 0 and command.source != 'observed' else
                 ('경계 손실' if not lf or not rf else ('전진 경로' if command is not None else '명령 없음')))))
        if active and command is not None and command.speed == 0.:
            state = '경계 경로 없음 · 정지'
        if active and state != last_event:
            events.append((t, state))
        last_event = state
        fan = False
        if command is not None and not command.reverse:
            minimum_radius = fp.wheelbase/np.tan(np.deg2rad(28.))+.175/2
            fan = cone_corner.endpoint_centerline(left, right, fp.centerline_ds, minimum_radius) is not None
        frames.append(dict(t=t, scan=scan, obstacles=points, active=active, state=state,
            left=left.copy() if lf else np.zeros((0,2)), right=right.copy() if rf else np.zeros((0,2)),
            stale_left=left.copy() if not lf else np.zeros((0,2)), stale_right=right.copy() if not rf else np.zeros((0,2)),
            left_age=None if lt is None else t-lt, right_age=None if rt is None else t-rt,
            command=command, callback_results=callback_results, fan=fan))
        previous_scan = t
    if not frames:
        raise ValueError('라이다 스캔이 없습니다.')
    digest = hashlib.sha256()
    for module in (cone_trace, cone_corner, cone_sorter, cone_follower, obstacle_extractor, cone_exit, cone_corridor, cone_recovery):
        digest.update(Path(module.__file__).read_bytes())
    digest.update(Path(__file__).read_bytes())
    digest.update(config_path.read_bytes())
    return dict(frames=frames, cameras=cameras, recorded=recorded, flags=flags, events=events,
                params=fp, path=str(path), source_hash=digest.hexdigest()[:12], commands=command_rows)
