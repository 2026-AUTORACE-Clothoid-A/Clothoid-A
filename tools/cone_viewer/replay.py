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
    from clothoid_lidar import cone_trace, cone_sorter, cone_follower, obstacle_extractor, cone_exit, cone_exit_pursuit, cone_odometry, cone_lane_connection
    for module in (cone_trace, cone_exit, cone_exit_pursuit, cone_sorter, cone_follower, obstacle_extractor, cone_odometry, cone_lane_connection):
        importlib.reload(module)
    config_path = ROOT/'ros2_ws/src/clothoid_lidar/config/lidar.yaml'
    config = yaml.safe_load(config_path.read_text())
    def params(cls, name):
        return cls(**{k: v for k, v in config[name]['ros__parameters'].items()
                      if k in cls.__dataclass_fields__})
    ep = params(obstacle_extractor.ExtractorParams, 'obstacle_extractor')
    sp = params(cone_sorter.SorterParams, 'cone_sorter')
    fp = params(cone_follower.FollowerParams, 'cone_follower')
    mission = cone_sorter.ConeMission(cone_sorter.ConeSorter(sp), hold_exit=True)
    follower = cone_follower.ConeFollower(fp)
    exit_memory = cone_exit.ExitPathMemory()
    odometry = cone_odometry.ConeOdometry(fp.rear_axle_x, fp.boundary_timeout)
    pose_generation = odometry.generation
    follower_config = config['cone_follower']['ros__parameters']
    xp = cone_exit_pursuit.ExitPursuitParams(speed=follower_config['exit_speed'],
        lfd=follower_config['exit_lfd'], wheelbase=fp.wheelbase, rear_axle_x=fp.rear_axle_x,
        centerline_ds=fp.centerline_ds, boundary_timeout=fp.boundary_timeout)

    def result_value(*args, source='observed', **kwargs):
        result = cone_follower.FollowResult(*args, **kwargs)
        result.source = source  # display metadata, never an input to main's controller
        return result
    connection = cone_lane_connection.LaneConnection(cone_lane_connection.ConnectionParams(**{
        k: follower_config['exit_connect_' + k]
        for k in cone_lane_connection.ConnectionParams.__dataclass_fields__}))
    odom_topic, imu_topic = follower_config.get('odom_topic', '/odom'), follower_config.get('imu_topic', '/imu')
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
                      if t == 'sensor_msgs/msg/LaserScan' or n in ('/tf_static', '/tf', '/cmd_vel', '/cone/mission_active', odom_topic, imu_topic, '/lane/cone_exit_ready', '/lane/cone_exit_candidate', '/avoid/active')
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
    lane_ready = False; lane_ready_time = None
    avoid_active = False; avoid_seen = avoid_last = None
    lane_candidate = None; lane_candidate_stamp = lane_candidate_time = None
    exit_reference_end = None
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
        if name == '/avoid/active':
            avoid_active, avoid_seen = bool(msg.data), t
            continue
        if name == '/lane/cone_exit_ready':
            lane_ready, lane_ready_time = bool(msg.data), t
            continue
        if name == '/lane/cone_exit_candidate':
            candidate_stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9-t0*1e-9
            if (msg.header.frame_id == 'base_link' and 0 <= t-candidate_stamp <= fp.boundary_timeout
                    and (lane_candidate_stamp is None or candidate_stamp > lane_candidate_stamp)
                    and all(math.isfinite(v) for v in (msg.twist.linear.x,msg.twist.angular.z))
                    and msg.twist.linear.x >= 0.):
                lane_candidate = msg.twist
                lane_candidate_stamp, lane_candidate_time = candidate_stamp, t
            continue
        if name in (odom_topic, imu_topic):
            header_time = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            sensor_time = header_time-t0*1e-9 if header_time else t
            if -.02 <= t-sensor_time <= fp.boundary_timeout:
                if name == odom_topic:
                    odometry.odom(sensor_time, msg.twist.twist.linear.x)
                else:
                    gyro = msg.angular_velocity.z if msg.angular_velocity_covariance[0] >= 0 else math.nan
                    odometry.imu(sensor_time, gyro)
            if pose_generation != odometry.generation:
                exit_memory.clear();exit_reference_end = None
                connection.clear()
                pose_generation = odometry.generation
                events.append((t, 'IMU/속도 수신 중단: 저장 경로 초기화'))
            pose = odometry.pose(t)
            pose_time = odometry.stamp if pose is not None else None
            if pose is not None:
                exit_memory.update_pose(pose)
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
        was_exiting = mission.exiting
        held = False
        if not mission.running:
            gate = config['cone_sorter']['ros__parameters']
            if avoid_active and avoid_seen is not None and t-avoid_seen <= gate.get('avoid_active_timeout', .5):
                avoid_last = t
                held = True
            elif avoid_last is not None:
                held = t-avoid_last < gate.get('avoid_release_hold', 2.)
        result = mission.update(points, t, may_start=not held)
        if mission.exiting and not was_exiting:
            connection.clear()
            lane_ready = False; lane_ready_time = None
            events.append((t, '콘 출구: 저장 경로 유지'))
        elif was_exiting and not mission.exiting:
            connection.clear()
            events.append((t, '콘 경계 재검출'))
        command = None; callback_results = []
        def control():
            nonlocal exit_reference_end
            if lt is None or rt is None:
                return None
            lf, rf = t-lt <= fp.boundary_timeout, t-rt <= fp.boundary_timeout
            current_pose = pose if pose_time is not None and 0 <= t-pose_time <= fp.boundary_timeout else None
            value = follower.decide(left, right, lf, rf)
            if value is not None:
                value.source = 'observed' if not value.reverse else 'main_reverse'
                if not value.reverse and len(value.center) and current_pose is not None:
                    if exit_memory.remember(value.center, current_pose, xp.lfd, xp.centerline_ds):
                        exit_reference_end = cone_exit.transform(value.center[-1:], current_pose)[0]
                command_rows.append((t, value.speed, value.steering))
                callback_results.append(value)
            return value
        if result is not None:
            # Match main's two boundary callbacks, including a still-fresh older side.
            if len(result.left):
                left, lt = result.left, t
                if active:command = control()
            if len(result.right):
                right, rt = result.right, t
                if active:command = control()
        if active and mission.exiting:
            fresh_pose = pose is not None and pose_time is not None and 0 <= t-pose_time <= xp.boundary_timeout
            current_pose = pose if fresh_pose else None
            arrived = current_pose is not None and exit_memory.handover_ready(current_pose, xp.rear_axle_x)
            if connection.active or arrived:
                candidate_fresh = (lane_candidate is not None
                    and 0 <= t-lane_candidate_time <= xp.boundary_timeout
                    and 0 <= t-lane_candidate_stamp <= xp.boundary_timeout)
                command = result_value(0.,0.,source='lane_connection_wait')
                if current_pose is not None and candidate_fresh:
                    if not connection.active:
                        last = command_rows[-1] if command_rows else (t,0.,0.)
                        connection.start(t,current_pose,last[2] if last[1]>0. else 0.,xp.wheelbase)
                    ready = lane_ready and lane_ready_time is not None and 0 <= t-lane_ready_time <= xp.boundary_timeout
                    velocity, angle, done = connection.step(t,current_pose,lane_candidate.linear.x,
                        lane_candidate.angular.z,xp.wheelbase,xp.speed,odometry.speed,
                        odometry.gyro-odometry.bias,ready,xp.boundary_timeout)
                    measured = math.atan(xp.wheelbase*(odometry.gyro-odometry.bias)/odometry.speed) if odometry.speed>.03 else 0.
                    arcs = [cone_lane_connection.forward_arc(a,xp.wheelbase,xp.rear_axle_x,xp.lfd,xp.centerline_ds)
                            for a in (angle,measured)]
                    clear = all(cone_exit.clearance(arc,points)>=follower_config.get('exit_clearance',.2) for arc in arcs)
                    if velocity>0. and clear:
                        command = result_value(velocity,angle,center=arcs[0],source='lane_connection')
                        if done:mission.finish_exit()
                    else:connection.pause(t)
                elif connection.active:connection.pause(t)
                command_rows.append((t,command.speed,command.steering));callback_results.append(command)
            else:
                center = exit_memory.remaining(pose, xp.rear_axle_x) if fresh_pose else np.zeros((0, 2))
                target = cone_exit_pursuit.look_ahead_point(center, xp)
                usable = (target is not None and target[0] > 0.
                          and cone_exit.clearance(center, points) >= config['cone_follower']['ros__parameters'].get('exit_clearance', .2))
                command = (result_value(xp.speed, cone_exit_pursuit.steering_from_target(target, xp),
                           center=center, target=target, source='exit_path') if usable else
                           result_value(0., 0., center=center, source='exit_wait'))
                command_rows.append((t, command.speed, command.steering));callback_results.append(command)
        observed_end = (cone_exit.transform(exit_reference_end[None, :], pose, True)[0]
                        if mission.exiting and exit_reference_end is not None and pose is not None else None)
        rear_remaining = (exit_memory.rear_axle_remaining(pose, fp.rear_axle_x)
                           if mission.exiting and pose is not None and pose_time is not None
                           and 0 <= t-pose_time <= fp.boundary_timeout else None)
        command = callback_results[-1] if callback_results else None
        if active != mission.running:
            connection.clear()
            exit_memory.clear();exit_reference_end = None
            if mission.running:start_scan = t
            events.append((t, '콘 시작' if mission.running else '콘 종료'))
        active = mission.running
        if not active:
            left = right = np.zeros((0, 2)); lt = rt = None
        lf, rf = lt is not None and t-lt <= fp.boundary_timeout, rt is not None and t-rt <= fp.boundary_timeout
        state = '콘 비활성' if not active else ('경계 수신 대기' if lt is None or rt is None else ('후진 명령' if command and command.reverse else
                ('보완 경로 주행' if command and command.speed > 0 and command.source != 'observed' else
                 ('경계 손실' if not lf or not rf else ('전진 경로' if command is not None else '명령 없음')))))
        if active and command is not None and command.speed == 0.:
            state = '경계 경로 없음 · 정지'
        if active and mission.exiting:
            state = '콘 출구 · 차선 준비 대기' if command is not None and command.speed == 0. else '콘 출구 · 저장 경로 추종'
            if command is not None and command.source.startswith('lane_connection'):
                state = '콘 출구 · 저속 차선 연결' if command.speed>0. else '콘 출구 · 차선 연결 입력/안전 조건 대기'
        if active and state != last_event:
            events.append((t, state))
        last_event = state
        fan = False
        frames.append(dict(t=t, scan=scan, obstacles=points, active=active, state=state,
            left=left.copy() if lf else np.zeros((0,2)), right=right.copy() if rf else np.zeros((0,2)),
            stale_left=left.copy() if not lf else np.zeros((0,2)), stale_right=right.copy() if not rf else np.zeros((0,2)),
            left_age=None if lt is None else t-lt, right_age=None if rt is None else t-rt,
            command=command, callback_results=callback_results, fan=fan, observed_end=observed_end,
            exiting=mission.exiting, rear_remaining=rear_remaining))
        previous_scan = t
    if not frames:
        raise ValueError('라이다 스캔이 없습니다.')
    digest = hashlib.sha256()
    for module in (cone_trace, cone_sorter, cone_follower, obstacle_extractor, cone_exit, cone_exit_pursuit, cone_odometry, cone_lane_connection):
        digest.update(Path(module.__file__).read_bytes())
    digest.update(Path(__file__).read_bytes())
    digest.update(config_path.read_bytes())
    return dict(frames=frames, cameras=cameras, recorded=recorded, flags=flags, events=events,
                params=fp, path=str(path), source_hash=digest.hexdigest()[:12], commands=command_rows)
