#!/usr/bin/env python3
"""Replay recorded inputs into mission callbacks, with every publisher captured in memory.

No executor or bag playback runs. This compares decision timing on the recorded
trajectory; changed commands do not change the recorded vehicle motion.
"""
import argparse
import json
import os
from pathlib import Path
import sqlite3
import sys
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[2]


def replay(bag, source_root=ROOT):
    for package in ('clothoid_mission', 'clothoid_finish', 'clothoid_parking',
                    'clothoid_lane', 'clothoid_common'):
        sys.path.insert(0, str(source_root/'ros2_ws/src'/package))
    os.environ['ROS_DOMAIN_ID'] = '229'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import cv2
    import rclpy
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from clothoid_mission.mission_node import MissionNode

    cv2.setNumThreads(1)
    dbs = sorted(bag.glob('*.db3')) if bag.is_dir() else [bag]
    if len(dbs) != 1:
        raise ValueError('select one SQLite bag file or an unsplit bag directory')
    db = sqlite3.connect(f'file:{dbs[0].resolve()}?mode=ro', uri=True)
    topics = {name: (i, kind) for i, name, kind in db.execute('select id,name,type from topics')}
    start, end = db.execute('select min(timestamp),max(timestamp) from messages').fetchone()
    image = next((n for n in ('/camera/color/image_10hz', '/camera/color/image_raw',
                              '/camera/color/image_raw/compressed')
                  if n in topics and db.execute('select 1 from messages where topic_id=? limit 1',
                                                (topics[n][0],)).fetchone()), None)
    if image is None:
        raise ValueError('no source camera frames')
    compressed = topics[image][1] == 'sensor_msgs/msg/CompressedImage'
    config = source_root/'ros2_ws/src'
    args = ['--ros-args']
    for path in (config/'clothoid_mission/config/mission.yaml',
                 config/'clothoid_lane/config/camera_limo2.yaml'):
        args += ['--params-file', str(path)]
    args += ['-p', f'compressed:={str(compressed).lower()}']
    for parameter, fallback in (('imu_topic', '/imu'), ('odom_topic', '/odom')):
        preferred = '/imu_25hz' if parameter == 'imu_topic' else '/odom_25hz'
        args += ['-p', f'{parameter}:={preferred if preferred in topics else fallback}']
    rclpy.init(args=args)
    node = MissionNode()
    captured = {}
    for key, publisher in list(vars(node).items()):
        if key.endswith('_pub') and hasattr(publisher, 'publish'):
            setattr(node, key, SimpleNamespace(publish=lambda msg, key=key: captured.update({key: msg})))
    clock = [start]
    node.now_sec = lambda: clock[0]*1e-9
    node._image_on = lambda on: None
    node.get_logger = lambda: SimpleNamespace(info=lambda *a, **k: None)
    callbacks = {
        image: node.on_image, '/camera/color/camera_info': node.on_info,
        node.get_parameter('imu_topic').value: node.on_imu,
        node.get_parameter('odom_topic').value: node.on_odom,
        '/detections': node.on_detections, '/parking_done': node.on_parking_done,
        '/parking_sign': node.on_parking_sign, '/stopline_near': node.on_parking_line,
        '/cone/mission_active': node.on_cone_active, '/avoid/active': node.on_avoid_active,
        '/parking/bay_candidate': node.on_parking_scene,
        '/parking/observing': node.on_parking_observing, '/parking/skipped': node.on_parking_skipped,
    }
    wanted = {topics[n][0]: (n, get_message(topics[n][1]))
              for n in set(callbacks) | {'/mission/state'} if n in topics}
    trace, recorded, events = [], [], []
    last = None

    def tick(stamp):
        nonlocal last
        clock[0] = stamp
        node.on_timer()
        state = json.loads(captured['mission_state_pub'].data)
        t = (stamp-start)*1e-9
        trace.append(dict(t=t, **state))
        key = (state['state'], tuple(state['armed']), state['line_phase'], state['bump_state'],
               state.get('entry_slow'), state.get('entry_reached'))
        if key != last:
            events.append(dict(t=round(t, 3), state=state['state'], armed=state['armed'],
                line_phase=state['line_phase'], bump=state['bump_state'],
                entry=state.get('entry_remaining'), entry_slow=state.get('entry_slow'),
                entry_reached=state.get('entry_reached'), speed=state['speed_limit']))
            last = key

    next_tick, frames = start, 0
    query = ('select timestamp,topic_id,data from messages where topic_id in ('+
             ','.join('?' for _ in wanted)+') order by timestamp,id')
    try:
        for stamp, topic, blob in db.execute(query, tuple(wanted)):
            while next_tick < stamp:
                tick(next_tick)
                next_tick += 50_000_000
            clock[0] = stamp
            name, cls = wanted[topic]
            message = deserialize_message(blob, cls)
            if name == '/mission/state':
                recorded.append(dict(t=(stamp-start)*1e-9, **json.loads(message.data)))
            else:
                callbacks[name](message)
            frames += name == image
        while next_tick <= end:
            tick(next_tick)
            next_tick += 50_000_000
    finally:
        node.destroy_node()
        rclpy.shutdown()
        db.close()
    return dict(bag=str(bag), source=str(source_root), frames=frames,
                events=events, trace=trace, recorded=recorded)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--source-root', type=Path, default=ROOT)
    args = parser.parse_args()
    result = replay(args.bag, args.source_root)
    args.output.write_text(json.dumps(result, ensure_ascii=False))
    print(json.dumps({k: result[k] for k in ('bag', 'source', 'frames', 'events')},
                     ensure_ascii=False, indent=2))
