#!/usr/bin/env python3
"""Read-only full-recording replay. No ROS commands and no recorded-time triggers.

Inputs: --inventory selected.json (list of {name,dbs,start_ns}) and --output dir.
Runs landmark candidates everywhere, even without P: a deliberate negative stress
check. Reports registration/support, not closed-loop parking success.
"""
import argparse
import json
import math
from pathlib import Path
import sqlite3
import sys
import time
import cv2
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, LaserScan
from nav_msgs.msg import Odometry

ROOT = Path(__file__).resolve().parents[2]
for package in ('clothoid_lane', 'clothoid_common', 'clothoid_parking'):
    sys.path.insert(0, str(ROOT/'ros2_ws/src'/package))
from clothoid_lane.lane_main import detector_defaults, make_detector
from clothoid_lane.lane_detector import make_warp_matrix
from clothoid_lane.parking_landmark import ParkingLandmark
from clothoid_parking.core import ParkingCore
from clothoid_parking.layout import LayoutParams


def intervals(times):
    groups = []
    for t in times:
        if not groups or t-groups[-1][-1] > .7:
            groups.append([t])
        else:
            groups[-1].append(t)
    return [[round(g[0], 3), round(g[-1], 3)] for g in groups]


def run(row, output):
    config = detector_defaults()
    for name, scope in [('lane.yaml', 'lane_main'), ('camera_limo2.yaml', '/**')]:
        config.update(yaml.safe_load((ROOT/'ros2_ws/src/clothoid_lane/config'/name).read_text())[scope]['ros__parameters'])
    lane = make_detector(config.__getitem__)
    inv = np.linalg.inv(make_warp_matrix(lane.p))
    visual = ParkingLandmark()
    core = ParkingCore(layout=LayoutParams())
    core.prepare(True)
    last = -math.inf
    frames = 0
    results = []
    measurements = []
    costs = {'vision': [], 'scan_and_selection': []}
    t0 = row['start_ns']*1e-9
    kinds = {'/imu': Imu, '/odom': Odometry, '/scan': LaserScan,
             '/camera/color/camera_info': CameraInfo, '/camera/color/image_raw/compressed': CompressedImage}
    for db in row['dbs']:
        con = sqlite3.connect('file:'+str(db)+'?mode=ro', uri=True)
        topics = {i:(n,kinds[n]) for i,n in con.execute('select id,name from topics') if n in kinds}
        query = 'select topic_id,timestamp,data from messages where topic_id in ('+','.join('?' for _ in topics)+') order by timestamp'
        for tid, ts, blob in con.execute(query, list(topics)):
            name, kind = topics[tid]
            m = deserialize_message(blob, kind)
            stamp = m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            stamp = stamp or ts*1e-9
            if name == '/imu':
                core.imu(stamp, m.angular_velocity.z)
            elif name == '/odom':
                core.odom(stamp, m.twist.twist.linear.x)
            elif name == '/camera/color/camera_info':
                visual.configure_camera(m.width, m.height, m.k, m.d)
            elif name == '/scan':
                previous = core.layout.last_scan
                start = time.perf_counter()
                core.scan(stamp, m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max)
                if core.layout.last_scan == previous:
                    continue
                costs['scan_and_selection'].append((time.perf_counter()-start)*1000)
                if core.layout.origin is not None:
                    results.append({'t': stamp-t0, 'supported': core.layout.support_count,
                                    'states': list(core.layout.states), 'selected':core.layout.selected,
                                    'origin': core.layout.origin.tolist(), 'yaw':core.layout.yaw,
                                    'reason': core.layout.reason})
            else:
                im = cv2.imdecode(np.frombuffer(m.data, np.uint8), 1)
                result = lane.detect(im, 0, 170, False)
                frames += 1
                if stamp-last < .2 or visual.lookup is None:
                    continue
                last = stamp
                start = time.perf_counter()
                gx, gy, _ = visual.lookup
                sides = []
                for side in (result.left_points, result.right_points):
                    if not len(side): continue
                    pixels = cv2.perspectiveTransform(np.asarray(side,np.float32).reshape(-1,1,2),inv).reshape(-1,2)
                    pixels = np.rint(pixels).astype(int)
                    valid=(pixels[:,0]>=0)&(pixels[:,0]<im.shape[1])&(pixels[:,1]>=0)&(pixels[:,1]<im.shape[0])
                    x,y = pixels[valid].T
                    sides.append(np.column_stack((gx[y,x],gy[y,x])))
                obs = visual.detect(im, sides)
                costs['vision'].append((time.perf_counter()-start)*1000)
                if obs:
                    accepted = core.observe_entry(stamp, obs['origin'], obs['angle'])
                    measurements.append({'t':stamp-t0, 'accepted':bool(accepted), 'observation':obs})
        con.close()
    summary = {'name':row['name'],'frames':frames,'candidates':len(measurements),
               'registered_frames':sum(r['accepted'] for r in measurements),
               'lidar_supported_scans':sum(r['supported']>=3 for r in results),
               'selected_scans':sum(r['selected'] is not None for r in results),
               'support_intervals': intervals([r['t'] for r in results if r['supported']>=3]),
               'selection_intervals': intervals([r['t'] for r in results if r['selected'] is not None]),
               'ms':{k:{'mean':float(np.mean(v)), 'p95':float(np.percentile(v,95))} for k,v in costs.items() if v},
               'scope':'full recording, preparation forced on to test negatives; fixed recorded motion, no driving success claim'}
    (output/(row['name']+'.json')).write_text(json.dumps({'summary':summary,'measurements':measurements,'scans':results},indent=2))
    print(json.dumps(summary),flush=True)
    return summary


if __name__ == '__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--inventory',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True);cv2.setNumThreads(1)
    result=[run(row,args.output) for row in json.loads(args.inventory.read_text())]
    (args.output/'summary.json').write_text(json.dumps(result,indent=2))
