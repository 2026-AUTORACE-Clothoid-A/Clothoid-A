"""Run the current perception/decision modules on a bag, frame by frame, with the repo yaml values.

    python3 run_pipeline.py <bag> <out.npz> [--snap t1,t2,...] [--snapdir DIR]

Camera: LaneDetector (side from lane.yaml initial_side), StopLineDetector, SpeedZoneDetector,
        StartLineDetector (parking.yaml). Lidar: extract_obstacles (lidar.yaml) -> ConeMission ->
        ConeFollower, plus a "vehicle-size" clustering (wider max_width) for the static obstacles.
Also IMU gyro z, /odom linear.x and angular.z, and message gaps.
--snap: bag times (s from start) where debug images of every detector are written.
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np
import yaml

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'ros2_ws', 'src')
for pkg in ('clothoid_lane', 'clothoid_lidar', 'clothoid_parking'):
    sys.path.insert(0, os.path.join(SRC, pkg))

from clothoid_lane.lane_detector import LaneDetector, LaneParams  # noqa: E402
from clothoid_lane.speed_zone import SpeedZoneDetector, SpeedZoneParams  # noqa: E402
from clothoid_lane.stopline import StopLineDetector, StopLineParams  # noqa: E402
from clothoid_lidar.cone_follower import ConeFollower, FollowerParams  # noqa: E402
from clothoid_lidar.cone_sorter import ConeMission, ConeSorter, SorterParams  # noqa: E402
from clothoid_lidar.obstacle_extractor import (ExtractorParams, extract_obstacles,  # noqa: E402
                                               transform_points)
from clothoid_parking.start_line import StartLineDetector, StartLineParams  # noqa: E402

from bagio import image_bgr, read  # noqa: E402


def params(pkg, file, node):
    with open(os.path.join(SRC, pkg, 'config', file)) as f:
        return yaml.safe_load(f)[node]['ros__parameters']


def build():
    ly = params('clothoid_lane', 'lane.yaml', 'lane_main')
    lp = LaneParams(x_h=ly['warp_x_h'], x_l=ly['warp_x_l'], y_h=ly['warp_y_h'], y_l=ly['warp_y_l'],
                    hsv_lower=tuple(ly['hsv_lower']), hsv_upper=tuple(ly['hsv_upper']),
                    canny_low=ly['canny_low'], canny_high=ly['canny_high'])
    sy = params('clothoid_lane', 'stopline.yaml', 'stopline')
    sp = StopLineParams(
        warp=LaneParams(x_h=sy['warp_x_h'], x_l=sy['warp_x_l'], y_h=sy['warp_y_h'], y_l=sy['warp_y_l']),
        crop_x=sy['crop_x'], crop_y=sy['crop_y'], hsv_lower=tuple(sy['hsv_lower']),
        hsv_upper=tuple(sy['hsv_upper']), gray_low=sy['gray_low'], gray_high=sy['gray_high'],
        min_tall_w=sy['min_tall_w'], min_wide_w=sy['min_wide_w'], min_count=sy['min_count'],
        min_white_ratio=sy['min_white_ratio'])
    zy = params('clothoid_lane', 'speed_zone.yaml', 'speed_zone')
    zp = SpeedZoneParams(**{k: (tuple(v) if isinstance(v, list) else v) for k, v in zy.items()
                            if k in SpeedZoneParams.__dataclass_fields__})
    py = params('clothoid_parking', 'parking.yaml', 'parking')
    pp = StartLineParams(**{k: (tuple(v) if isinstance(v, list) else v) for k, v in py.items()
                            if k in StartLineParams.__dataclass_fields__})
    ey = params('clothoid_lidar', 'lidar.yaml', 'obstacle_extractor')
    ep = ExtractorParams(**{k: v for k, v in ey.items() if k in ExtractorParams.__dataclass_fields__})
    vp = ExtractorParams(**{**ep.__dict__, 'max_width': 0.45, 'min_points': 3, 'cone_radius': 0.0})
    vp_narrow = ExtractorParams(**{**vp.__dict__, 'max_width': ep.max_width})
    oy = params('clothoid_lidar', 'lidar.yaml', 'cone_sorter')
    so = SorterParams(**{k: v for k, v in oy.items() if k in SorterParams.__dataclass_fields__})
    fy = params('clothoid_lidar', 'lidar.yaml', 'cone_follower')
    fo = FollowerParams(**{k: v for k, v in fy.items() if k in FollowerParams.__dataclass_fields__})
    return dict(lane=LaneDetector(lp), side=ly['initial_side'], lane_width=ly['lane_width'],
                max_steer=ly.get('max_steer_rad', 0.48869), stop=StopLineDetector(sp),
                zone=SpeedZoneDetector(zp), start=StartLineDetector(pp), start_row=pp.trigger_row,
                ep=ep, vp=vp, vpn=vp_narrow, mission=ConeMission(ConeSorter(so)), follower=ConeFollower(fo))


def lane_steer(res, det, max_steer):
    x = res.target_x - det.p.warp_w / 2
    y = det.p.warp_h - res.target_y
    s = -math.atan2(x, max(y, 1e-6)) / (math.pi / 2)
    return max(-max_steer, min(max_steer, s))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--snap', default='')
    ap.add_argument('--snapdir', default='')
    a = ap.parse_args()
    snaps = sorted(float(s) for s in a.snap.split(',') if s)
    m = build()
    LIDAR_TF = (0.103, 0.0, 0.0)

    cam, lid, imu, odo, gaps = [], [], [], [], {}
    last = {}
    t0 = None
    for topic, t, msg in read(a.bag, ['/camera/color/image_raw', '/scan', '/imu', '/odom']):
        t0 = t if t0 is None else t0
        rt = t - t0
        if topic in last:
            gaps.setdefault(topic, []).append((rt, rt - last[topic]))
        last[topic] = rt
        if topic == '/imu':
            imu.append((rt, msg.angular_velocity.z))
        elif topic == '/odom':
            odo.append((rt, msg.twist.twist.linear.x, msg.twist.twist.angular.z))
        elif topic == '/camera/color/image_raw':
            bgr = image_bgr(msg)
            snap = bool(snaps) and rt >= snaps[0]
            lr = m['lane'].detect(bgr, m['side'], m['lane_width'], snap)
            st = m['stop'].detect(bgr, snap)
            zn = m['zone'].detect(bgr, snap)
            sl = m['start'].detect(bgr, snap)
            steer = lane_steer(lr, m['lane'], m['max_steer']) if lr.found else np.nan
            mean_bgr = bgr.reshape(-1, 3).mean(axis=0)
            cam.append((rt, lr.found, steer, lr.target_x, st.found, st.count, st.white_ratio,
                        zn.found, zn.area, sl.center_y if sl.center_y is not None else -1,
                        *mean_bgr))
            if snap:
                tag = f'{snaps.pop(0):06.1f}'
                d = a.snapdir
                cv2.imwrite(f'{d}/{tag}_cam.jpg', bgr)
                cv2.imwrite(f'{d}/{tag}_lane.png', lr.debug_image)
                cv2.imwrite(f'{d}/{tag}_stop.png', st.debug_image)
                cv2.imwrite(f'{d}/{tag}_zone.jpg', zn.debug_image)
                cv2.imwrite(f'{d}/{tag}_start.png', sl.debug_image)
        elif topic == '/scan':
            pts = extract_obstacles(msg.ranges, msg.angle_min, msg.angle_increment, m['ep'])
            cones = transform_points(pts, *LIDAR_TF)
            veh = transform_points(
                extract_obstacles(msg.ranges, msg.angle_min, msg.angle_increment, m['vp']), *LIDAR_TF)
            was = m['mission'].running
            res = m['mission'].update(cones)
            running = m['mission'].running
            speed, steer, rev = np.nan, np.nan, False
            if res is not None:
                fr = m['follower'].decide(res.left, res.right, len(res.left) > 0, len(res.right) > 0)
                if fr is not None:
                    speed, steer, rev = fr.speed, fr.steering, fr.reverse
            ahead = cones[(cones[:, 0] > 0) & (cones[:, 0] < 1.5) & (np.abs(cones[:, 1]) < 0.8)] \
                if len(cones) else cones
            vahead = veh[(veh[:, 0] > 0) & (veh[:, 0] < 1.5) & (np.abs(veh[:, 1]) < 0.8)] \
                if len(veh) else veh
            vn = transform_points(
                extract_obstacles(msg.ranges, msg.angle_min, msg.angle_increment, m['vpn']), *LIDAR_TF)
            vnahead = vn[(vn[:, 0] > 0) & (vn[:, 0] < 1.5) & (np.abs(vn[:, 1]) < 0.8)] if len(vn) else vn
            wide = len(vahead) - len(vnahead)   # clusters 0.15..0.45 m wide ahead (vehicle-size)
            lid.append((rt, len(cones), len(ahead), wide, running, running != was,
                        0 if res is None else len(res.left), 0 if res is None else len(res.right),
                        speed, steer, rev, len(msg.ranges)))
    np.savez(a.out, cam=np.array(cam, float), lid=np.array(lid, float), imu=np.array(imu, float),
             odo=np.array(odo, float),
             **{'gap' + k.replace('/', '_'): np.array(v, float) for k, v in gaps.items()})
    print(a.out, len(cam), len(lid))


if __name__ == '__main__':
    main()
