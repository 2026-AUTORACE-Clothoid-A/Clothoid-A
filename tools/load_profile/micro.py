#!/usr/bin/env python3
"""Cost of single image operations on real bag frames, with OpenCV's default threads and with one.

    source tools/load_profile/env.sh   # PYTHONPATH (LIDAR_SRC for section_cues)
    python3 tools/load_profile/micro.py <bag dir or .db3> [--camera limo2]

Times are on this computer; the Orin NX is about x2~3 slower per core (docs/PIPELINE_LOAD_PLAN.md).
"""
import argparse
import array
import os
import sqlite3
import sys
import time

import cv2
import numpy as np
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from gaps import db_path  # noqa: E402

from clothoid_common.image_utils import bgr_to_image, image_to_bgr  # noqa: E402
from clothoid_lane.lane_main import detector_defaults, make_detector  # noqa: E402

SRC = os.path.join(HERE, '..', '..', 'ros2_ws', 'src')


def bench(name, fn, n, frames):
    t = time.perf_counter()
    for i in range(n):
        fn(i % frames)
    print(f'  {name:46s} {(time.perf_counter() - t) / n * 1e3:7.2f} ms')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--camera', default='limo2')
    ap.add_argument('--offset', type=int, default=300, help='first frame to use')
    a = ap.parse_args()

    db = sqlite3.connect(db_path(a.bag))
    tid = db.execute("select id from topics where name='/camera/color/image_raw'").fetchone()[0]
    raws = [r[0] for r in db.execute('select data from messages where topic_id=? limit 60 offset ?',
                                     (tid, a.offset))]
    msgs = [deserialize_message(r, Image) for r in raws]
    frames = [image_to_bgr(m).copy() for m in msgs]
    n = len(frames)
    print(f'{msgs[0].encoding} {msgs[0].width}x{msgs[0].height}, {len(raws[0])} bytes, {n} frames')

    cfg = detector_defaults()
    for f in ('lane.yaml', f'camera_{a.camera}.yaml'):
        with open(os.path.join(SRC, 'clothoid_lane', 'config', f)) as fh:
            profile = yaml.safe_load(fh)
            for name in ('/**', 'lane_main'):
                values = profile.get(name, {}).get('ros__parameters', {})
                cfg.update({k: v for k, v in values.items() if k in cfg})
    det = make_detector(lambda k: cfg[k])
    small = [cv2.resize(f, (320, 240)) for f in frames]
    m = cv2.getPerspectiveTransform(np.float32([[60, 91], [580, 91], [2, 240], [638, 240]]),
                                    np.float32([[0, 0], [320, 0], [0, 240], [320, 240]]))
    try:
        from clothoid_lidar.section_cues import (CameraModel, ColourParams, WhiteParams,
                                                 find_entry_line, find_lane_lines)
        cues = (CameraModel(), WhiteParams(), ColourParams())
    except ImportError:
        cues = None
        print('  (no clothoid_lidar.section_cues: set LIDAR_SRC to the static-avoid branch)')

    for threads in (0, 1):
        if threads:
            cv2.setNumThreads(1)
        print(f'--- cv2 threads {cv2.getNumThreads()}')
        bench('deserialize Image (rclpy)', lambda i: deserialize_message(raws[i], Image), 60, n)
        bench('GaussianBlur 5x5 640x480', lambda i: cv2.GaussianBlur(frames[i], (5, 5), 0), 60, n)
        bench('warpPerspective 640x480 -> 320x240',
              lambda i: cv2.warpPerspective(frames[i], m, (320, 240)), 60, n)
        bench('cvtColor HSV 640x480', lambda i: cv2.cvtColor(frames[i], cv2.COLOR_BGR2HSV), 60, n)
        bench('cvtColor HSV 320x240', lambda i: cv2.cvtColor(small[i], cv2.COLOR_BGR2HSV), 60, n)
        bench('lane_tracker.detect (debug off)', lambda i: det.detect(frames[i], 0, 170.0, False), 60, n)
        bench('lane_tracker.detect (debug on)', lambda i: det.detect(frames[i], 0, 170.0, True), 60, n)
        dbg = det.detect(frames[0], 0, 170.0, True).debug_image
        bench('bgr_to_image 320x240', lambda i: bgr_to_image(dbg, msgs[0].header), 60, n)
        bench('bgr_to_image 640x480', lambda i: bgr_to_image(frames[i], msgs[0].header), 60, n)
        if cues:
            cam, wp, cp = cues
            bench('section_cues.find_entry_line', lambda i: find_entry_line(frames[i], cam, wp), 60, n)
            bench('section_cues.find_lane_lines',
                  lambda i: find_lane_lines(frames[i], cam, cp, 0.0, np.radians(20)), 60, n)
        cost = np.random.randint(0, 100, (450, 278)).astype(np.int8)   # static_avoid's costmap

        def grid_list(_):
            g = OccupancyGrid()
            g.data = cost.T.ravel().tolist()
            serialize_message(g)

        def grid_array(_):
            g = OccupancyGrid()
            g.data = array.array('b', cost.T.ravel().tobytes())
            serialize_message(g)
        bench('OccupancyGrid 450x278 .tolist() + serialize', grid_list, 10, n)
        bench('OccupancyGrid 450x278 array.array + serialize', grid_array, 10, n)


if __name__ == '__main__':
    main()
