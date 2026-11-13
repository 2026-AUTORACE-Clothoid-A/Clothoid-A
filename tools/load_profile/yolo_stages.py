#!/usr/bin/env python3
"""Where the yolo node's CPU goes, on the car, without ROS: bag frames -> the same calls as the node.

    source scripts/env_jetson.sh dl_env && source tools/load_profile/env.sh
    python3 tools/load_profile/yolo_stages.py <bag> --model ros2_ws/src/clothoid_yolo/models/best.pt
    python3 tools/load_profile/yolo_stages.py <bag> --model .../best_fp16.engine --rate 10

Per frame it times image_to_bgr (the node's conversion), Ultralytics' own preprocess / inference /
postprocess (`result.speed`), and the process CPU time against the wall time:
- CPU ms ~ wall ms during inference: the CPU spins while it waits for the GPU (CUDA sync)
- CPU ms << wall ms: the CPU is idle while the GPU works; then a busy node only means the input rate
  (30 Hz) is above what it can do, and a lower input rate (10 Hz topic) cuts the CPU in proportion
`--rate 0` runs frames back to back (= the saturated node), `--rate 10` paces them like a 10 Hz topic.
"""
import argparse
import os
import sqlite3
import sys
import time

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from gaps import db_path  # noqa: E402

from clothoid_common.image_utils import image_to_bgr  # noqa: E402


def stats(name, v):
    v = np.asarray(v)
    print(f'  {name:28s} mean {v.mean():7.2f}  p95 {np.percentile(v, 95):7.2f}  max {v.max():7.2f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--model', required=True, help='.pt / .onnx / .engine')
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--device', default=None, help="'' auto, 'cuda:0', 'cpu'")
    ap.add_argument('--half', action='store_true', help='fp16 (for .pt; an fp16 .engine already is)')
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--frames', type=int, default=300)
    ap.add_argument('--offset', type=int, default=0, help='first frame in the bag')
    ap.add_argument('--rate', type=float, default=0.0, help='input Hz; 0 = back to back')
    a = ap.parse_args()

    from ultralytics import YOLO
    db = sqlite3.connect(db_path(a.bag))
    tid = db.execute("select id from topics where name='/camera/color/image_raw'").fetchone()[0]
    raws = [r[0] for r in db.execute('select data from messages where topic_id=? limit ? offset ?',
                                     (tid, a.frames, a.offset))]
    print(f'{len(raws)} frames from {a.bag}; model {a.model} imgsz {a.imgsz} half {a.half} '
          f'rate {a.rate or "max"} Hz')
    model = YOLO(a.model, task='detect')
    first = image_to_bgr(deserialize_message(raws[0], Image))
    for _ in range(10):   # warm-up: CUDA context, TensorRT, memory pools
        model.predict(first, imgsz=a.imgsz, conf=a.conf, device=a.device, half=a.half, verbose=False)

    rows = {k: [] for k in ('deserialize', 'image_to_bgr', 'preprocess', 'inference', 'postprocess',
                            'predict wall', 'predict cpu', 'frame wall', 'frame cpu')}
    period = 1.0 / a.rate if a.rate else 0.0
    cpu0, wall0 = time.process_time(), time.perf_counter()
    next_t = wall0
    for raw in raws:
        if period:
            next_t += period
        fw, fc = time.perf_counter(), time.process_time()
        msg = deserialize_message(raw, Image)
        t1 = time.perf_counter()
        bgr = image_to_bgr(msg)
        t2, c2 = time.perf_counter(), time.process_time()
        r = model.predict(bgr, imgsz=a.imgsz, conf=a.conf, device=a.device, half=a.half, verbose=False)[0]
        t3, c3 = time.perf_counter(), time.process_time()
        rows['deserialize'].append((t1 - fw) * 1e3)
        rows['image_to_bgr'].append((t2 - t1) * 1e3)
        for k in ('preprocess', 'inference', 'postprocess'):
            rows[k].append(r.speed.get(k, 0.0))
        rows['predict wall'].append((t3 - t2) * 1e3)
        rows['predict cpu'].append((c3 - c2) * 1e3)
        rows['frame wall'].append((t3 - fw) * 1e3)
        rows['frame cpu'].append((c3 - fc) * 1e3)
        if period:
            time.sleep(max(0.0, next_t - time.perf_counter()))
    cpu, wall = time.process_time() - cpu0, time.perf_counter() - wall0

    print('ms per frame:')
    for k, v in rows.items():
        stats(k, v)
    pw, pc = np.mean(rows['predict wall']), np.mean(rows['predict cpu'])
    print(f'process CPU {100 * cpu / wall:.0f} % of one core over {wall:.1f} s '
          f'({len(raws) / wall:.1f} frames/s); all threads count, so >100 % is possible')
    print(f'predict: CPU {pc:.1f} ms of {pw:.1f} ms wall ({100 * pc / pw:.0f} %) -> '
          + ('the CPU is busy for the whole predict (spin-wait on the GPU or CPU-bound pre/post)'
             if pc > 0.8 * pw else 'the CPU waits idle for part of the predict'))


if __name__ == '__main__':
    main()
