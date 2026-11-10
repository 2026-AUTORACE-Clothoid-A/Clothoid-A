#!/usr/bin/env python3
"""Run one Clothoid node class unchanged, wrapping its on_*/publish_* methods with timers.

    python3 harness.py <module>:<Class> <label> <seconds> [--ros-args ...]

Prints per-callback count / mean / p50 / p95 / max (ms), callback start gaps for timers, and the
process CPU share (includes rclpy deserialization, which the callback timers do not see).
"""
import importlib
import json
import os
import sys
import time
from collections import defaultdict

import rclpy
from rclpy.executors import SingleThreadedExecutor

spec, label, seconds = sys.argv[1], sys.argv[2], float(sys.argv[3])
mod_name, cls_name = spec.split(':')
cls = getattr(importlib.import_module(mod_name), cls_name)

durations = defaultdict(list)
starts = defaultdict(list)


def wrap(name, fn):
    def inner(self, *a, **k):
        t0 = time.perf_counter()
        try:
            return fn(self, *a, **k)
        finally:
            t1 = time.perf_counter()
            durations[name].append((t1 - t0) * 1e3)
            starts[name].append(t0)
    return inner


for name in list(vars(cls)):
    if (name.startswith('on_') or name.startswith('publish_')) and name != 'on_parameters' \
            and callable(getattr(cls, name)):
        setattr(cls, name, wrap(name, getattr(cls, name)))

# the nodes' main() sets one OpenCV thread (opencv_threads, lane_shared's shared_opencv_threads); the
# harness does not run main(), so do it here. OPENCV_THREADS=0 keeps OpenCV's default pool.
_threads = int(os.environ.get('OPENCV_THREADS', '1'))
if _threads:
    import cv2
    cv2.setNumThreads(_threads)
rclpy.init(args=sys.argv[4:])
node = cls()
ex = SingleThreadedExecutor()
ex.add_node(node)
cpu0, wall0 = time.process_time(), time.perf_counter()
first = None
while time.perf_counter() - wall0 < seconds:
    ex.spin_once(timeout_sec=0.1)
cpu, wall = time.process_time() - cpu0, time.perf_counter() - wall0


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(q * len(v)))] if v else 0.0


out = {'label': label, 'cpu_pct': 100.0 * cpu / wall, 'wall': wall, 'callbacks': {}}
for name, v in sorted(durations.items()):
    s = starts[name]
    gaps = [(b - a) * 1e3 for a, b in zip(s, s[1:])]
    active = s[-1] - s[0] if len(s) > 1 else 0.0
    out['callbacks'][name] = {
        'n': len(v), 'hz': (len(v) - 1) / active if active > 0 else 0.0,
        'mean': sum(v) / len(v), 'p50': pct(v, .5), 'p95': pct(v, .95), 'max': max(v),
        'share_pct': 100.0 * sum(v) / 1e3 / wall,
        'gap_p95': pct(gaps, .95), 'gap_max': max(gaps) if gaps else 0.0}
print('RESULT ' + json.dumps(out), flush=True)
node.destroy_node()
rclpy.shutdown()
