#!/usr/bin/env python3
"""Topic rates and gaps from the recording times of a car bag (sqlite3), and whether gaps line up.

    python3 tools/load_profile/gaps.py <bag dir or .db3>                    # rates, gaps, /rosout
    python3 tools/load_profile/gaps.py <bag> --split 6.4                    # before / after 6.4 s
    python3 tools/load_profile/gaps.py <bag> --watch /avoid/lane --gap 0.25 # what others did in its gaps
    python3 tools/load_profile/gaps.py <bag> --duration                     # bag length in s (run.sh)

A gap only in one node's outputs while the camera, imu and scan keep coming = that node is stuck
(its own executor), not the whole computer. Times are seconds from the start of the bag.
"""
import argparse
import glob
import os
import sqlite3

import numpy as np

TOPICS = ['/camera/color/image_raw', '/scan', '/imu', '/odom', '/cmd_vel_lane', '/cmd_vel_cone', '/cmd_vel_avoid',
          '/avoid/active', '/avoid/lane']
CONTEXT = ['/camera/color/image_raw', '/imu', '/scan', '/cmd_vel_lane', '/avoid/active']


def db_path(path):
    if os.path.isdir(path):
        found = sorted(glob.glob(os.path.join(path, '*.db3')))
        if not found:
            raise SystemExit(f'no .db3 in {path}')
        return found[0]
    return path


def stats(name, t):
    g = np.diff(t)
    return (f'{name:26s} n={len(t):5d} {len(t) / (t[-1] - t[0]):6.1f} Hz  gap p50 {np.median(g) * 1e3:6.1f}'
            f' p95 {np.percentile(g, 95) * 1e3:6.1f} max {g.max() * 1e3:6.0f} ms')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--topics', nargs='*', default=TOPICS)
    ap.add_argument('--split', type=float, nargs='*', default=[],
                    help='seconds where to cut the bag into windows, e.g. the avoid START / END')
    ap.add_argument('--watch', default='/avoid/lane')
    ap.add_argument('--gap', type=float, default=0.25, help='s, gaps of --watch to list')
    ap.add_argument('--timeout', type=float, default=0.3, help='s, twist_mux timeout to count on cmd topics')
    ap.add_argument('--duration', action='store_true')
    ap.add_argument('--no-rosout', action='store_true')
    a = ap.parse_args()

    db = sqlite3.connect(db_path(a.bag))
    t0, t1 = db.execute('select min(timestamp), max(timestamp) from messages').fetchone()
    if a.duration:
        print(f'{(t1 - t0) * 1e-9:.1f}')
        return
    topics = dict(db.execute('select id, name from topics'))
    by = {}
    for tid, ts in db.execute('select topic_id, timestamp from messages order by timestamp'):
        by.setdefault(topics[tid], []).append((ts - t0) * 1e-9)
    by = {k: np.array(v) for k, v in by.items()}

    edges = [0.0] + sorted(a.split) + [(t1 - t0) * 1e-9 + 1e-3]
    for lo, hi in zip(edges, edges[1:]):
        print(f'--- {lo:.1f} ~ {hi:.1f} s')
        for name in a.topics:
            t = by.get(name, np.zeros(0))
            t = t[(t >= lo) & (t < hi)]
            if len(t) < 3:
                continue
            line = stats(name, t)
            if name.startswith('/cmd_vel'):
                line += f'  gaps > {a.timeout} s: {int((np.diff(t) > a.timeout).sum())}'
            print(line)

    if a.watch in by:
        print(f'\n{a.watch} gaps > {a.gap * 1e3:.0f} ms and the longest gap of other topics around them:')
        t = by[a.watch]
        for start, d in zip(t[:-1], np.diff(t)):
            if d <= a.gap:
                continue
            line = f'  t={start:6.2f} s gap {d * 1e3:4.0f} ms |'
            for name in CONTEXT:
                if name == a.watch or name not in by:
                    continue
                w = by[name][(by[name] >= start - 0.05) & (by[name] <= start + d + 0.05)]
                mg = np.diff(w).max() * 1e3 if len(w) > 1 else float('nan')
                line += f' {name.rsplit("/", 1)[-1]} {mg:4.0f}'
            print(line)

    rosout = [k for k, v in topics.items() if v == '/rosout']
    if rosout and not a.no_rosout:
        from rcl_interfaces.msg import Log
        from rclpy.serialization import deserialize_message
        print()
        for ts, data in db.execute('select timestamp, data from messages where topic_id=? '
                                   'order by timestamp', (rosout[0],)):
            m = deserialize_message(data, Log)
            print(f'  rosout {(ts - t0) * 1e-9:6.2f} [{m.name}] {m.msg[:110]}')


if __name__ == '__main__':
    main()
