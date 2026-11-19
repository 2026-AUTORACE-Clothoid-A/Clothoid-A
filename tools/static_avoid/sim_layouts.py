#!/usr/bin/env python3
"""Closed-loop comparison of static_avoid settings in the simulation of test_static_avoid.py (car 7's
measured steering table, ray-cast lidar, the real StaticAvoid).

    python3 tools/static_avoid/sim_layouts.py                       # current defaults only
    python3 tools/static_avoid/sim_layouts.py --conf 'now={}' \\
        --conf 'seq={"sequence": true}' --conf 'seq r.65={"sequence": true, "lc_radius": 0.65}'

Layouts: the 3 tape-measured ones of the 11-08 bags x sign left / right x seed 1, 2 (12 runs) and 40
random ones (lane 2 LIMO rear 0.8~3.0 m from the entry line, one in lane 1 or 3 at 1.5~3.2 m, seed 7)
x left / right (80 runs). Failed = touched a LIMO (HIT) or came within 3 cm (gap<3), on the island,
stuck before the end, or not in the sign's lane at the end. Every run's lane sequence is recorded.
"""
import argparse
import json
import os
import sys
from dataclasses import replace
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..', '..', 'ros2_ws', 'src', 'clothoid_lidar')
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.join(PKG, 'test'))
from clothoid_lidar.static_avoid import AvoidParams, lane_of  # noqa: E402
import test_static_avoid as sim  # noqa: E402

W = 0.50
HALF = sim.LIMO_LEN / 2
FIXED = {'run1-2': [(0.0, 1.00), (-W, 1.82)],       # (lane y, rear of the LIMO from the entry line)
         'run3': [(0.0, 1.00), (W, 1.82)],
         'run6-9': [(0.0, 3.00), (-W, 2.50)]}


def random_layouts(seed):
    rng = np.random.default_rng(seed)
    return [[(0.0, round(float(rng.uniform(0.8, 3.0)), 2)),
             (float(rng.choice([W, -W])), round(float(rng.uniform(1.5, 3.2)), 2))] for _ in range(40)]


def dense_layouts(seed, count):
    """One LIMO in every lane (rear 0.8~3.4 m) and a fourth in lane 1 or 3 half the time, as the
    hard 11-12 / 11-13 car layouts: some need a quick swerve between two of them."""
    rng = np.random.default_rng(seed)
    out = []
    while len(out) < count:
        lay = [(y, round(float(rng.uniform(0.8, 3.4)), 2)) for y in (W, 0.0, -W)]
        if rng.random() < 0.5:
            y = float(rng.choice([W, -W]))
            r = round(float(rng.uniform(0.8, 3.4)), 2)
            if all(abs(r - r2) > 0.6 for y2, r2 in lay if y2 == y):
                lay.append((y, r))
        out.append(lay)
    return out


RANDOM = random_layouts(7)
DENSE = []
CONFS = {}


def sequence_of(track):
    out = []
    for _, _, d, _ in track:
        lane = lane_of(d, AvoidParams())
        if not out or out[-1] != lane:
            out.append(lane)
    return '-'.join(map(str, out))


def job(args):
    name, layout, direction, seed = args
    conf = dict(CONFS[name])
    sign_at = conf.pop('sign_at', None)          # the sign first seen when the car passes x (m)
    steer_gain = conf.pop('steer_gain', 1.0)     # real wheel angle x this (car 7's table x gain)
    control_every = conf.pop('control_every', 1)  # control every n-th 0.05 s step (busy node)
    line_start = conf.pop('line_start', False)    # race start: the entry line starts it (course map)
    lane_speed = conf.pop('lane_speed', 0.3)      # lane_main's speed before the start (car: 0.8)
    sign_view = conf.pop('sign_view', None)       # [half_deg, range]: the sign read only in that view
    misread = conf.pop('sign_misread', None)      # [half_deg, range, secs]: first the wrong way for secs
    p = replace(AvoidParams(), lane_width=W, **conf)
    track = []
    flag = direction if sign_at is None else (direction, sign_at)
    if sign_view is not None:
        flag = ('view', direction, float(sign_view[0]), float(sign_view[1]))
    if misread is not None:
        flag = ('misread', direction, float(misread[0]), float(misread[1]), float(misread[2]))
    o = sim.run([(rear + HALF, y) for y, rear in layout], direction=flag, lane_width=W,
                params=p, seed=seed, track=track, steer_gain=steer_gain,
                control_every=control_every, line_start=line_start,
                lane_speed=lane_speed)
    why = []
    if o['min_gap'] <= 0.0:
        why.append('HIT')
    elif o['min_gap'] <= 0.03:
        why.append('gap<3')
    if not o['island_ok']:
        why.append('island')
    tip = bool(o.get('island_tip'))
    if o['x'] <= 6.0:
        why.append('stuck')
    elif o['final_lane'] != {0: 1, 1: 3}[direction]:
        why.append('lane')
    return why, float(o['min_gap']), sequence_of(track), tip


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--conf', action='append', default=[],
                    help='name={AvoidParams overrides as JSON, plus "sign_at": x, "steer_gain": g, '
                         '"control_every": n}; repeatable')
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--fails', action='store_true', help='list every failed random layout')
    ap.add_argument('--seed', type=int, default=7, help='random layouts')
    ap.add_argument('--dense', type=int, default=0,
                    help='also this many dense layouts (a LIMO in every lane, x sign left / right)')
    a = ap.parse_args()
    global RANDOM, DENSE
    RANDOM = random_layouts(a.seed)
    DENSE = dense_layouts(a.seed, a.dense)
    for c in a.conf or ['now={}']:
        name, js = c.split('=', 1)
        CONFS[name] = {k: tuple(v) if isinstance(v, list) else v for k, v in json.loads(js).items()}
    jobs = []
    for name in CONFS:
        for key, layout in FIXED.items():
            for d in (0, 1):
                for seed in (1, 2):
                    jobs.append((name, 'F', key, d, seed, layout))
        for i, layout in enumerate(RANDOM):
            for d in (0, 1):
                jobs.append((name, 'R', i, d, i + 1, layout))
        for i, layout in enumerate(DENSE):
            for d in (0, 1):
                jobs.append((name, 'D', i, d, i + 1, layout))
    with Pool(a.workers) as pool:
        res = pool.map(job, [(j[0], j[5], j[3], j[4]) for j in jobs])
    print(f'{"setting":24s} | 1108 | random | HIT gap<3 island stuck lane (tip) | gap p10 | '
          'sequences, 1108 sign R seed 1')
    for name in CONFS:
        rows = [(j, r) for j, r in zip(jobs, res) if j[0] == name]
        fixed = [r for j, r in rows if j[1] == 'F']
        rand = [r for j, r in rows if j[1] == 'R']
        dense = [r for j, r in rows if j[1] == 'D']
        allr = fixed + rand
        count = {k: sum(k in r[0] for r in allr) for k in ('HIT', 'gap<3', 'island', 'stuck', 'lane')}
        gaps = [r[1] for r in allr if np.isfinite(r[1])]
        count['tip'] = sum(r[3] for r in allr)
        seqs = {j[2]: r[2] for j, r in rows if j[1] == 'F' and j[3] == 1 and j[4] == 1}
        print(f'{name:24s} | {sum(not r[0] for r in fixed):2d}/12 | {sum(not r[0] for r in rand):2d}/80 | '
              f'{count["HIT"]:3d} {count["gap<3"]:5d} {count["island"]:6d} {count["stuck"]:5d} '
              f'{count["lane"]:4d} ({count["tip"]:2d}) | {100 * np.percentile(gaps, 10):4.1f} cm | {seqs}')
        if dense:
            dc = {k: sum(k in r[0] for r in dense) for k in ('HIT', 'gap<3', 'island', 'stuck', 'lane')}
            print(f'{"":24s}   dense {sum(not r[0] for r in dense)}/{len(dense)}: '
                  + ' '.join(f'{k} {v}' for k, v in dc.items()))
        if a.fails:
            for j, r in rows:
                if j[1] == 'D' and r[0]:
                    print(f'    dense {j[2]} {j[5]}, sign {"LR"[j[3]]}: {" ".join(r[0])} ({r[2]})')
            for j, r in rows:
                if j[1] == 'R' and r[0]:
                    (_, r2), (side, ro) = j[5]
                    print(f'    L2 {r2:.2f}, {"L1" if side > 0 else "L3"} {ro:.2f}, sign '
                          f'{"LR"[j[3]]}: {" ".join(r[0])} ({r[2]})')


if __name__ == '__main__':
    main()
