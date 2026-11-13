#!/usr/bin/env python3
"""How well the car follows the avoid path, in the closed-loop simulation of test_static_avoid.py
(car 7's measured steering table, 10 Hz lidar ray-cast, the real StaticAvoid). The car has never
driven the avoidance itself (11-08 ~ 11-10 were all shadow runs), so this is the only tracking data.

    python3 tools/static_avoid/sim_tracking.py

Varies the pure pursuit look-ahead, the steering response (x0.55: the car turning about half of what
is asked, an old note from the /odom yaw) and the control rate (20 Hz set, 8~9 Hz seen on the car).
Error = how far the car's base_link is, at the same s, from the path planned `AGE` s before (the
plan is redone from the car every scan, so the current one always starts at the car).
"""
import os
import sys
from dataclasses import replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..', '..', 'ros2_ws', 'src', 'clothoid_lidar')
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.join(PKG, 'test'))
from clothoid_lidar.static_avoid import AvoidParams  # noqa: E402
import test_static_avoid as sim  # noqa: E402

AGE = 0.5
W = 0.50
HALF = sim.LIMO_LEN / 2
# tape layouts of the 11-08 bags: (lane y, rear of the LIMO from the entry line)
LAYOUTS = {
    'run1-2 (L2 1.00, L3 1.82)': [(0.0, 1.00), (-W, 1.82)],
    'run3 (L2 1.00, L1 1.82)': [(0.0, 1.00), (W, 1.82)],
    'run6-9 (L2 3.00, L3 2.50)': [(0.0, 3.00), (-W, 2.50)],
}


def one(lookahead, gain, every, direction, layout, seed):
    p = replace(AvoidParams(), lane_width=W, lookahead=lookahead)
    limos = [(rear + HALF, y) for y, rear in layout]
    track = []
    out = sim.run(limos, direction=direction, lane_width=W, params=p, seed=seed, steer_gain=gain,
                  control_every=every, track=track)
    want = {0: 1, 1: 3}[direction]
    ok = (out['activated'] and out['min_gap'] > 0.03 and out['island_ok']
          and out['max_abs_y'] < 1.5 * W and out['x'] > 6.0 and out['final_lane'] == want)
    errors = []
    for t, s, d, _ in track:
        old = [path for t0, _, _, path in track if t0 <= t - AGE]
        if old and old[-1][0, 0] <= s <= old[-1][-1, 0]:
            errors.append(d - float(np.interp(s, old[-1][:, 0], old[-1][:, 1])))
    return np.array(errors), out, ok


def main():
    rows = []
    for lookahead in (0.45, 0.30):
        for gain in (1.0, 0.55):
            for every, hz in ((1, 20), (3, 7)):
                errs, gaps, fails = [], [], []
                for name, layout in LAYOUTS.items():
                    for direction in (0, 1):
                        for seed in (1, 2):
                            e, out, ok = one(lookahead, gain, every, direction, layout, seed)
                            errs.append(np.abs(e))
                            gaps.append(out['min_gap'])
                            if not ok:
                                fails.append(f'{name.split()[0]}/{"LR"[direction]}')
                e = np.concatenate(errs) if errs else np.zeros(1)
                rows.append((lookahead, gain, hz, np.median(e), np.percentile(e, 90), e.max(),
                             min(gaps), fails))
    print('lookahead  steer  control | path error median  p90   max | min gap to a LIMO | failed')
    for la, g, hz, med, p90, mx, gap, fails in rows:
        print(f'  {la:.2f} m   x{g:.2f}  {hz:2d} Hz   |   {100 * med:5.1f} cm {100 * p90:5.1f} '
              f'{100 * mx:5.1f} | {100 * gap:5.1f} cm | {len(fails)}/12 {" ".join(fails)}')


if __name__ == '__main__':
    main()
