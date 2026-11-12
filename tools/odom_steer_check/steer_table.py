#!/usr/bin/env python3
"""Steering table from a forward/reverse bag recorded by steer_table.sh.

For every constant /cmd_vel run: chassis value sent, real inner wheel angle from the IMU gyro
(steady part, first --settle s dropped), the value the chassis reports back (from /odom), and
R_rear. Then one row per value: left/right x forward/reverse, and a YAML block for limo_base.
    python3 steer_table.py <bag_dir> [--scale 1.06] [--car 8] [--settle 1.0]
"""
import argparse
import math
import os
import sys
import warnings

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_drive import L, central_to_inner, read_bag, runs  # noqa: E402


def inner_of(rate, v):
    """Signed inner wheel angle of a bicycle with speed v turning at `rate` (+ = left)."""
    if abs(v) < 1e-3:
        return float('nan')
    return float(central_to_inner(math.atan(L * rate / v)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--scale', type=float, default=1.06, help='steering_scale limo_base ran with (launch default 1.06)')
    ap.add_argument('--max-inner', type=float, default=0.48869)
    ap.add_argument('--settle', type=float, default=1.0)
    ap.add_argument('--car', default='?')
    a = ap.parse_args()
    warnings.simplefilter("ignore", RuntimeWarning)

    cmd, odom, imu = read_bag(a.bag)
    if len(cmd) == 0:
        sys.exit('no /cmd_vel in the bag')
    rows = []
    print(f'{a.bag}  car {a.car}  scale {a.scale}')
    print(' run  t[s]        v      z    | value  side dir | odom v  gyro   | real inner  fb value | R_rear  note')
    for k, (t0, t1, v, z) in enumerate(runs(cmd), 1):
        c = math.atan(L * z / v)
        inner_cmd = max(-a.max_inner, min(a.max_inner, float(central_to_inner(c))))
        value = inner_cmd / a.scale
        lo = t0 + a.settle
        mo = (odom[:, 0] >= lo) & (odom[:, 0] <= t1)
        mi = (imu[:, 0] >= lo) & (imu[:, 0] <= t1)
        note = ''
        if t1 - lo < 0.8 or mo.sum() < 5 or mi.sum() < 5:
            note = 'too short'
            continue
        ov, ow, g = odom[mo, 1].mean(), odom[mo, 2].mean(), imu[mi, 1].mean()
        if abs(ov) < 0.6 * abs(v):
            note = 'SLOW (lifted / blocked?)'
        if np.std(imu[mi, 1]) > 0.15:
            note += ' gyro noisy'
        real = inner_of(g, ov)
        fb = inner_of(ow, ov) / a.scale
        side = 'L' if value > 1e-4 else 'R' if value < -1e-4 else '0'
        direction = 'fwd' if v > 0 else 'rev'
        r = ov / g if abs(g) > 1e-3 else float('inf')
        print(f'{k:4d} {t0:6.1f}-{t1:6.1f} {v:+.2f} {z:+.3f} | {value:+.3f}  {side}   {direction} | '
              f'{ov:+.3f} {g:+.3f} | {math.degrees(real):+6.1f}°   {fb:+.3f}  | {r:+6.2f}  {note}')
        if not note:
            rows.append((round(abs(value), 3), side, direction, math.degrees(real)))

    if not rows:
        sys.exit('no usable runs')
    print('\nreal inner wheel angle [deg] per value (fwd / rev, mean).  0-row = straight drift')
    print(' value |   left fwd   rev   mean |  right fwd   rev   mean | L-R')
    table = []
    for val in sorted({r[0] for r in rows}):
        cell = {}
        for side in ('L', 'R', '0'):
            for d in ('fwd', 'rev'):
                x = [r[3] for r in rows if r[0] == val and r[1] == side and r[2] == d]
                cell[side, d] = np.mean(x) if x else float('nan')
        if val == 0:
            print(f' {val:.3f} |  straight fwd {cell["0", "fwd"]:+5.1f}  rev {cell["0", "rev"]:+5.1f}')
            continue
        lm = np.nanmean([cell['L', 'fwd'], cell['L', 'rev']])
        rm = -np.nanmean([cell['R', 'fwd'], cell['R', 'rev']])
        table.append((val, lm, rm))
        print(f' {val:.3f} |   {cell["L", "fwd"]:+6.1f} {cell["L", "rev"]:+6.1f} {lm:6.1f} |'
              f'   {-cell["R", "fwd"]:6.1f} {-cell["R", "rev"]:6.1f} {rm:6.1f} | {lm - rm:+5.1f}')
    print('\n# limo_base steering table candidate (value sent -> real inner angle, deg)')
    print(f'car_{a.car}:')
    print('  value:     [' + ', '.join(f'{t[0]:.3f}' for t in table) + ']')
    print('  left_deg:  [' + ', '.join(f'{t[1]:.1f}' for t in table) + ']')
    print('  right_deg: [' + ', '.join(f'{t[2]:.1f}' for t in table) + ']')


if __name__ == '__main__':
    main()
