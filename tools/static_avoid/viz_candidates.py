#!/usr/bin/env python3
"""Draw what the static avoid planners choose, in the simulation of test_static_avoid.py.

    python3 tools/static_avoid/viz_candidates.py out.png

Per layout (sign right): left, at the start of the avoidance, the one-stage candidates (2 m, scored by
where they end) and the lane sequence chosen (lane centre to lane centre, curvature limited, into lane
3 before the island); right, the closed-loop drives of both.
"""
import os
import sys
from dataclasses import replace

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..', '..', 'ros2_ws', 'src', 'clothoid_lidar')
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.join(PKG, 'test'))
from clothoid_lidar.static_avoid import LETHAL, AvoidParams, Costmap, plan_paths  # noqa: E402
import test_static_avoid as sim  # noqa: E402

W = 0.5
HALF = sim.LIMO_LEN / 2
ISLAND = (4.4, 6.15)
LAYOUTS = {'1108 run 1-2 (L2 1.00, L3 1.82)': [(1.00 + HALF, 0.0), (1.82 + HALF, -W)],
           '1108 run 6-9 (L2 3.00, L3 2.50)': [(3.00 + HALF, 0.0), (2.50 + HALF, -W)],
           'random (L2 1.27, L3 2.54)': [(1.27 + HALF, 0.0), (2.54 + HALF, -W)]}


def road(ax, limos, title):
    for y in (-1.5 * W, 1.5 * W):
        ax.axhline(y, color='goldenrod', lw=2)
    for y in (-0.5 * W, 0.5 * W):
        ax.axhline(y, color='0.6', lw=1, ls=(0, (6, 6)))
    ax.axvline(0, color='0.3', lw=2)
    tip = 0.25
    ax.add_patch(Polygon([(ISLAND[0], 0), (ISLAND[0] + tip, W / 2), (ISLAND[1], W / 2),
                          (ISLAND[1], -W / 2), (ISLAND[0] + tip, -W / 2)],
                         fc='0.85', ec='goldenrod', hatch='//'))
    for x, y in limos:
        ax.add_patch(Rectangle((x - HALF, y - sim.LIMO_WID / 2), sim.LIMO_LEN, sim.LIMO_WID,
                               fc='tab:blue', ec='k'))
    for y, n in ((W, 'lane 1 (A)'), (0, 'lane 2'), (-W, 'lane 3 (B)')):
        ax.text(-0.75, y - 0.03, n, fontsize=7)
    ax.set_xlim(-0.8, 5.2)
    ax.set_ylim(-0.9, 0.9)
    ax.set_aspect('equal')
    ax.set_title(title, fontsize=9)


def costmap_for(limos, car):
    p = AvoidParams()
    cm = Costmap(p)
    rng = np.random.default_rng(0)
    o = np.array([car[0] + sim.LIDAR_X, car[1]])
    for _ in range(8):                         # cells need repeated hits
        ang, r = sim.raycast(o, 0.0, limos, rng)
        ok = np.isfinite(r)
        hits = np.column_stack((o[0] + r[ok] * np.cos(ang[ok]), o[1] + r[ok] * np.sin(ang[ok])))
        cm.update(hits, o, np.column_stack((np.cos(ang[~ok]), np.sin(ang[~ok]))))
    return cm


def snapshot(ax, limos, car, title):
    road(ax, limos, title)
    cm = costmap_for(limos, car)
    ext = (AvoidParams().grid_s_min, AvoidParams().grid_s_min + cm.ns * AvoidParams().resolution,
           -cm.d_max, cm.d_max)
    ax.imshow(np.where(cm.cost.T >= LETHAL, 1.0, np.nan), origin='lower', extent=ext, cmap='Reds',
              vmin=0, vmax=1.3, alpha=0.45, aspect='auto')
    one = plan_paths((car[0], car[1], 0.0), 0.0, cm, 1, None, replace(AvoidParams(), sequence=False),
                     keep_candidates=True, island=ISLAND, forbid_island=True)
    for c, cost in one.candidates:
        ax.plot(c[:, 0], c[:, 1], color='tab:orange' if np.isfinite(cost) else '0.8', lw=0.4,
                alpha=0.4)
    ax.plot(one.path[:, 0], one.path[:, 1], color='tab:orange', lw=2.5, label='one-stage: chosen')
    seq = plan_paths((car[0], car[1], 0.0), 0.0, cm, 1, None, AvoidParams(), island=ISLAND,
                     forbid_island=True)
    if seq.sequence is not None:
        lanes = '-'.join(map(str, [2] + seq.sequence.lanes))
        ax.plot(seq.path[:, 0], seq.path[:, 1], color='tab:purple', lw=2.5,
                label=f'lane sequence: {lanes}')
    ax.plot(*car, 'k^', ms=9)
    ax.legend(fontsize=7, loc='lower left')


def drive(ax, limos, title):
    road(ax, limos, title)
    for seq, col in ((False, 'tab:orange'), (True, 'tab:purple')):
        tr = []
        o = sim.run(limos, direction=1, params=replace(AvoidParams(), sequence=seq), seed=1, track=tr)
        xy = np.array([(s, d) for _, s, d, _ in tr])
        ok = o['x'] > 6.0 and o['island_ok'] and o['min_gap'] > 0.03
        lab = (('lane sequence' if seq else 'one-stage') + (' ok' if ok else ' FAIL')
               + f', gap {100 * o["min_gap"]:.0f} cm')
        ax.plot(xy[:, 0], xy[:, 1], color=col, lw=2.2, label=lab)
        if o['x'] <= 6.0:
            ax.plot(*xy[-1], 'x', color=col, ms=12, mew=3)
    ax.legend(fontsize=7, loc='lower left')


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'viz_candidates.png'
    fig, axs = plt.subplots(len(LAYOUTS), 2, figsize=(15, 3.8 * len(LAYOUTS)))
    for row, (name, limos) in enumerate(LAYOUTS.items()):
        snapshot(axs[row, 0], limos, (-0.1, 0.0), f'{name}, sign right: plans at the start')
        drive(axs[row, 1], limos, f'{name}: closed loop (car 7 steering)')
    fig.tight_layout()
    fig.savefig(out, dpi=85)
    print('saved', out)


if __name__ == '__main__':
    main()
