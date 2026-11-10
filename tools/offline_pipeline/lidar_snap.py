"""Single-scan BEV views: raw points, cone clusters, vehicle-size clusters, sorter boundaries.

    python3 lidar_snap.py <bag_dir> <out.png> "<bag>@<sec>@<title>" ...

Example (2026-11-05 bags):
    python3 lidar_snap.py ~/autorace_bags/1105 lidar_scenes.png "01_slow_run_all@44@static obstacles" \
        "01_slow_run_all@91@cone course" "03_slow_all@95@parking area"
The sorter runs over every scan up to the chosen time, so "course running" matches the real node.
"""
import math
import sys
import os
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt, numpy as np
from run_pipeline import build
from clothoid_lidar.obstacle_extractor import extract_obstacles, transform_points
from clothoid_lidar.cone_sorter import ConeMission, ConeSorter
from bagio import read
TF = (0.103, 0.0, 0.0)
bag_dir, out_png = sys.argv[1], sys.argv[2]
jobs = []
for spec in sys.argv[3:]:
    bag, at, title = (spec.split('@', 2) + [''])[:3]
    jobs.append((bag, float(at), title))
cols = min(3, len(jobs))
rows = math.ceil(len(jobs) / cols)
fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 6 * rows), squeeze=False)
for ax in axes.flat[len(jobs):]:
    ax.axis('off')
for ax, (bag, at, title) in zip(axes.flat, jobs):
    m = build()
    t0 = None; scan = None; res = None
    for topic, t, msg in read(os.path.join(bag_dir, bag), ['/camera/color/image_raw', '/scan', '/imu', '/odom']):
        t0 = t if t0 is None else t0
        if topic != '/scan':
            continue
        cones = transform_points(extract_obstacles(msg.ranges, msg.angle_min, msg.angle_increment, m['ep']), *TF)
        res = m['mission'].update(cones)
        if t - t0 >= at:
            scan = msg; break
    r = np.asarray(scan.ranges); a = scan.angle_min + scan.angle_increment * np.arange(r.size)
    ok = np.isfinite(r) & (r > 0.05)
    ax.scatter(r[ok]*np.cos(a[ok]) + 0.103, r[ok]*np.sin(a[ok]), s=3, c='0.6', label='scan')
    veh = transform_points(extract_obstacles(scan.ranges, scan.angle_min, scan.angle_increment, m['vp']), *TF)
    vn = transform_points(extract_obstacles(scan.ranges, scan.angle_min, scan.angle_increment, m['vpn']), *TF)
    wide = [p for p in veh if not any(np.hypot(*(p - q)) < 0.05 for q in vn)]
    if len(cones): ax.scatter(cones[:, 0], cones[:, 1], s=40, facecolors='none', edgecolors='C1', label='cone clusters (current params)')
    if wide: w = np.array(wide); ax.scatter(w[:, 0], w[:, 1], s=120, marker='s', facecolors='none', edgecolors='purple', label='0.15-0.45 m wide clusters')
    if res is not None and m['mission'].running:
        if len(res.left): ax.plot(res.left[:, 0], res.left[:, 1], 'b.-', label='sorter LEFT')
        if len(res.right): ax.plot(res.right[:, 0], res.right[:, 1], 'r.-', label='sorter RIGHT')
    ax.plot(0, 0, 'k^', ms=10)
    ax.add_patch(plt.Rectangle((0.0, -0.6), 1.3, 1.2, color='orange', alpha=0.08))
    ax.set_xlim(-0.5, 2.5); ax.set_ylim(-2, 2); ax.set_aspect('equal'); ax.grid(alpha=0.3)
    ax.set_title(f'{bag[:2]} @ {at:.1f}s: {title}\ncone course running={m["mission"].running}', fontsize=10)
    ax.legend(fontsize=7, loc='lower right')
fig.suptitle('Single lidar scans in base_link (x forward, y left). Shaded = cone-course start zone (x 0-1.3, |y|<0.6, >=6 cones)')
fig.tight_layout(); fig.savefig(out_png, dpi=85)
print('ok')
