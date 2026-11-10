"""Per-bag timeline and dead-reckoned lap map from run_pipeline.py output (+ the bag for scans).

    python3 plots.py <bag> <npz> <outdir> <tag>

Writes <outdir>/timeline_<tag>.png and <outdir>/map_<tag>.png. The map uses /imu gyro yaw and
/odom linear.x (the /odom yaw is ~1.8x too large, see docs), so it drifts slowly over a lap.
"""
import math
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from bagio import read  # noqa: E402

bag, npz, outdir, tag = sys.argv[1:5]
d = np.load(npz)
cam, lid, imu, odo = d['cam'], d['lid'], d['imu'], d['odo']
tc, tl = cam[:, 0], lid[:, 0]


def spans(t, mask, join=0.15):
    out, s, last = [], None, None
    for ti, m in zip(t, mask):
        if m:
            if s is None:
                s = ti
            elif ti - last > join:
                out.append((s, last))
                s = ti
            last = ti
    if s is not None:
        out.append((s, last))
    return out


# ---------------- timeline ----------------
fig, ax = plt.subplots(6, 1, figsize=(16, 13), sharex=True,
                       gridspec_kw={'height_ratios': [1.3, 1.6, 1.2, 1.2, 1.0, 1.0]})
a = ax[0]
a.plot(tc, np.degrees(cam[:, 2]), lw=0.6, color='C0', label='lane steering command (deg)')
for s, e in spans(tc, cam[:, 1] == 0):
    a.axvspan(s, e, color='red', alpha=0.25, lw=0)
a.set_ylabel('lane')
a.plot([], [], color='red', alpha=0.4, lw=6, label='lane not found (no /cmd_vel_lane)')
a.legend(loc='upper right', fontsize=8)

a = ax[1]
rows = [('stopline (/stopline_detected)', cam[:, 4] == 1, 'C0', tc),
        ('speed_zone (red HSV)', cam[:, 7] == 1, 'C3', tc),
        ('parking start line', cam[:, 9] > 5, 'C2', tc),
        ('cone course active', lid[:, 4] == 1, 'C1', tl),
        ('cone follower REVERSE', lid[:, 10] == 1, 'k', tl),
        ('vehicle-size cluster ahead', lid[:, 3] > 0, 'C4', tl)]
for i, (name, m, c, t) in enumerate(rows):
    for s, e in spans(t, m, 0.15 if t is tc else 0.25):
        a.broken_barh([(s, max(e - s, 0.08))], (i - 0.35, 0.7), color=c)
a.set_yticks(range(len(rows)))
a.set_yticklabels([r[0] for r in rows], fontsize=8)
a.invert_yaxis()
a.set_ylabel('detectors')

a = ax[2]
a.plot(tl, lid[:, 1], lw=0.7, label='cone-size clusters (all)')
a.plot(tl, lid[:, 2], lw=0.7, label='cone-size ahead (x<1.5, |y|<0.8)')
a.plot(tl, lid[:, 3], lw=0.9, label='vehicle-size ahead (0.15-0.45 m wide)')
a.set_ylabel('lidar clusters')
a.legend(loc='upper right', fontsize=8)

a = ax[3]
a.plot(odo[:, 0], odo[:, 1], lw=0.6, label='/odom linear.x (m/s)')
a.axhline(0.2, color='C3', ls='--', lw=0.8, label='0.2 m/s slow limit')
a.axhline(0.4, color='grey', ls=':', lw=0.8, label='0.4 m/s')
a.set_ylabel('speed')
a.legend(loc='upper right', fontsize=8)

a = ax[4]
a.plot(imu[:, 0], imu[:, 1], lw=0.5, label='/imu gyro z (rad/s)')
a.plot(odo[:, 0], odo[:, 2], lw=0.5, alpha=0.7, label='/odom angular.z (from steering)')
a.set_ylabel('yaw rate')
a.legend(loc='upper right', fontsize=8)

a = ax[5]
b, g, r = cam[:, 10], cam[:, 11], cam[:, 12]
a.plot(tc, r, color='r', lw=0.6, label='R mean')
a.plot(tc, g, color='g', lw=0.6, label='G mean')
a.plot(tc, b, color='b', lw=0.6, label='B mean')
a.plot(tc, (b - r), color='k', lw=0.8, label='B - R (colour cast)')
a.set_ylabel('image colour')
a.set_xlabel('bag time (s)')
a.legend(loc='upper right', fontsize=8, ncol=4)
fig.suptitle(f'{tag}: current pipeline run offline on the manual-driving bag')
fig.tight_layout()
fig.savefig(f'{outdir}/timeline_{tag}.png', dpi=90)
plt.close(fig)

# ---------------- dead-reckoned map ----------------
# yaw from integrated gyro (the /odom yaw is ~1.8x too large), distance from /odom linear.x
ti, wz = imu[:, 0], imu[:, 1]
yaw_i = np.concatenate([[0.0], np.cumsum(0.5 * (wz[1:] + wz[:-1]) * np.diff(ti))])
to, v = odo[:, 0], odo[:, 1]
yaw_o = np.interp(to, ti, yaw_i)
dt = np.diff(to, prepend=to[0])
x = np.cumsum(v * np.cos(yaw_o) * dt)
y = np.cumsum(v * np.sin(yaw_o) * dt)


def pose(t):
    return np.interp(t, to, x), np.interp(t, to, y), np.interp(t, ti, yaw_i)


_, T0, _ = next(read(bag, ['/camera/color/image_raw', '/scan', '/imu', '/odom']))
pts = []
for k, (_, t, msg) in enumerate(read(bag, ['/scan'])):
    if k % 3:
        continue
    r = np.asarray(msg.ranges)
    ang = msg.angle_min + msg.angle_increment * np.arange(r.size)
    ok = np.isfinite(r) & (r > 0.1) & (r < 2.5)
    px, py, pyaw = pose(t - T0)  # same relative time base as run_pipeline.py
    lx = r[ok] * np.cos(ang[ok]) + 0.103
    ly = r[ok] * np.sin(ang[ok])
    pts.append(np.column_stack([px + lx * math.cos(pyaw) - ly * math.sin(pyaw),
                                py + lx * math.sin(pyaw) + ly * math.cos(pyaw)]))
P = np.vstack(pts) if pts else np.zeros((0, 2))

fig, a = plt.subplots(figsize=(14, 10))
a.scatter(P[:, 0], P[:, 1], s=0.2, c='0.75', lw=0, label='lidar points (dead reckoned)')
cx, cy = np.interp(tc, to, x), np.interp(tc, to, y)
lost = cam[:, 1] == 0
a.scatter(cx[~lost], cy[~lost], s=2, c='C0', label='lane found')
a.scatter(cx[lost], cy[lost], s=6, c='red', label='lane NOT found')
for name, m, c, mk in [('stopline', cam[:, 4] == 1, 'blue', 's'),
                       ('speed_zone (red)', cam[:, 7] == 1, 'magenta', 'D'),
                       ('parking start line', cam[:, 9] > 5, 'green', '^')]:
    a.scatter(cx[m], cy[m] + 0.0, s=14, c=c, marker=mk, label=name, alpha=0.6)
lx_, ly_ = np.interp(tl, to, x), np.interp(tl, to, y)
m = lid[:, 4] == 1
a.scatter(lx_[m], ly_[m], s=30, facecolors='none', edgecolors='orange', label='cone course active')
m = lid[:, 3] > 0
a.scatter(lx_[m], ly_[m], s=12, c='purple', marker='x', label='vehicle-size cluster ahead')
for tt in range(0, int(tc[-1]) + 1, 10):
    a.annotate(f'{tt}s', (np.interp(tt, to, x), np.interp(tt, to, y)), fontsize=8)
a.plot(x[:1], y[:1], 'k*', ms=14, label='start')
a.set_aspect('equal')
a.legend(loc='best', fontsize=8, markerscale=2)
a.set_title(f'{tag}: lap reconstructed from /imu yaw + /odom speed (drift grows along the lap)')
a.set_xlabel('x (m)')
a.set_ylabel('y (m)')
fig.tight_layout()
fig.savefig(f'{outdir}/map_{tag}.png', dpi=90)
print('ok', tag)
