"""Compare lane metrics of run_pipeline.py outputs, e.g. before and after a lane change.

    python3 compare.py before/01.npz after/01.npz [--labels before,after]

Per npz:
  found %         frames where LaneDetector.found was true
  loss>0.5s       lane lost for longer than 0.5 s (count of such gaps)
  straight steer  median steering angle in degrees on straights; a straight is where the /imu
                  yaw rate, smoothed over 0.5 s, stays under 0.05 rad/s, so no hand-labelled segments
                  are needed. Near 0 is good; a steady offset means the target point is off centre
  jumps>5deg      consecutive found frames whose steering differs by more than 5 deg
Steering is lane_steer() of run_pipeline.py (-1..1 = -90..90 deg of the target direction).
"""
import argparse

import numpy as np

STRAIGHT_RATE = 0.05   # rad/s
SMOOTH_SEC = 0.5
LOSS_SEC = 0.5
JUMP_DEG = 5.0


def metrics(path):
    d = np.load(path)
    cam, imu = d['cam'], d['imu']
    t, found, steer = cam[:, 0], cam[:, 1] > 0, cam[:, 2] * 90.0
    # lane losses longer than LOSS_SEC
    losses, start = 0, None
    for ti, f in zip(t, found):
        if not f and start is None:
            start = ti
        elif f and start is not None:
            losses += ti - start > LOSS_SEC
            start = None
    if start is not None and t[-1] - start > LOSS_SEC:
        losses += 1
    # straights from the gyro
    dt = np.median(np.diff(imu[:, 0]))
    n = max(1, int(SMOOTH_SEC / dt))
    rate = np.convolve(imu[:, 1], np.ones(n) / n, mode='same')
    straight = np.abs(np.interp(t, imu[:, 0], rate)) < STRAIGHT_RATE
    k = straight & found
    s = steer[found]
    return {'frames': len(t), 'found %': 100 * found.mean(), 'loss>0.5s': losses,
            'straight steer': float(np.median(steer[k])) if k.any() else float('nan'),
            'jumps>5deg': int((np.abs(np.diff(s)) > JUMP_DEG).sum())}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('npz', nargs='+')
    ap.add_argument('--labels', default='')
    a = ap.parse_args()
    labels = a.labels.split(',') if a.labels else a.npz
    print(f'{"run":40s} {"frames":>6s} {"found %":>8s} {"loss>0.5s":>9s} {"straight steer":>14s} {"jumps>5deg":>10s}')
    for label, path in zip(labels, a.npz):
        m = metrics(path)
        print(f'{label:40s} {m["frames"]:6d} {m["found %"]:8.0f} {m["loss>0.5s"]:9d} '
              f'{m["straight steer"]:13.1f}° {m["jumps>5deg"]:10d}')


if __name__ == '__main__':
    main()
