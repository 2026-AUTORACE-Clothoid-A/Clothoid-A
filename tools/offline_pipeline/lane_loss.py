"""Why does the lane detector report "not found"? Per frame, which window (left / right) got pixels.

    python3 lane_loss.py <bag> [--every 2] [--hsv 18,70,100,32,255,255] [--segments 01]

Re-runs the LaneDetector sliding window with the lane.yaml values and records whether the left and
the right window found pixels at least once (the detector needs both, even when it follows one side).
Prints, per track segment, how many frames were lost and why. `--hsv` also runs the same frames with
that HSV mask (Canny still on) for comparison. `--segments 01` uses the hand-labelled segment times of
the 2026-11-05 bag 01 (see the README), otherwise 10 s bins.
"""
import argparse
import dataclasses

import cv2
import numpy as np

from bagio import image_bgr, read
from run_pipeline import build

SEGMENTS_01 = [(0, 8, 'start straight'), (8, 12, 'slow mark'), (12, 24, 'top straight'),
               (24, 28, 'top-right corner'), (28, 37, 'zigzag + crosswalk'), (37, 40, 'corner'),
               (40, 62, '3-lane obstacles'), (62, 75, 'fork A/B'), (75, 80, 'after fork'),
               (80, 97, 'cone course'), (97, 105, 'left side / corner'), (105, 114, 'inner crosswalk'),
               (114, 125, 'inner straight'), (125, 1e9, 'parking')]


def windows(det, bgr, side, lane_width):
    """(left_found, right_found, left_start_x, right_start_x) of one frame, like LaneDetector.detect."""
    p = det.p
    blur = cv2.GaussianBlur(bgr, (5, 5), 0)
    hsv = cv2.inRange(cv2.cvtColor(blur, cv2.COLOR_BGR2HSV), np.array(p.hsv_lower), np.array(p.hsv_upper))
    merged = cv2.bitwise_or(cv2.Canny(blur, p.canny_low, p.canny_high), hsv)
    warped = cv2.warpPerspective(merged, det._matrix, (p.warp_w, p.warp_h))
    h, w = warped.shape
    hist = np.sum(warped[h // 2:], axis=0)
    mid = w // 2
    lx, rx = int(np.argmax(hist[:mid])), int(np.argmax(hist[mid:]) + mid)
    ny, nx = warped.nonzero()
    wh = h // p.nwindows
    lf = rf = False
    lc, rc = lx, rx
    for k in range(p.nwindows):
        yl, yh = h - (k + 1) * wh, h - k * wh
        rows = (ny >= yl) & (ny < yh)
        gl = (rows & (nx >= lc - p.margin) & (nx < lc + p.margin)).nonzero()[0]
        gr = (rows & (nx >= rc - p.margin) & (nx < rc + p.margin)).nonzero()[0]
        if len(gl) > p.min_pixels:
            lc, lf = int(np.mean(nx[gl])), True
        if len(gr) > p.min_pixels:
            rc, rf = int(np.mean(nx[gr])), True
    return lf, rf, lx, rx


ap = argparse.ArgumentParser()
ap.add_argument('bag')
ap.add_argument('--every', type=int, default=2)
ap.add_argument('--hsv', default='')
ap.add_argument('--segments', default='')
a = ap.parse_args()

m = build()
dets = {'lane.yaml': m['lane']}
if a.hsv:
    v = [int(x) for x in a.hsv.split(',')]
    dets[f'hsv {a.hsv}'] = type(m['lane'])(dataclasses.replace(m['lane'].p, hsv_lower=tuple(v[:3]),
                                                                hsv_upper=tuple(v[3:])))
rows = {k: [] for k in dets}
t0 = None
for _, t, msg in read(a.bag, ['/camera/color/image_raw'], every={'/camera/color/image_raw': a.every}):
    t0 = t if t0 is None else t0
    bgr = image_bgr(msg)
    for name, det in dets.items():
        rows[name].append((t - t0, *windows(det, bgr, m['side'], m['lane_width'])))

segs = SEGMENTS_01 if a.segments == '01' else None
for name, r in rows.items():
    r = np.array(r, float)
    t, lf, rf = r[:, 0], r[:, 1] > 0, r[:, 2] > 0
    if segs is None:
        segs = [(s, s + 10, f'{s:.0f}-{s + 10:.0f} s') for s in range(0, int(t[-1]) + 1, 10)]
    print(f'== {name}: found {100 * np.mean(lf & rf):.0f}% of {len(t)} frames')
    print(f'{"segment":22s} frames  lost  left-missing  right-missing  both-missing')
    for s, e, label in segs:
        k = (t >= s) & (t < e)
        if not k.any():
            continue
        lost = k & ~(lf & rf)
        print(f'{label:22s} {k.sum():6d} {100 * lost.sum() / k.sum():4.0f}%  '
              f'{(k & ~lf & rf).sum():12d} {(k & lf & ~rf).sum():14d} {(k & ~lf & ~rf).sum():13d}')
