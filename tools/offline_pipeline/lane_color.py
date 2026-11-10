"""Measure the lane line colour and the lane width in the lane_main warp image from a bag.

    python3 lane_color.py <bag> [--from 0] [--to 1e9] [--every 6] [--hsv 18,70,100,32,255,255]

1. Colour: in the lower half of each sampled frame, pixels that look yellow (H 15-40, S>=60, V>=100)
   are summarised as H/S/V percentiles, next to how many pixels the lane.yaml mask keeps.
2. Width: with the lane.yaml warp and the --hsv mask, the histogram peaks of the bottom half of the
   warp image give the left/right line x; the median distance is what `lane_width` should be.
   Only use straight segments (--from/--to), curves widen the lane in the warp.
"""
import argparse

import cv2
import numpy as np

from bagio import image_bgr, read
from run_pipeline import build

ap = argparse.ArgumentParser()
ap.add_argument('bag')
ap.add_argument('--from', dest='t_from', type=float, default=0.0)
ap.add_argument('--to', dest='t_to', type=float, default=1e9)
ap.add_argument('--every', type=int, default=6)
ap.add_argument('--hsv', default='18,70,100,32,255,255')
ap.add_argument('--camera', default='limo2', help='camera profile: limo1 | limo2 (car 7)')
a = ap.parse_args()
lo_hi = [int(v) for v in a.hsv.split(',')]
lo, hi = np.array(lo_hi[:3]), np.array(lo_hi[3:])

det = build(a.camera)['lane']
p = det.p
t0 = None
hsv_stats, widths = [], []
for _, t, msg in read(a.bag, ['/camera/color/image_raw'], every={'/camera/color/image_raw': a.every}):
    t0 = t if t0 is None else t0
    if not a.t_from <= t - t0 <= a.t_to:
        continue
    bgr = image_bgr(msg)
    hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)
    lower = hsv[hsv.shape[0] // 2:]
    yellow = (lower[..., 0] >= 15) & (lower[..., 0] <= 40) & (lower[..., 1] >= 60) & (lower[..., 2] >= 100)
    if yellow.sum() > 200:
        repo = cv2.inRange(lower, np.array(p.hsv_lower), np.array(p.hsv_upper))
        hsv_stats.append((*(lower[..., c][yellow] for c in range(3)), yellow.sum(), np.count_nonzero(repo)))
    mask = cv2.inRange(hsv, lo, hi)
    warped = cv2.warpPerspective(mask, det._matrix, (p.warp_w, p.warp_h))
    hist = np.sum(warped[p.warp_h // 2:], axis=0)
    mid = hist.size // 2
    if hist[:mid].max() > 0 and hist[mid:].max() > 0:
        left, right = int(np.argmax(hist[:mid])), int(np.argmax(hist[mid:]) + mid)
        widths.append((left, right, right - left))

if hsv_stats:
    h = np.concatenate([s[0] for s in hsv_stats])
    s_ = np.concatenate([s[1] for s in hsv_stats])
    v = np.concatenate([s[2] for s in hsv_stats])
    kept = sum(s[4] for s in hsv_stats) / max(sum(s[3] for s in hsv_stats), 1)
    print(f'yellow-looking pixels in {len(hsv_stats)} frames: H p10/50/90 {np.percentile(h, [10, 50, 90])}, '
          f'S {np.percentile(s_, [10, 50, 90])}, V {np.percentile(v, [10, 50, 90])}')
    print(f'lane.yaml mask {p.hsv_lower}..{p.hsv_upper} keeps {100 * kept:.1f}% of them')
if widths:
    w = np.array(widths)
    print(f'lane width in warp ({len(w)} frames, mask {a.hsv}): median {np.median(w[:, 2]):.0f} px '
          f'(p10 {np.percentile(w[:, 2], 10):.0f}, p90 {np.percentile(w[:, 2], 90):.0f}), '
          f'left x {np.median(w[:, 0]):.0f}, right x {np.median(w[:, 1]):.0f}, warp_w {p.warp_w}')
else:
    print('no frame with both lines under the --hsv mask')
