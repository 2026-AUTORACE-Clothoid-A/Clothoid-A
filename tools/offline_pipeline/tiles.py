"""Tile the per-time debug snapshots of one bag: camera | lane warp | stopline | start line.

    python3 tiles.py <snapdir from run_pipeline.py --snapdir> <out.png>
"""
import glob, os, sys
import cv2, numpy as np
d, out = sys.argv[1], sys.argv[2]
rows = []
for cam in sorted(glob.glob(f'{d}/*_cam.jpg')):
    tag = os.path.basename(cam)[:6]
    def load(name, size):
        p = f'{d}/{tag}_{name}'
        img = cv2.imread(p) if os.path.exists(p) else None
        return np.zeros((size[1], size[0], 3), np.uint8) if img is None else cv2.resize(img, size)
    c = load('zone.jpg', (320, 240))
    cv2.putText(c, f'{float(tag):.0f}s', (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    row = [c, load('lane.png', (320, 240)), load('stop.png', (320, 240)), load('start.png', (320, 240))]
    for im, label in zip(row[1:], ['lane (side=right)', 'stopline', 'parking start line']):
        cv2.putText(im, label, (5, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    rows.append(np.hstack([np.pad(r, ((2, 2), (2, 2), (0, 0))) for r in row]))
cv2.imwrite(out, np.vstack(rows))
print(out, len(rows))
