"""Contact sheet: one camera frame every `step` seconds, labelled with bag time.

    python3 contact.py <bag> <out.png> [step_sec=4]
"""
import sys

import cv2
import numpy as np

from bagio import image_bgr, read

bag, out, step = sys.argv[1], sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 4.0
frames, t0, next_t = [], None, 0.0
for _, t, msg in read(bag, ['/camera/color/image_raw']):
    t0 = t if t0 is None else t0
    if t - t0 >= next_t:
        img = cv2.resize(image_bgr(msg), (320, 240))
        cv2.putText(img, f'{t - t0:.0f}s', (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        frames.append(img)
        next_t += step
cols = 8
while len(frames) % cols:
    frames.append(np.zeros_like(frames[0]))
rows = [np.hstack(frames[i:i + cols]) for i in range(0, len(frames), cols)]
cv2.imwrite(out, np.vstack(rows))
print(out, len(frames))
