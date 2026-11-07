"""sensor_msgs/Image <-> numpy helpers (no cv_bridge needed)."""
import array

import cv2
import numpy as np
from sensor_msgs.msg import Image


def image_to_bgr(msg: Image) -> np.ndarray:
    """Convert a sensor_msgs/Image to a BGR numpy array."""
    channels = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1}.get(msg.encoding)
    if channels is None:
        raise ValueError(f'unsupported image encoding: {msg.encoding}')
    frame = np.frombuffer(msg.data, dtype=np.uint8)
    frame = frame.reshape(msg.height, msg.step)[:, :msg.width * channels]
    frame = frame.reshape(msg.height, msg.width, channels)
    if msg.encoding == 'bgr8':
        return frame
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if msg.encoding == 'bgra8':
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    if msg.encoding == 'rgba8':
        return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)


def bgr_to_image(frame: np.ndarray, header) -> Image:
    """Wrap a BGR (or single channel) numpy array in an Image message."""
    out = Image()
    out.header = header
    out.height, out.width = frame.shape[:2]
    if frame.ndim == 2:
        out.encoding = 'mono8'
        out.step = out.width
    else:
        out.encoding = 'bgr8'
        out.step = out.width * 3
    # array, not bytes: on Humble the bytes path checks every element (206 ms for one 640x480 frame)
    out.data = array.array('B', np.ascontiguousarray(frame).tobytes())
    return out
