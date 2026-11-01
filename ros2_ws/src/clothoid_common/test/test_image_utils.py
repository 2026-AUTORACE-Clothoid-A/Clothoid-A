"""Image conversion tests. They need ROS message packages, so they are skipped on a plain PC
and run inside the dev container / on the Jetson: `pytest ros2_ws/src/clothoid_common/test`."""
import os
import sys

import numpy as np
import pytest

pytest.importorskip('sensor_msgs')
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from clothoid_common.image_utils import bgr_to_image, image_to_bgr  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import Header  # noqa: E402


def test_bgr_round_trip():
    frame = np.random.default_rng(0).integers(0, 255, (48, 64, 3), dtype=np.uint8)
    assert np.array_equal(image_to_bgr(bgr_to_image(frame, Header())), frame)


def test_rgb_is_converted_to_bgr():
    msg = Image(height=1, width=1, encoding='rgb8', step=3, data=bytes([10, 20, 30]))
    assert image_to_bgr(msg).tolist() == [[[30, 20, 10]]]


def test_row_padding_is_dropped():
    # step larger than width * 3 (row padding)
    msg = Image(height=2, width=1, encoding='bgr8', step=4,
                data=bytes([1, 2, 3, 0, 4, 5, 6, 0]))
    assert image_to_bgr(msg).tolist() == [[[1, 2, 3]], [[4, 5, 6]]]


def test_unsupported_encoding_raises():
    with pytest.raises(ValueError):
        image_to_bgr(Image(height=1, width=1, encoding='16UC1', step=2, data=bytes(2)))


def test_mono_image_message():
    out = bgr_to_image(np.zeros((4, 6), dtype=np.uint8), Header())
    assert out.encoding == 'mono8' and out.step == 6
