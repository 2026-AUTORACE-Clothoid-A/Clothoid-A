"""Shared frames must retain ownership until every detector has finished."""
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np

HERE = Path(__file__).parent
for package in ('clothoid_lane', 'clothoid_common'):
    sys.path.insert(0, str(HERE.parents[1] / package))

from clothoid_lane import lane_main


def test_both_detectors_receive_same_frame_and_finish_before_next_callback(monkeypatch):
    frame = np.arange(60, dtype=np.uint8).reshape(4, 5, 3)
    original = frame.copy()
    order = []
    conversions = []
    header = object()

    def decode(msg):
        conversions.append(msg)
        return frame

    def stopline(bgr):
        assert bgr is frame
        order.append('stopline')

    def lane(bgr, received_header):
        assert order == ['stopline']
        assert bgr is frame and received_header is header
        order.append('lane')

    monkeypatch.setattr(lane_main, 'image_to_bgr', decode)
    receiver = SimpleNamespace(image_observers=[stopline], process_bgr=lane)
    msg = SimpleNamespace(header=header)
    lane_main.LaneMainNode.on_image(receiver, msg)
    assert order == ['stopline', 'lane']
    assert conversions == [msg]
    assert np.array_equal(frame, original)


def test_standalone_lane_keeps_one_conversion_and_its_header(monkeypatch):
    frame = np.zeros((4, 5, 3), np.uint8)
    calls = []
    monkeypatch.setattr(lane_main, 'image_to_bgr', lambda msg: frame)
    msg = SimpleNamespace(header=object())
    receiver = SimpleNamespace(image_observers=[], process_bgr=lambda bgr,h: calls.append((bgr,h)))
    lane_main.LaneMainNode.on_image(receiver, msg)
    assert len(calls) == 1 and calls[0][0] is frame and calls[0][1] is msg.header
