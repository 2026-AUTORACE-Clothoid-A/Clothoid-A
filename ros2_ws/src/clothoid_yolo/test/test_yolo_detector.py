"""Post-processing tests for the YOLO wrapper: `pytest ros2_ws/src/clothoid_yolo/test`."""
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from clothoid_yolo.yolo_detector import (  # noqa: E402
    KNOWN_CLASSES, Box, UltralyticsBackend, boxes_from_results, filter_boxes)


def fake_result():
    names = {0: 'sign_left', 1: 'traffic_light_red', 2: 'cat'}
    boxes = SimpleNamespace(
        xyxy=np.array([[10.4, 20.6, 110.0, 220.0], [300, 100, 340, 180], [0, 0, 5, 5]]),
        conf=np.array([0.91, 0.40, 0.99]),
        cls=np.array([0, 1, 2]))
    return SimpleNamespace(names=names, boxes=boxes)


def test_boxes_from_results_converts_names_and_rounds_pixels():
    boxes = boxes_from_results(fake_result())
    assert boxes[0] == Box('sign_left', pytest.approx(0.91), 10, 21, 110, 220)
    assert [b.class_name for b in boxes] == ['sign_left', 'traffic_light_red', 'cat']


def test_boxes_from_results_handles_no_detections():
    empty = SimpleNamespace(names={0: 'x'}, boxes=SimpleNamespace(
        xyxy=np.zeros((0, 4)), conf=np.zeros(0), cls=np.zeros(0)))
    assert boxes_from_results(empty) == []


def test_filter_drops_unknown_classes_and_reports_them():
    kept, unknown = filter_boxes(boxes_from_results(fake_result()), min_confidence=0.3)
    assert [b.class_name for b in kept] == ['sign_left', 'traffic_light_red']
    assert unknown == ['cat']


def test_filter_applies_confidence_threshold():
    kept, _ = filter_boxes(boxes_from_results(fake_result()), min_confidence=0.5)
    assert [b.class_name for b in kept] == ['sign_left']


def test_filter_applies_aliases_before_checking_known():
    boxes = [Box('red', 0.9, 0, 0, 10, 10), Box('sign_slow', 0.8, 0, 0, 10, 10)]
    kept, unknown = filter_boxes(boxes, 0.5, aliases={'red': 'traffic_light_red'})
    assert [b.class_name for b in kept] == ['traffic_light_red', 'sign_slow']
    assert unknown == []


def test_known_classes_match_the_message_definition():
    msg = os.path.join(os.path.dirname(__file__), '..', '..', 'clothoid_interfaces',
                       'msg', 'Detection.msg')
    text = open(msg, encoding='utf-8').read()
    for name in KNOWN_CLASSES:
        assert name in text


def test_missing_ultralytics_gives_a_clear_error(monkeypatch):
    monkeypatch.setitem(sys.modules, 'ultralytics', None)  # import raises ImportError
    with pytest.raises(RuntimeError, match='ultralytics is not installed'):
        UltralyticsBackend('model.pt')
