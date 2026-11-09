"""Regressions for false boxes recorded on LIMO2, 2026-11-10."""
from types import SimpleNamespace
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from clothoid_mission.mission_logic import DetectionGate, classes_seen


def box(name='sign_bump', coords=(357, 229, 369, 241), confidence=.53):
    return SimpleNamespace(class_name=name, confidence=confidence,
                           **dict(zip(('x_min', 'y_min', 'x_max', 'y_max'), coords)))


def step(gate, t, boxes):
    return gate.update(t, boxes, classes_seen(boxes))


def test_recorded_single_bump_and_clipped_floor_box_are_rejected():
    gate = DetectionGate()
    assert not step(gate, 0, [box()])
    floor = box(coords=(349, 468, 372, 480), confidence=.9)
    for t in (.1, .2, .3):
        assert not step(gate, t, [floor])


def test_three_distinct_frames_with_one_dropout_confirm_real_sign():
    gate = DetectionGate()
    real = box('sign_pedestrian', (385, 227, 401, 243), .81)
    assert not step(gate, 0, [real])
    assert not step(gate, .06, [])
    assert not step(gate, .12, [real])
    assert step(gate, .18, [real]) == {'sign_pedestrian'}
    assert not step(gate, .24, [])  # a confirmed box does not persist forever


def test_stale_or_spatially_unrelated_sightings_do_not_accumulate():
    gate = DetectionGate()
    a = box(coords=(300, 220, 320, 240), confidence=.9)
    b = box(coords=(360, 220, 380, 240), confidence=.9)
    assert not step(gate, 0, [a])
    assert not step(gate, .1, [b])
    assert not step(gate, .2, [a])
    assert not step(gate, 1, [a])


def test_green_also_requires_confirmation_and_survives_clock_reset():
    gate = DetectionGate()
    green = box('traffic_light_green', (310, 200, 330, 230), .9)
    for t in (1, 1.06):
        assert not step(gate, t, [green])
    assert step(gate, 1.12, [green]) == {'traffic_light_green'}
    assert not step(gate, 0, [green])
