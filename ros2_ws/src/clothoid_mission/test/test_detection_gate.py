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


def test_fork_sign_confirms_on_two_frames_others_still_need_three():
    """1114: the fork sign (~20 px) showed between the cone stacks for a moment and never got 3 hits."""
    fork = ('sign_left', 'sign_right')
    gate = DetectionGate(class_hits={n: 2 for n in fork})
    left = box('sign_left', (120, 200, 140, 220), .8)
    ped = box('sign_pedestrian', (385, 227, 401, 243), .81)
    assert not step(gate, 0, [left, ped])
    assert step(gate, .1, [left, ped]) == {'sign_left'}
    assert step(gate, .2, [ped]) == {'sign_pedestrian'}
    assert gate.required('sign_left') == 2 and gate.required('sign_pedestrian') == 3


def test_fork_sign_two_frames_too_far_apart_do_not_confirm():
    gate = DetectionGate(class_hits={'sign_left': 2})
    left = box('sign_left', (120, 200, 140, 220), .8)
    assert not step(gate, 0, [left])
    assert not step(gate, .6, [left])   # beyond max_age 0.5 s


def test_box_reason_names_the_failed_check():
    gate = DetectionGate()
    assert gate.box_reason(box(confidence=.5)).startswith('conf')
    assert gate.box_reason(box(coords=(300, 200, 310, 210), confidence=.9)).startswith('h ')
    assert gate.box_reason(box(coords=(0, 200, 30, 240), confidence=.9)) == 'edge'
    assert gate.box_reason(box(coords=(300, 200, 330, 240), confidence=.9)) is None


FORK_GATE = dict(class_hits={'sign_left': 2, 'sign_right': 2},
                 class_max_age={'sign_left': 1.0, 'sign_right': 1.0},
                 edge_exempt=('sign_left', 'sign_right'))


def test_fork_sign_second_frame_within_one_second_confirms_others_keep_half_a_second():
    # 1115_1709 17:12:43.16: sign_left (conf 0.67, h 21) at YOLO 10 Hz, no second frame within 0.5 s
    gate = DetectionGate(**FORK_GATE)
    left = box('sign_left', (180, 230, 201, 251), .67)
    bump = box('sign_bump', (400, 230, 421, 251), .9)
    assert not step(gate, 0.0, [left, bump])
    for t in (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7):   # misses in between: more than `window` frames
        assert not step(gate, t, [])
    assert not step(gate, 0.75, [bump])             # bump: 0.75 s apart, still 1/3 within 0.5 s
    assert step(gate, 0.8, [left]) == {'sign_left'}
    assert not step(gate, 2.0, [left])              # 1.2 s after the last one: starts over


def test_fork_sign_on_the_image_border_counts_other_signs_do_not():
    # 1115_1709 17:12:52.39: sign_left conf 0.85, h 65, box 0,235,38,300 dropped as 'edge'
    gate = DetectionGate(**FORK_GATE)
    left = box('sign_left', (0, 235, 38, 300), .85)
    bump = box('sign_bump', (0, 235, 38, 300), .85)
    assert gate.box_reason(left) is None
    assert gate.box_reason(bump) == 'edge'
    assert not step(gate, 0.0, [left])
    assert step(gate, 0.1, [box('sign_left', (0, 238, 35, 301), .8)]) == {'sign_left'}


def test_fork_sign_keeps_the_confidence_limit():
    # 1115_1709 17:12:44.96: conf 0.58 < 0.60 stays out
    gate = DetectionGate(**FORK_GATE)
    assert gate.box_reason(box('sign_left', (180, 230, 201, 251), .58)) == 'conf 0.58 < 0.60'


def test_1115_1726_fork_sign_sweeping_across_the_image_confirms_at_0_55():
    # /detections 17:27:18.04 / 18.53 / 18.94: the box moves ~100 px between frames while the car swerves
    seq = [(18.04, (459, 231, 478, 252), .65), (18.53, (353, 231, 372, 252), .59),
           (18.94, (273, 231, 293, 253), .55)]
    strict = DetectionGate(**FORK_GATE, class_confidence={'sign_left': .55})
    loose = DetectionGate(**FORK_GATE, class_confidence={'sign_left': .55}, any_x=('sign_left', 'sign_right'))
    got = {}
    for t, coords, conf in seq:
        b = [box('sign_left', coords, conf)]
        got.setdefault('strict', []).append(step(strict, t, b))
        got.setdefault('loose', []).append(step(loose, t, b))
    assert got['strict'] == [set(), set(), set()]
    assert got['loose'] == [set(), {'sign_left'}, {'sign_left'}]
    assert loose.box_reason(box('sign_left', (273, 231, 293, 253), .54)) == 'conf 0.54 < 0.55'
    assert loose.box_reason(box('sign_bump', (273, 231, 293, 253), .58)) == 'conf 0.58 < 0.60'


def test_any_x_still_needs_the_same_row_and_size():
    gate = DetectionGate(**FORK_GATE, any_x=('sign_left',))
    assert not step(gate, 0.0, [box('sign_left', (100, 230, 120, 251), .9)])
    assert not step(gate, 0.1, [box('sign_left', (500, 330, 520, 351), .9)])   # 100 px lower
    assert not step(gate, 0.2, [box('sign_left', (500, 200, 560, 290), .9)])   # 4x taller


def test_sign_log_key_drops_the_numbers_and_slows_far_sign_repeats():
    from clothoid_mission.mission_node import sign_log_key
    assert sign_log_key('IGNORED: invalid width or h 10 < 14') == sign_log_key('IGNORED: invalid width or h 9 < 14')
    assert sign_log_key('IGNORED: conf 0.58 < 0.60') == ('conf', 1.)
    assert sign_log_key('IGNORED: size') == ('size', 5.)
    assert sign_log_key('IGNORED: offset') == ('offset', 5.)
    assert sign_log_key('ARMED') == ('ARMED', 1.)
    assert sign_log_key('IGNORED: association/overlap')[0] == 'association/overlap'
