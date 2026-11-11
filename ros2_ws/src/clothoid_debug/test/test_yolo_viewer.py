import numpy as np

from clothoid_debug.yolo_viewer import Box, Replay, SIZE_CLASSES


def values(**over):
    v = {'yolo_hz': 0.0, 'model_conf': 0.25, 'min_confidence': 0.5, 'rate_hz': 20.0,
         'input_timeout': 0.5, 'light_max_offset_height_ratio': 3.5,
         'sign_max_offset_height_ratio': 5.0, 'fork_max_offset_height_ratio': 10.0,
         'detection_confirm_hits': 3, 'detection_window': 5, 'detection_max_age': 0.5,
         'detection_confidence': 0.6, 'sign_min_height': 14, 'light_min_height': 8,
         'detection_edge_margin': 2, 'slow_speed': 0.2, 'slow_hold_sec': 4.0,
         'bump_pitch_thresh': 3.5, 'bump_level_thresh': 1.2, 'bump_level_duration': 0.3,
         'bump_arm_timeout': 6.0, 'bump_cooldown': 4.0, 'pedestrian_wait_sec': 3.5,
         'stop_on_yellow': True, 'arm_timeout_sec': 8.0, 'cooldown_sec': 15.0,
         'use_stop_line': True, 'approach_speed': 0.2, 'fork_left_direction': 0,
         'fork_right_direction': 1, 'parking_at_stopline': False}
    for name in SIZE_CLASSES.values():
        v[f'{name}_min_area'], v[f'{name}_min_height'] = 2500.0, 40.0
    v.update(over)
    return v


def replay(boxes, **over):
    stamps = [i / 30.0 for i in range(len(boxes))]
    r = Replay(stamps, boxes, np.zeros((0, 2)), 320.0, 640, 480)
    r.run(values(**over))
    return r


def test_slow_sign_confirmed_on_third_frame():
    big = Box('sign_slow', 0.9, 290, 100, 350, 160)
    r = replay([[big]] * 4)
    assert [f[0][1] for f in r.results] == ['wait', 'wait', 'ok', 'ok']
    assert r.outputs[-1]['slow'] and r.outputs[-1]['speed_limit'] == 0.2


def test_reasons():
    small = Box('sign_slow', 0.9, 300, 100, 320, 120)        # 20 x 20 < 2500 px2
    weak = Box('sign_bump', 0.4, 290, 100, 350, 160)         # below min_confidence
    aside = Box('traffic_light_red', 0.9, 600, 100, 610, 120)  # 285 px off centre, h 20
    fates = replay([[small, weak, aside]]).results[0]
    assert fates[0][1] == 'drop' and fates[0][2].startswith('size')
    assert fates[1][1] == 'model'
    assert fates[2][1] == 'drop' and fates[2][2].startswith('side offset')


def test_yolo_hz_keeps_fewer_frames():
    big = Box('sign_slow', 0.9, 290, 100, 350, 160)
    r = replay([[big]] * 30, yolo_hz=10.0)
    assert len(r.used) == 10
