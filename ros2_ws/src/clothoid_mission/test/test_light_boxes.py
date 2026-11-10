"""Mission regressions using light and sign boxes recorded on the simulator track."""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from clothoid_mission.mission_logic import (  # noqa: E402
    STOPPED_LIGHT, Inputs, MissionLogic, MissionParams, classes_seen)


def box(name, x0, y0, x1, y1):
    return SimpleNamespace(class_name=name, x_min=x0, y_min=y0, x_max=x1, y_max=y1)


def test_adjacent_red_does_not_arm_a_stop_at_the_bump_line():
    seen = classes_seen([box('traffic_light_red', 568, 209, 587, 258),
                         box('sign_bump', 209, 223, 237, 251)])
    logic = MissionLogic(MissionParams())
    out = logic.step(Inputs(now=0.0, seen=seen))
    assert seen == {'sign_bump'}
    assert out.speed_limit == 0.2 and not out.stop_arm
    assert not logic.step(Inputs(now=1.0, line_phase='arrived')).stop


def test_own_red_still_arms_and_stops():
    seen = classes_seen([box('traffic_light_red', 526, 200, 559, 277)])
    logic = MissionLogic(MissionParams())
    assert logic.step(Inputs(now=0.0, seen=seen)).stop_arm
    assert logic.step(Inputs(now=1.0, line_phase='arrived')).state == STOPPED_LIGHT


def test_only_an_accepted_green_releases_the_light_wait():
    logic = MissionLogic(MissionParams())
    logic.step(Inputs(now=0.0, seen=classes_seen([
        box('traffic_light_red', 526, 200, 559, 277)])))
    logic.step(Inputs(now=1.0, line_phase='arrived'))
    side_green = classes_seen([box('traffic_light_green', 568, 209, 587, 258)])
    assert logic.step(Inputs(now=2.0, seen=side_green)).stop
    own_green = classes_seen([box('traffic_light_green', 526, 200, 559, 277)])
    assert not logic.step(Inputs(now=3.0, seen=own_green)).stop


def test_small_central_light_is_allowed_and_camera_centre_is_used():
    light = box('traffic_light_red', 95, 200, 105, 210)
    assert classes_seen([light], camera_center_x=100.0) == {'traffic_light_red'}
    assert classes_seen([light], camera_center_x=319.9) == set()


def test_invalid_light_boxes_are_ignored_and_filter_can_be_disabled():
    assert classes_seen([box('traffic_light_red', 310, 200, 330, 200),
                         box('traffic_light_green', 320, 200, 320, 250)]) == set()
    assert classes_seen([box('traffic_light_red', 568, 209, 587, 258)],
                        light_max_offset_height_ratio=0.0) == {'traffic_light_red'}


def test_adjacent_parking_sign_does_not_arm_early_parking():
    seen = classes_seen([box('sign_parking', 425, 225, 434, 235),
                         box('sign_bump', 215, 223, 239, 248)])
    assert seen == {'sign_bump'}
    logic = MissionLogic(MissionParams())
    assert logic.step(Inputs(now=0.0, seen=seen)).speed_limit == .2
    assert not logic.step(Inputs(now=1.0, parking_line=True)).parking_flag


def test_own_signs_and_edge_fork_sign_are_retained():
    seen = classes_seen([box('sign_pedestrian', 208, 222, 236, 255),
                         box('sign_slow', 210, 222, 240, 255),
                         box('sign_parking', 365, 225, 375, 237),
                         box('sign_left', 2, 223, 34, 255)])
    assert seen == {'sign_pedestrian', 'sign_slow', 'sign_parking', 'sign_left'}


def test_sign_and_fork_limits_can_be_tuned_independently():
    parking = box('sign_parking', 425, 225, 434, 235)
    fork = box('sign_right', 2, 223, 34, 255)
    assert classes_seen([parking, fork], sign_max_offset_height_ratio=0.0,
                        fork_max_offset_height_ratio=5.0) == {'sign_parking'}
