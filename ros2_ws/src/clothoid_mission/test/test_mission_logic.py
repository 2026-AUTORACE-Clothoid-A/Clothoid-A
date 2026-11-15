"""Scenario tests for the mission state logic: `pytest ros2_ws/src/clothoid_mission/test`."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from clothoid_mission.mission_logic import (  # noqa: E402
    DRIVE, STOPPED_LIGHT, STOPPED_PEDESTRIAN, Inputs, MissionLogic, MissionParams)


def run(logic, now, *seen, stopline=False, line=False, phase=None, speed=None):
    return logic.step(Inputs(now=now, seen=set(seen), stopline=stopline, parking_line=line,
                             line_phase=phase, line_speed=speed))


def flag_params(**kw):
    """2025 behaviour: stop on the stripe flag (the scenarios below were written for it)."""
    kw.setdefault('require_cone_for_parking', False)
    return MissionParams(use_stop_line=False, require_scene_confirmation=False, slow_arms_bump=False, **kw)


def test_idle_drive_has_no_outputs():
    out = run(MissionLogic(flag_params()), 0.0)
    assert out.state == DRIVE and not out.stop
    assert out.speed_limit is None and out.direction is None and not out.parking_flag


def test_slow_sign_limits_speed_then_expires():
    logic = MissionLogic(flag_params(slow_speed=0.2, slow_hold_sec=4.0))
    assert run(logic, 0.0, 'sign_slow').speed_limit == 0.2
    assert run(logic, 3.9).speed_limit == 0.2          # sign out of view, still inside the hold
    assert run(logic, 4.1).speed_limit is None
    assert run(logic, 5.0, 'sign_slow').speed_limit == 0.2   # seen again


def test_pedestrian_sign_stops_at_stop_line_waits_and_goes():
    logic = MissionLogic(flag_params(pedestrian_wait_sec=3.5))
    assert not run(logic, 0.0, 'sign_pedestrian').stop        # armed, no stop line yet
    assert not run(logic, 1.0).stop
    out = run(logic, 2.0, stopline=True)
    assert out.stop and out.state == STOPPED_PEDESTRIAN
    assert run(logic, 5.0).stop                                # still waiting (3.0 s)
    assert not run(logic, 5.6).stop                            # 3.6 s: go
    assert run(logic, 5.7).state == DRIVE


def test_stop_line_without_a_sign_does_not_stop():
    assert not run(MissionLogic(flag_params()), 0.0, stopline=True).stop


def test_cooldown_prevents_a_second_stop_for_the_same_mission():
    logic = MissionLogic(flag_params(pedestrian_wait_sec=1.0, cooldown_sec=15.0))
    run(logic, 0.0, 'sign_pedestrian', stopline=True)
    run(logic, 1.5)                                            # released at 1.5
    assert not run(logic, 3.0, 'sign_pedestrian', stopline=True).stop
    assert run(logic, 17.0, 'sign_pedestrian', stopline=True).stop   # cooldown over


def test_arm_expires_without_a_stop_line():
    logic = MissionLogic(flag_params(arm_timeout_sec=8.0))
    run(logic, 0.0, 'sign_pedestrian')
    assert not run(logic, 9.0, stopline=True).stop


def test_red_light_holds_until_green():
    logic = MissionLogic(flag_params())
    run(logic, 0.0, 'traffic_light_red')
    out = run(logic, 1.0, 'traffic_light_red', stopline=True)
    assert out.stop and out.state == STOPPED_LIGHT
    assert run(logic, 20.0, 'traffic_light_red').stop          # red: no timeout
    assert not run(logic, 21.0, 'traffic_light_green').stop


def test_green_light_on_approach_does_not_stop():
    logic = MissionLogic(flag_params())
    run(logic, 0.0, 'traffic_light_red')
    run(logic, 0.5, 'traffic_light_green')                     # turned green before the line
    assert not run(logic, 1.0, stopline=True).stop


def test_yellow_light_follows_the_stop_on_yellow_switch():
    stops = MissionLogic(flag_params(stop_on_yellow=True))
    run(stops, 0.0, 'traffic_light_yellow')
    assert run(stops, 1.0, stopline=True).stop

    passes = MissionLogic(flag_params(stop_on_yellow=False))
    run(passes, 0.0, 'traffic_light_yellow')
    assert not run(passes, 1.0, stopline=True).stop


def test_light_takes_priority_over_pedestrian_when_both_armed():
    logic = MissionLogic(flag_params())
    run(logic, 0.0, 'sign_pedestrian', 'traffic_light_red')
    assert run(logic, 1.0, stopline=True).state == STOPPED_LIGHT


def test_fork_signs_set_the_direction_for_lane_main():
    logic = MissionLogic(flag_params(fork_left_direction=0, fork_right_direction=1))
    assert run(logic, 0.0, 'sign_left').direction == 0
    assert run(logic, 1.0).direction == 0                      # latched after the sign leaves view
    assert run(logic, 2.0, 'sign_right').direction == 1


def test_parking_sign_arms_and_the_stop_line_raises_the_flag():
    logic = MissionLogic(flag_params())
    assert not run(logic, 0.0, 'sign_parking').parking_flag    # sign seen from afar: armed only
    assert not run(logic, 2.0).parking_flag                      # sign out of view, still armed
    assert not run(logic, 2.5, stopline=True).parking_flag       # crosswalk stripes: not this one
    assert run(logic, 3.0, line=True).parking_flag               # the single stop line in front
    assert run(logic, 10.0).parking_flag                         # latched


def test_stop_line_without_the_sign_does_not_park():
    logic = MissionLogic(flag_params())
    assert not run(logic, 0.0, line=True).parking_flag


def test_parking_at_the_sign_when_asked():
    logic = MissionLogic(flag_params(parking_at_stopline=False))
    assert run(logic, 0.0, 'sign_parking').parking_flag


# ---- stop on clothoid_finish stop_line (use_stop_line, the default) ----

def test_sign_arms_stop_line_and_its_arrival_holds_the_car():
    logic = MissionLogic(MissionParams(pedestrian_wait_sec=3.5))
    out = run(logic, 0.0, 'sign_pedestrian')
    assert out.stop_arm and not out.stop and out.speed_limit is None  # observe before committing the line
    assert run(logic, 1.0, phase='search', speed=0.2).stop_arm
    out = run(logic, 2.0, phase='commit', speed=0.09)
    assert out.speed_limit == 0.09 and not out.stop                    # finish's ramp
    out = run(logic, 3.0, phase='arrived', speed=0.0)
    assert out.stop and out.state == STOPPED_PEDESTRIAN and not out.stop_arm
    assert run(logic, 6.0).stop
    out = run(logic, 6.6)
    assert not out.stop and not out.stop_arm and out.speed_limit is None


def test_stripes_alone_do_not_stop_with_stop_line():
    logic = MissionLogic(MissionParams())
    run(logic, 0.0, 'sign_pedestrian')
    assert not run(logic, 1.0, stopline=True, phase='search').stop


def test_committed_line_keeps_the_arm_past_the_timeout():
    logic = MissionLogic(MissionParams(arm_timeout_sec=8.0))
    run(logic, 0.0, 'traffic_light_red')
    assert run(logic, 9.0, phase='commit', speed=0.06).stop_arm
    assert run(logic, 12.0, phase='arrived').state == STOPPED_LIGHT
    assert not run(logic, 13.0, 'traffic_light_green').stop


def test_search_without_a_line_expires():
    logic = MissionLogic(MissionParams(arm_timeout_sec=8.0))
    run(logic, 0.0, 'sign_pedestrian')
    assert not run(logic, 9.0, phase='search').stop_arm


def test_cone_gate_blocks_parking_before_cone_and_unlocks_after():
    logic = MissionLogic(MissionParams(parking_at_stopline=False, cone_confirm_sec=1.5,
                                       require_scene_confirmation=False))
    # Seen near the bump / start before cone course: ignored
    assert not logic.step(Inputs(now=0.0, seen={'sign_parking'})).parking_flag
    assert not logic.parking_armed
    # Brief cone_active (< 1.5 s) does not latch, and dropping to False resets the timer
    logic.step(Inputs(now=1.0, cone_active=True))
    logic.step(Inputs(now=2.0, cone_active=True))
    assert not logic.cone_entered
    logic.step(Inputs(now=2.1, cone_active=False))
    assert not logic.cone_entered
    # Sustained cone_active >= 1.5 s latches cone_entered, and it stays unlocked after leaving
    logic.step(Inputs(now=3.0, cone_active=True))
    assert not logic.cone_entered
    logic.step(Inputs(now=4.5, cone_active=True))
    assert logic.cone_entered
    assert not logic.step(Inputs(now=5.0, cone_active=False)).parking_flag
    assert logic.step(Inputs(now=6.0, seen={'sign_parking'}, cone_active=False)).parking_flag


def test_cone_gate_blocks_bump_after_cone_entry():
    logic = MissionLogic(MissionParams(slow_speed=0.2, cone_confirm_sec=1.5))
    # Before cone: bump arms normally; brief cone_active (< 1.5 s) does not block bump
    assert logic.step(Inputs(now=0.0, seen={'sign_bump'})).speed_limit == 0.2
    assert logic.step(Inputs(now=1.0, cone_active=True)).speed_limit == 0.2
    # Once cone_active holds >= 1.5 s, cone_entered latches and clears/blocks bump
    assert logic.step(Inputs(now=2.5, cone_active=True)).speed_limit is None
    assert logic.cone_entered
    # After cone: sign_bump is ignored even after cone_active returns False
    assert logic.step(Inputs(now=3.0, seen={'sign_bump'}, cone_active=False)).speed_limit is None


def test_cone_gate_keeps_scene_handover_checks_after_unlock():
    logic = MissionLogic(MissionParams())
    evidence = dict(seen={'sign_parking'}, parking_line=True, parking_scene=True)
    assert not logic.step(Inputs(now=0., **evidence)).parking_prepare
    assert not logic.parking_flag
    logic.step(Inputs(now=1., cone_active=True))
    logic.step(Inputs(now=2.5, cone_active=True))
    out = logic.step(Inputs(now=2.6, seen={'sign_parking'}))
    assert out.parking_prepare and not out.parking_flag
    assert not logic.step(Inputs(now=2.7, parking_line=True)).parking_flag
    assert logic.step(Inputs(now=2.8, parking_line=True, parking_scene=True)).parking_flag


def test_cone_active_during_and_right_after_avoidance_does_not_latch():
    """1114_0028: a false cone start during the static avoidance latched cone_entered."""
    logic = MissionLogic(MissionParams(cone_confirm_sec=1.5, avoid_release_hold=2.0))
    for t in (0.0, 1.0, 2.0, 3.0):
        logic.step(Inputs(now=t, cone_active=True, avoid_active=True))
    assert not logic.cone_entered
    # avoidance ended at 3.0: cone_active up to 5.0 is still ignored
    logic.step(Inputs(now=4.0, cone_active=True))
    logic.step(Inputs(now=4.9, cone_active=True))
    assert not logic.cone_entered
    # the real cone course afterwards latches as before (1.5 s from 5.0)
    logic.step(Inputs(now=5.0, cone_active=True))
    logic.step(Inputs(now=6.0, cone_active=True))
    assert not logic.cone_entered
    logic.step(Inputs(now=6.5, cone_active=True))
    assert logic.cone_entered


def test_parking_uses_line_stop_without_crosswalk_and_without_slowing():
    logic = MissionLogic(MissionParams(parking_at_stopline=True, require_cone_for_parking=False,
                                       require_scene_confirmation=False))
    out = run(logic, 0.0, 'sign_parking')
    assert out.stop_arm and not out.stop_require_crosswalk
    assert not out.parking_flag and out.speed_limit is None
    out = run(logic, 1.0, phase='commit', speed=0.1)
    assert out.stop_arm and not out.parking_flag and out.speed_limit is None
    out = run(logic, 2.0, phase='arrived', speed=0.0)
    assert out.parking_flag and not out.stop and not out.stop_arm


@pytest.mark.parametrize('sign', ['sign_slow', 'sign_bump'])
def test_mission_stopline_arrival_does_not_bypass_parking_scene_confirmation(sign):
    logic = MissionLogic(MissionParams(require_cone_for_parking=False,
                                       slow_at_stopline=True, bump_at_stopline=True))
    out = logic.step(Inputs(now=0.0, seen={'sign_parking', sign}))
    assert out.parking_prepare and out.stop_arm and not out.parking_flag
    # A shared line arrival can start slowing, but cannot replace parking's separate entry.
    out = logic.step(Inputs(now=1.0, line_phase='arrived', parking_scene=True))
    assert out.speed_limit == 0.2 and not out.parking_flag
    assert out.parking_prepare and not out.stop_arm
    out = logic.step(Inputs(now=1.1, parking_line=True, parking_scene=True))
    assert out.parking_flag


def test_slow_at_stopline_waits_for_arrival_and_expires_without_line():
    logic = MissionLogic(MissionParams(slow_at_stopline=True, slow_speed=0.2, slow_hold_sec=4.0,
                                       arm_timeout_sec=8.0))
    out = run(logic, 0.0, 'sign_slow')
    assert out.stop_arm and not out.stop_require_crosswalk and out.speed_limit is None
    out = run(logic, 1.5, phase='commit', speed=0.1)
    assert out.stop_arm and out.speed_limit is None
    out = run(logic, 2.0, phase='arrived', speed=0.0)
    assert out.speed_limit == 0.2 and not out.stop_arm
    assert run(logic, 5.9).speed_limit == 0.2
    assert run(logic, 6.1).speed_limit is None

    # Timeout without seeing a stop line
    assert run(logic, 7.0, 'sign_slow').stop_arm
    assert not run(logic, 15.5, phase='search').stop_arm
    assert run(logic, 15.6, phase='arrived').speed_limit is None


def test_bump_at_stopline_waits_for_line_or_pitch_fallback():
    from clothoid_mission.mission_logic import BUMP_IDLE, BUMP_APPROACHING, BUMP_ON
    logic = MissionLogic(MissionParams(bump_at_stopline=True, slow_speed=0.2,
                                       bump_pitch_thresh=3.5, bump_arm_timeout_sec=8.0))
    out = logic.step(Inputs(now=0.0, seen={'sign_bump'}, imu_pitch=0.0))
    assert logic.bump_state == BUMP_IDLE and out.stop_arm and not out.stop_require_crosswalk
    assert out.speed_limit is None

    out = logic.step(Inputs(now=1.5, line_phase='arrived', imu_pitch=0.0))
    assert logic.bump_state == BUMP_APPROACHING and out.speed_limit == 0.2 and not out.stop_arm

    # Fallback: if stop line is missed but vehicle hits bump, IMU pitch still enters BUMP_ON
    logic2 = MissionLogic(MissionParams(bump_at_stopline=True, slow_speed=0.2, bump_pitch_thresh=3.5))
    logic2.step(Inputs(now=0.0, seen={'sign_bump'}, imu_pitch=0.0))
    out2 = logic2.step(Inputs(now=0.8, line_phase='search', imu_pitch=4.2))
    assert logic2.bump_state == BUMP_ON and out2.speed_limit == 0.2 and out2.steering_lock


@pytest.mark.parametrize('sign, timeout', [('sign_slow', 8.0), ('sign_bump', 12.0)])
def test_visible_sign_refreshes_stopline_arm_until_the_last_sighting_expires(sign, timeout):
    logic = MissionLogic(MissionParams(slow_at_stopline=True, bump_at_stopline=True,
                                       arm_timeout_sec=timeout, bump_arm_timeout_sec=timeout))
    assert run(logic, 0.0, sign).stop_arm
    assert run(logic, timeout - 0.1, sign, phase='search').stop_arm
    # The first sighting is stale, but the sign was just confirmed again.
    out = run(logic, timeout + 0.1, phase='search')
    assert out.stop_arm and out.speed_limit is None
    # With no further sightings or committed line, the arm still expires.
    out = run(logic, 2 * timeout, phase='search')
    assert not out.stop_arm and out.speed_limit is None
    assert run(logic, 2 * timeout + 0.1, phase='arrived').speed_limit is None
