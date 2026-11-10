"""Scenario tests for the mission state logic: `pytest ros2_ws/src/clothoid_mission/test`."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from clothoid_mission.mission_logic import (  # noqa: E402
    DRIVE, STOPPED_LIGHT, STOPPED_PEDESTRIAN, Inputs, MissionLogic, MissionParams)


def run(logic, now, *seen, stopline=False, line=False, phase=None, speed=None):
    return logic.step(Inputs(now=now, seen=set(seen), stopline=stopline, parking_line=line,
                             line_phase=phase, line_speed=speed))


def flag_params(**kw):
    """2025 behaviour: stop on the stripe flag (the scenarios below were written for it)."""
    return MissionParams(use_stop_line=False, **kw)


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
    assert out.stop_arm and not out.stop and out.speed_limit == 0.2   # until stop_line answers
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
