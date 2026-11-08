"""Scenario tests for the mission state logic: `pytest ros2_ws/src/clothoid_mission/test`."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from clothoid_mission.mission_logic import (  # noqa: E402
    DRIVE, STOPPED_LIGHT, STOPPED_PEDESTRIAN, Inputs, MissionLogic, MissionParams)


def run(logic, now, *seen, stopline=False):
    return logic.step(Inputs(now=now, seen=set(seen), stopline=stopline))


def test_idle_drive_has_no_outputs():
    out = run(MissionLogic(MissionParams()), 0.0)
    assert out.state == DRIVE and not out.stop
    assert out.speed_limit is None and out.direction is None and not out.parking_flag


def test_slow_sign_limits_speed_then_expires():
    logic = MissionLogic(MissionParams(slow_speed=0.2, slow_hold_sec=4.0))
    assert run(logic, 0.0, 'sign_slow').speed_limit == 0.2
    assert run(logic, 3.9).speed_limit == 0.2          # sign out of view, still inside the hold
    assert run(logic, 4.1).speed_limit is None
    assert run(logic, 5.0, 'sign_slow').speed_limit == 0.2   # seen again


def test_pedestrian_sign_stops_at_stop_line_waits_and_goes():
    logic = MissionLogic(MissionParams(pedestrian_wait_sec=3.5))
    assert not run(logic, 0.0, 'sign_pedestrian').stop        # armed, no stop line yet
    assert not run(logic, 1.0).stop
    out = run(logic, 2.0, stopline=True)
    assert out.stop and out.state == STOPPED_PEDESTRIAN
    assert run(logic, 5.0).stop                                # still waiting (3.0 s)
    assert not run(logic, 5.6).stop                            # 3.6 s: go
    assert run(logic, 5.7).state == DRIVE


def test_stop_line_without_a_sign_does_not_stop():
    assert not run(MissionLogic(MissionParams()), 0.0, stopline=True).stop


def test_cooldown_prevents_a_second_stop_for_the_same_mission():
    logic = MissionLogic(MissionParams(pedestrian_wait_sec=1.0, cooldown_sec=15.0))
    run(logic, 0.0, 'sign_pedestrian', stopline=True)
    run(logic, 1.5)                                            # released at 1.5
    assert not run(logic, 3.0, 'sign_pedestrian', stopline=True).stop
    assert run(logic, 17.0, 'sign_pedestrian', stopline=True).stop   # cooldown over


def test_arm_expires_without_a_stop_line():
    logic = MissionLogic(MissionParams(arm_timeout_sec=8.0))
    run(logic, 0.0, 'sign_pedestrian')
    assert not run(logic, 9.0, stopline=True).stop


def test_red_light_holds_until_green():
    logic = MissionLogic(MissionParams())
    run(logic, 0.0, 'traffic_light_red')
    out = run(logic, 1.0, 'traffic_light_red', stopline=True)
    assert out.stop and out.state == STOPPED_LIGHT
    assert run(logic, 20.0, 'traffic_light_red').stop          # red: no timeout
    assert not run(logic, 21.0, 'traffic_light_green').stop


def test_green_light_on_approach_does_not_stop():
    logic = MissionLogic(MissionParams())
    run(logic, 0.0, 'traffic_light_red')
    run(logic, 0.5, 'traffic_light_green')                     # turned green before the line
    assert not run(logic, 1.0, stopline=True).stop


def test_yellow_light_follows_the_stop_on_yellow_switch():
    stops = MissionLogic(MissionParams(stop_on_yellow=True))
    run(stops, 0.0, 'traffic_light_yellow')
    assert run(stops, 1.0, stopline=True).stop

    passes = MissionLogic(MissionParams(stop_on_yellow=False))
    run(passes, 0.0, 'traffic_light_yellow')
    assert not run(passes, 1.0, stopline=True).stop


def test_light_takes_priority_over_pedestrian_when_both_armed():
    logic = MissionLogic(MissionParams())
    run(logic, 0.0, 'sign_pedestrian', 'traffic_light_red')
    assert run(logic, 1.0, stopline=True).state == STOPPED_LIGHT


def test_fork_signs_set_the_direction_for_lane_main():
    logic = MissionLogic(MissionParams(fork_left_direction=0, fork_right_direction=1))
    assert run(logic, 0.0, 'sign_left').direction == 0
    assert run(logic, 1.0).direction == 0                      # latched after the sign leaves view
    assert run(logic, 2.0, 'sign_right').direction == 1


def test_parking_sign_arms_and_the_stop_line_raises_the_flag():
    logic = MissionLogic(MissionParams())
    assert not run(logic, 0.0, 'sign_parking').parking_flag    # sign seen from afar: armed only
    assert not run(logic, 2.0).parking_flag                      # sign out of view, still armed
    assert run(logic, 3.0, stopline=True).parking_flag           # stop line before the bays
    assert run(logic, 10.0).parking_flag                         # latched


def test_stop_line_without_the_sign_does_not_park():
    logic = MissionLogic(MissionParams())
    assert not run(logic, 0.0, stopline=True).parking_flag


def test_parking_at_the_sign_when_asked():
    logic = MissionLogic(MissionParams(parking_at_stopline=False))
    assert run(logic, 0.0, 'sign_parking').parking_flag
