"""Scenario tests for the speed bump logic: `pytest ros2_ws/src/clothoid_mission/test`."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from clothoid_mission.speed_bump import (  # noqa: E402
    IDLE, ON_BUMP, PASSED, SIGN, SLOW, SpeedBump, SpeedBumpParams, quaternion_to_pitch_deg)

LEVEL = -0.4     # the limo2 IMU reads about -0.4 deg on a flat floor


def pitch_run(bump, pitches, t0, dt=0.01):
    """Feed one pitch sample every `dt` s (the IMU runs at 100 Hz). Returns (end time, states)."""
    states = []
    for i, p in enumerate(pitches):
        states.append(bump.on_pitch(p, t0 + i * dt))
    return t0 + len(pitches) * dt, states


def see_sign(bump, t, n=2):
    for i in range(n):
        bump.on_detections([(SIGN, 0.9)], t + i * 0.1)
    return t + n * 0.1


def bump_profile():
    """Pitch over a bump: front up, level on top, rear up (nose down), level."""
    return ([LEVEL + 5.0] * 40 + [LEVEL] * 30 + [LEVEL - 5.0] * 40 + [LEVEL] * 50)


def fresh():
    bump = SpeedBump(SpeedBumpParams())
    t, _ = pitch_run(bump, [LEVEL] * 300, 0.0)     # 3 s of flat floor: baseline settles
    return bump, t


def test_quaternion_pitch():
    assert abs(quaternion_to_pitch_deg(0.0, 0.0, 0.0, 1.0)) < 1e-9
    # 10 deg about y: (0, sin 5deg, 0, cos 5deg)
    assert abs(quaternion_to_pitch_deg(0.0, 0.0871557, 0.0, 0.9961947) - 10.0) < 0.01


def test_no_sign_no_limit_even_on_a_bump():
    bump, t = fresh()
    _, states = pitch_run(bump, bump_profile(), t)
    assert set(states) == {IDLE} and bump.limit is None


def test_sign_slows_then_whole_bump_releases():
    bump, t = fresh()
    t = see_sign(bump, t)
    assert bump.state == SLOW and bump.limit == 0.2
    t, _ = pitch_run(bump, [LEVEL] * 200, t)              # 2 s driving to the bump, still slow
    assert bump.state == SLOW
    t, states = pitch_run(bump, bump_profile(), t)
    assert ON_BUMP in states
    assert bump.state == PASSED and bump.limit is None


def test_level_on_top_of_the_bump_does_not_release():
    # front wheels down again: level, but the rear wheels are still before the bump
    bump, t = fresh()
    t = see_sign(bump, t)
    t, _ = pitch_run(bump, [LEVEL + 5.0] * 40 + [LEVEL] * 100, t)   # 1 s level on top
    assert bump.state == ON_BUMP and bump.limit == 0.2


def test_one_sign_frame_is_not_enough():
    bump, t = fresh()
    bump.on_detections([(SIGN, 0.9)], t)
    bump.on_detections([], t + 0.1)
    bump.on_detections([(SIGN, 0.9)], t + 0.2)
    assert bump.state == IDLE


def test_unsure_sign_and_other_classes_are_ignored():
    bump, t = fresh()
    for i in range(5):
        bump.on_detections([(SIGN, 0.3), ('sign_slow', 0.99)], t + i * 0.1)
    assert bump.state == IDLE


def test_no_bump_after_the_sign_gives_up():
    bump, t = fresh()
    t = see_sign(bump, t)
    assert bump.update(t + 9.0) == SLOW
    assert bump.update(t + 10.5) == IDLE and bump.limit is None


def test_stuck_in_on_bump_counts_as_passed():
    bump, t = fresh()
    t = see_sign(bump, t)
    t, _ = pitch_run(bump, [LEVEL + 5.0] * 10, t)
    assert bump.state == ON_BUMP
    assert bump.update(t + 5.1) == PASSED


def test_cooldown_then_ready_again():
    bump, t = fresh()
    t = see_sign(bump, t)
    t, _ = pitch_run(bump, bump_profile(), t)
    assert bump.state == PASSED
    see_sign(bump, t + 1.0)                     # still in cooldown
    assert bump.state == PASSED
    assert bump.update(t + 3.1) == IDLE
    see_sign(bump, t + 4.0)
    assert bump.state == SLOW


def test_baseline_follows_the_floor_but_not_the_bump():
    bump = SpeedBump(SpeedBumpParams())
    pitch_run(bump, [1.5] * 500, 0.0)          # a car whose IMU reads +1.5 on the flat
    assert abs(bump.baseline - 1.5) < 0.05
    t = see_sign(bump, 5.0)
    t, _ = pitch_run(bump, [1.5 + 5.0] * 40, t)
    assert bump.state == ON_BUMP and abs(bump.baseline - 1.5) < 0.05   # frozen while slowing
