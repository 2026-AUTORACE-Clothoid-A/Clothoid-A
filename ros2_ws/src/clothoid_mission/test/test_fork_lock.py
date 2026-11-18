"""fork_lock_after_avoid (1118 PM-5): left/right boxes past the static obstacle section are misdetections."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from clothoid_mission.mission_logic import Inputs, MissionLogic, MissionParams  # noqa: E402


def logic():
    return MissionLogic(MissionParams(fork_lock_after_avoid=True, avoid_release_hold=2.))


def test_direction_is_kept_after_the_avoidance_ended():
    m = logic()
    m.step(Inputs(now=0., avoid_active=True))
    assert m.step(Inputs(now=1., avoid_active=True, seen={'sign_right'})).direction == 1
    assert m.step(Inputs(now=1.3, seen={'sign_left'})).direction == 0     # a 0.3 s gap in /avoid/active
    m.step(Inputs(now=1.4, avoid_active=True, seen={'sign_right'}))
    m.step(Inputs(now=2.0))                                             # ended 0.6 s ago: locks on 1
    assert m.fork_locked
    assert m.step(Inputs(now=6., seen={'sign_left'})).direction == 1


def test_no_lock_before_the_avoidance_nor_without_a_direction():
    m = logic()
    assert m.step(Inputs(now=0., seen={'sign_left'})).direction == 0      # never active: no lock
    assert m.step(Inputs(now=1., seen={'sign_right'})).direction == 1
    m = logic()
    m.step(Inputs(now=0., avoid_active=True))
    m.step(Inputs(now=5.))                                              # ended, no direction yet
    assert not m.fork_locked
    assert m.step(Inputs(now=6., seen={'sign_left'})).direction == 0


def test_off_by_default():
    m = MissionLogic(MissionParams())
    m.step(Inputs(now=0., avoid_active=True, seen={'sign_right'}))
    m.step(Inputs(now=5.))
    assert m.step(Inputs(now=6., seen={'sign_left'})).direction == 0
