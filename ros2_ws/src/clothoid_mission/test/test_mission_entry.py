import pytest

from clothoid_mission.scene_evidence import FloorObservation, MissionEntryEvidence
from clothoid_mission.mission_logic import Inputs, MissionLogic, MissionParams


def confirmed_entry():
    gate = MissionEntryEvidence()
    for t, distance in ((0., 0.), (.1, .08), (.2, .16)):
        gate.observe(t, distance, FloorObservation((1.-distance,)))
    return gate


def test_start_boundary_remains_distinct_from_later_crosswalk_stop_and_tail():
    gate = confirmed_entry()
    assert gate.entry_at == pytest.approx(1.)
    gate.observe(.3, .24, FloorObservation((1.8-.24,), ((1.9-.24, 2.3-.24),)))
    assert gate.entry_at == pytest.approx(1.)
    assert not gate.reached(.89, .1)
    assert gate.reached(.9, .1)


def test_crosswalk_stop_and_far_boundary_cannot_become_mission_entry():
    gate = MissionEntryEvidence()
    for t in (0., .1, .2):
        gate.observe(t, 0., FloorObservation((.5, 1.), ((.6, .95),)))
    assert gate.entry_at is None
    # Bars disappeared below the camera, but their tail line is still not an entry.
    for t in (.3, .4, .5):
        gate.observe(t, .6, FloorObservation((.4,)))
    assert gate.entry_at is None


def test_crosswalk_context_can_reject_an_early_unclassified_stop_line():
    gate = confirmed_entry()
    gate.observe(.3, .2, FloorObservation((.8,), ((.9, 1.2),)))
    assert gate.entry_at is None
    assert not gate.should_slow(.8, .1, .8, .2, .25, 1.5)


def test_duplicate_stale_and_inconsistent_lines_cannot_start_entry():
    gate = MissionEntryEvidence()
    for t in (0., 0., .5, 1.):
        gate.observe(t, 0., FloorObservation((1.,)))
    assert gate.entry_at is None
    gate.observe(1.1, 0., FloorObservation((.6,)))
    gate.observe(1.2, 0., FloorObservation((1.,)))
    assert gate.entry_at is None


def test_braking_lead_uses_measured_speed_and_does_not_oscillate_after_slowing():
    fast, slow = confirmed_entry(), confirmed_entry()
    assert not fast.should_slow(.2, .1, .8, .2, .25, 1.5)
    assert fast.should_slow(.5, .1, .8, .2, .25, 1.5)
    assert not fast.reached(.5, .1)
    assert not slow.should_slow(.5, .1, .2, .2, .25, 1.5)
    assert fast.should_slow(.55, .1, .2, .2, .25, 1.5)


def test_missing_speed_does_not_postpone_braking_at_a_confirmed_boundary():
    assert confirmed_entry().should_slow(.3, .1, None, .2, .25, 1.5)


@pytest.mark.parametrize('sign', ['sign_pedestrian', 'traffic_light_red'])
def test_mission_entry_prepares_without_slowing_or_completing_a_crosswalk_stop(sign):
    logic = MissionLogic(MissionParams())
    out = logic.step(Inputs(0., seen={sign}))
    assert out.entry_arm and out.stop_arm and out.speed_limit is None
    out = logic.step(Inputs(.1, entry_slow=True, entry_reached=True))
    assert out.speed_limit is None and not out.stop and out.stop_arm
    out = logic.step(Inputs(.2, entry_slow=True, line_phase='commit', line_speed=.08))
    assert out.speed_limit == .08 and not out.stop
    assert logic.step(Inputs(.3, line_phase='arrived')).stop


def test_entry_evidence_without_a_mission_candidate_has_no_effect():
    out = MissionLogic(MissionParams()).step(Inputs(0., entry_slow=True, entry_reached=True))
    assert out.speed_limit is None and not out.stop and not out.entry_arm


def test_slow_hold_starts_at_front_wheel_entry_after_approach_braking():
    logic = MissionLogic(MissionParams(slow_at_stopline=True, slow_hold_sec=4.))
    out = logic.step(Inputs(0., seen={'sign_slow'}))
    assert out.entry_arm and not out.stop_arm and out.speed_limit is None
    out = logic.step(Inputs(1., entry_slow=True))
    assert out.speed_limit == .2 and out.entry_arm
    logic.step(Inputs(3., entry_slow=True, entry_reached=True))
    assert logic.step(Inputs(6.9)).speed_limit == .2
    assert logic.step(Inputs(7.1)).speed_limit is None
