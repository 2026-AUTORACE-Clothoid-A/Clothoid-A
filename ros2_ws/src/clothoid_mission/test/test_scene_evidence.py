from clothoid_mission.scene_evidence import FloorObservation, ParkingEntryEvidence
from clothoid_mission.mission_logic import Inputs, MissionLogic, MissionParams


def test_crosswalk_tail_cannot_become_an_entry_after_bars_leave_the_image():
    gate = ParkingEntryEvidence()
    gate.observe(0., 0., FloorObservation((.4,), ((.5, .9),)))
    for t in (.1, .2, .3):
        gate.observe(t, .4, FloorObservation((.5,)))  # same crosswalk's far line
    assert gate.entry_at is None
    assert not gate.reached(.8, .161)


def test_separate_entry_confirms_and_requires_actual_motion_to_the_line():
    gate = ParkingEntryEvidence()
    gate.observe(0., 0., FloorObservation((), ((.4, .8),)))
    for t, distance in ((.1, .5), (.2, .55), (.3, .6)):
        gate.observe(t, distance, FloorObservation((1.5 - distance,)))
    assert gate.entry_at == 1.5
    assert not gate.reached(.6, .161)
    assert gate.reached(1.31, .161)
    assert not gate.reached(5., .161)


def test_stale_duplicate_and_inconsistent_lines_do_not_confirm():
    gate = ParkingEntryEvidence()
    for t in (0., 0., .5, 1.):
        gate.observe(t, 0., FloorObservation((.8,)))
    assert gate.entry_at is None
    gate.observe(1.1, 0., FloorObservation((.5,)))
    gate.observe(1.2, 0., FloorObservation((.8,)))
    assert gate.entry_at is None


def test_parking_entry_limits_speed_progressively_until_line_evidence_expires():
    gate = ParkingEntryEvidence(max_travel=.5)
    cap = lambda travel: gate.approach_limit(travel, .16, .2, .20, 1.0)
    assert cap(0.) is None
    for t in (1., 1.1, 1.2):
        gate.observe(t, 0., FloorObservation((1.,)))
    assert cap(0.) > .8
    assert .2 < cap(.60) < .8
    assert cap(.80) == .2
    assert cap(.90) == .2
    assert cap(1.35) is None


def test_parking_needs_sign_entry_and_bay_but_no_previous_mission():
    logic = MissionLogic(MissionParams(require_cone_for_parking=False))
    out = logic.step(Inputs(0., seen={'sign_parking'}, parking_line=True))
    assert out.parking_prepare and not out.parking_flag
    assert not logic.step(Inputs(.1, parking_scene=True)).parking_flag
    assert logic.step(Inputs(.2, parking_scene=True, parking_line=True)).parking_flag


def test_old_parking_candidate_expires_instead_of_latching_forever():
    logic = MissionLogic(MissionParams(arm_timeout_sec=2.))
    logic.step(Inputs(0., seen={'sign_parking'}))
    out = logic.step(Inputs(3., parking_line=True, parking_scene=True))
    assert not out.parking_prepare and not out.parking_flag


def test_sign_alone_observes_without_reducing_speed_until_line_commit():
    logic = MissionLogic(MissionParams(require_cone_for_parking=False))
    out = logic.step(Inputs(0., seen={'sign_pedestrian'}))
    assert out.stop_arm and out.speed_limit is None and not out.stop
    assert logic.step(Inputs(.1, line_phase='search', line_speed=.2)).speed_limit is None
    assert logic.step(Inputs(.2, line_phase='commit', line_speed=.1)).speed_limit == .1


def test_fork_preference_does_not_require_success_of_other_missions():
    logic = MissionLogic(MissionParams(require_cone_for_parking=False))
    assert logic.step(Inputs(0., seen={'sign_left'})).direction == 0
    assert logic.step(Inputs(1.3, seen={'sign_right'})).direction == 1
    out = logic.step(Inputs(1.4, seen={'sign_parking'}, parking_scene=True, parking_line=True))
    assert out.parking_flag


def test_conflicting_fork_instructions_do_not_choose_an_arbitrary_side():
    logic = MissionLogic(MissionParams(require_cone_for_parking=False))
    assert logic.step(Inputs(0., seen={'sign_right'})).direction == 1
    assert logic.step(Inputs(.1, seen={'sign_left','sign_right'})).direction == 1


def test_either_warning_prepares_slow_traversal_and_imu_confirms_contact():
    for sign in ('sign_bump', 'sign_slow'):
        logic = MissionLogic(MissionParams(slow_arms_bump=True))
        out = logic.step(Inputs(0., seen={sign}, imu_pitch=0.))
        assert out.speed_limit == .2 and not out.steering_lock
        assert logic.step(Inputs(5., imu_pitch=0.)).speed_limit == .2
        assert logic.step(Inputs(6., imu_pitch=-5.)).steering_lock


def test_default_round_slow_warning_expires_without_arming_bump_traversal():
    logic = MissionLogic(MissionParams(require_cone_for_parking=False))
    out = logic.step(Inputs(0., seen={'sign_slow'}, imu_pitch=0.))
    assert out.speed_limit == .2 and logic.bump_state == 'idle'
    out = logic.step(Inputs(5., imu_pitch=-5.))
    assert out.speed_limit is None and not out.steering_lock
    assert logic.bump_state == 'idle'


def test_parking_does_not_take_control_while_waiting_at_a_red_signal():
    logic = MissionLogic(MissionParams(require_cone_for_parking=False))
    logic.state = 'stopped_light'
    out = logic.step(Inputs(0., seen={'sign_parking'}, parking_line=True, parking_scene=True))
    assert out.stop and not out.parking_flag


def test_disabled_parking_does_not_prepare_sensors_or_take_control():
    logic = MissionLogic(MissionParams(parking_enabled=False))
    out = logic.step(Inputs(0., seen={'sign_parking'}, parking_line=True, parking_scene=True))
    assert not out.parking_prepare and not out.parking_flag
