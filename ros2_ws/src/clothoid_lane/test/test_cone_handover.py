import pytest
from clothoid_lane.cone_handover import ConeHandover


def armed():
    h = ConeHandover()
    h.on_cone(True, 0.)
    h.on_cone(False, 1., travel=0.)
    return h


AUTO = object()


def frame(h, t, found=True, raw=(120., 120.), used=(120., 120.),
          aligned=True, travel=AUTO):
    if travel is AUTO:
        travel = max(0., .4*(t-1.))
    h.observe(t, found, raw, used, 8., .3, aligned=aligned, travel=travel)
    return h.speed(t, .8, .4, .3)


def test_no_exit_no_effect():
    h = ConeHandover()
    h.on_cone(False, 1., travel=0.)
    assert frame(h, 1.) == .8


def test_time_alone_never_releases():
    h = armed()
    for t in [1., 2., 100.]:
        assert frame(h, t, used=(20., 120.)) == .4
    assert h.recovering


def test_continuous_agreement_releases_with_ramp():
    h = armed()
    speeds = [frame(h, 1 + i * .05) for i in range(80)]
    assert speeds[:7] == [.4] * 7
    assert speeds == sorted(speeds)
    assert speeds[-1] == .8
    assert not h.recovering


@pytest.mark.parametrize('kwargs', [dict(found=False),
    dict(used=(0., 120.)), dict(used=None), dict(raw=(float('nan'), 120.))])
def test_loss_lag_lock_invalid_reset_evidence(kwargs):
    h = armed()
    for i in range(6):frame(h, 1 + i * .05)
    assert frame(h, 1.3, **kwargs) == .4
    assert frame(h, 1.35) == .4
    assert h.release_since is None


def test_ramp_requires_continued_observation():
    h = armed()
    for i in range(10):frame(h, 1 + i * .05)
    assert h.release_since is not None
    assert frame(h, 1.5, found=False) == .4
    assert h.release_since is None


def test_target_agreement_on_a_curve_does_not_release_speed():
    h = armed()
    for i in range(80):
        target=(80. + 6*i, 120.)
        speed=frame(h, 1+i*.05, raw=target, used=target, aligned=False)
    assert speed == .4


def test_frame_gap_and_clock_rewind_do_not_count_as_evidence():
    h = armed()
    frame(h, 1.)
    assert frame(h, 2.) == .4
    assert frame(h, .5) == .4
    assert h.release_since is None


def test_other_limits_and_new_cone_are_respected():
    h = armed()
    assert h.speed(1.1, .2, .4, .3) == .2
    assert h.speed(1.1, 0., .4, .3) == 0.
    h.on_cone(True, 1.2)
    assert h.speed(1.3, .8, .4, .3) == .8


def test_repeated_false_does_not_reset_confirmation():
    h = armed()
    for i in range(80):
        t=1+i*.05
        h.on_cone(False,t)
        speed=frame(h,t)
    assert speed == .8


def test_prepare_handover_while_cone_still_active():
    h=ConeHandover();h.on_cone(True,0.);h.request_exit(True)
    for i in range(80):
        t=1+i*.05
        h.on_cone(True,t)  # heartbeat must not erase lane evidence
        speed=frame(h,t)
    assert h.ready and speed==.4 and h.cone_active
    h.on_cone(False,2.)
    h.request_exit(False)
    for i in range(80):speed=frame(h,2+i*.05)
    assert speed==.8


@pytest.mark.parametrize('kwargs', [dict(aligned=False), dict(travel=None),
                                    dict(travel=0.), dict(travel=float('nan'))])
def test_time_and_lane_detection_cannot_replace_alignment_and_measured_travel(kwargs):
    h = armed()
    for i in range(300):
        assert frame(h, 1+i*.05, **kwargs) == .4
    assert h.recovering


def test_lane_handover_can_be_ready_before_acceleration_is_allowed():
    h = ConeHandover()
    h.on_cone(True, 0.)
    h.request_exit(True)
    for i in range(20):
        frame(h, 1+i*.05, aligned=False, travel=0.)
    assert h.ready
    h.on_cone(False, 2., travel=0.)
    for i in range(30):
        assert frame(h, 2+i*.05, travel=.3*i*.05) == .4


@pytest.mark.parametrize('failure', [dict(found=False), dict(aligned=False), dict(travel=None)])
def test_loss_or_misalignment_during_ramp_restores_cap(failure):
    h = armed()
    for i in range(45):
        speed = frame(h, 1+i*.05)
    assert .4 < speed < .8
    assert frame(h, 3.25, **failure) == .4
    # A single good frame does not resume a partly completed ramp.
    assert frame(h, 3.3) == .4


def test_odometry_reset_restarts_distance_and_alignment_checks():
    h = armed()
    for i in range(45):
        frame(h, 1+i*.05)
    assert frame(h, 3.25, travel=0.) == .4
    for i in range(20):
        assert frame(h, 3.3+i*.05, travel=.01*i) == .4
