import math
import pytest
from clothoid_parking.approach import ParkingApproach, ApproachParams
from clothoid_parking.layout import ParkingLayout, LayoutParams
from clothoid_parking.planner import PlanParams
from clothoid_parking.tracker import Pose2D


def fixture():
    layout=ParkingLayout(LayoutParams(control_enabled=True,target_bay=2))
    for t in (1.,1.2,1.4):layout.observe(t,(1.,-.25),0.,Pose2D())
    layout.last_scan=1.5;layout.support_count=3
    # This fixture isolates the approach timing policy; occupied/unknown are tested below.
    layout.motion_clear=lambda *args:True
    return ParkingApproach(),layout,Pose2D(),PlanParams()


def update(a,now,preparing,active,l,pose,p,**kwargs):
    if 'entry_distance' not in kwargs:
        kwargs['entry_distance'] = None if l.origin is None else (l.origin[0]-pose.x)*math.cos(l.yaw)+(l.origin[1]-pose.y)*math.sin(l.yaw)-a.p.entry_to_grid
    return a.update(now,preparing,active,l,pose,p,**kwargs)


def test_unrelated_crosswalk_grid_does_not_slow_lane():
    a,l,pose,p=fixture()
    assert update(a,1.5,True,False,l,pose,p,entry_distance=None) is None
    assert update(a,1.5,True,False,l,pose,p,entry_distance=2.) is None
    assert not a.observing


def test_sign_alone_or_unconfirmed_layout_does_not_slow_lane():
    a,l,pose,p=fixture();l.support_count=0
    assert update(a,1.5,True,False,l,pose,p) is None
    assert not a.observing


def test_registered_layout_uses_turn_braking_before_the_line_and_slow_speed_after():
    a,l,pose,p=fixture()
    assert l.selected is None
    before_line = update(a,1.5,True,False,l,pose,p)
    assert before_line > a.p.speed and a.observing
    pose.x = .40  # bumper has reached the observed entry line
    assert update(a,1.55,True,False,l,pose,p) == a.p.speed


def test_limit_brakes_before_turn_then_skip_is_bounded():
    a,l,pose,p=fixture();b=l.bays()[1]
    pose.x=b.face_x+p.rear_offset-p.turn_radius-p.turn_lead-.10
    cap=update(a,1.5,True,False,l,pose,p)
    assert 0<cap<.2
    pose.x+=.03
    assert update(a,1.55,True,False,l,pose,p)==0
    l.last_scan=4.6
    assert update(a,4.6,True,False,l,pose,p) is None and a.skipped
    assert update(a,4.7,True,False,l,pose,p) is None


def test_missing_registration_during_approach_holds_before_release():
    a,l,pose,p=fixture();update(a,1.5,True,False,l,pose,p)
    l.origin=None
    assert update(a,1.6,True,False,l,pose,p)==0
    assert update(a,4.7,True,False,l,pose,p) is None and a.skipped


def test_skip_and_limit_reset_after_prepare_ends_not_fake_done():
    a,l,pose,p=fixture();a.skipped=True
    assert update(a,2.,False,False,l,pose,p) is None
    assert not a.skipped and not a.observing


def test_manoeuvre_handover_releases_lane_limit():
    a,l,pose,p=fixture();update(a,1.5,True,False,l,pose,p)
    assert update(a,1.6,True,True,l,pose,p) is None and not a.observing


def test_already_passed_turn_does_not_request_reverse_or_turn():
    a,l,pose,p=fixture();pose.x=3.
    assert update(a,1.5,True,False,l,pose,p) is None and a.skipped


def test_invalid_config_is_rejected():
    with pytest.raises(ValueError):ParkingApproach(ApproachParams(deceleration=0.))
    with pytest.raises(ValueError):ParkingApproach(ApproachParams(speed=math.nan))


def test_occupied_or_unobserved_lane_envelope_holds_before_handover():
    a,l,pose,p=fixture();l.motion_clear=lambda *args:False;l.motion_reason='occupied swept footprint'
    assert update(a,1.5,True,False,l,pose,p)==0.
    assert 'occupied' in a.reason
    l.last_scan=4.6
    assert update(a,4.6,True,False,l,pose,p)==0. and not a.skipped
    l.motion_clear=lambda *args:True
    assert update(a,4.7,True,False,l,pose,p)>a.p.speed
