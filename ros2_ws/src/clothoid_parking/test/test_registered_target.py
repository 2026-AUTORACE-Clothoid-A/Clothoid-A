"""Target geometry must not follow obstacle placement; unknown is not free."""
import math
import numpy as np
import pytest
from clothoid_parking.layout import ParkingLayout, LayoutParams
from clothoid_parking.core import ParkingCore
from clothoid_parking.planner import PlanParams, Phase
from clothoid_parking.tracker import Pose2D


def registered(target=2):
    layout=ParkingLayout(LayoutParams(control_enabled=True,target_bay=target))
    for t in (.1,.2,.3): layout.observe(t,(1.,-.25),0.,Pose2D())
    return layout


def fill(layout, points, occupied=False, now=1.):
    for k in np.floor(points/layout.p.grid).astype(int): layout.cells[tuple(k)]=(now,occupied)


def test_control_cannot_silently_select_an_unspecified_bay():
    with pytest.raises(ValueError,match='designated'): ParkingLayout(LayoutParams(control_enabled=True))
    with pytest.raises(ValueError): ParkingLayout(LayoutParams(target_bay=7))


def test_displaced_and_rotated_obstacles_do_not_move_registered_targets():
    layout=registered()
    before=np.array([[b.x,b.y,b.row_yaw] for b in layout.bays()])
    # Two separated car silhouettes shifted away from nominal column centres.
    for x,y in ((1.37,-.34),(2.16,-.55)):
        fill(layout,np.array([[x+d,y+.3*d] for d in np.linspace(-.12,.12,12)]),True)
    assert layout.supported(1.)
    assert np.array_equal(before,[[b.x,b.y,b.row_yaw] for b in layout.bays()])


def test_continuous_wall_does_not_corroborate_grid():
    layout=registered()
    fill(layout,np.array([[x,-.5] for x in np.arange(1.01,2.55,.02)]),True)
    assert not layout.supported(1.)


def test_occupied_fixed_target_never_falls_back_to_another_bay(monkeypatch):
    layout=registered()
    monkeypatch.setattr(layout,'supported',lambda now:True)
    monkeypatch.setattr(layout,'coverage',lambda points,now:'free')
    for now in (.6,.8,1.,1.2,1.4):
        layout.last_scan=now
        layout.choose(now,Pose2D(),PlanParams())
    assert layout.selected==1
    monkeypatch.setattr(layout,'coverage',lambda points,now:'occupied' if np.mean(points[:,0])>1.5 else 'free')
    layout.last_scan=1.6
    assert layout.choose(1.6,Pose2D(),PlanParams()) is None
    assert layout.selected is None


def test_current_unknown_envelope_is_not_a_new_obstacle_but_hits_still_veto():
    layout=registered();pose=Pose2D()
    points=np.concatenate([layout.footprint(x,0.,0.) for x in np.linspace(0,.20,100)])
    fill(layout,points)
    current=set(map(tuple,np.floor(layout.footprint(0.,0.,0.)/layout.p.grid).astype(int)))
    for k in current: layout.cells.pop(k,None)
    assert layout.motion_clear(1.,pose,.2,0.,.2)
    # Even inside the current safety envelope an actual return may not be ignored.
    fill(layout,np.array([[.24,0.]]),True)
    assert not layout.motion_clear(1.,pose,.2,0.,.2)
    layout.cells.clear()
    assert not layout.motion_clear(1.,pose,.2,0.,.2)


def test_outside_bay_obstacle_blocks_approach():
    layout=registered();pose=Pose2D()
    points=np.concatenate([layout.footprint(x,0.,0.) for x in np.linspace(0,.2,100)])
    fill(layout,points)
    assert layout.motion_clear(1.,pose,.2,0.,.2)
    fill(layout,np.array([[.4,0.]]),True)
    assert not layout.motion_clear(1.,pose,.2,0.,.2)


def test_depth_only_is_not_success_when_car_overhangs_bay():
    core=ParkingCore(layout=LayoutParams(control_enabled=True,target_bay=2))
    core.layout=registered();bay=core.layout.bays()[1]
    core.planner.bay=bay
    core.planner.phase=Phase.ENTER
    core.layout.last_scan=1.
    core.tracker.rear=Pose2D(bay.x+.20, bay.y+core.tracker.rear_offset, bay.entry_yaw)
    assert core.command(1.)==(0.,0.)
    assert core.phase==Phase.ABORT
    assert 'corners' in core.planner.reason


def test_unknown_target_stops_before_turn_without_false_parking():
    core=ParkingCore(layout=LayoutParams(control_enabled=True,target_bay=2))
    core.layout=registered();bay=core.layout.bays()[1]
    core.planner.start(0.,Pose2D())
    core.layout.last_scan=1.;core.layout.support_count=3
    core.tracker.rear=Pose2D(bay.x-core.planner.p.turn_radius-core.planner.p.turn_lead-.05-core.tracker.rear_offset,0.,0.)
    assert core.command(1.)==(0.,0.)
    assert core.phase==Phase.SEARCH and core.planner.bay is None
