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


def test_search_retains_free_evidence_before_it_expires_at_turn(monkeypatch):
    core=ParkingCore(layout=LayoutParams(control_enabled=True,target_bay=2))
    core.layout=registered()
    core.set_flag(.4,True)
    monkeypatch.setattr(core.layout,'supported',lambda now:True)
    # Three actual scan updates corroborate the supplied visual registration.
    for now in (.6,.8,1.):
        core.odom(now,0.)
        core.scan(now,[1.],0.,.1,.02,12.)
    assert core.phase==Phase.SEARCH and core.layout.retaining
    key=next(iter(core.layout.retained_free))
    saved=set(core.layout.retained_free)
    # Move the test clock beyond the old free-cell lifetime while scans remain
    # live. The turn transition must not replace the approach memory.
    core.layout.cells.clear()
    core.layout.last_scan=3.
    core.planner.bay=core.layout.bays()[1]
    def start_turn(*args):
        core.planner.phase=Phase.TURN
        return .2,-.41
    monkeypatch.setattr(core.planner,'step',start_turn)
    core.command(3.)
    assert saved<=core.layout.retained_free and key in core.layout.retained_free
    core.set_flag(3.1,False)
    assert not core.layout.retained_free


@pytest.mark.parametrize('blocked', ['none', 'target', 'path'])
@pytest.mark.parametrize('active', [False, True])
def test_provisional_target_does_not_bypass_occupied_or_unknown_motion(monkeypatch, blocked, active):
    core=ParkingCore(layout=LayoutParams(control_enabled=True,target_bay=2))
    if active:
        core.set_flag(.4,True)
    else:
        core.prepare(True)
    core.layout=registered()
    monkeypatch.setattr(core.layout,'supported',lambda now:True)
    # Model an occluded target and a known route obstruction separately.
    def coverage(points,now):
        if blocked=='target':return 'occupied'
        if blocked=='path' and np.ptp(points[:,0])>.5:return 'occupied'
        return 'unknown'
    monkeypatch.setattr(core.layout,'coverage',coverage)
    for now in (.6,.8,1.):
        core.odom(now,0.)
        core.scan(now,[1.],0.,.1,.02,12.)
    if blocked=='none':
        assert core.approach_bay is not None
        assert (core.planner.bay is not None)==active
        assert core.layout.selected is None  # not published as a confirmed free bay
    else:
        assert core.planner.bay is None and core.approach_bay is None
    # Regardless of provisional geometry, an unobserved near-term sweep stops.
    assert core.command(1.)==((0.,0.) if active else None)


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


def parked_core():
    core = ParkingCore(layout=LayoutParams(control_enabled=True, target_bay=2))
    core.layout = registered()
    bay = core.layout.bays()[1]
    core.planner.bay = bay
    core.planner._arc_end = Pose2D(bay.x, bay.y+.10, bay.entry_yaw)
    core.planner._go(Phase.PARKED, 1., 'parked')
    place(core, Pose2D(bay.x, bay.y, bay.entry_yaw))
    return core


def place(core, pose):
    offset = core.tracker.rear_offset
    core.tracker.rear = Pose2D(pose.x-offset*math.cos(pose.yaw),
                               pose.y-offset*math.sin(pose.yaw), pose.yaw)


def test_registered_reverse_exit_completes_without_rear_free_space_evidence():
    core = parked_core()
    assert not core.layout.cells
    core.layout.last_scan = 2.
    assert core.command(2.) == (0., 0.)
    assert core.phase == Phase.PARKED
    core.layout.last_scan = 4.1
    assert core.command(4.1)[0] < 0
    assert core.phase == Phase.EXIT_BACK
    place(core, core.planner._arc_end)
    core.layout.last_scan = 4.3
    assert core.command(4.3)[0] < 0
    assert core.phase == Phase.EXIT_TURN
    place(core, Pose2D(core.tracker.pose.x, core.tracker.pose.y, core.planner.bay.row_yaw))
    core.layout.last_scan = 4.5
    assert core.command(4.5) is None
    assert core.phase == Phase.DONE


@pytest.mark.parametrize('phase', [Phase.SEARCH, Phase.TURN, Phase.ENTER])
def test_entry_still_requires_observed_free_space(phase):
    core = parked_core()
    core.planner._go(phase, 1., 'entry')
    bay = core.planner.bay
    place(core, Pose2D(bay.x-.60, 0., 0.) if phase != Phase.ENTER
          else Pose2D(bay.x, bay.y+.08, bay.entry_yaw))
    core.layout.last_scan = 1.1
    core.layout.support_count = core.layout.p.confirm_scans
    assert core.command(1.1) == (0., 0.)
    assert core.phase == phase
    assert core.layout.reason == 'holding: unobserved swept footprint'


def test_reverse_depth_adjustment_is_not_exempt_from_entry_checks():
    core = parked_core()
    core.planner._go(Phase.ENTER, 1., 'depth adjustment')
    bay = core.planner.bay
    place(core, Pose2D(bay.x, bay.y-.08, bay.entry_yaw))
    core.layout.last_scan = 1.1
    assert core.command(1.1) == (0., 0.)
    assert core.phase == Phase.ENTER
    assert core.layout.reason == 'holding: unobserved swept footprint'


def test_exit_still_holds_when_scans_stop_arriving():
    core = parked_core()
    core.layout.last_scan = 4.1
    assert core.command(4.1)[0] < 0
    assert core.phase == Phase.EXIT_BACK
    assert core.command(4.7) == (0., 0.)
    assert core.phase == Phase.EXIT_BACK


@pytest.mark.parametrize('phase', [Phase.EXIT_BACK, Phase.EXIT_TURN])
def test_exit_timeout_does_not_report_success(phase):
    core = parked_core()
    core.planner._go(phase, 4., 'exit')
    now = 4.+core.planner.p.exit_max_sec+.1
    core.layout.last_scan = now
    assert core.command(now) == (0., 0.)
    assert core.phase == Phase.ABORT
