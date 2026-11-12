"""Preparation must not confuse occluded parking space with a blocked lane."""
import math
from types import SimpleNamespace
import pytest
from clothoid_parking.exploration import Exploration, ExploreParams
from clothoid_parking.layout import ParkingLayout, LayoutParams, unique_cells
from clothoid_parking.planner import PlanParams
from clothoid_parking.tracker import Pose2D
from test_six_bays import observed, evidence


def test_no_sign_only_slowdown_and_stale_hold_after_registration():
    policy = Exploration()
    layout = ParkingLayout()
    assert policy.update(1., layout, Pose2D(), PlanParams(), .8) is None
    layout = observed(1)
    evidence(layout, 1.5)
    assert policy.update(1.5, layout, Pose2D(), PlanParams(), .8) is not None
    assert policy.update(2.1, layout, Pose2D(), PlanParams(), .8) == 0.
    layout.origin = None
    assert policy.update(2.2, layout, Pose2D(), PlanParams(), .8) == 0.


def test_unknown_goal_can_be_passed_from_clear_lane():
    layout = observed(1)
    evidence(layout, 1.6)
    # Never use this goal's missing evidence to certify a bay.
    b = layout.bays()[0]
    for key in map(tuple, unique_cells(layout.footprint(b.x,b.y,b.entry_yaw), layout.p.grid)):
        layout.cells.pop(key, None)
    policy = Exploration()
    assert policy.update(1.6, layout, Pose2D(), PlanParams(), .2) > 0
    assert layout.coverage(layout.footprint(b.x,b.y,b.entry_yaw), 1.6) == 'unknown'
    evidence(layout, 1.8)
    assert policy.update(1.8, layout, Pose2D(1.,0.,0.), PlanParams(), .2) is None
    assert policy.skipped


def test_lane_obstacle_stops_exploration_not_skip():
    layout = observed(1)
    evidence(layout, 1.6, (.3, 0.))
    policy = Exploration()
    assert policy.update(1.6, layout, Pose2D(), PlanParams(), .2) == 0
    assert not policy.skipped


def test_newly_swept_unknown_and_any_hit_still_veto():
    layout = ParkingLayout()
    pose = Pose2D()
    assert layout.swept_coverage(layout.footprint(0,0,0), 1., pose) == 'free'
    assert layout.swept_coverage(layout.footprint(.1,0,0), 1., pose) == 'unknown'
    layout.cells[(0,0)] = (1., True)
    assert layout.swept_coverage(layout.footprint(0,0,0), 1., pose) == 'occupied'
    # It remains unknown in the map; this exception cannot approve a goal.
    layout.cells.clear()
    assert layout.coverage(layout.footprint(0,0,0),1.) == 'unknown'


def test_passed_target_not_selected_even_if_observed_free():
    layout = observed(1)
    for now in (1.6,1.8,2.,2.2,2.4):
        evidence(layout,now)
        assert layout.choose(now, Pose2D(1.2,0,0), PlanParams()) is None
    assert layout.states[0] == 'passed'


def test_speed_cap_includes_distance_and_stays_below_far_limit():
    layout = observed(1)
    policy = Exploration()
    evidence(layout,1.6)
    far = policy.update(1.6,layout,Pose2D(),PlanParams(),.2)
    evidence(layout,1.8)
    near = policy.update(1.8,layout,Pose2D(.7,0,0),PlanParams(),.2)
    assert near == pytest.approx(policy.p.speed)
    assert far > near
