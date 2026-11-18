"""Sensor-selected bay/depth policy; no simulator vacancy is passed to the core."""
import math

import numpy as np
import pytest

from clothoid_parking.core import ParkingCore
from clothoid_parking.layout import LayoutParams, ParkingLayout
from clothoid_parking.planner import Phase, PlanParams, Planner
from clothoid_parking.tracker import Pose2D


def registered():
    layout = ParkingLayout(LayoutParams(control_enabled=True))
    for t in (1., 1.2, 1.4):
        layout.observe(t, (1., -.25), 0., Pose2D())
    return layout


def fill(layout, points, now, occupied=False):
    for key in np.floor(points/layout.p.grid).astype(int):
        layout.cells[tuple(key)] = (now, occupied)


@pytest.mark.parametrize('goal', range(6))
def test_all_six_goals_selected_from_evidence_with_clear_access(monkeypatch, goal):
    layout, pose, plan = registered(), Pose2D(), PlanParams()
    monkeypatch.setattr(layout, 'supported', lambda now: True)
    bays = layout.bays()
    empty = {goal} | ({goal-3} if goal >= 3 else set())
    for now in (1.6, 1.8, 2., 2.2, 2.4):
        fill(layout, layout.path(bays[goal], pose, plan), now)
        for i, bay in enumerate(bays):
            fill(layout, layout.footprint(bay.x, bay.y, bay.entry_yaw), now, i not in empty)
        layout.last_scan = now
        selected = layout.choose(now, pose, plan)
    assert selected == bays[goal]
    assert layout.selected == goal
    assert layout.empty_bays == sorted(i+1 for i in empty)


def test_occluded_inner_goal_is_not_mistaken_for_a_front_goal(monkeypatch):
    layout, pose, plan = registered(), Pose2D(), PlanParams()
    monkeypatch.setattr(layout, 'supported', lambda now: True)
    front = layout.bays()[1]
    for now in (1.6, 1.8, 2., 2.2, 2.4):
        fill(layout, layout.footprint(front.x, front.y, front.entry_yaw), now)
        layout.last_scan = now
        assert layout.choose(now, pose, plan) is None
    assert layout.empty_bays == [2]
    assert layout.observation_bay(pose, plan) is not None


def test_five_occupied_bays_do_not_certify_the_hidden_sixth(monkeypatch):
    layout, pose, plan = registered(), Pose2D(), PlanParams()
    monkeypatch.setattr(layout, 'supported', lambda now: True)
    for now in (1.6, 1.8, 2., 2.2, 2.4):
        for bay in layout.bays()[:5]:
            fill(layout, layout.footprint(bay.x, bay.y, bay.entry_yaw), now, True)
        layout.last_scan = now
        assert layout.choose(now, pose, plan) is None
    assert layout.empty_bays == []


def test_free_inner_goal_with_blocked_front_is_diagnosed_but_not_selected(monkeypatch):
    layout, pose, plan = registered(), Pose2D(), PlanParams()
    monkeypatch.setattr(layout, 'supported', lambda now: True)
    front, inner = layout.bays()[1], layout.bays()[4]
    for now in (1.6, 1.8, 2., 2.2, 2.4):
        fill(layout, layout.path(inner, pose, plan), now)
        fill(layout, layout.footprint(inner.x, inner.y, inner.entry_yaw), now)
        fill(layout, layout.footprint(front.x, front.y, front.entry_yaw), now, True)
        layout.last_scan = now
        assert layout.choose(now, pose, plan) is None
    assert layout.empty_bays == [5]
    assert layout.states[4] == 'path_occupied'


@pytest.mark.parametrize('column', range(3))
@pytest.mark.parametrize('heading', [-.07, 0., .07])
def test_second_row_keeps_same_turn_point_and_adds_only_depth(column, heading):
    layout, pose, plan = registered(), Pose2D(yaw=heading), PlanParams(entry_heading_compensation=True)
    front, inner = layout.bays()[column], layout.bays()[column+3]
    planner = Planner(plan)
    planner.start(0., pose)
    planner.bay = front
    turn_at_front = planner.turn_remaining(pose)
    planner.bay = inner
    assert planner.turn_remaining(pose) == pytest.approx(turn_at_front)
    assert inner.entry_yaw == front.entry_yaw
    assert math.hypot(inner.x-front.x, inner.y-front.y) == pytest.approx(.62)


def test_search_without_a_confirmed_goal_times_out_without_success():
    core = ParkingCore(layout=LayoutParams(control_enabled=True))
    core.set_flag(0., True)
    core.layout.last_scan = 21.
    assert core.command(21.) == (0., 0.)
    assert core.phase == Phase.ABORT


def test_same_scan_cannot_confirm_an_automatic_goal(monkeypatch):
    layout, pose, plan = registered(), Pose2D(), PlanParams()
    monkeypatch.setattr(layout, 'supported', lambda now: True)
    layout.support_count = 3
    front, back = layout.bays()[1], layout.bays()[4]
    fill(layout, layout.path(front, pose, plan), 2.)
    fill(layout, layout.footprint(back.x, back.y, back.entry_yaw), 2., True)
    layout.last_scan = 2.
    for _ in range(20):
        assert layout.choose(2., pose, plan) is None
    assert layout.free_counts[1] == 1


@pytest.mark.parametrize('index', range(6))
def test_goal_is_always_the_fixed_bay_centre(index):
    layout = ParkingLayout(LayoutParams(control_enabled=True))
    for t in (1., 1.2, 1.4):
        layout.observe(t, (1., -.25), 0., Pose2D())
    bay, goal = layout.bays()[index], layout.goal(index)
    assert goal == bay
    assert layout.inside_bay(bay, Pose2D(goal.x, goal.y, goal.entry_yaw))
    assert not layout.inside_bay(bay, Pose2D(goal.x+.15, goal.y, goal.entry_yaw))


def test_fixed_reference_survives_lidar_occlusion_but_goal_evidence_does_not(monkeypatch):
    layout, pose, plan = registered(), Pose2D(), PlanParams()
    layout.reference_fixed = True
    layout.support_count = 3
    monkeypatch.setattr(layout, 'supported', lambda now: False)
    for now in (1.6, 1.8, 2.):
        layout.last_scan = now
        layout.choose(now, pose, plan)
    assert layout.support_count == 3
    assert layout.selected is None and layout.empty_bays == []
    layout.reference_fixed = False
    layout.last_scan = 2.2
    layout.choose(2.2, pose, plan)
    assert layout.support_count == 0


@pytest.mark.parametrize('lateral,sign', [(-.02, 1), (.04, -1)])
def test_automatic_search_returns_towards_measured_road_centre(monkeypatch, lateral, sign):
    core = ParkingCore(layout=LayoutParams(control_enabled=True))
    core.layout = registered()
    core.set_flag(.4, True)
    core.layout.support_count = 3
    core.layout.last_scan = 1.
    core.tracker.rear.y = lateral
    monkeypatch.setattr(core.layout, 'observation_bay', lambda *args: core.layout.goal(1))
    monkeypatch.setattr(core.layout, 'motion_clear', lambda *args: True)
    speed, steer = core.command(1.)
    assert speed > 0 and steer*sign > 0
    assert abs(steer) <= core.planner.p.max_steer


def test_registered_hold_does_not_count_final_braking_as_stationary_time():
    core = ParkingCore(plan=PlanParams(exit_enabled=False), layout=LayoutParams(control_enabled=True))
    core.layout = registered()
    core.layout.selected = 1
    core.planner.bay = core.layout.goal(1)
    yaw = core.planner.bay.entry_yaw
    core.tracker.rear = Pose2D(core.planner.bay.x-core.tracker.rear_offset*math.cos(yaw),
                              core.planner.bay.y-core.tracker.rear_offset*math.sin(yaw), yaw)
    core.planner._go(Phase.PARKED, 0., 'test')
    core.tracker.speed = .1
    core.layout.last_scan = 1.
    assert core.command(1.) == (0., 0.)
    core.tracker.speed = 0.
    core.layout.last_scan = 3.1
    assert core.command(3.1) == (0., 0.) and core.phase == Phase.PARKED
    core.layout.last_scan = 4.1
    assert core.command(4.1) is None and core.phase == Phase.DONE


def test_goal_memory_requires_measured_free_cells_and_a_new_hit_revokes_it():
    layout = registered()
    bay = layout.bays()[1]
    points = layout.footprint(bay.x, bay.y, bay.entry_yaw)
    layout.retain_free(1.5, True)
    assert layout.goal_coverage(points, 1.5) == 'unknown'
    fill(layout, points, 1.5)
    layout.retain_free(1.5, True)
    assert layout.coverage(points, 4.) == 'unknown'
    assert layout.goal_coverage(points, 4.) == 'free'
    distance, angle = math.hypot(bay.x, bay.y), math.atan2(bay.y, bay.x)
    layout.scan(4.1, [distance], angle, 0., .02, 12., Pose2D(), 0.)
    assert layout.goal_coverage(points, 4.1) == 'unknown'
    layout.scan(4.4, [distance], angle, 0., .02, 12., Pose2D(), 0.)
    assert layout.goal_coverage(points, 4.4) == 'occupied'
