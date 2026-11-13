"""Measured road alignment, collision geometry and handover invariants."""
import math
from dataclasses import replace
import numpy as np
from clothoid_common.parking_geometry import PARKING_GEOMETRY
from clothoid_parking.core import ParkingCore
from clothoid_parking.layout import LayoutParams, ParkingLayout
from clothoid_parking.tracker import Pose2D


def layout():
    obj=ParkingLayout(LayoutParams(control_enabled=True,target_bay=2))
    for t in (1.,1.2,1.4):obj.observe(t,(1.,-.25),0.,Pose2D())
    obj.support_count=3
    return obj


def test_registration_tolerance_does_not_enlarge_collision_body():
    a=ParkingLayout(LayoutParams(position_tolerance=.07))
    b=ParkingLayout(LayoutParams(position_tolerance=.15))
    assert np.array_equal(a._footprint_offsets,b._footprint_offsets)
    assert np.allclose(np.ptp(a._footprint_offsets,axis=0),(.322+.06,.220+.06))


def test_side_lines_correct_lateral_without_inventing_along_road_location():
    obj=layout();before=obj.origin.copy()
    segments=[np.array([[1.2,-.22],[1.7,-.22]]),
              np.array([[1.2,.30],[1.7,.30]])]
    for t in (1.6,1.8,2.,2.2,2.4):obj.refine_boundaries(t,segments,Pose2D())
    assert -.25 < obj.origin[1] < -.22
    assert obj.origin[0]==before[0]
    assert obj.yaw==0.
    assert not obj.refine_boundaries(2.4,segments,Pose2D())


def test_side_line_heading_is_refined_and_disagreeing_line_is_rejected():
    obj=layout();obj.yaw=.025
    segments=[np.array([[1.1,-.25],[1.5,-.25]]),
              np.array([[1.1,.27],[1.5,.27]])]
    for t in (1.6,1.8,2.,2.2,2.4):obj.refine_boundaries(t,segments,Pose2D())
    assert abs(obj.yaw)<.025
    before=(obj.origin.copy(),obj.yaw)
    assert not obj.refine_boundaries(2.6,[np.array([[1.,.8],[1.5,.8]])],Pose2D())
    assert np.array_equal(obj.origin,before[0]) and obj.yaw==before[1]


def test_same_frame_anchor_does_not_erase_an_accepted_side_refinement():
    obj=layout()
    segments=[np.array([[1.2,-.22],[1.7,-.22]])]
    for t in (1.6,1.8,2.):obj.refine_boundaries(t,segments,Pose2D())
    lateral=obj.origin[1]
    assert lateral>-.25
    assert obj.observe(2.,(1.,-.25),0.,Pose2D())
    assert obj.origin[1]==lateral


def test_side_observations_can_accumulate_but_never_apply_without_lidar_support():
    obj=layout();obj.support_count=0;before=obj.origin.copy()
    segments=[np.array([[1.2,-.22],[1.7,-.22]])]
    for t in (1.6,1.8,2.):assert not obj.refine_boundaries(t,segments,Pose2D())
    assert np.array_equal(obj.origin,before)
    obj.support_count=3
    assert obj.refine_boundaries(2.2,segments,Pose2D())


def test_parking_flag_freezes_both_anchor_and_side_line_updates():
    core=ParkingCore(layout=LayoutParams(control_enabled=True,target_bay=2))
    core.prepare(True);core.layout=layout()
    for t in (1.4,1.6,1.8,2.):core.odom(t,0.)
    lines=[np.array([[1.2,-.22],[1.7,-.22]])]
    for t in (1.6,1.8,2.):core.observe_side_lines(t,lines)
    before=(core.layout.origin.copy(),core.layout.yaw)
    core.set_flag(2.,True);core.odom(2.2,0.)
    assert not core.observe_side_lines(2.2,lines)
    assert not core.observe_entry(2.2,(1.02,-.20),.02)
    assert np.array_equal(core.layout.origin,before[0]) and core.layout.yaw==before[1]


def test_rule_stage_keeps_lane_width_separate_from_bay_dimensions():
    geometry=PARKING_GEOMETRY
    assert geometry.road_width==.50 and geometry.boundary_spacing==.52
    lines={name:np.array(points) for name,points,_ in geometry.stage_lines()}
    assert np.allclose(lines['entry'][:,0],-.470207)
    assert np.allclose(lines['finish'][:,0],2.039797)
    assert np.isclose(np.ptp(lines['entry'][:,1]),.52)


def test_one_visible_boundary_never_rotates_registered_map():
    obj=layout();before=obj.yaw
    points=np.array([[1.1,-.22],[1.6,-.20]])
    for t in (1.6,1.8,2.,2.2,2.4):obj.refine_boundaries(t,[points],Pose2D())
    assert obj.origin[1]>-.25
    assert obj.yaw==before


def test_disagreeing_near_and_far_boundaries_do_not_average_into_a_correction():
    obj=layout();before=obj.origin.copy()
    lines=[np.array([[1.1,-.25],[1.6,-.22]]),np.array([[1.1,.27],[1.6,.24]])]
    for t in (1.6,1.8,2.,2.2):assert not obj.refine_boundaries(t,lines,Pose2D())
    assert np.array_equal(obj.origin,before) and obj.yaw==0.
