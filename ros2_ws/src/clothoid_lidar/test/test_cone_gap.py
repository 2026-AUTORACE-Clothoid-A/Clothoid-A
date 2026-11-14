import numpy as np
import pytest
from clothoid_lidar.cone_trace import _bridge_inner_gap, _neighbours
from clothoid_lidar.cone_sorter import SorterParams, LEFT, RIGHT, ConeSorter, ConeMission, SortResult


def fixture(sign=1.):
    # A visible inner corner has a large terminal gap and two supporting cones.
    points = [(.3,.3),(.55,.3),(.58,-.2),(.75,-.4),(.95,-.5),
              (0.,-.5),(.2,-.75),(.45,-.95),(.75,-1.),(1.05,-.82)]
    return [(x, sign*y) for x,y in points]


@pytest.mark.parametrize('side',[LEFT, RIGHT])
def test_observed_corner_gap_is_bridged_and_keeps_side_identity(side):
    p=SorterParams();points=fixture(side);graph=_neighbours(points,p.max_n_neighbors,p.max_dist_between_cones)
    chain=_bridge_inner_gap(points,[0,1],[5,6,7,8,9],graph,p,side)
    assert chain == [0,1,2,3]
    assert all(i not in chain for i in (5,6,7,8,9))


def test_no_bridge_without_a_current_continuation():
    p=SorterParams();points=fixture();points[3]=(1.8,-.3);points[4]=(2.,-.5)
    graph=_neighbours(points,p.max_n_neighbors,p.max_dist_between_cones)
    assert _bridge_inner_gap(points,[0,1],[5,6,7,8,9],graph,p,LEFT)==[0,1]


def test_normal_gap_is_not_relaxed_globally():
    p=SorterParams();points=fixture();points[2]=(.58,0.);graph=_neighbours(points,p.max_n_neighbors,p.max_dist_between_cones)
    assert 2 in graph[1]
    assert _bridge_inner_gap(points,[0,1],[5,6,7,8,9],graph,p,LEFT)==[0,1]


def test_opposite_wall_must_show_the_corner():
    p=SorterParams();points=fixture()
    points[5:]=[(0.,-.7),(.2,-.7),(.4,-.7),(.6,-.7),(.8,-.7)]
    graph=_neighbours(points,p.max_n_neighbors,p.max_dist_between_cones)
    assert _bridge_inner_gap(points,[0,1],[5,6,7,8,9],graph,p,LEFT)==[0,1]


def test_bridge_may_not_cross_or_reach_the_opposite_boundary():
    p=SorterParams();points=fixture();points[5]=(.3,-.05);points[6]=(.8,-.05)
    graph=_neighbours(points,p.max_n_neighbors,p.max_dist_between_cones)
    assert _bridge_inner_gap(points,[0,1],[5,6],graph,p,LEFT)==[0,1]


def test_outward_detour_is_not_bridged():
    p=SorterParams();points=[(x,-y) for x,y in fixture()];graph=_neighbours(points,p.max_n_neighbors,p.max_dist_between_cones)
    assert _bridge_inner_gap(points,[0,1],[5,6,7,8,9],graph,p,LEFT)==[0,1]


def test_supported_tail_must_not_converge_into_opposite_wall():
    p=SorterParams();points=fixture();points[5]=(.8,-.6);points[6]=(1.,-.6)
    graph=_neighbours(points,p.max_n_neighbors,p.max_dist_between_cones)
    assert _bridge_inner_gap(points,[0,1],[5,6],graph,p,LEFT)==[0,1]


def test_disabled_bridge_keeps_normal_trace_and_seed_history():
    points=np.array(fixture());mission=ConeMission(ConeSorter(SorterParams()))
    mission.running=True
    mission.previous=SortResult(left=points[:2],right=points[5:])
    first=mission.update(points,0.)
    np.testing.assert_allclose(first.left,mission.previous.left)
    # The comparison helper still exists, but cannot mutate the retained trace.
    extended=mission.sorter.extend_corners(points,first)
    assert len(extended.left)>len(first.left)
    np.testing.assert_allclose(mission.previous.left,points[:2])
    second=mission.update(points,.1)
    np.testing.assert_allclose(second.left,first.left)
    np.testing.assert_allclose(mission.previous.left,points[:2])
