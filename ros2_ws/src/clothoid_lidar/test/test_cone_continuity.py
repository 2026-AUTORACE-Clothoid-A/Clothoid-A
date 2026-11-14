import numpy as np
import pytest

from clothoid_lidar.cone_sorter import ConeMission, ConeSorter, SorterParams, SortResult, LEFT
from clothoid_lidar.cone_corner import endpoint_centerline, _curvature
from clothoid_lidar.cone_follower import compute_steering, FollowerParams


def test_observed_boundary_survives_entry_angle_gate():
    sorter = ConeSorter(SorterParams())
    previous = SortResult(left=np.array([[.6, .2]]), right=np.array([[.1, -.4]]))
    cones = np.array([[.65, .19], [.1, -.4]])
    assert sorter.select_first_cone(cones, LEFT) is None
    result = sorter.sort(cones, previous)
    np.testing.assert_allclose(result.left, cones[:1])
    # No persistence when the observation disappears.
    assert not len(sorter.sort(cones[1:], result).left)


def test_far_or_ambiguous_observation_is_not_tracked():
    sorter = ConeSorter(SorterParams())
    prior = np.array([[.6, .2]])
    assert sorter._tracked_seed(np.array([[1., .2]]), prior, np.zeros((0, 2))) is None
    assert sorter._tracked_seed(prior, prior, prior) is None


def test_scan_gap_clears_tracking_and_course_end_is_immediate():
    mission = ConeMission(ConeSorter(SorterParams()))
    cones = np.array([[.3, .35], [.6, .35], [.9, .35], [.3, -.35], [.6, -.35], [.9, -.35]])
    mission.update(cones, 0.)
    assert mission.running
    moved = cones+[.15, 0.]
    assert len(mission.update(moved, .1).left)
    # These do not pass strict seed angle; a stale history cannot rescue them.
    assert not len(mission.update(moved, 1.).left)
    assert mission.update(np.zeros((0, 2)), 1.1) is None
    assert not mission.running and mission.previous is None


@pytest.mark.parametrize('sign', [1., -1.])
def test_one_inner_cone_and_outer_semicircle(sign):
    anchor = np.array([[.3, .5]])
    angles = np.linspace(-np.pi/2, np.pi/2, 13)
    outer = anchor+np.column_stack((np.cos(angles), np.sin(angles)))
    left, right = (anchor, outer) if sign == 1 else (outer*[1., -1.], anchor*[1., -1.])
    path = endpoint_centerline(left, right, .05)
    assert path is not None
    np.testing.assert_allclose(path[0], (left[0]+right[0])/2)
    np.testing.assert_allclose(path[-1], (left[-1]+right[-1])/2)
    assert np.max(_curvature((anchor+outer)/2)) == pytest.approx(2.)


def test_single_anchor_rejects_disordered_or_too_tight_outer_wall():
    anchor = np.array([[.3, .5]])
    angles = np.linspace(-np.pi/2, 0., 7)
    outer = anchor+.6*np.column_stack((np.cos(angles), np.sin(angles)))
    assert endpoint_centerline(anchor, outer, .05) is None
    outer = anchor+np.column_stack((np.cos(angles), np.sin(angles)))
    outer[[2, 3]] = outer[[3, 2]]
    assert endpoint_centerline(anchor, outer, .05) is None


def test_far_target_uses_its_actual_rear_axle_distance():
    params = FollowerParams(lfd=.4)
    center = np.array([[.1, .1], [.45, .825]])
    delta = compute_steering(center, params)
    # The selected target is (.45,.825); it is almost a metre from the rear
    # axle, rather than the configured .4 m of path used to select it.
    assert delta == pytest.approx(np.arctan2(.4*.825, .55**2+.825**2))
