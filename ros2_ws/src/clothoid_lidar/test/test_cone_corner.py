import numpy as np
import pytest

from clothoid_lidar.cone_corner import endpoint_centerline, _distance
from clothoid_lidar.cone_follower import FollowerParams, build_center_line
from clothoid_lidar.cone_sorter import ConeSorter, SorterParams


def corner(sign=1., width=1.0):
    inner = np.column_stack(([-.15, .10, .35, .60], np.full(4, width/2)))
    straight = np.column_stack(([-.15, .10, .35], np.full(3, -width/2)))
    angles = np.linspace(-np.pi/2, 0., 7)
    arc = inner[-1]+width*np.column_stack((np.cos(angles), np.sin(angles)))
    outer = np.vstack((straight, arc))
    if sign == 1:
        return inner, outer
    return outer*[1., -1.], inner*[1., -1.]


@pytest.mark.parametrize('sign', [1., -1.])
def test_endpoint_fan_reaches_outer_corner_in_order(sign):
    left, right = corner(sign)
    path = endpoint_centerline(left, right, .05)
    assert path is not None
    inner, outer = (left, right) if sign == 1 else (right, left)
    np.testing.assert_allclose(path[-1], (inner[-1]+outer[-1])/2)
    assert np.min(_distance(path, left)) > .25
    assert np.min(_distance(path, right)) > .25
    assert np.max(np.linalg.norm(np.diff(path, axis=0), axis=1)) <= .05+1e-9
    np.testing.assert_allclose(build_center_line(left, right, FollowerParams()), path)


def test_parallel_walls_and_merely_truncated_wall_keep_existing_path():
    left = np.column_stack((np.arange(5)*.2, np.full(5, .35)))
    right = np.column_stack((np.arange(8)*.2, np.full(8, -.35)))
    assert endpoint_centerline(left, right, .05) is None
    np.testing.assert_allclose(build_center_line(left, right, FollowerParams()),
                               build_center_line(left, right, FollowerParams(corner_endpoint_enabled=False)))


def test_rotated_corner_is_not_tied_to_vehicle_x_axis():
    left, right = corner()
    angle = .35
    rot = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    baseline = endpoint_centerline(left, right, .05)
    actual = endpoint_centerline(left@rot.T, right@rot.T, .05)
    assert actual is not None
    np.testing.assert_allclose(actual, baseline@rot.T, atol=1e-10)


def test_missing_opposite_wall_and_narrow_opening_are_not_fanned():
    left, right = corner()
    assert endpoint_centerline(left, np.zeros((0, 2)), .05) is None
    left, right = corner(width=.4)
    assert endpoint_centerline(left, right, .05) is None


def test_irregular_outer_wall_is_rejected():
    left, right = corner()
    right[5, 1] -= .5
    assert endpoint_centerline(left, right, .05) is None


def test_turn_too_tight_for_vehicle_is_not_fanned():
    left, right = corner(width=.65)
    assert endpoint_centerline(left, right, .05) is None


def test_bent_inner_wall_is_rejected():
    left, right = corner()
    left[1, 1] += .2
    assert endpoint_centerline(left, right, .05) is None


def test_short_straight_endpoint_retains_two_observed_cones():
    cones = np.array([[.05, .4], [.3, .4], [.05, -.4], [.3, -.4], [.55, -.4]])
    result = ConeSorter(SorterParams()).sort(cones)
    np.testing.assert_allclose(result.left, cones[:2])
    assert len(result.right) == 3


def test_two_point_inner_end_can_support_observed_outer_turn():
    left, right = corner()
    left = left[[0, -1]]
    assert endpoint_centerline(left, right, .05) is not None


def test_radial_measurement_jitter_does_not_destroy_corner_path():
    left, right = corner()
    angle = np.linspace(-np.pi/2, 0., 7)
    radii = 1.0+.03*np.sin(np.arange(7)*2.)
    right[-7:] = left[-1]+radii[:, None]*np.column_stack((np.cos(angle), np.sin(angle)))
    path = endpoint_centerline(left, right, .05)
    assert path is not None
    assert _distance(path, left).min() > .25
    assert _distance(path, right).min() > .25


def test_outer_wall_can_pass_the_inner_straight_half_plane_after_corner():
    left, right = corner()
    angle = np.linspace(-np.pi/2, np.deg2rad(10), 8)
    right = np.vstack((right[:3], left[-1]+1.0*np.column_stack((np.cos(angle), np.sin(angle)))))
    assert right[-1, 1] > left[-1, 1]
    assert endpoint_centerline(left, right, .05) is not None
