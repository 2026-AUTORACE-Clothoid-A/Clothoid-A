"""Geometry and forward-path regressions for rear-axle target selection."""
import numpy as np
import pytest
from unittest.mock import patch

from clothoid_lidar.cone_follower import (
    ConeFollower, FollowerParams, compute_steering, look_ahead_point,
)


def test_straight_target_is_measured_from_rear_axle_and_interpolated():
    p = FollowerParams(lfd=.3, rear_axle_x=.1)
    path = np.array([[-.4, 0.], [.13, 0.], [.63, 0.]])
    original = path.copy()
    np.testing.assert_allclose(look_ahead_point(path, p), [.2, 0.], atol=1e-12)
    np.testing.assert_array_equal(path, original)


@pytest.mark.parametrize('side', [1., -1.])
def test_lateral_error_changes_intersection_without_changing_radius(side):
    p = FollowerParams(lfd=.3)
    path = np.array([[-.6, side*.18], [1., side*.18]])
    target = look_ahead_point(path, p)
    np.testing.assert_allclose(target, [.14, side*.18], atol=1e-12)
    assert np.linalg.norm(target-[-.1, 0.]) == pytest.approx(.3)
    assert np.sign(compute_steering(path, p)) == side


def test_target_is_invariant_to_polyline_sampling():
    p = FollowerParams()
    coarse = np.array([[-.4, .05], [.1, .08], [.7, .5]])
    dense = np.vstack([np.linspace(a, b, 17, endpoint=False)
                       for a, b in zip(coarse, coarse[1:])] + [coarse[-1:]])
    np.testing.assert_allclose(look_ahead_point(coarse, p), look_ahead_point(dense, p), atol=1e-12)


def test_target_moves_continuously_across_old_vertex_selection_boundary():
    p = FollowerParams()
    path = np.column_stack((np.arange(-.4, 1., .05), np.full(28, .1)))
    previous = None
    for travel in np.linspace(0., .1, 101):
        target = look_ahead_point(path-[travel, 0.], p)
        assert target is not None
        world = target+[travel, 0.]
        if previous is not None:
            assert np.linalg.norm(world-previous) == pytest.approx(.001)
        previous = world


def test_short_remaining_path_uses_endpoint_without_extrapolation():
    p = FollowerParams()
    path = np.array([[-.2, 0.], [.04, .06], [.09, .08]])
    np.testing.assert_array_equal(look_ahead_point(path, p), path[-1])
    assert np.isfinite(compute_steering(path, p))


def test_outside_circle_uses_closest_projection_on_existing_forward_segment():
    p = FollowerParams()
    path = np.array([[.3, -.4], [.3, .5]])
    np.testing.assert_allclose(look_ahead_point(path, p), [.3, 0.], atol=1e-12)


def test_first_forward_outward_crossing_precedes_later_crossing():
    p = FollowerParams()
    path = np.array([[-.1, 0.], [.6, 0.], [.6, .6], [0., .1], [.8, .1]])
    np.testing.assert_allclose(look_ahead_point(path, p), [.2, 0.], atol=1e-12)


def test_tangent_intersection_is_a_valid_target():
    p = FollowerParams()
    x, y = .08, .24  # .3 m from the rear axle
    step = np.array([-.24, .18])
    path = np.array([x, y])+np.array([-.5, .5])[:, None]*step
    np.testing.assert_allclose(look_ahead_point(path, p), [x, y], atol=1e-9)


@pytest.mark.parametrize('path', [
    np.zeros((0, 2)), [[.2, 0.]], [[.2, 0.], [.2, 0.]],
    [[-.6, 0.], [-.3, 0.]], [[1., 0.], [-1., 0.]],
    [[.1, np.nan], [.5, 0.]], [[.1, 0.], [.5, np.inf]],
])
def test_no_target_for_empty_degenerate_backward_or_nonfinite_path(path):
    p = FollowerParams()
    assert look_ahead_point(np.asarray(path), p) is None
    assert compute_steering(np.asarray(path), p) == 0.


def test_repeated_vertices_do_not_change_target():
    p = FollowerParams()
    path = np.array([[-.1, 0.], [-.1, 0.], [.8, .1], [.8, .1]])
    np.testing.assert_allclose(look_ahead_point(path, p), look_ahead_point(path[[0, 2]], p))


@pytest.mark.parametrize('lfd', [0., -1., np.nan, np.inf])
def test_invalid_lookahead_radius_is_rejected(lfd):
    with pytest.raises(ValueError, match='lfd'):
        FollowerParams(lfd=lfd)


@pytest.mark.parametrize('x', [[.02, .08], [.45, .7]])
def test_missing_circle_intersection_alone_does_not_stop_or_reverse(x):
    left = np.column_stack((x, [.4, .4]))
    right = np.column_stack((x, [-.4, -.4]))
    p = FollowerParams(speed=.25, lfd=.3, steering_gain=1.1)
    follower = ConeFollower(p)
    result = follower.decide(left, right, True, True, (0., 0., 0.),
                             np.vstack((left, right)), allow_reverse=True)
    assert result.speed == .25 and not result.reverse
    assert result.target is not None and np.isfinite(result.steering)


@pytest.mark.parametrize('retained', [False, True])
def test_forward_command_and_visualisation_share_one_target_search(retained):
    from clothoid_lidar import cone_follower

    x = np.linspace(.1, 1.2, 12)
    left = np.column_stack((x, np.full_like(x, .4)))
    right = np.column_stack((x, np.full_like(x, -.4)))
    follower = ConeFollower(FollowerParams())
    follower.decide(left, right, True, True, (0., 0., 0.), np.vstack((left, right)))
    visible_right = np.zeros((0, 2)) if retained else right
    with patch.object(cone_follower, 'look_ahead_point', wraps=look_ahead_point) as search:
        result = follower.decide(left, visible_right, True, not retained,
                                 (0., 0., 0.), np.vstack((left, right)))
        assert search.call_count == 1
    assert result.source == ('retained_boundary' if retained else 'observed')
    np.testing.assert_array_equal(result.target, look_ahead_point(result.center, follower.p))
    assert result.steering == compute_steering(result.center, follower.p)
