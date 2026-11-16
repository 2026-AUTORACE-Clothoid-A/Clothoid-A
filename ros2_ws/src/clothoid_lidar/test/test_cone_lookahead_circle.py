"""Geometry and forward-path regressions for rear-axle target selection."""
import numpy as np
import pytest
from unittest.mock import patch

from clothoid_lidar.cone_exit_pursuit import (
    ExitPursuitParams, compute_steering, look_ahead_point,
)


def test_straight_target_is_measured_from_rear_axle_and_interpolated():
    p = ExitPursuitParams(lfd=.3, rear_axle_x=.1)
    path = np.array([[-.4, 0.], [.13, 0.], [.63, 0.]])
    original = path.copy()
    np.testing.assert_allclose(look_ahead_point(path, p), [.2, 0.], atol=1e-12)
    np.testing.assert_array_equal(path, original)


@pytest.mark.parametrize('side', [1., -1.])
def test_lateral_error_changes_intersection_without_changing_radius(side):
    p = ExitPursuitParams(lfd=.3)
    path = np.array([[-.6, side*.18], [1., side*.18]])
    target = look_ahead_point(path, p)
    np.testing.assert_allclose(target, [.14, side*.18], atol=1e-12)
    assert np.linalg.norm(target-[-.1, 0.]) == pytest.approx(.3)
    assert np.sign(compute_steering(path, p)) == side


def test_target_is_invariant_to_polyline_sampling():
    p = ExitPursuitParams()
    coarse = np.array([[-.4, .05], [.1, .08], [.7, .5]])
    dense = np.vstack([np.linspace(a, b, 17, endpoint=False)
                       for a, b in zip(coarse, coarse[1:])] + [coarse[-1:]])
    np.testing.assert_allclose(look_ahead_point(coarse, p), look_ahead_point(dense, p), atol=1e-12)


def test_target_moves_continuously_across_old_vertex_selection_boundary():
    p = ExitPursuitParams()
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
    p = ExitPursuitParams()
    path = np.array([[-.2, 0.], [.04, .06], [.09, .08]])
    np.testing.assert_array_equal(look_ahead_point(path, p), path[-1])
    assert np.isfinite(compute_steering(path, p))


def test_outside_circle_uses_closest_projection_on_existing_forward_segment():
    p = ExitPursuitParams()
    path = np.array([[.3, -.4], [.3, .5]])
    np.testing.assert_allclose(look_ahead_point(path, p), [.3, 0.], atol=1e-12)


def test_first_forward_outward_crossing_precedes_later_crossing():
    p = ExitPursuitParams(lfd=.3)
    path = np.array([[-.1, 0.], [.6, 0.], [.6, .6], [0., .1], [.8, .1]])
    np.testing.assert_allclose(look_ahead_point(path, p), [.2, 0.], atol=1e-12)


def test_tangent_intersection_is_a_valid_target():
    p = ExitPursuitParams(lfd=.3)
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
    p = ExitPursuitParams()
    assert look_ahead_point(np.asarray(path), p) is None
    assert compute_steering(np.asarray(path), p) == 0.


def test_repeated_vertices_do_not_change_target():
    p = ExitPursuitParams()
    path = np.array([[-.1, 0.], [-.1, 0.], [.8, .1], [.8, .1]])
    np.testing.assert_allclose(look_ahead_point(path, p), look_ahead_point(path[[0, 2]], p))


@pytest.mark.parametrize('lfd', [0., -1., np.nan, np.inf])
def test_invalid_lookahead_radius_is_rejected(lfd):
    with pytest.raises(ValueError, match='lfd'):
        ExitPursuitParams(lfd=lfd)
