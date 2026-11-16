import numpy as np
import pytest

from clothoid_lidar.cone_exit_pursuit import ExitPursuitParams, compute_steering, look_ahead_point


@pytest.mark.parametrize('side', [1., -1.])
def test_gain_scales_curvature_without_changing_target(side):
    path = np.array([[0., 0.], [.2, .05*side], [.5, .2*side], [.8, .3*side]])
    baseline = ExitPursuitParams(lfd=.4)
    higher = ExitPursuitParams(lfd=.4, steering_gain=1.1)
    np.testing.assert_array_equal(look_ahead_point(path, baseline), look_ahead_point(path, higher))
    a, b = compute_steering(path, baseline), compute_steering(path, higher)
    assert np.tan(b)/higher.wheelbase == pytest.approx(1.1*np.tan(a)/baseline.wheelbase)
    assert np.sign(a) == np.sign(b) == side


def test_gain_does_not_turn_on_straight_path_or_missing_target():
    p = ExitPursuitParams(steering_gain=1.1)
    assert compute_steering(np.array([[0., 0.], [.5, 0.], [1., 0.]]), p) == 0.
    assert compute_steering(np.zeros((0, 2)), p) == 0.


@pytest.mark.parametrize('value', [0., -1., float('nan'), float('inf')])
def test_invalid_gain_is_rejected(value):
    with pytest.raises(ValueError, match='steering_gain'):
        ExitPursuitParams(steering_gain=value)
