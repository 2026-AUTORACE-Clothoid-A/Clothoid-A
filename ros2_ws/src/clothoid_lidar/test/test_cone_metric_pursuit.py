import math
import numpy as np
import pytest

from clothoid_lidar.cone_follower import FollowerParams, compute_steering, look_ahead_point


@pytest.mark.parametrize('side', [1., -1.])
@pytest.mark.parametrize('radius', [.5, .8])
@pytest.mark.parametrize('angle', [.35, .8])
def test_targets_on_same_rear_axle_circle_require_same_steering(side, radius, angle):
    p=FollowerParams(lfd=.3, steering_gain=1.1)
    target=np.array([radius*math.sin(angle)-p.rear_axle_x, side*radius*(1-math.cos(angle))])
    path=np.array([[0.,0.],target])
    np.testing.assert_array_equal(look_ahead_point(path,p),target)
    assert compute_steering(path,p)==pytest.approx(side*math.atan(p.steering_gain*p.wheelbase/radius))


def test_lfd_does_not_change_steering_when_selected_target_is_unchanged():
    path=np.array([[0.,0.],[.5,.2]])
    short,long=FollowerParams(lfd=.2),FollowerParams(lfd=.6)
    np.testing.assert_array_equal(look_ahead_point(path,short),look_ahead_point(path,long))
    assert compute_steering(path,short)==pytest.approx(compute_steering(path,long))


def test_target_at_rear_axle_does_not_produce_a_nonfinite_command():
    p=FollowerParams()
    path=np.array([[0.,0.],[-p.rear_axle_x,0.]])
    assert compute_steering(path,p)==0.
