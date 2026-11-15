import numpy as np
import pytest

from clothoid_lidar.cone_follower import ConeFollower, FollowerParams


@pytest.mark.parametrize('left_fresh,right_fresh', [(True, False), (False, True)])
def test_disabled_reverse_stops_on_either_boundary_loss(left_fresh, right_fresh):
    follower = ConeFollower(FollowerParams(reverse_speed=0.0))
    left = np.array([[.1, .4], [.4, .4]])
    right = np.array([[.1, -.4], [.4, -.4]])
    result = follower.decide(left, right, left_fresh, right_fresh)
    assert result is not None
    assert result.speed == 0.0 and result.steering == 0.0
    assert not result.reverse
    assert len(result.center) == 0 and result.target is None
