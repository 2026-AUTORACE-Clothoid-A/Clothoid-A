import numpy as np
import pytest

from clothoid_lidar.cone_follower import ConeFollower, FollowerParams
from clothoid_lidar.cone_recovery import current_boundaries, recovery_confirmed, reverse_sweep_clear


def test_new_scan_is_used_immediately_without_mixing_old_boundary():
    assert current_boundaries(2, 1, True, True) == (True, False)
    assert current_boundaries(1, 2, True, True) == (False, True)
    assert current_boundaries(2, 2, True, True) == (True, True)
    assert not recovery_confirmed(2, 1, 1, True, False)
    assert recovery_confirmed(3, 1, 2, True, False)


def test_cones_present_but_two_isolated_seeds_are_not_a_forward_path():
    f = ConeFollower(FollowerParams())
    left, right = np.array([[.2, .4]]), np.array([[.2, -.4]])
    r = f.decide(left, right, True, True, (0.,0.,0.), np.vstack((left,right)), allow_reverse=True)
    assert r.reverse and r.source == 'boundary_recovery'


def test_no_reverse_on_missing_cones_pose_or_unconfirmed_frame():
    left, right = np.array([[.2, .4]]), np.array([[.2, -.4]])
    for pose, obstacles, allow in [(None, right, True), ((0.,0.,0.), None, True),
                                   ((0.,0.,0.), np.zeros((0,2)), True), ((0.,0.,0.), right, False)]:
        r = ConeFollower(FollowerParams()).decide(left,right,False,False,pose,obstacles,allow_reverse=allow)
        assert r.speed == 0. and not r.reverse


def test_recovery_does_not_replace_usable_memory_and_is_bounded_by_travel():
    x=np.arange(0.,1.21,.2);left=np.column_stack((x,np.full(len(x),.4)));right=left*[1.,-1.]
    f=ConeFollower(FollowerParams(lfd=.4))
    f.decide(left,right,True,True,(0.,0.,0.),np.vstack((left,right)))
    r=f.decide(left,right,False,True,(.1,0.,0.),right,allow_reverse=True)
    assert r.speed > 0. and not r.reverse
    f.reset()
    seed=np.array([[.2,.4]])
    r=f.decide(seed,seed,False,False,(0.,0.,0.),seed,allow_reverse=True)
    assert r.reverse
    f.corridor.update_pose((-.2,0.,0.));f.corridor.update_pose((-.41,0.,0.))
    r=f.decide(seed,seed,False,False,(-.41,0.,0.),seed,allow_reverse=True)
    assert not r.reverse and r.source == 'recovery_limit'


def clear(ranges, angle_min=-np.pi, increment=np.pi/180, laser_pose=(0.,0.,0.), steering=.3):
    return reverse_sweep_clear(ranges,angle_min,increment,.05,3.,laser_pose,steering,.2,.1,.4)


def test_reverse_sweep_requires_known_rear_and_respects_laser_transform():
    scan=np.full(361,3.)
    assert clear(scan)
    assert clear(scan,laser_pose=(.12,0.,np.pi))
    assert clear(np.full(361,np.inf))
    assert not clear(np.full(221,3.),angle_min=np.deg2rad(-110))
    scan[:90]=np.nan;scan[-90:]=np.nan
    assert not clear(scan)


def test_reverse_sweep_stops_for_rear_obstacle():
    scan=np.full(361,3.);scan[0]=scan[-1]=.3
    assert not clear(scan,steering=0.)
