import numpy as np
import pytest
from clothoid_lidar.cone_path_relaxation import PathRelaxation
from clothoid_lidar.cone_corner import _curvature, _distance
from clothoid_lidar.cone_exit import transform


def scene(shape):
    x = np.linspace(0., 2., 41)
    y = {'straight':np.zeros_like(x), 'left':.18*x*x,
         'right':-.18*x*x, 's':.14*np.sin(3*x)}[shape]
    center=np.column_stack((x,y))
    tangent=np.gradient(center,axis=0)
    normal=np.column_stack((-tangent[:,1],tangent[:,0]))/np.linalg.norm(tangent,axis=1)[:,None]
    return center,center+.4*normal,center-.4*normal


@pytest.mark.parametrize('shape',['straight','left','right','s'])
def test_curvature_and_observed_clearance(shape):
    c,l,r=scene(shape)
    got=PathRelaxation().refine(c,l,r,np.vstack((l,r)),(0.,0.,0.),.05)
    assert np.max(_curvature(got))<=max(np.max(_curvature(c)), 1/(.2/np.tan(np.deg2rad(28.))+.175/2))+1e-6
    assert min(_distance(got,l).min(),_distance(got,r).min())>.25
    np.testing.assert_allclose(got[[0,-1]],c[[0,-1]])
    if shape=='straight':np.testing.assert_allclose(got,c,atol=1e-12)
    elif shape=='s':assert np.max(np.linalg.norm(got-c,axis=1))>1e-5


def test_mirror_and_rotation_do_not_change_rule():
    c,l,r=scene('left');base=PathRelaxation().refine(c,l,r,np.vstack((l,r)),(0.,0.,0.),.05)
    flip=np.array([1.,-1.]);mirrored=PathRelaxation().refine(c*flip,r*flip,l*flip,np.vstack((l,r))*flip,(0.,0.,0.),.05)
    np.testing.assert_allclose(mirrored,base*flip,atol=1e-10)
    pose=(.3,-.2,.7)
    got=PathRelaxation().refine(transform(c,pose),transform(l,pose),transform(r,pose),transform(np.vstack((l,r)),pose),(0.,0.,0.),.05)
    np.testing.assert_allclose(got,transform(base,pose),atol=1e-10)


def test_missing_or_blocked_observation_does_not_keep_old_offset():
    c,l,r=scene('left');f=PathRelaxation()
    f.refine(c,l,r,np.vstack((l,r)),(0.,0.,0.),.05)
    got=f.refine(c,l,r,np.vstack((l,r,c[20:21])),(0.,0.,0.),.05)
    np.testing.assert_array_equal(got,c)
    np.testing.assert_array_equal(f.refine(c,l,r,None,None,.05),c)
    assert f.world is None


def test_pose_transformed_history_and_reset():
    c,l,r=scene('left');f=PathRelaxation()
    f.refine(c,l,r,np.vstack((l,r)),(0.,0.,0.),.05)
    pose=(.01,0.,.01)
    got=f.refine(transform(c,pose,True),transform(l,pose,True),transform(r,pose,True),transform(np.vstack((l,r)),pose,True),pose,.05)
    assert np.isfinite(got).all()
    f.clear();assert f.world is None


def test_zero_lateral_allowance_keeps_path_and_refreshes_pose_history():
    c, l, r = scene('s')
    f = PathRelaxation()
    f.refine(c, l, r, np.vstack((l, r)), (0., 0., 0.), .05)
    # Neither side leaves room for a lateral update inside the body envelope.
    x = np.linspace(0., 1., 21)
    narrow = np.column_stack((x, np.zeros_like(x)))
    left, right = narrow+[0., .2], narrow-[0., .2]
    pose = (.02, .01, .03)
    result = f.refine(narrow, left, right, np.vstack((left, right)), pose, .05)
    np.testing.assert_array_equal(result, narrow)
    np.testing.assert_array_equal(f.world, transform(narrow, pose))
    np.testing.assert_array_equal(f.pose, pose)
