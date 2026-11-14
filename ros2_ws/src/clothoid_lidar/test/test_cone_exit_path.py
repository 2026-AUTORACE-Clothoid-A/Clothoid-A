import numpy as np
from clothoid_lidar.cone_exit import ExitPathMemory, transform, clearance
from clothoid_lidar.cone_sorter import ConeMission,ConeSorter,SorterParams


def line():
    return np.column_stack((np.arange(0., .81, .05),np.zeros(17)))


def test_pose_round_trip():
    p=line();pose=(1.,2.,.8)
    assert np.allclose(transform(transform(p,pose),pose,True),p)


def test_short_or_backward_path_cannot_replace_observed_exit():
    m=ExitPathMemory();assert m.remember(line(),(0,0,0),.4,.05)
    saved=m.world.copy()
    assert not m.remember([[.1,0],[.2,0]],(.1,0,0),.4,.05)
    assert not m.remember(-line(),(.1,0,0),.4,.05)
    assert np.array_equal(saved,m.world)


def test_path_moves_with_ego_and_extension_is_bounded():
    m=ExitPathMemory();m.remember(line(),(0,0,0),.4,.05)
    assert np.allclose(m.world[-1],[1.2,0])
    p=m.remaining((.3,0,0),.1)
    assert np.allclose(p[-1],[.9,0])
    assert len(m.remaining((1.3,0,0),.1))==0


def test_total_travel_expires_memory_even_after_loop():
    m=ExitPathMemory();m.remember(line(),(0,0,0),.4,.05)
    for x in [.5,0,.5,0]:m.update_pose((x,0,0))
    assert len(m.remaining((0,0,0),.1))==0


def test_clearance_checks_between_samples():
    assert clearance([[0,0],[1,0]],[[.5,.1]])<.101
    assert clearance([[0,0],[1,0]],[[.5,.4]])>.39


def test_count_loss_enters_exit_and_requires_ack():
    m=ConeMission(ConeSorter(SorterParams()),hold_exit=True)
    cones=np.array([[x,y]for y in [-.4,.4]for x in [.2,.5,.8]])
    m.update(cones,0.)
    assert m.running and not m.exiting
    assert not m.finish_exit()
    m.update(cones[:3],.1)
    assert m.running and m.exiting
    m.update(np.zeros((0,2)),.2)
    assert m.running and m.exiting
    assert m.finish_exit()
    assert not m.running and not m.exiting


def test_recovered_cone_zone_resumes_normal_tracking():
    m=ConeMission(ConeSorter(SorterParams()),hold_exit=True)
    cones=np.array([[x,y]for y in [-.4,.4]for x in [.2,.5,.8]])
    m.update(cones,0.);m.update(cones[:3],.1);m.update(cones,.2)
    assert m.running and not m.exiting
