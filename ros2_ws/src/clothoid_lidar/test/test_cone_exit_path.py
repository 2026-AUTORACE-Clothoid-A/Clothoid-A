import numpy as np
import pytest
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


def test_static_avoid_start_hold_preserves_timestamped_exit_handshake():
    m = ConeMission(ConeSorter(SorterParams()), hold_exit=True)
    cones = np.array([[x, y] for y in [-.4, .4] for x in [.2, .5, .8]])
    m.update(cones, 0., may_start=False)
    assert not m.running
    m.update(cones, .1, may_start=True)
    assert m.running and not m.exiting
    assert m.update(cones, .2, may_start=False) is not None
    m.update(np.zeros((0, 2)), .3, may_start=False)
    assert m.running and m.exiting
    assert m.finish_exit()
    m.update(cones, .4, may_start=False)
    assert not m.running and not m.exiting


def test_lane_handover_waits_for_rear_axle_to_reach_observed_endpoint():
    m=ExitPathMemory();assert m.remember(line(),(0.,0.,0.),.35,.05)
    assert m.front_axle_remaining((.69,0.,0.),.1,.2)==pytest.approx(.01)
    assert m.front_axle_remaining((.70,0.,0.),.1,.2)==pytest.approx(0.)
    assert m.rear_axle_remaining((.70,0.,0.),.1)==pytest.approx(.20)
    assert not m.handover_ready((.70,0.,0.),.1)
    assert not m.handover_ready((.89,0.,0.),.1)
    assert m.handover_ready((.90,0.,0.),.1)
    assert m.handover_ready((.91,0.,0.),.1)
    # The extrapolated tail still lies 35 cm beyond the observed endpoint.
    np.testing.assert_allclose(m.world[-1],[1.15,0.])
    assert len(m.remaining((.91,0.,0.),.1))>=2


@pytest.mark.parametrize('angle',[-2.,-.7,.7,2.])
def test_progress_is_in_observed_exit_direction_not_world_x(angle):
    m=ExitPathMemory();origin=(1.,-2.,angle)
    assert m.remember(line(),origin,.35,.05)
    before=transform(np.array([[.89,0.]]),origin)[0]
    after=transform(np.array([[.91,0.]]),origin)[0]
    assert not m.handover_ready((*before,angle),.1)
    assert m.handover_ready((*after,angle),.1)


def test_progress_uses_rear_offset_and_vehicle_heading():
    m=ExitPathMemory();m.remember(line(),(0.,0.,0.),.35,.05)
    assert m.front_axle_remaining((.7,0.,0.),.15,.25)==pytest.approx(0.)
    assert not m.handover_ready((.94,0.,0.),.15)
    assert m.handover_ready((.95,0.,0.),.15)
    assert not m.handover_ready((.79,0.,np.pi/2),.1)
    assert m.handover_ready((.80,0.,np.pi/2),.1)
    # Crossing the progress plane is allowed without exactly hitting one point.
    assert m.handover_ready((.91,.05,0.),.1)


@pytest.mark.parametrize('heading',[0.,-.45,.45])
def test_front_crossing_on_a_curve_cannot_release_rear_axle_pursuit(heading):
    m=ExitPathMemory();m.remember(line(),(0.,0.,0.),.35,.05)
    # Front axle is just past the endpoint while the car is still finishing the turn.
    base=np.array([.801,.08])-.1*np.array([np.cos(heading),np.sin(heading)])
    pose=(*base,heading)
    assert m.front_axle_remaining(pose,.1,.2)<0.
    assert m.rear_axle_remaining(pose,.1)>.17
    assert not m.handover_ready(pose,.1)


@pytest.mark.parametrize('pose',[None,(np.nan,0.,0.),(0.,0.,np.inf)])
def test_known_endpoint_cannot_be_released_without_valid_pose(pose):
    m=ExitPathMemory();m.remember(line(),(0.,0.,0.),.35,.05)
    assert m.front_axle_remaining(pose,.1,.2) is None
    assert m.rear_axle_remaining(pose,.1) is None
    assert not m.handover_ready(pose,.1)


def test_rejected_observation_preserves_endpoint_and_clear_removes_it():
    m=ExitPathMemory();m.remember(line(),(0.,0.,0.),.35,.05)
    endpoint=m.observed_end.copy();direction=m.end_direction.copy()
    assert not m.remember([[.1,0.],[.2,0.]],(.1,0.,0.),.35,.05)
    np.testing.assert_array_equal(m.observed_end,endpoint)
    np.testing.assert_array_equal(m.end_direction,direction)
    m.clear()
    assert m.observed_end is None and m.end_direction is None
    # Lane readiness alone cannot certify rear-axle arrival at an unknown endpoint.
    assert not m.handover_ready(None,.1)
    assert not m.handover_ready((1.,0.,0.),.1)
