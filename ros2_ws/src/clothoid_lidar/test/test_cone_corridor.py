import numpy as np
import pytest

from clothoid_lidar.cone_corridor import CorridorMemory, offset_center, width_samples
from clothoid_lidar.cone_exit import transform
from clothoid_lidar.cone_follower import ConeFollower, FollowerParams


def walls():
    x = np.arange(.0, 1.21, .2)
    return np.column_stack((x, np.full(len(x), .4))), np.column_stack((x, np.full(len(x), -.4)))


def trained():
    left, right = walls()
    f = ConeFollower(FollowerParams(reverse_speed=0., lfd=.4))
    r = f.decide(left, right, True, True, (0., 0., 0.), np.vstack((left, right)))
    assert r.speed > 0. and f.corridor.width == pytest.approx(.8)
    return f, left, right


def test_width_is_perpendicular_and_does_not_use_nonoverlapping_endpoints():
    left, right = walls()
    assert np.median(width_samples(left, right)) == pytest.approx(.8)
    assert not width_samples(left[:2], right[-2:])


@pytest.mark.parametrize('side', [1, -1])
def test_offset_center_has_correct_inward_side(side):
    left, right = walls()
    wall = left if side == 1 else right
    center = offset_center(wall, side, .8, .05)
    assert len(center) > 2
    assert np.max(np.abs(center[:, 1])) < 1e-9


@pytest.mark.parametrize('missing', ['left', 'right'])
def test_missing_side_keeps_forward_path_and_compensates_translation_rotation(missing):
    f, left, right = trained()
    pose = (.25, .03, .15)
    left, right = transform(left, pose, True), transform(right, pose, True)
    r = f.decide(left, right, missing != 'left', missing != 'right', pose, np.vstack((left, right)))
    assert r.speed > 0. and not r.reverse
    assert r.source == 'retained_boundary'
    assert np.max(np.abs(transform(r.center, pose)[:, 1])) < 1e-8


def test_inferred_paths_do_not_train_width_or_refresh_saved_geometry():
    f, left, right = trained()
    world = f.corridor.path.copy()
    samples = list(f.corridor.widths)
    for x in np.arange(.1, .6, .1):
        pose = (x, 0., 0.)
        r = f.decide(left, transform(right, pose, True), False, True, pose, np.zeros((0, 2)))
        assert r.speed > 0.
    np.testing.assert_array_equal(f.corridor.path, world)
    assert list(f.corridor.widths) == samples


def test_width_offset_can_extend_to_current_observations_after_saved_path_is_used():
    f, left, right = trained()
    f.corridor.update_pose((.6, 0., 0.))
    pose = (1.3, 0., 0.)
    current = np.array([[.1, -.4], [.4, -.4], [.7, -.4], [1., -.4]])
    r = f.decide(left, current, False, True, pose, current)
    assert r.speed > 0.
    # After both retained walls/path are behind, use the freshly observed wall.
    assert r.source == 'width_offset'
    assert np.max(np.abs(r.center[:, 1])) < 1e-8


def test_blocked_inference_and_stale_pose_do_not_command_forward():
    f, left, right = trained()
    for pose, obstacles in [(None, right), ((0., 0., 0.), None),
                            ((0., 0., 0.), np.array([[.1, 0.], [.4, 0.], [.7, 0.], [1., 0.]]))]:
        r = f.decide(left, right, False, True, pose, obstacles)
        assert r.speed == 0. and not r.reverse


def test_inference_expires_by_travel_and_clears_on_new_mission():
    f, left, right = trained()
    for x in [.8, 0., .8, 0.]:
        f.corridor.update_pose((x, 0., 0.))
    r = f.decide(left, right, False, True, (0., 0., 0.), right)
    assert r.speed == 0.
    f.reset()
    assert f.corridor.width is None and f.corridor.path is None


def test_pose_jump_discards_saved_corridor():
    f, left, right = trained()
    r = f.decide(left, right, False, True, (5., 0., 0.), right)
    assert r.speed == 0. and f.corridor.path is None
