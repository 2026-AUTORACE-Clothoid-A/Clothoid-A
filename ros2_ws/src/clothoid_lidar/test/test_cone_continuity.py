import numpy as np
import pytest

from clothoid_lidar.cone_sorter import ConeMission, ConeSorter, SorterParams, SortResult, LEFT


def test_observed_boundary_survives_entry_angle_gate():
    sorter = ConeSorter(SorterParams())
    previous = SortResult(left=np.array([[.6, .2]]), right=np.array([[.1, -.4]]))
    cones = np.array([[.65, .19], [.1, -.4]])
    assert sorter.select_first_cone(cones, LEFT) is None
    result = sorter.sort(cones, previous)
    np.testing.assert_allclose(result.left, cones[:1])
    # No persistence when the observation disappears.
    assert not len(sorter.sort(cones[1:], result).left)


def test_far_or_ambiguous_observation_is_not_tracked():
    sorter = ConeSorter(SorterParams())
    prior = np.array([[.6, .2]])
    assert sorter._tracked_seed(np.array([[1., .2]]), prior, np.zeros((0, 2))) is None
    assert sorter._tracked_seed(prior, prior, prior) is None


def test_scan_gap_clears_tracking_and_course_end_is_immediate():
    mission = ConeMission(ConeSorter(SorterParams()))
    cones = np.array([[.3, .35], [.6, .35], [.9, .35], [.3, -.35], [.6, -.35], [.9, -.35]])
    mission.update(cones, 0.)
    assert mission.running
    moved = cones+[.15, 0.]
    assert len(mission.update(moved, .1).left)
    # These do not pass strict seed angle; a stale history cannot rescue them.
    assert not len(mission.update(moved, 1.).left)
    assert mission.update(np.zeros((0, 2)), 1.1) is None
    assert not mission.running and mission.previous is None
