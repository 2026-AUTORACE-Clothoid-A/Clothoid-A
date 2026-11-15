"""Prefer nearby feasible continuations without turning distance into a veto."""
import pytest

from clothoid_lidar.cone_sorter import SorterParams
from clothoid_lidar.cone_trace import (
    _can_add, _chains, _cost, _neighbours, _skip_distance_cost, best_chains,
)


@pytest.mark.parametrize('side', [1, -1])
def test_near_corner_beats_equal_length_far_smooth_branch(side):
    # Rounded geometry of the 050802 fork; both four-cone chains are eligible.
    points = [(x, side*y) for x, y in [
        (.0073, .4343), (.2197, .3854), (.4792, .3968),
        (.6103, .2600), (.8038, .5292)]]
    p = SorterParams()
    graph = _neighbours(points, p.max_n_neighbors, p.max_dist_between_cones)
    chains = _chains(points, graph, 0, side, p)
    assert (0, 1, 2, 3) in chains
    assert (0, 1, 2, 4) in chains
    selected = best_chains(points, 0, None, p, left=side)[0]
    assert selected == [0, 1, 2, 3]


def test_visited_and_ineligible_near_points_do_not_add_cost():
    points = [(0., .4), (.2, .4), (.4, .4), (.6, .4),
              (.39, .4), (.4, .41)]
    chain = (0, 1, 2, 3)
    p = SorterParams()
    graph = _neighbours(points, p.max_n_neighbors, p.max_dist_between_cones)
    for candidate in (1, 4, 5):
        assert not _can_add(points, chain, 2, candidate, 1, p)
    # Test the final decision separately: its only legal continuation is .2 m.
    cache = {chain[:1]: .2, chain[:2]: .2}
    assert _skip_distance_cost(points, chain, 1, p, graph, cache) == pytest.approx(0.)


def test_far_branch_can_still_win_with_supported_continuation():
    points = [(.0073, .4343), (.2197, .3854), (.4792, .3968),
              (.6103, .2600), (.8038, .5292), (1.0038, .6092),
              (1.2038, .6892), (1.4038, .7692)]
    p = SorterParams()
    graph = _neighbours(points, p.max_n_neighbors, p.max_dist_between_cones)
    near, far = (0, 1, 2, 3), (0, 1, 2, 4, 5, 6, 7)
    chains = _chains(points, graph, 0, 1, p)
    assert near in chains and far in chains
    assert _cost(points, far, 1, p, graph) < _cost(points, near, 1, p, graph)


def test_prefix_cache_does_not_change_scores():
    points = [(0., .4), (.2, .4), (.4, .4), (.6, .4), (.6, .5)]
    p = SorterParams()
    graph = _neighbours(points, p.max_n_neighbors, p.max_dist_between_cones)
    cache = {}
    for chain in _chains(points, graph, 0, 1, p):
        assert _cost(points, chain, 1, p, graph, cache) == _cost(points, chain, 1, p, graph)
