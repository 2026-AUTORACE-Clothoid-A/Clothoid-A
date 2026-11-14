"""Boundary selection must tolerate a small wiggle before a real corner."""
import math
import numpy as np
import pytest
from clothoid_lidar.cone_sorter import SorterParams
from clothoid_lidar.cone_trace import _chains, _cost, _neighbours


@pytest.mark.parametrize('side', [1, -1])
def test_small_wiggle_then_corner_beats_far_straight_branch(side):
    points = [(x, side*y) for x,y in [
        (.06,.44),(.28,.38),(.55,.375),(.67,.23),(.8,0.),(.88,-.25),(.925,-.45),
        (.875,.48)]]
    p = SorterParams()
    graph = _neighbours(points, p.max_n_neighbors, p.max_dist_between_cones)
    chains = _chains(points, graph, 0, side, p)
    selected = min(chains, key=lambda chain: _cost(points, chain, side, p))
    assert selected == (0,1,2,3,4,5,6)
    # The alternative remains eligible; the fix changes ranking, not visibility.
    assert (0,1,2,7) in chains


def chain_with_turns(turns):
    heading = 0.
    points = [(0.,.4),(.2,.4)]
    for turn in turns:
        heading += math.radians(turn)
        points.append((points[-1][0] + .2*math.cos(heading),
                       points[-1][1] + .2*math.sin(heading)))
    return points


def test_large_zigzag_still_costs_more_than_small_wiggle():
    p = SorterParams()
    small = chain_with_turns([5.,-40.,-10.])
    large = chain_with_turns([40.,-40.,-10.])
    indices = tuple(range(len(small)))
    assert _cost(small,indices,1,p) < _cost(large,indices,1,p)


def test_straight_boundary_still_beats_zigzag_of_same_length():
    p = SorterParams()
    straight = chain_with_turns([0.,0.,0.])
    zigzag = chain_with_turns([30.,-30.,30.])
    indices = tuple(range(len(straight)))
    assert _cost(straight,indices,1,p) < _cost(zigzag,indices,1,p)
