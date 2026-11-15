import numpy as np
from clothoid_lidar.cone_sorter import SorterParams, ConeSorter, ConeMission, SortResult


def fixture(sign=1.):
    # A visible inner corner has a large terminal gap and two supporting cones.
    points = [(.3,.3),(.55,.3),(.58,-.2),(.75,-.4),(.95,-.5),
              (0.,-.5),(.2,-.75),(.45,-.95),(.75,-1.),(1.05,-.82)]
    return [(x, sign*y) for x,y in points]


def test_observed_trace_keeps_side_identity_and_seed_history():
    points=np.array(fixture());mission=ConeMission(ConeSorter(SorterParams()))
    mission.running=True
    mission.previous=SortResult(left=points[:2],right=points[5:])
    first=mission.update(points,0.)
    np.testing.assert_allclose(first.left,mission.previous.left)
    np.testing.assert_allclose(mission.previous.left,points[:2])
    second=mission.update(points,.1)
    np.testing.assert_allclose(second.left,first.left)
    np.testing.assert_allclose(mission.previous.left,points[:2])
