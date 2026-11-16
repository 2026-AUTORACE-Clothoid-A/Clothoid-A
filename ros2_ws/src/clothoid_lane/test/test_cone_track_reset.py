"""Cone connection reacquires roles once without changing ordinary tracking."""
from types import SimpleNamespace
from unittest.mock import Mock

from std_msgs.msg import Bool
from clothoid_lane.cone_handover import ConeHandover
from clothoid_lane.lane_main import LaneMainNode
from clothoid_lane.lane_tracker import LaneTracker, _Side
from clothoid_lane.lane_detector import SIDE_LEFT, SIDE_RIGHT


def test_reset_only_boundary_memory():
    detector = LaneTracker.__new__(LaneTracker)
    detector._sides = {SIDE_LEFT: _Side(), SIDE_RIGHT: _Side()}
    detector._sides[SIDE_RIGHT].bottom_x = 120.
    detector._zigzag_left = 8
    detector.reset_tracks()
    assert all(s.bottom_x is None and s.missed == 0 for s in detector._sides.values())
    assert detector._zigzag_left == 8


def test_connection_edge_resets_once_and_rearms_for_next_mission():
    detector = LaneTracker.__new__(LaneTracker)
    detector.reset_tracks = Mock()
    h = ConeHandover()
    n = SimpleNamespace(detector=detector, cone_handover=h, cone_connecting=False,
                        cone_ready_pub=Mock(), get_logger=Mock(return_value=Mock()))
    LaneMainNode.on_cone_connecting(n, Bool(data=True))
    detector.reset_tracks.assert_not_called()
    h.on_cone(True, 0.)
    h.request_exit(True)
    h.hits = 10
    h.release_since = 1.
    for _ in range(20):
        LaneMainNode.on_cone_connecting(n, Bool(data=True))
    detector.reset_tracks.assert_called_once()
    assert h.hits == 0 and not h.ready
    assert n.cone_ready_pub.publish.call_args.args[0].data is False
    LaneMainNode.on_cone_connecting(n, Bool(data=False))
    LaneMainNode.on_cone_connecting(n, Bool(data=True))
    assert detector.reset_tracks.call_count == 2
