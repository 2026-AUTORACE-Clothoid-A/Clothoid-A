"""The view must not display expired evidence as free space."""
import pytest
pytest.importorskip('rclpy')
from builtin_interfaces.msg import Time
from clothoid_parking.core import ParkingCore
from clothoid_parking.layout import LayoutParams
from clothoid_parking.viz import observation_markers


def test_map_displays_fresh_evidence_and_expires_old_cells():
    core = ParkingCore(layout=LayoutParams())
    core.layout.cells = {(0, 0): (10., False), (1, 0): (10., True), (2, 0): (7., False)}
    markers = {m.ns: m for m in observation_markers(core, Time(), 10.1).markers}
    assert len(markers['observed_free'].points) == 1
    assert len(markers['observed_occupied'].points) == 1
    assert 'grey=unknown' in markers['observation_status'].text
    expired = {m.ns: m for m in observation_markers(core, Time(), 12.).markers}
    assert not expired['observed_free'].points and not expired['observed_occupied'].points
    assert all(m.header.frame_id == 'base_link' for m in markers.values())
