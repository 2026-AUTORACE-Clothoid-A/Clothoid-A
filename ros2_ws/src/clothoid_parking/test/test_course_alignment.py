"""Parking collision geometry regression checks."""
import numpy as np
from clothoid_common.parking_geometry import PARKING_GEOMETRY
from clothoid_parking.layout import LayoutParams, ParkingLayout


def test_registration_tolerance_does_not_enlarge_collision_body():
    a=ParkingLayout(LayoutParams(position_tolerance=.07))
    b=ParkingLayout(LayoutParams(position_tolerance=.15))
    assert np.array_equal(a._footprint_offsets,b._footprint_offsets)
    assert np.allclose(np.ptp(a._footprint_offsets,axis=0),(.322+.06,.220+.06))


def test_rule_stage_keeps_lane_width_separate_from_bay_dimensions():
    geometry=PARKING_GEOMETRY
    assert geometry.road_width==.50 and geometry.boundary_spacing==.52
    lines={name:np.array(points) for name,points,_ in geometry.stage_lines()}
    assert np.allclose(lines['entry'][:,0],-.470207)
    assert np.allclose(lines['finish'][:,0],2.039797)
    assert np.isclose(np.ptp(lines['entry'][:,1]),.52)
