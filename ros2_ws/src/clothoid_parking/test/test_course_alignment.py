"""Parking collision geometry regression checks."""
import numpy as np
from clothoid_parking.layout import LayoutParams, ParkingLayout


def test_registration_tolerance_does_not_enlarge_collision_body():
    a=ParkingLayout(LayoutParams(position_tolerance=.07))
    b=ParkingLayout(LayoutParams(position_tolerance=.15))
    assert np.array_equal(a._footprint_offsets,b._footprint_offsets)
    assert np.allclose(np.ptp(a._footprint_offsets,axis=0),(.322+.06,.220+.06))
