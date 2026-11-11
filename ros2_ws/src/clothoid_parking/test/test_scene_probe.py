import math

import numpy as np

import scene
from clothoid_parking.bay_finder import BayParams
from clothoid_parking.scene_probe import ParkingSceneProbe


def test_real_scan_geometry_requires_several_distinct_scans():
    probe = ParkingSceneProbe(BayParams())
    segments = scene.segments(scene.parked_cars(1), True)
    confirmed = []
    for now in (0., .4, .8):
        ranges, angle, increment = scene.scan(segments, 5.7 + .1 * now, scene.LANE_Y, 0.)
        _, bay = probe.scan(now, ranges, angle, increment)
        confirmed.append(bay)
    assert confirmed[0] is None and confirmed[1] is None
    assert confirmed[2] is not None


def test_duplicate_scan_cannot_confirm_and_gaps_discard_history():
    probe = ParkingSceneProbe(BayParams())
    segments = scene.segments(scene.parked_cars(1), True)
    ranges, angle, increment = scene.scan(segments, 5.5, scene.LANE_Y, 0.)
    for now in (0., 0., 0., 2., 2.4):
        _, bay = probe.scan(now, ranges, angle, increment)
        assert bay is None


def test_empty_and_fully_occupied_scenes_are_not_bays():
    empty_probe = ParkingSceneProbe(BayParams())
    for now in (0., .4, .8):
        assert empty_probe.scan(now, [float('inf')] * 360, -math.pi, .02)[1] is None
    for cars in ([(x, scene.UPPER_Y, math.pi / 2) for x in scene.COLUMNS],):
        probe = ParkingSceneProbe(BayParams())
        segments = scene.segments(cars, False)
        for now in (0., .4, .8):
            ranges, angle, increment = scene.scan(segments, 5.5, scene.LANE_Y, 0.)
            assert probe.scan(now, ranges, angle, increment)[1] is None
