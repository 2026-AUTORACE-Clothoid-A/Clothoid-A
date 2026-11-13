"""Geometric and evidence invariants; recorded-data checks live in tools/parking_review."""
import math
import numpy as np
from clothoid_parking.layout import ParkingLayout, LayoutParams
from clothoid_parking.planner import PlanParams
from clothoid_parking.tracker import Pose2D


def registered():
    layout = ParkingLayout()
    for t in (1., 1.2, 1.4):
        layout.observe(t, (1., -.25), 0., Pose2D())
    return layout


def test_six_bays_are_rigidly_registered_not_fixed_in_world():
    layout = ParkingLayout()
    pose = Pose2D(4., 8., math.pi/2)
    for t in (1., 1.2, 1.4):
        layout.observe(t, (1., -.25), 0., pose)
    bays = layout.bays()
    assert len(bays) == 6
    assert np.allclose(layout.origin, (4.25, 9.))
    assert math.isclose(bays[1].y-bays[0].y, .52)
    assert math.isclose(bays[3].x-bays[0].x, .62)
    assert math.isclose(bays[0].entry_yaw, 0.)


def test_duplicate_or_inconsistent_frames_do_not_confirm():
    layout = ParkingLayout()
    for _ in range(10): layout.observe(1., (1., 0.), 0., Pose2D())
    assert layout.origin is None
    layout.observe(1.2, (2., 0.), 0., Pose2D())
    layout.observe(1.4, (1., 0.), 0., Pose2D())
    assert layout.origin is None


def test_registration_disagreement_revokes_existing_target():
    layout = registered()
    assert layout.origin is not None
    layout.observe(1.6, (1.3, -.25), 0., Pose2D())
    assert layout.origin is None and layout.selected is None
    for t in (1.8, 2.): layout.observe(t, (1.3, -.25), 0., Pose2D())
    assert layout.origin is not None and layout.selected is None
    assert layout.support_count == 0


def test_no_return_is_unknown_and_finite_rays_clear_old_obstacles():
    layout = ParkingLayout()
    pose = Pose2D()
    layout.scan(1., [np.inf, np.nan, 0.], -.1, .1, .02, 12., pose, 0.)
    assert layout.coverage([[.4, 0.]], 1.) == 'unknown'
    layout.scan(1.3, [.4], 0., .1, .02, 12., pose, 0.)
    assert layout.coverage([[.4, 0.]], 1.3) == 'unknown'
    layout.scan(1.6, [.4], 0., .1, .02, 12., pose, 0.)
    assert layout.coverage([[.4, 0.]], 1.6) == 'occupied'
    layout.scan(1.9, [1.], 0., .1, .02, 12., pose, 0.)
    assert layout.coverage([[.4, 0.]], 1.9) == 'free'
    assert layout.coverage([[.4, 0.]], 3.5) == 'unknown'


def test_hit_confirmation_needs_distinct_scans_and_expires():
    layout = ParkingLayout()
    def scan(t, ranges):
        return layout.scan(t, ranges, 0., 0., .02, 12., Pose2D(), 0.)
    scan(1., [.4]*8)
    assert layout.coverage([[.4, 0.]], 1.) == 'unknown'
    assert not scan(1., [.4])
    scan(1.3, [.4])
    assert layout.coverage([[.4, 0.]], 1.3) == 'occupied'
    scan(3., [.4])
    assert layout.coverage([[.4, 0.]], 3.) == 'unknown'
    layout.reset()
    scan(3.3, [.4])
    assert layout.coverage([[.4, 0.]], 3.3) == 'unknown'


def test_clearing_resets_hits_and_new_return_revokes_free_evidence():
    layout = ParkingLayout()
    def scan(t, r):
        layout.scan(t, [r], 0., 0., .02, 12., Pose2D(), 0.)
    scan(1., .4)
    scan(1.3, 1.)
    assert layout.coverage([[.4, 0.]], 1.3) == 'free'
    scan(1.6, .4)
    assert layout.coverage([[.4, 0.]], 1.6) == 'unknown'
    scan(1.9, .4)
    assert layout.coverage([[.4, 0.]], 1.9) == 'occupied'


def test_missing_obstacles_cannot_certify_empty_layout():
    layout = registered()
    for t in (1.5, 1.8, 2.):
        assert layout.choose(t, Pose2D(), PlanParams()) is None
    assert layout.selected is None


def fill(layout, points, stamp, occupied=False):
    for key in np.floor(points/layout.p.grid).astype(int):
        layout.cells[tuple(key)] = (stamp, occupied)


def test_inner_bay_route_crosses_front_bay_and_blockage_is_not_empty():
    layout = registered()
    front, inner = layout.bays()[0], layout.bays()[3]
    p = PlanParams()
    path = layout.path(inner, Pose2D(), p)
    fill(layout, path, 2.)
    assert layout.coverage(path, 2.) == 'free'
    fill(layout, np.array([[front.x, front.y]]), 2., True)
    assert layout.coverage(path, 2.) == 'occupied'


def test_all_six_targets_have_a_geometric_route_when_aisle_open():
    layout = registered()
    for bay in layout.bays():
        assert layout.path(bay, Pose2D(), PlanParams()) is not None
    # This checks geometry only, not physical success or lidar visibility.


def test_obstacle_and_unknown_stop_forward_and_reverse_envelopes():
    layout = registered()
    pose = Pose2D()
    assert not layout.motion_clear(2., pose, .2, 0., .2)
    assert not layout.motion_clear(2., pose, -.2, 0., .2)
    points = np.concatenate([layout.footprint(x, 0., 0.) for x in np.linspace(-.25, .25, 80)])
    fill(layout, points, 2.)
    assert layout.motion_clear(2., pose, .2, 0., .2)
    assert layout.motion_clear(2., pose, -.2, 0., .2)
    fill(layout, np.array([[.3, 0.]]), 2., True)
    assert not layout.motion_clear(2., pose, .2, 0., .2)


def test_packed_grid_coordinates_keep_negative_cells_distinct():
    from clothoid_parking.layout import unique_cells
    pts = np.array([[-.01, -.01], [-.01, .01], [.01, -.01], [.01, .01]])
    assert set(map(tuple, unique_cells(pts, .04))) == {(-1,-1),(-1,0),(0,-1),(0,0)}


def test_duplicate_scan_cannot_build_confidence(monkeypatch):
    layout = registered()
    monkeypatch.setattr(layout, 'supported', lambda now: True)
    layout.last_scan = 2.
    for _ in range(10): layout.choose(2., Pose2D(), PlanParams())
    assert layout.support_count == 1


def test_expired_registration_clears_diagnostic_support():
    layout = registered()
    layout.support_count = 3
    layout.states = ['free']*6
    assert layout.choose(30., Pose2D(), PlanParams()) is None
    assert layout.support_count == 0 and layout.states == ['unknown']*6


def test_strict_completion_does_not_report_timeout_as_parked():
    from clothoid_parking.planner import Planner, Phase
    planner = Planner(PlanParams(strict_completion=True))
    planner.start(0., Pose2D())
    planner.bay = registered().bays()[0]
    planner.phase = Phase.ENTER
    assert planner.step(20., Pose2D()) == (0., 0.)
    assert planner.phase == Phase.ABORT
