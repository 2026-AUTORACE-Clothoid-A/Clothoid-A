"""Selection/hand-over invariants; these do not stand in for sensor or driving validation."""
import numpy as np
import pytest
from clothoid_parking.layout import ParkingLayout, LayoutParams
from clothoid_parking.core import ParkingCore
from clothoid_parking.planner import PlanParams, Phase
from clothoid_parking.tracker import Pose2D


def observed(target):
    layout = ParkingLayout(LayoutParams(target_bay=target, control_enabled=True))
    for t in (1., 1.2, 1.4):
        layout.observe(t, (1., -.25), 0., Pose2D())
    return layout


def evidence(layout, now, blocked=None):
    # Explicitly supplied free/occupied cells isolate selection from ray casting.
    x, y = np.meshgrid(np.arange(-.5, 3.5, .02), np.arange(-2., .5, .02))
    for key in np.unique(np.floor(np.column_stack((x.ravel(), y.ravel()))/.04).astype(int), axis=0):
        layout.cells[tuple(key)] = (now, False)
    column = (layout.p.target_bay-1) % 3
    for i, bay in enumerate(layout.bays()[3:]):
        if i == column:
            continue
        for dx in (-.04, 0., .04):
            key = tuple(np.floor(np.array([bay.x+dx, bay.y])/.04).astype(int))
            layout.cells[key] = (now, True)
    if blocked is not None:
        layout.cells[tuple(np.floor(np.asarray(blocked)/.04).astype(int))] = (now, True)
    layout.last_scan = now


@pytest.mark.parametrize('target', range(1, 7))
def test_each_requested_bay_can_be_selected_without_two_flanks(target):
    layout = observed(target)
    for now in (1.6, 1.8, 2., 2.2, 2.4):
        evidence(layout, now)
        result = layout.choose(now, Pose2D(), PlanParams())
    assert result == layout.bays()[target-1]
    assert layout.selected == target-1


def test_requested_inner_bay_never_bypasses_blocked_front_bay():
    layout = observed(5)
    front = layout.bays()[1]
    for now in (1.6, 1.8, 2., 2.2, 2.4):
        evidence(layout, now, (front.x, front.y))
        assert layout.choose(now, Pose2D(), PlanParams()) is None


def test_automatic_selection_prefers_clear_front_row_over_inner_target():
    layout = observed(5)
    for now in (1.6, 1.8, 2., 2.2, 2.4):
        evidence(layout, now)
        layout.choose(now, Pose2D(), PlanParams())
    assert layout.selected == 4
    layout.p.target_bay = 0
    layout.cells = {k: (2.6, occupied) for k, (_, occupied) in layout.cells.items()}
    layout.last_scan = 2.6
    layout.choose(2.6, Pose2D(), PlanParams())
    assert layout.selected in (0, 1, 2)


def test_unobserved_and_expired_evidence_cannot_start_motion():
    layout = observed(1)
    core = ParkingCore(layout=layout.p)
    core.layout = layout
    core.set_flag(1.5, True)
    assert core.command(1.5) == (0., 0.)
    for now in (1.6, 1.8, 2., 2.2, 2.4):
        evidence(layout, now)
        layout.choose(now, Pose2D(), core.planner.p)
    assert layout.selected == 0
    assert core.command(3.) == (0., 0.)
    layout.choose(30., Pose2D(), core.planner.p)
    assert layout.bays() == []


def test_requested_number_is_validated():
    with pytest.raises(ValueError):
        ParkingLayout(LayoutParams(target_bay=7))


def test_scans_between_camera_frames_keep_pending_registration():
    layout = ParkingLayout()
    for t in (1., 1.2, 1.4):
        layout.observe(t, (1., -.25), 0., Pose2D())
        layout.choose(t+.01, Pose2D(), PlanParams())
    assert len(layout.bays()) == 6


def test_grid_probe_publishes_verified_target_without_legacy_gap(monkeypatch):
    rclpy = pytest.importorskip('rclpy')
    from sensor_msgs.msg import LaserScan
    from clothoid_parking.parking_node import ParkingNode
    rclpy.init(args=['--ros-args', '-p', 'layout.control_enabled:=true'])
    node = ParkingNode()
    try:
        now = node.now()
        layout = observed(1)
        for t in (now-.8, now-.6, now-.4, now-.2, now):
            layout.last_observation = t
            evidence(layout, t)
            layout.choose(t, Pose2D(), node.core.planner.p)
        node.core.layout = layout
        monkeypatch.setattr(node.probe, 'scan', lambda *a: pytest.fail('legacy probe used'))
        sent = []
        monkeypatch.setattr(node.candidate_pub, 'publish', sent.append)
        node.on_probe_scan(LaserScan())
        assert len(sent) == 1
        assert sent[0].pose.position.x == pytest.approx(layout.bays()[0].face_x)
        assert node.core.phase == Phase.IDLE
    finally:
        node.destroy_node()
        rclpy.shutdown()
