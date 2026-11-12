"""Legacy reference geometry remains fixed while the recorded vehicle turns."""
import math

from clothoid_parking.bay_finder import Bay, BayParams
from clothoid_parking.planner import Phase, PlanParams
from clothoid_parking.preview import ParkingPreview, entry_path
from clothoid_parking.tracker import Pose2D


def test_reference_uses_legacy_turn_lead_radius_and_rear_axle_goal():
    plan = PlanParams()
    bay = Bay(1., -.5, 0., -math.pi/2, .5, 1., -.3)
    points = entry_path(Pose2D(), bay, plan)
    assert points[0] == (-plan.rear_offset, 0.)
    assert math.isclose(points[1][0], 1.-plan.turn_radius-plan.turn_lead)
    assert math.isclose(points[1][1], 0., abs_tol=1e-9)
    assert math.isclose(points[-2][0], 1.-plan.turn_lead)
    assert math.isclose(points[-2][1], -plan.turn_radius)
    assert math.isclose(points[-1][1], -.5+plan.rear_offset)


def test_preview_keeps_world_reference_after_turn_starts():
    plan = PlanParams(rear_offset=0.)
    preview = ParkingPreview(plan, BayParams(), .103)
    preview.core.set_flag(0., True)
    preview.core.planner.bay = Bay(1., -.5, 0., -math.pi/2, .5, 1., -.3)
    preview.core.tracker.rear.x = 1.-plan.turn_radius-plan.turn_lead+.001
    assert preview.step(.1) is None
    assert preview.core.phase == Phase.TURN
    reference = list(preview.path)
    preview.core.tracker.rear = Pose2D(.65, -.03, -.3)
    preview.step(.2)
    assert preview.path == reference
