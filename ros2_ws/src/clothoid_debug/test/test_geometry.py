from clothoid_debug.geometry import GroundProjection, arc_points
from clothoid_lane.lane_detector import LaneParams, make_warp_matrix


def projection():
    warp = make_warp_matrix(LaneParams(x_h=-28, x_l=370, y_h=70, y_l=40))
    return GroundProjection(warp, (489.14, 489.14, 322.0, 217.0), 0.187, 0.080, 0.23, 0.05)


def test_warp_columns_are_parallel_on_the_floor():
    # the warp was set from straight lines, so a warped column must be straight ahead on the floor
    p = projection()
    near, far = p.warp_to_floor(60, 230), p.warp_to_floor(60, 10)
    assert far[0] > near[0]
    assert abs(far[1] - near[1]) < 0.02


def test_right_of_image_is_negative_y():
    p = projection()
    assert p.warp_to_floor(300, 120)[1] < 0.0 < p.warp_to_floor(20, 120)[1]


def test_arc_turns_to_the_curvature_side():
    assert arc_points(1.0, 0.0)[-1][1] > 0.0
    assert arc_points(-1.0, 0.0)[-1][1] < 0.0
    assert arc_points(0.0, -0.1)[-1] == (0.9, 0.0)
