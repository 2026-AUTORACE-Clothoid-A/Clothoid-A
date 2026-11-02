"""Offline tests for the lidar cone pipeline: `pytest ros2_ws/src/clothoid_lidar/test`."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from clothoid_lidar.cone_follower import (  # noqa: E402
    ConeFollower, FollowerParams, build_center_line, compute_steering, order_by_mst)
from clothoid_lidar.cone_sorter import (  # noqa: E402
    LEFT, RIGHT, ConeMission, ConeSorter, SorterParams)
from clothoid_lidar.obstacle_extractor import (  # noqa: E402
    ExtractorParams, extract_obstacles, transform_points)

ANGLE_INC = np.deg2rad(0.88)   # ~410 beams per turn like the Tmini Plus
ANGLES = np.arange(-np.pi, np.pi, ANGLE_INC)


def raycast(circles):
    """Ranges of a 2D scan hitting circles given as (x, y, radius)."""
    ranges = np.full(ANGLES.size, np.inf)
    for cx, cy, r in circles:
        for i, a in enumerate(ANGLES):
            d = np.array([np.cos(a), np.sin(a)])
            b = d @ np.array([cx, cy])
            disc = b * b - (cx * cx + cy * cy - r * r)
            if disc < 0 or b < 0:
                continue
            ranges[i] = min(ranges[i], b - np.sqrt(disc))
    return ranges


def extract(circles, **kw):
    return extract_obstacles(raycast(circles), ANGLES[0], ANGLE_INC, ExtractorParams(**kw))


def gate(xs, half_width=0.35):
    left = np.array([[x, half_width] for x in xs])
    right = np.array([[x, -half_width] for x in xs])
    return left, right


def curved_gate(sign=1, radius=1.5, half_width=0.35):
    """Boundaries of a corridor that curves left (sign=1) or right (sign=-1) from the origin."""
    theta = np.arange(0.15, 0.95, 0.15)
    centre = np.column_stack((radius * np.sin(theta), sign * radius * (1 - np.cos(theta))))
    normal = np.column_stack((-np.sin(theta), sign * np.cos(theta)))
    return centre + normal * half_width * sign, centre - normal * half_width * sign


# --- extractor ---------------------------------------------------------------------

def test_extractor_finds_cone_centres():
    cones = [(0.6, 0.3, 0.04), (0.9, -0.3, 0.04), (1.3, 0.2, 0.04)]
    found = extract(cones)
    assert len(found) == 3
    for cx, cy, _ in cones:
        assert np.min(np.linalg.norm(found - [cx, cy], axis=1)) < 0.03


def test_extractor_ignores_wide_objects():
    wall = [(1.0, y, 0.04) for y in np.arange(-0.4, 0.41, 0.04)]  # a 0.8 m wall of beads
    assert len(extract(wall)) == 0


def test_extractor_ignores_targets_outside_range_and_fov():
    assert len(extract([(2.6, 0.0, 0.04)])) == 0           # beyond range_max
    assert len(extract([(-0.8, 0.0, 0.04)])) == 0          # behind the car, outside +-110 deg


def test_extractor_empty_scan():
    assert extract([]).shape == (0, 2)


def test_transform_points_applies_lidar_pose():
    out = transform_points(np.array([[1.0, 0.0]]), tx=0.1, ty=0.0, yaw=np.pi / 2)
    assert out == pytest.approx(np.array([[0.1, 1.0]]), abs=1e-9)


# --- sorter ------------------------------------------------------------------------

def test_first_cone_selection_per_side():
    left, right = gate([0.3, 0.65, 1.0, 1.35])
    cones = np.vstack((left, right))
    sorter = ConeSorter(SorterParams())
    assert np.allclose(cones[sorter.select_first_cone(cones, LEFT)], [0.3, 0.35])
    assert np.allclose(cones[sorter.select_first_cone(cones, RIGHT)], [0.3, -0.35])


def test_first_cone_missing_when_too_far():
    cones = np.array([[1.5, 0.9], [1.5, -0.9]])
    sorter = ConeSorter(SorterParams())
    assert sorter.select_first_cone(cones, LEFT) is None
    assert sorter.select_first_cone(cones, RIGHT) is None


def test_sort_splits_left_and_right_in_driving_order():
    left, right = gate([0.3, 0.65, 1.0, 1.35, 1.7])
    cones = np.vstack((left, right))
    np.random.default_rng(0).shuffle(cones)
    result = ConeSorter(SorterParams()).sort(cones)
    assert np.allclose(result.left, left)
    assert np.allclose(result.right, right)


def test_sort_with_two_cones_sends_only_first_cones():
    cones = np.array([[0.3, 0.35], [0.3, -0.35]])
    result = ConeSorter(SorterParams()).sort(cones)
    assert result.left.shape == (1, 2)
    assert result.right.shape == (1, 2)


def test_mission_starts_continues_and_ends():
    left, right = gate([0.3, 0.65, 1.0, 1.35, 1.7])
    cones = np.vstack((left, right))
    mission = ConeMission(ConeSorter(SorterParams()))

    assert mission.update(np.zeros((0, 2))) is None and not mission.running
    assert mission.update(cones) is None and mission.running   # start frame sends nothing yet
    assert mission.update(cones) is not None                    # running, boundaries sent
    assert mission.update(np.zeros((0, 2))) is None and not mission.running  # left the course


def test_mission_does_not_start_with_too_few_cones_in_zone():
    left, right = gate([0.3, 0.65])  # 4 cones in the start box, 6 needed
    mission = ConeMission(ConeSorter(SorterParams()))
    mission.update(np.vstack((left, right)))
    assert not mission.running


# --- follower ----------------------------------------------------------------------

def test_mst_order_runs_from_the_vehicle_outwards():
    points = np.array([[1.2, 0.0], [0.3, 0.0], [0.9, 0.0], [0.6, 0.0]])
    ordered = order_by_mst(points)
    assert np.allclose(ordered[:, 0], [0.3, 0.6, 0.9, 1.2])


def test_straight_gate_steers_straight():
    left, right = gate([0.3, 0.65, 1.0, 1.35, 1.7])
    params = FollowerParams()
    center = build_center_line(left, right, params)
    assert len(center) > 5
    assert np.abs(center[:, 1]).max() < 0.02
    assert compute_steering(center, params) == pytest.approx(0.0, abs=0.02)


def test_left_curve_steers_left_and_right_curve_steers_right():
    params = FollowerParams()
    left, right = curved_gate(+1)
    assert compute_steering(build_center_line(left, right, params), params) > 0.05
    left, right = curved_gate(-1)
    assert compute_steering(build_center_line(left, right, params), params) < -0.05


def test_pure_pursuit_uses_rear_axle_offset_and_wheelbase_separately():
    # straight diagonal line; the look-ahead (0.3 m of path) lands on (0.3, 0.15)
    center = np.array([[0.1 * i, 0.05 * i] for i in range(8)])
    target = np.array([0.3, 0.15])

    def expected(rear_axle_x, wheelbase, lfd=0.3):
        theta = np.arctan2(target[1], target[0] + rear_axle_x)
        return np.arctan2(2.0 * wheelbase * np.sin(theta), lfd)

    base = FollowerParams(rear_axle_x=0.1, wheelbase=0.2)
    assert compute_steering(center, base) == pytest.approx(expected(0.1, 0.2))
    # each parameter moves the result on its own
    assert compute_steering(center, FollowerParams(rear_axle_x=0.25, wheelbase=0.2)) \
        == pytest.approx(expected(0.25, 0.2))
    assert compute_steering(center, FollowerParams(rear_axle_x=0.1, wheelbase=0.25)) \
        == pytest.approx(expected(0.1, 0.25))
    assert expected(0.1, 0.2) != pytest.approx(expected(0.25, 0.2))
    assert expected(0.1, 0.2) != pytest.approx(expected(0.1, 0.25))


def test_decide_drives_when_both_boundaries_are_fresh():
    left, right = gate([0.3, 0.65, 1.0, 1.35, 1.7])
    result = ConeFollower(FollowerParams(speed=0.3)).decide(left, right, True, True)
    assert result is not None and not result.reverse
    assert result.speed == 0.3
    assert len(result.center) > 0


def test_decide_reverses_with_full_lock_when_one_boundary_is_missing():
    left, right = gate([0.3, 0.65, 1.0])
    follower = ConeFollower(FollowerParams(reverse_speed=-0.2, reverse_steer_deg=21.0))
    only_right = follower.decide(left, right, False, True)
    only_left = follower.decide(left, right, True, False)
    assert only_right.reverse and only_right.speed == -0.2
    assert only_right.steering == pytest.approx(-np.deg2rad(21.0))
    assert only_left.steering == pytest.approx(np.deg2rad(21.0))


def test_decide_publishes_nothing_when_all_boundaries_are_stale():
    left, right = gate([0.3, 0.65, 1.0])
    assert ConeFollower(FollowerParams()).decide(left, right, False, False) is None


def test_collinear_cones_do_not_crash():
    left = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
    right = np.array([[3.0, 0.0], [4.0, 0.0]])
    center = build_center_line(left, right, FollowerParams())
    assert center.ndim == 2


def test_lsq_window_is_forced_odd_and_at_least_five():
    assert FollowerParams(lsq_window_size=2).lsq_window_size == 5
    assert FollowerParams(lsq_window_size=8).lsq_window_size == 9
