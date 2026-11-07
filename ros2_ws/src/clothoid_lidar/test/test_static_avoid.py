"""Closed-loop checks of static_avoid on a simulated 3-lane section.

Car: rear-axle bicycle model driven through limo_base's conversion with steering_scale 1.06 and the
real wheel angle from car 7's measured table (nonlinear, right turns ~2 deg more), /odom 1.4 % short.
Lidar: LIMO rectangles ray-cast at +-110 deg with 239~248 beams and 3 mm noise, 10 Hz.
Before activation a lane_main stand-in follows the nearest lane centre. After the hand-over it does
what lane_main is meant to do at the island: it moves to the A/B lane of the fork sign (nearest of
lanes 1 / 3 without one). The white entry line is at x = 0; the island at 4.35~6.10 m after it.
"""
import math
from dataclasses import replace

import numpy as np
import pytest

from clothoid_lidar.obstacle_extractor import ExtractorParams, extract_obstacles, transform_points
from clothoid_lidar.static_avoid import (ACTIVE, IDLE, AvoidParams, Costmap, Inhibit, PoseTracker,
                                         StaticAvoid, lane_of, plan_paths, pure_pursuit)

L, TRACK, SCALE, MAX_INNER = 0.2, 0.175, 1.06, 0.48869
LIDAR_X = 0.103
LIMO_LEN, LIMO_WID = 0.32, 0.22
ISLAND = (4.35, 6.10)                      # section start = x 0
LIMO_PARAMS = ExtractorParams(cluster_gap=0.08, min_points=3, max_width=0.45, min_width=0.12,
                              cone_radius=0.1)
# car 7, chassis value -> real inner wheel angle (deg), forward/reverse mean (ODOM_STEERING_TEST.md)
VALUES = [0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.444]
LEFT = [0.0, 2.1, 4.1, 7.4, 11.3, 15.9, 20.1, 24.2, 25.5, 28.1]
RIGHT = [0.0, 3.5, 5.8, 9.6, 13.7, 17.6, 21.9, 24.9, 26.4, 27.7]


def central_to_inner(c):
    if abs(c) < 1e-9:
        return 0.0
    return math.copysign(math.atan(L / (L / math.tan(abs(c)) - TRACK / 2)), c)


def inner_to_central(i):
    if abs(i) < 1e-9:
        return 0.0
    return math.copysign(math.atan(L / (L / math.tan(abs(i)) + TRACK / 2)), i)


def real_central(v, wz):
    """What the car really steers for a /cmd_vel, through limo_base and car 7's linkage."""
    if abs(v) < 1e-3:
        return 0.0
    inner = central_to_inner(math.atan(L * wz / v))
    inner = max(-MAX_INNER, min(MAX_INNER, inner))
    value = abs(inner) / SCALE
    real = math.radians(np.interp(value, VALUES, LEFT if inner > 0 else RIGHT))
    return inner_to_central(math.copysign(real, inner))


def rect(cx, cy, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(length / 2, width / 2), (length / 2, -width / 2), (-length / 2, -width / 2),
           (-length / 2, width / 2)]
    return np.array([(cx + c * x - s * y, cy + s * x + c * y) for x, y in pts])


def raycast(origin, yaw, limos, rng):
    n = int(rng.integers(239, 249))
    ang = np.linspace(-math.radians(110), math.radians(110), n)
    world = yaw + ang
    d = np.column_stack((np.cos(world), np.sin(world)))
    r = np.full(n, np.inf)
    for x, y in limos:
        poly = rect(x, y, 0.0, LIMO_LEN, LIMO_WID)
        for k in range(4):
            a, b = poly[k], poly[(k + 1) % 4]
            e = b - a
            w = a - origin
            den = d[:, 0] * (-e[1]) - d[:, 1] * (-e[0])
            ok = np.abs(den) > 1e-9
            t = np.where(ok, (w[0] * (-e[1]) - w[1] * (-e[0])) / np.where(ok, den, 1), np.inf)
            u = np.where(ok, (d[:, 0] * w[1] - d[:, 1] * w[0]) / np.where(ok, den, 1), -1)
            hit = ok & (t > 0) & (u >= 0) & (u <= 1)
            r = np.where(hit, np.minimum(r, t), r)
    r = np.where(r > 8.0, np.inf, r)
    noisy = r + rng.normal(0, 0.003, n)
    return ang, np.where(np.isfinite(r), noisy, np.inf)


def min_gap(car_poly, limos):
    """Smallest distance between the car rectangle and any LIMO rectangle (sampled outlines)."""
    def outline(poly):
        pts = []
        for k in range(4):
            a, b = poly[k], poly[(k + 1) % 4]
            for f in np.linspace(0, 1, 20, endpoint=False):
                pts.append(a + f * (b - a))
        return np.array(pts)
    car = outline(car_poly)
    best = np.inf
    for x, y in limos:
        lp = rect(x, y, 0.0, LIMO_LEN, LIMO_WID)
        inside = (np.abs(car[:, 0] - x) < LIMO_LEN / 2) & (np.abs(car[:, 1] - y) < LIMO_WID / 2)
        if inside.any():
            return -1.0
        best = min(best, float(np.min(np.linalg.norm(car[:, None] - outline(lp)[None], axis=2))))
    return best


def run(limos, direction=0, lane_width=0.50, heading0_deg=0.0, inhibit=None, remove_at=None,
        seed=1, t_max=45.0, params=None, see_entry=True):
    """Drive the section once. Returns a dict of what happened.
    direction: 0 / 1 / None, or (value, x) for a fork sign first seen when the car passes x."""
    rng = np.random.default_rng(seed)
    w = lane_width
    p = params or replace(AvoidParams(), lane_width=w)
    avoid = StaticAvoid(p)
    tracker = PoseTracker()
    xr, yr, th = -2.5 - 0.1, 0.0, math.radians(heading0_deg)   # rear axle
    dt, t = 0.05, 0.0
    limos = list(limos)
    out = dict(min_gap=np.inf, max_abs_y=0.0, island_ok=True, activated=False, events=[],
               stops=0)
    released = False
    entry_odo = None
    cmd = (0.3, 0.0)
    while t < t_max:
        bx, by = xr + 0.1 * math.cos(th), yr + 0.1 * math.sin(th)
        if bx > 6.3:
            break
        if remove_at is not None and t >= remove_at[0] and remove_at[1] in limos:
            limos.remove(remove_at[1])
        tracker.imu(t, th)
        tracker.odom(t, 0.986 * cmd[0])
        if see_entry and entry_odo is None and bx >= 0.0:
            entry_odo = tracker.distance          # the camera's entry-line crossing
            avoid.on_entry_crossing(tracker.latest())
        entry_back = tracker.distance - entry_odo if entry_odo is not None else None
        inh = inhibit or Inhibit()
        flag = direction
        if isinstance(direction, tuple):
            flag = direction[0] if bx >= direction[1] else None
        if int(round(t / dt)) % 2 == 0:                       # 10 Hz scan
            ox, oy = bx + LIDAR_X * math.cos(th), by + LIDAR_X * math.sin(th)
            ang, r = raycast(np.array([ox, oy]), th, limos, rng)
            obstacles = transform_points(
                extract_obstacles(r, ang[0], ang[1] - ang[0], LIMO_PARAMS), LIDAR_X, 0.0, 0.0)
            pose = tracker.latest()
            if avoid.on_limos(t, obstacles, inh, cmd[0], pose, entry_back):
                out['activated'] = True
                out['events'].append((round(t, 2), 'start', round(bx, 2)))
                out['island_source'] = avoid.island_source
            ok = np.isfinite(r)
            hits = np.column_stack((r[ok] * np.cos(ang[ok]) + LIDAR_X, r[ok] * np.sin(ang[ok])))
            free = np.column_stack((np.cos(ang[~ok]), np.sin(ang[~ok])))
            avoid.on_scan(hits, np.array([LIDAR_X, 0.0]), pose, pose, flag,
                          free_dirs_base=free)
        res = avoid.control(tracker.latest(), inh)
        if res is None:
            if avoid.event and (not out['events'] or out['events'][-1][1] != avoid.event):
                out['events'].append((round(t, 2), avoid.event, round(bx, 2)))
                if avoid.event.startswith('hand over'):
                    released = True
                    out['handover_x'] = bx
            # lane_main stand-in: nearest lane centre; after the hand-over the A/B lane
            centre = min((w, 0.0, -w), key=lambda c: abs(c - by))
            if released:
                centre = {0: w, 1: -w}.get(flag, w if by >= 0 else -w)
            xs = bx + 0.05 * np.arange(40)
            path = np.column_stack((xs, np.full(40, centre)))
            delta = pure_pursuit(path, (bx, by, th), p)
            cmd = (0.3, 0.3 * math.tan(delta) / L)
        else:
            cmd = res
            out['stops'] += int(res[0] == 0.0)
        delta = real_central(cmd[0], cmd[1])
        xr += cmd[0] * math.cos(th) * dt
        yr += cmd[0] * math.sin(th) * dt
        th += cmd[0] * math.tan(delta) / L * dt
        t += dt
        car = rect(bx, by, th, 0.32, 0.22)
        out['min_gap'] = min(out['min_gap'], min_gap(car, limos) if limos else np.inf)
        out['max_abs_y'] = max(out['max_abs_y'], float(np.abs(car[:, 1]).max()))
        if ISLAND[0] <= bx <= ISLAND[1] and np.abs(car[:, 1]).min() < w / 2 - 0.02:
            out['island_ok'] = False
    out.update(final_y=by, final_lane=lane_of(by, p), x=bx, released=released, t=t,
               state=avoid.state, island_source_end=avoid.island_source)
    return out


def check(out, w, lane=None):
    assert out['activated'], out
    assert out['min_gap'] > 0.03, out
    assert out['max_abs_y'] < 1.5 * w, out           # never over the yellow outer line
    assert out['island_ok'], out
    assert out['x'] > 6.0, out
    if lane is not None:
        assert out['final_lane'] == lane, out


@pytest.mark.parametrize('direction, lane', [(0, 1), (1, 3)])
def test_one_limo_then_fork_lane(direction, lane):
    out = run([(1.2, 0.0)], direction=direction)
    check(out, 0.50, lane)
    assert out['released']


def test_no_direction_never_on_island():
    out = run([(1.2, 0.0)], direction=None)
    check(out, 0.50)
    assert out['final_lane'] in (1, 3)


def test_staggered_limos():
    w = 0.50
    limos = [(1.0, 0.0), (2.6, w), (2.6, 0.0), (4.2, w)]      # lane 3 is the way through
    out = run(limos, direction=1)
    check(out, w, 3)


def test_left_target_around_two_rows():
    w = 0.50
    limos = [(1.0, 0.0), (1.0, -w), (2.8, 0.0), (2.8, -w)]    # lane 1 open all the way
    out = run(limos, direction=0)
    check(out, w, 1)


def test_wider_lanes_and_heading_error():
    out = run([(1.2, 0.0)], direction=0, lane_width=0.55, heading0_deg=3.0)
    check(out, 0.55, 1)


def test_all_lanes_blocked_waits_then_goes():
    w = 0.50
    wall = [(1.4, w), (1.4, 0.0), (1.4, -w)]
    out = run(wall, direction=1, remove_at=(14.0, (1.4, -w)))
    check(out, w, 3)
    assert out['stops'] > 0


@pytest.mark.parametrize('inhibit', [Inhibit(cone_active=True), Inhibit(red_ratio=0.25),
                                     Inhibit(parking=True), Inhibit(state_command=True)])
def test_inhibited_never_starts(inhibit):
    out = run([(1.2, 0.0)], direction=0, inhibit=inhibit, t_max=6.0)
    assert not out['activated']


def test_low_red_ratio_does_not_inhibit():
    out = run([(1.2, 0.0)], direction=0, inhibit=Inhibit(red_ratio=0.03))
    check(out, 0.50, 1)


# ------------------------------------------------------------------------------------ units
def test_trigger_needs_consecutive_scans_and_rearm():
    a = StaticAvoid(AvoidParams())
    limo = np.array([[1.0, 0.05]])
    pose = (0.0, 0.0, 0.0)
    assert not a.on_limos(0.0, limo, Inhibit(), 0.3, pose)
    assert not a.on_limos(0.1, limo, Inhibit(), 0.3, pose)
    assert a.on_limos(0.2, limo, Inhibit(), 0.3, pose)
    assert a.state == ACTIVE
    a.stop('test')
    assert a.state == IDLE
    # the same LIMO still ahead: no new start until the lane was clear for rearm_clear
    for k in range(5):
        assert not a.on_limos(0.3 + 0.1 * k, limo, Inhibit(), 0.3, pose)
    t = 1.0
    while t < 2.2:
        a.on_limos(t, np.zeros((0, 2)), Inhibit(), 0.3, pose)
        t += 0.1
    for k in range(3):
        started = a.on_limos(t + 0.1 * k, limo, Inhibit(), 0.3, pose)
    assert started


def test_trigger_ignores_side_lanes_and_far_and_standing():
    a = StaticAvoid(AvoidParams())
    pose = (0.0, 0.0, 0.0)
    for k in range(5):
        assert not a.on_limos(0.1 * k, np.array([[1.0, 0.5], [2.5, 0.0]]), Inhibit(), 0.3, pose)
        assert not a.on_limos(1 + 0.1 * k, np.array([[1.0, 0.0]]), Inhibit(), 0.0, pose)


def test_running_avoidance_aborts_on_inhibit():
    a = StaticAvoid(AvoidParams())
    a.activate((0.0, 0.0, 0.0))
    assert a.control((0.0, 0.0, 0.0), Inhibit()) is not None
    assert a.control((0.1, 0.0, 0.0), Inhibit(cone_active=True)) is None
    assert a.state == IDLE and a.event.startswith('abort')


def test_costmap_forgets_a_moved_obstacle():
    p = AvoidParams()
    cm = Costmap(p)
    origin = np.array([0.0, 0.0])
    blob = np.array([[1.0, y] for y in np.arange(-0.1, 0.1, 0.01)])
    for _ in range(3):
        cm.update(blob, origin)
    assert cm.occupied_in(0.9, 1.4, -0.2, 0.2)
    far = np.array([[2.0, y] for y in np.arange(-0.1, 0.1, 0.01)])   # beams now pass through
    for _ in range(12):
        cm.update(far, origin)
    assert not cm.occupied_in(0.95, 1.05, -0.05, 0.05)


def test_planner_prefers_target_lane_and_keeps_off_the_line():
    p = AvoidParams()
    cm = Costmap(p)
    plan = plan_paths((0.0, 0.0, 0.0), 0.0, cm, 1, None, p)
    assert plan.feasible and plan.end_offset == pytest.approx(-p.lane_width, abs=0.051)
    assert np.abs(plan.path[:, 1]).max() < 1.5 * p.lane_width - p.car_half_width


@pytest.mark.parametrize('flag, lane', [(0, 1), (1, 3)])
def test_fork_sign_seen_late(flag, lane):
    """The sign is read only when the car is 2.2 m into the section, after the first LIMO."""
    out = run([(1.2, 0.0)], direction=(flag, 2.2))
    check(out, 0.50, lane)


def test_hands_over_before_the_island():
    for limos in ([(1.2, 0.0)], [(1.0, 0.0), (2.6, 0.5), (2.6, 0.0), (4.2, 0.5)]):
        out = run(limos, direction=1)
        check(out, 0.50, 3)
        assert out['island_source_end'] == 'entry line'    # crossed after the start
        assert out['handover_x'] < ISLAND[0], out


def test_limo_close_to_the_island_is_still_avoided():
    w = 0.50
    out = run([(1.2, 0.0), (3.7, -w)], direction=1)     # last LIMO in lane 3 just before the island
    check(out, w, 3)


def test_without_the_entry_line_the_guess_is_used():
    out = run([(1.2, 0.0)], direction=0, see_entry=False)
    check(out, 0.50, 1)
    assert out['island_source_end'] == 'estimate'


def test_no_new_start_at_the_island():
    a = StaticAvoid(AvoidParams())
    a.activate((0.0, 0.0, 0.0), entry_back=0.7)          # island 3.7~5.45 m
    a.stop('hand over 1.00 m before the island (entry line), lane 2')
    limo = np.array([[1.0, 0.0]])
    for k in range(5):                                    # something blocks the lane at the island
        assert not a.on_limos(10 + 0.1 * k, limo, Inhibit(), 0.3, (3.2, 0.0, 0.0))
    started = [a.on_limos(20 + 0.1 * k, limo, Inhibit(), 0.3, (12.0, 0.0, 0.0))
               for k in range(5)]                          # far beyond the section: normal again
    assert any(started)
