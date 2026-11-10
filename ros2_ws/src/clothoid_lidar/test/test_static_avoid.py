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
from clothoid_lidar.section_cues import LaneLine, LaneObservation
from clothoid_lidar.static_avoid import (ACTIVE, IDLE, AvoidParams, Costmap, Inhibit, PoseTracker,
                                         SectionFrame, StaticAvoid, lane_of, line_residuals,
                                         plan_paths, pure_pursuit)

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
        seed=1, t_max=45.0, params=None, see_entry=True, skew_deg=None, start_fit=False, steer_gain=1.0,
        control_every=1, track=None):
    """Drive the section once. Returns a dict of what happened.
    direction: 0 / 1 / None, or (value, x) for a fork sign first seen when the car passes x.
    skew_deg: the section frame is wrong by this much until the camera lane fit, 0.5 m past the entry
    line (a start while leaving the corner). start_fit: corrected 0.3 s after the start instead, as the
    camera start fit does (the entry-line fit stands for a start fit that failed).
    steer_gain: the real wheel angle is this times car 7's table (tools/static_avoid/sim_tracking.py).
    control_every: control() every n-th 0.05 s step, the command held in between (a busy node).
    track: list to append (t, s, d of the car, the plan's path) at each control step while avoiding."""
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
            if avoid.state == ACTIVE:
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
            entry_pose = tracker.at_distance(entry_odo) if entry_odo is not None else None
            if avoid.on_limos(t, obstacles, inh, cmd[0], pose, entry_back, entry_pose):
                out['activated'] = True
                out['events'].append((round(t, 2), 'start', round(bx, 2)))
                out['island_source'] = avoid.island_source
                if skew_deg is not None:
                    avoid.frame = SectionFrame(pose[0], pose[1], pose[2] + math.radians(skew_deg))
                    out['t_start'] = t
            fit_now = (t >= out.get('t_start', t) + 0.3) if start_fit else bx >= 0.5
            if skew_deg is not None and avoid.state == ACTIVE and fit_now and avoid.lane_frame is None:
                # the lane is the world x axis; origin on lane 2 centre at the entry line
                avoid.on_lane_fit(SectionFrame(pose[0] - bx, pose[1] - by, 0.0))
            ok = np.isfinite(r)
            hits = np.column_stack((r[ok] * np.cos(ang[ok]) + LIDAR_X, r[ok] * np.sin(ang[ok])))
            free = np.column_stack((np.cos(ang[~ok]), np.sin(ang[~ok])))
            avoid.on_scan(hits, np.array([LIDAR_X, 0.0]), pose, pose, flag,
                          free_dirs_base=free)
        if int(round(t / dt)) % control_every:
            res = 'hold'
        else:
            res = avoid.control(tracker.latest(), inh, t)
        if res == 'hold':
            pass
        elif res is None:
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
            if track is not None and avoid.plan is not None and avoid.plan.feasible:
                track.append((t, *avoid.lane_pose(tracker.latest())[:2], avoid.plan.path.copy()))
        delta = steer_gain * real_central(cmd[0], cmd[1])
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
    out = run([(1.2, 0.0), (3.2, -w)], direction=1)     # last LIMO in lane 3 before the island
    check(out, w, 3)


def test_without_the_entry_line_the_guess_is_used():
    out = run([(1.2, 0.0)], direction=0, see_entry=False)
    check(out, 0.50, 1)
    assert out['island_source_end'] == 'estimate'


def finished_section():
    a = StaticAvoid(AvoidParams())
    a.activate((0.0, 0.0, 0.0), entry_back=0.7)          # island 3.7~5.45 m
    a.seen_entry = (-0.7, 0.0, 0.0)
    a.stop('hand over 1.00 m before the island (entry line), lane 2', True)
    return a


def test_no_new_start_after_reaching_the_fork():
    a = finished_section()
    limo = np.array([[1.0, 0.0]])
    for k in range(5):                                    # something blocks the lane at the island
        assert not a.on_limos(10 + 0.1 * k, limo, Inhibit(), 0.3, (3.2, 0.0, 0.0))
    for k in range(5):                                    # far on, still the same entry line
        assert not a.on_limos(20 + 0.1 * k, limo, Inhibit(), 0.3, (12.0, 0.0, 0.0),
                              entry_back=2.0, entry_pose=(-0.7, 0.0, 0.0))
    assert a.locked


def test_new_line_without_a_blocker_keeps_the_lock():
    a = finished_section()
    for k in range(5):
        assert not a.on_limos(20 + 0.1 * k, np.zeros((0, 2)), Inhibit(), 0.3, (12.0, 0.0, 0.0),
                              entry_back=1.0, entry_pose=(11.0, 0.0, 0.0))
    assert a.locked


def test_retry_after_a_new_entry_line_starts_a_new_section():
    a = finished_section()
    limo = np.array([[1.0, 0.0]])
    started = [a.on_limos(20 + 0.1 * k, limo, Inhibit(), 0.3, (-5.0, 3.0, 1.0), entry_back=1.0,
                          entry_pose=(-6.0, 3.0, 1.0)) for k in range(4)]
    assert any(started) and not a.locked
    assert a.frame.x0 == pytest.approx(-6.0) and a.island_source == 'entry line'


def test_one_scan_after_a_new_line_keeps_the_finished_section():
    a = finished_section()
    frame = a.frame
    assert not a.on_limos(20.0, np.array([[1.0, 0.0]]), Inhibit(), 0.3, (12.0, 0.0, 0.0),
                          entry_back=1.0, entry_pose=(11.0, 0.0, 0.0))
    assert a.locked and a.frame is frame


def test_handover_mid_section_does_not_lock():
    a = StaticAvoid(AvoidParams())
    a.activate((0.0, 0.0, 0.0))
    a.stop('hand over in lane 1')
    assert not a.locked


def test_alignment_is_not_waited_for_right_at_the_island():
    a = StaticAvoid(AvoidParams())
    a.activate((0.0, 0.0, 0.0), entry_back=0.0, entry_pose=(0.0, 0.0, 0.0))
    a.plan = plan_paths((3.6, 0.25, 0.0), 3.6, a.costmap, None, None, a.p, island=a.island)
    assert a.control((4.0, 0.25, 0.0), Inhibit()) is not None    # 0.4 m before it: still waits
    assert a.control((4.15, 0.25, 0.0), Inhibit()) is None       # 0.25 m before it: goes
    assert a.locked


def test_sign_stand_at_the_island_does_not_start():
    a = StaticAvoid(AvoidParams())
    sign = np.array([[1.2, 0.0]])                 # in lane 2, 1.2 m ahead
    for k in range(5):                            # car 3.1 m past the entry line: sign at 4.3 m
        assert not a.on_limos(0.1 * k, sign, Inhibit(), 0.3, (0.0, 0.0, 0.0), entry_back=3.1)
    started = [a.on_limos(1 + 0.1 * k, sign, Inhibit(), 0.3, (0.0, 0.0, 0.0), entry_back=1.0)
               for k in range(5)]                 # 1.0 m past the line: a LIMO at 2.2 m
    assert any(started)


def test_white_line_far_past_the_start_is_not_the_entry():
    a = StaticAvoid(AvoidParams())
    a.activate((0.0, 0.0, 0.0))
    a.on_entry_crossing((3.0, 0.0, 0.0))           # a line 3 m past the start: not the entry
    assert a.island_source == 'estimate'
    a.on_entry_crossing((0.5, 0.0, 0.0))
    assert a.island_source == 'entry line' and a.entry_s == pytest.approx(0.5)


def test_skewed_frame_is_corrected_by_the_start_fit():
    """Run 10 of 1108: started leaving the corner, the frame 12 deg off the lanes, corrected by the
    camera start fit."""
    w = 0.50
    out = run([(1.5, 0.0), (3.5, 0.0), (3.0, -w)], direction=0, skew_deg=-12.0, start_fit=True)
    check(out, w, 1)


def test_skewed_frame_without_a_start_fit_needs_the_route():
    """The start fit failed: the entry-line fit turns the frame 0.5 m before the lane 2 obstacle.
    One-stage paths stop there; the two-stage route (option `route`) goes around."""
    w = 0.50
    stuck = run([(1.5, 0.0), (3.5, 0.0), (3.0, -w)], direction=0, skew_deg=-12.0)
    assert stuck['x'] < 2.0 and stuck['stops'] > 0
    out = run([(1.5, 0.0), (3.5, 0.0), (3.0, -w)], direction=0, skew_deg=-12.0,
              params=replace(AvoidParams(), route=True))
    check(out, w, 1)


def test_reverse_recovery_backs_up_only_when_allowed():
    blocked = StaticAvoid(AvoidParams(recovery_reverse=True))
    blocked.activate((0.0, 0.0, 0.0))
    blocked.plan = None
    cmds = [blocked.control((0.0, 0.0, 0.0), Inhibit(), 0.1 * k) for k in range(20)]
    assert cmds[0] == (0.0, 0.0) and any(v < 0.0 for v, _ in cmds)
    assert blocked.event == 'stuck: backing up'
    off = StaticAvoid(AvoidParams())
    off.activate((0.0, 0.0, 0.0))
    off.plan = None
    assert all(off.control((0.0, 0.0, 0.0), Inhibit(), 0.1 * k) == (0.0, 0.0) for k in range(30))


def test_reverse_stops_after_its_distance():
    a = StaticAvoid(AvoidParams(recovery_reverse=True))
    a.activate((0.0, 0.0, 0.0))
    a.plan = None
    for k in range(16):
        a.control((0.0, 0.0, 0.0), Inhibit(), 0.1 * k)
    assert a.control((-0.25, 0.0, 0.0), Inhibit(), 1.7) == (0.0, 0.0)      # backed 0.25 m >= 0.2
    assert a.reversed == pytest.approx(0.25)


def test_lane_frame_moves_the_lanes_not_the_costmap():
    p = AvoidParams()
    a = StaticAvoid(p)
    a.activate((0.0, 0.0, 0.0))
    yaw = math.radians(-12.0)
    assert a.on_lane_fit(SectionFrame(0.2, 0.0, yaw))
    assert a.island == (p.island_from_entry_min, p.island_from_entry_max)
    s, d, psi = a.lane_pose((0.2, 0.0, yaw))
    assert s == pytest.approx(0.0) and d == pytest.approx(0.0) and psi == pytest.approx(0.0)
    # an obstacle on lane 2 in lane coordinates, seen in the unrotated costmap
    hits = SectionFrame(0.2, 0.0, yaw).points_from(np.array([[2.0, y] for y in np.arange(-.1, .1, .01)]))
    for _ in range(3):
        a.costmap.update(hits, np.array([0.0, 0.0]))
    assert not a.road_clear(1.8, 2.3) and a.road_clear(3.0, 3.5)


def test_lane_fit_ignores_other_lines_and_big_turns():
    a = StaticAvoid(AvoidParams())
    a.activate((0.0, 0.0, 0.0))
    assert not a.on_lane_fit(SectionFrame(-1.7, 0.0, 0.0))        # stop line before the corner
    assert not a.on_lane_fit(SectionFrame(4.0, 0.0, 0.0))         # a line on the A / B road
    assert not a.on_lane_fit(SectionFrame(0.1, 0.0, math.radians(25.0)))
    assert a.lane_frame is None


def test_slow_update_pulls_towards_the_lines_and_is_limited():
    a = StaticAvoid(AvoidParams(lane_step_dist=0.01))   # sideways part is off by default
    a.activate((0.0, 0.0, 0.0))
    a.on_lane_fit(SectionFrame(0.0, 0.0, 0.0))
    # lines say the lanes are turned 3 deg left and the car is 4 cm right of lane 2 centre
    obs = LaneObservation(math.radians(3.0), [LaneLine(0.29, 'white', 50, 0.3),
                                              LaneLine(-0.21, 'white', 50, 0.3)])
    assert a.update_lane((1.0, 0.0, 0.0), obs)
    assert a.lane_frame.yaw0 == pytest.approx(math.radians(0.5))       # one step only
    assert a.lane_pose((1.0, 0.0, 0.0))[1] == pytest.approx(-0.008, abs=2e-3)
    far = LaneObservation(math.radians(9.0), obs.lines)           # not lane lines
    assert not a.update_lane((1.0, 0.0, 0.0), far)


def test_line_residuals_match_nearest_expected_line():
    p = AvoidParams()
    obs = LaneObservation(0.0, [LaneLine(0.27, 'white', 40, 0.3), LaneLine(0.12, 'white', 40, 0.3)])
    r = line_residuals(0.0, obs, p)
    assert len(r) == 1 and r[0] == pytest.approx(-0.02)


def test_known_direction_hands_over_only_in_the_target_lane():
    a = StaticAvoid(AvoidParams())
    a.activate((0.0, 0.0, 0.0), entry_back=0.0, entry_pose=(0.0, 0.0, 0.0))
    a.direction = 0
    a.plan = plan_paths((3.5, 0.0, 0.0), 3.5, a.costmap, 0, None, a.p, island=a.island)
    assert a.control((3.6, 0.0, 0.0), Inhibit()) is not None     # lane 2, island 0.8 m ahead: keep on
    assert a.state == ACTIVE
    assert a.control((3.6, 0.5, 0.0), Inhibit()) is None         # lane 1 = target: hand over
    assert a.event.startswith('hand over in target lane 1')


def test_unknown_direction_waits_for_the_car_to_be_aligned():
    a = StaticAvoid(AvoidParams())
    a.activate((0.0, 0.0, 0.0), entry_back=0.0, entry_pose=(0.0, 0.0, 0.0))
    a.plan = plan_paths((3.6, 0.25, 0.0), 3.6, a.costmap, None, None, a.p, island=a.island)
    assert a.control((3.6, 0.25, 0.0), Inhibit()) is not None    # on the line between two lanes
    assert a.control((3.6, 0.5, 0.0), Inhibit()) is None


def test_island_forbidden_when_measured_unless_nothing_else():
    p = AvoidParams()
    cm = Costmap(p)
    # direction left, right beside the island: nothing may cross it
    plan = plan_paths((4.2, 0.45, 0.0), 4.2, cm, 0, None, p, island=(4.4, 6.15), forbid_island=True)
    on = (plan.path[:, 0] >= 4.4) & (np.abs(plan.path[:, 1]) < 0.25 + p.car_half_width)
    assert plan.feasible and not on.any()
    # on the island already: still a plan, it must not freeze
    plan = plan_paths((5.0, 0.0, 0.0), 5.0, cm, 0, None, p, island=(4.4, 6.15), forbid_island=True)
    assert plan.feasible


def _active_at_origin(p=None):
    a = StaticAvoid(p or AvoidParams())
    a.activate((0.0, 0.0, 0.0))
    return a


def test_start_fit_turns_the_lanes_and_keeps_the_island():
    a = _active_at_origin()
    island = a.island
    yaw = math.radians(-9.0)
    assert a.on_start_fit(SectionFrame(0.0, 0.0, yaw))
    assert a.island == island and a.island_source == 'estimate'
    assert a.lane_pose((2.0, 0.0, 0.0))[1] != 0.0
    assert not a.on_start_fit(SectionFrame(0.0, 0.0, 0.0))        # once


def test_start_fit_refuses_a_big_turn():
    a = _active_at_origin()
    assert not a.on_start_fit(SectionFrame(0.0, 0.0, math.radians(25.0)))
    assert a.lane_frame is None


def test_entry_line_fit_replaces_the_start_fit():
    a = _active_at_origin()
    a.on_start_fit(SectionFrame(0.0, 0.0, math.radians(-9.0)))
    assert a.on_lane_fit(SectionFrame(0.3, 0.0, math.radians(-8.0)))
    assert a.entry_s == 0.0 and a.island_source == 'entry line'


def test_tracking_runs_without_a_start_fit():
    a = _active_at_origin()
    a.track_from_start_heading()
    assert a.lane_frame is a.frame


@pytest.mark.parametrize('direction', [None, 0, 1])
def test_past_the_island_front_always_hands_over_and_locks(direction):
    a = _active_at_origin()
    a.on_lane_fit(SectionFrame(0.0, 0.0, 0.0))
    a.direction = direction
    a.plan = None
    assert a.control((a.island[0] + 0.01, 0.2, math.radians(30.0)), Inhibit()) is None
    assert a.state != ACTIVE and a.locked


def test_estimated_island_gets_a_margin():
    a = _active_at_origin()
    a.direction = 1
    a.plan = None
    s = a.island[0] + 0.2
    assert a.control((s, 0.1, math.radians(30.0)), Inhibit()) is not None
    assert a.state == ACTIVE
    assert a.control((a.island[0] + a.p.island_past_estimate, 0.1, math.radians(30.0)), Inhibit()) is None


ENTRY = (0.0, 0.0, 0.0)
NOTHING = np.zeros((0, 2))


def _watch(a, back, yaw=0.0):
    return a.on_limos(0.0, NOTHING, Inhibit(), 0.3, (back, 0.0, yaw), entry_back=back, entry_pose=ENTRY)


def test_section_is_done_without_any_obstacle():
    a = StaticAvoid(AvoidParams())
    _watch(a, 4.3)
    assert not a.done and not a.locked
    _watch(a, 4.5)
    assert a.done and a.locked and a.state == IDLE


def test_a_line_followed_by_a_corner_is_not_the_entry():
    a = StaticAvoid(AvoidParams())
    _watch(a, 4.5, yaw=math.radians(80.0))
    assert not a.done and not a.locked


def test_a_hand_over_at_the_island_is_done():
    a = _active_at_origin()
    a.on_lane_fit(SectionFrame(0.0, 0.0, 0.0))
    a.plan = None
    a.control((a.island[0] + 0.01, 0.0, 0.0), Inhibit())
    assert a.done


def test_a_retry_from_a_new_line_clears_done():
    a = StaticAvoid(AvoidParams())
    _watch(a, 4.5)
    new = (20.0, 0.0, 0.0)
    limo = np.array([[1.0, 0.0]])
    started = [a.on_limos(0.1 * k, limo, Inhibit(), 0.3, (21.0, 0.0, 0.0), entry_back=1.0,
                          entry_pose=new) for k in range(5)]
    assert any(started) and not a.done and a.state == ACTIVE


def _drive(tracker, keep, legacy=False, v=0.3, rate=50.0, time=4.0):
    """Straight at v, /odom at `rate` Hz with the driver's accumulated position; keep(k) says whether
    message k reaches the node."""
    tracker.imu(0.0, 0.0)
    n = int(time * rate)
    for k in range(n + 1):
        t = k / rate
        if keep(k):
            tracker.odom(t, v, None if legacy else (v * t, 0.0))
    return v * n / rate


def test_odometer_keeps_the_distance_when_messages_are_dropped():
    tr = PoseTracker()
    true = _drive(tr, lambda k: k % 25 == 0 or k == 200)          # one message per 0.5 s
    assert tr.distance == pytest.approx(true, abs=0.01)
    old = PoseTracker()
    _drive(old, lambda k: k % 25 == 0 or k == 200, legacy=True)    # the 11-10 odometer
    assert old.distance < 0.3 * true


def test_odometer_skips_a_long_silence_and_a_reset():
    tr = PoseTracker()
    tr.imu(0.0, 0.0)
    tr.odom(0.0, 0.3, (0.0, 0.0))
    tr.odom(0.02, 0.3, (0.006, 0.0))
    tr.odom(3.0, 0.3, (0.9, 0.0))           # 3 s without /odom: not trusted
    tr.odom(3.02, 0.3, (0.906, 0.0))
    tr.odom(3.04, 0.3, (5.0, 0.0))          # driver reset jump
    tr.odom(3.06, 0.3, (5.006, 0.0))
    assert tr.distance == pytest.approx(0.018, abs=1e-6)


def test_odometer_goes_back_when_reversing():
    tr = PoseTracker()
    tr.imu(0.0, 0.0)
    tr.odom(0.0, -0.2, (0.0, 0.0))
    tr.odom(0.5, -0.2, (-0.1, 0.0))
    assert tr.distance == pytest.approx(-0.1) and tr.x == pytest.approx(-0.1)
