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
        control_every=1, track=None, line_start=False, lane_speed=0.3):
    """Drive the section once. Returns a dict of what happened.
    direction: 0 / 1 / None, or (value, x) for a fork sign first seen when the car passes x.
    skew_deg: the section frame is wrong by this much until the camera lane fit, 0.5 m past the entry
    line (a start while leaving the corner). start_fit: corrected 0.3 s after the start instead, as the
    camera start fit does (the entry-line fit stands for a start fit that failed).
    steer_gain: the real wheel angle is this times car 7's table (tools/static_avoid/sim_tracking.py).
    control_every: control() every n-th 0.05 s step, the command held in between (a busy node).
    track: list to append (t, s, d of the car, the plan's path) at each control step while avoiding.
    line_start: the race start (course map): the entry line starts the mission, lane_main drives up
    to it at approach_speed, a blocker alone starts nothing."""
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
            if line_start:
                avoid.request_line_start(tracker.at_distance(entry_odo))
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
            if avoid.on_limos(t, obstacles, inh, cmd[0], pose, entry_back, entry_pose, not line_start):
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
            v = p.approach_speed if line_start and not released else lane_speed
            cmd = (v, v * math.tan(delta) / L)
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
            # the painted island narrows to a point over its first 0.25 m: the car over that tip only
            # is counted apart (allowed when the A / B lane cannot be reached earlier)
            tip = min((bx - ISLAND[0]) / 0.25, 1.0) * (w / 2 - 0.02)
            if np.abs(car[:, 1]).min() < tip or bx > ISLAND[0] + 0.25:
                out['island_ok'] = False
            else:
                out['island_tip'] = True
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


def test_hands_over_before_the_island_when_asked():
    for limos in ([(1.2, 0.0)], [(1.0, 0.0), (2.6, 0.5), (2.6, 0.0), (4.2, 0.5)]):
        out = run(limos, direction=1, params=AvoidParams(handover_after_island=False))
        check(out, 0.50, 3)
        assert out['island_source_end'] == 'entry line'    # crossed after the start
        assert out['handover_x'] < ISLAND[0], out


def test_hands_over_beside_the_island_in_the_sign_lane():
    """1112 auto 05: handed over in lane 1 before the island, lane_main turned for lane 2 into the
    island's sign. Now the avoidance drives on beside the island and hands over there."""
    for direction, lane, limos in ((0, 1, [(1.2, 0.0)]), (1, 3, [(1.2, 0.0)]),
                                   (1, 3, [(1.0, 0.0), (2.6, 0.5), (2.6, 0.0), (4.2, 0.5)])):
        out = run(limos, direction=direction)
        check(out, 0.50, lane)
        assert ISLAND[0] + 0.55 <= out['handover_x'] <= ISLAND[0] + 1.3, out
        assert 'beside the island, lane %d' % lane in out['events'][-1][1], out


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
    a = StaticAvoid(AvoidParams(handover_after_island=False))
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
    a.on_entry_crossing((3.5, 0.0, 0.0))           # past entry_late_max (3 m): not the entry
    assert a.island_source == 'estimate'
    a.on_entry_crossing((0.5, 0.0, 0.0))
    assert a.island_source == 'entry line' and a.entry_s == pytest.approx(0.5)


def test_skewed_frame_is_corrected_by_the_start_fit():
    """Run 10 of 1108: started leaving the corner, the frame 12 deg off the lanes, corrected by the
    camera start fit."""
    w = 0.50
    out = run([(1.5, 0.0), (3.5, 0.0), (3.0, -w)], direction=0, skew_deg=-12.0, start_fit=True)
    check(out, w, 1)


def test_skewed_frame_without_a_start_fit():
    """The start fit failed: the entry-line fit turns the frame 0.5 m before the lane 2 obstacle.
    One-stage paths stop there; the lane sequence (and the two-stage route) go around."""
    w = 0.50
    limos = [(1.5, 0.0), (3.5, 0.0), (3.0, -w)]
    stuck = run(limos, direction=0, skew_deg=-12.0,
                params=replace(AvoidParams(), sequence=False, escape=False))
    assert stuck['x'] < 2.0 and stuck['stops'] > 0
    check(run(limos, direction=0, skew_deg=-12.0), w, 1)
    check(run(limos, direction=0, skew_deg=-12.0,
              params=replace(AvoidParams(), sequence=False, route=True)), w, 1)


def _lanes_driven(track):
    out = []
    for _, _, d, _ in track:
        lane = lane_of(d, AvoidParams())
        if not out or out[-1] != lane:
            out.append(lane)
    return out


@pytest.mark.parametrize('layout', [[(1.16, 0.0), (1.98, -0.50)],      # 1108 run 1-2: L2 1.00, L3 1.82
                                    [(3.16, 0.0), (2.66, -0.50)]])     # 1108 run 6-9: L2 3.00, L3 2.50
def test_sign_right_goes_round_by_lane_1_then_into_lane_3(layout):
    track = []
    out = run(layout, direction=1, track=track)
    check(out, 0.50, 3)
    assert _lanes_driven(track) == [2, 1, 2, 3]
    assert out['min_gap'] > 0.10


def test_lane_sequence_is_kept_while_free():
    """Re-planned every scan from the car, the chosen lane changes must not come and go."""
    import clothoid_lidar.static_avoid as sa
    seen = []
    orig = sa.plan_paths

    def spy(*a, **k):
        plan = orig(*a, **k)
        if plan.sequence is not None:
            seen.append(tuple((x, y) for x, y, _ in plan.sequence.changes))
        return plan
    sa.plan_paths = spy
    try:
        run([(3.16, 0.0), (2.66, -0.50)], direction=1)
    finally:
        sa.plan_paths = orig
    switches = sum(1 for a, b in zip(seen, seen[1:]) if a != b and set(b) - set(a))
    assert seen and switches <= 2, seen


def test_s_curve_is_never_sharper_than_its_radius():
    from clothoid_lidar.lane_sequence import s_curve, s_curve_length
    for delta in (0.5, -0.5, 1.0):
        length = s_curve_length(delta, 0.55, math.radians(50))
        u = np.linspace(0.0, length, 400)
        off, head = s_curve(u, delta, 0.55, math.radians(50))
        assert off[-1] == pytest.approx(delta, abs=1e-6)
        curvature = np.abs(np.gradient(head, u)) * np.cos(head)    # d heading / d arc length
        assert curvature[2:-2].max() <= 1.0 / 0.55 + 0.05


def test_target_lane_limo_just_before_the_island_is_reached_over_the_tip_at_most():
    w = 0.50
    out = run([(1.2, 0.0), (3.36, -w)], direction=1)        # lane 3 LIMO rear 3.2 m: very late
    check(out, w, 3)


def _came(a, poses):
    for pose in poses:
        a.remember(pose)


def test_reverse_recovery_backs_up_only_when_allowed():
    way = [(-0.5 + 0.02 * k, 0.0, 0.0) for k in range(26)]
    blocked = StaticAvoid(AvoidParams(recovery_reverse=True, escape=False))
    blocked.activate((0.0, 0.0, 0.0))
    _came(blocked, way)
    blocked.plan = None
    cmds = [blocked.control((0.0, 0.0, 0.0), Inhibit(), 0.1 * k) for k in range(20)]
    assert cmds[0] == (0.0, 0.0) and any(v < 0.0 for v, _ in cmds)
    assert blocked.event == 'stuck: backing up along the way the car came'
    off = StaticAvoid(AvoidParams(recovery_reverse=False, escape=False))
    off.activate((0.0, 0.0, 0.0))
    _came(off, way)
    off.plan = None
    assert all(off.control((0.0, 0.0, 0.0), Inhibit(), 0.1 * k) == (0.0, 0.0) for k in range(30))
    # no record of the way the car came (started standing): what is behind is unknown, no back-up
    blind = StaticAvoid(AvoidParams(recovery_reverse=True, escape=False))
    blind.activate((0.0, 0.0, 0.0))
    blind.plan = None
    assert all(blind.control((0.0, 0.0, 0.0), Inhibit(), 0.1 * k) == (0.0, 0.0) for k in range(30))


def test_reverse_stops_after_its_distance():
    a = StaticAvoid(AvoidParams(recovery_reverse=True, escape=False))
    a.activate((0.0, 0.0, 0.0))
    _came(a, [(-0.5 + 0.02 * k, 0.0, 0.0) for k in range(26)])
    a.plan = None
    for k in range(17):
        a.control((0.0, 0.0, 0.0), Inhibit(), 0.1 * k)
    assert a.control((-0.25, 0.0, 0.0), Inhibit(), 1.8) == (0.0, 0.0)      # backed 0.25 m >= 0.2
    assert a.reversed == pytest.approx(0.25)


def test_reverse_follows_the_way_the_car_came():
    """Came in on a left curve (R 0.6 m): the back-up retraces it instead of going straight back."""
    R = 0.6
    way = [(R * math.sin(th) + 0.1 * math.cos(th), R * (1 - math.cos(th)) + 0.1 * math.sin(th), th)
           for th in np.linspace(0.0, 0.8, 41)]           # rear axle on the circle, base 0.1 m ahead
    for gain in (1.0, 0.8):
        a = StaticAvoid(AvoidParams(recovery_reverse=True, escape=False))
        a.activate(way[0])
        _came(a, way)
        a.plan = None
        x, y, th = way[-1]
        xr, yr = x - 0.1 * math.cos(th), y - 0.1 * math.sin(th)
        worst, t, moved = 0.0, 0.0, 0.0
        for _ in range(200):
            cmd = a.control((xr + 0.1 * math.cos(th), yr + 0.1 * math.sin(th), th), Inhibit(), t)
            t += 0.05
            if cmd is None or (a.reversed > 0.0 and a.backing_from is None):
                break
            v, wz = cmd
            delta = gain * math.atan(L * wz / v) if abs(v) > 1e-6 else 0.0
            xr += v * math.cos(th) * 0.05
            yr += v * math.sin(th) * 0.05
            th += v * math.tan(delta) / L * 0.05
            moved += abs(v) * 0.05
            bx, by = xr + 0.1 * math.cos(th), yr + 0.1 * math.sin(th)
            worst = max(worst, min(math.hypot(bx - px, by - py) for px, py, _ in way))
        assert 0.18 <= a.reversed <= 0.26, (gain, a.reversed)
        assert worst < 0.02, (gain, worst)           # within 2 cm of the way it came
        assert th < 0.8 - 0.2                        # turned back as it came in (straight: 0.8)


def test_escape_margin_and_swerve_margin_are_two_values():
    p = AvoidParams()
    assert p.escape_margin == 0.0 and p.swerve_margin == 0.15
    assert p.recovery_reverse and p.escape                  # on by default
    from clothoid_lidar.static_avoid import escape_distance
    assert escape_distance(p) == pytest.approx(0.595, abs=0.002)
    assert escape_distance(replace(p, escape_margin=-0.03)) == pytest.approx(0.595, abs=0.002)


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
    assert not a.on_lane_fit(SectionFrame(0.1, 0.0, math.radians(45.0)))
    assert a.lane_frame is None
    off = StaticAvoid(AvoidParams(reframe=False))       # the old limit: 15 deg
    off.activate((0.0, 0.0, 0.0))
    assert not off.on_lane_fit(SectionFrame(0.1, 0.0, math.radians(25.0)))


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
    a = StaticAvoid(AvoidParams(handover_after_island=False))
    a.activate((0.0, 0.0, 0.0), entry_back=0.0, entry_pose=(0.0, 0.0, 0.0))
    a.direction = 0
    a.plan = plan_paths((3.5, 0.0, 0.0), 3.5, a.costmap, 0, None, a.p, island=a.island)
    assert a.control((3.6, 0.0, 0.0), Inhibit()) is not None     # lane 2, island 0.8 m ahead: keep on
    assert a.state == ACTIVE
    assert a.control((3.6, 0.5, 0.0), Inhibit()) is None         # lane 1 = target: hand over
    assert a.event.startswith('hand over in target lane 1')


def test_unknown_direction_waits_for_the_car_to_be_aligned():
    a = StaticAvoid(AvoidParams(handover_after_island=False))
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
    assert not a.on_start_fit(SectionFrame(0.0, 0.0, math.radians(45.0)))
    assert a.lane_frame is None


def test_a_start_in_the_corner_turns_the_section_frame():
    """1112 runs: started 25 deg before the end of the corner. The entry fit says so: the section
    frame turns to the lane, the LIMO seen before stays where it is in the world, and the car
    driving along lane 1 is in lane 1 (it read lane 3 all the way)."""
    a = StaticAvoid(AvoidParams())
    a.activate((0.0, 0.0, math.radians(25.0)))          # heading 25 deg left of the lane
    limo = np.array([[1.5, 0.0]])                       # world, lane 2 ahead
    origin = np.array([0.1, 0.0])
    a.costmap.update(a.frame.points_to(limo), a.frame.points_to(origin[None])[0])
    a.costmap.update(a.frame.points_to(limo), a.frame.points_to(origin[None])[0])
    old_hit = a.costmap.occupied_at(a.frame.points_to(limo)).item()
    assert a.on_lane_fit(SectionFrame(0.2, 0.0, 0.0))
    assert 'turned' in a.event and a.frame.yaw0 == pytest.approx(0.0)
    assert old_hit and a.costmap.occupied_at(a.frame.points_to(limo)).item()
    assert lane_of(a.lane_pose((3.0, 0.5, 0.0))[1], a.p) == 1
    assert a.plan is None and a.sequence is None


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
    a = _active_at_origin(AvoidParams(handover_after_island=False))
    a.on_lane_fit(SectionFrame(0.0, 0.0, 0.0))
    a.direction = direction
    a.plan = None
    assert a.control((a.island[0] + 0.01, 0.2, math.radians(30.0)), Inhibit()) is None
    assert a.state != ACTIVE and a.locked


def test_estimated_island_gets_a_margin():
    a = _active_at_origin(AvoidParams(handover_after_island=False))
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
    a = _active_at_origin(AvoidParams(handover_after_island=False))
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


def test_poses_by_distance_after_backing_up():
    """1112 run 01: pushed back by hand below where the odometer started, then an entry line there
    (the node died on at_distance -> None)."""
    tr = PoseTracker()
    tr.imu(0.0, 0.0)
    t = 0.0
    for k in range(51):                     # forward 0.5 m
        tr.odom(t, 0.3, (0.01 * k, 0.0))
        t += 0.02
    tr.imu(t, 0.3)                          # backs up at another heading: the latest pass is used
    for k in range(1, 81):                  # back 0.8 m, to -0.3 m
        tr.odom(t, -0.3, (0.5 - 0.01 * k, 0.0))
        t += 0.02
    assert tr.distance == pytest.approx(-0.3)
    pose = tr.at_distance(-0.2)
    assert pose is not None and pose[2] == pytest.approx(0.3)
    assert tr.at_distance(0.2)[2] == pytest.approx(0.3)
    tr.imu(t, 0.0)
    for k in range(1, 31):                  # forward again
        tr.odom(t, 0.3, (-0.3 + 0.01 * k, 0.0))
        t += 0.02
    assert tr.at_distance(-0.1)[2] == pytest.approx(0.0)
    assert tr.at_distance(0.2)[2] == pytest.approx(0.3)
    keys = [e[0] for e in tr.by_distance]
    assert keys == sorted(keys) and min(np.diff(keys)) > 0.005


def test_heading_at_an_image_time_follows_the_imu_through_an_odom_gap():
    """/odom silent for 1 s in a turn (1112 car: depth 1 behind the images): the heading at an
    image's time is the IMU's, not a straight line between the two /odom messages."""
    tr = PoseTracker()
    for k in range(101):                    # IMU 100 Hz, turning 40 deg/s
        t = 0.01 * k
        tr.imu(t, math.radians(-40.0 * t))
        if k in (0, 100):
            tr.odom(t, 0.3, (0.3 * t, 0.0))
    assert math.degrees(tr.at(0.5)[2]) == pytest.approx(-20.0, abs=0.1)
    tr2 = PoseTracker()
    for k in range(101):                    # turned at the start of the gap, straight after
        t = 0.01 * k
        tr2.imu(t, math.radians(-40.0 * min(t, 0.5)))
        if k in (0, 100):
            tr2.odom(t, 0.3, (0.3 * t, 0.0))
    assert math.degrees(tr2.at(0.75)[2]) == pytest.approx(-20.0, abs=0.1)   # the trace says -15
    assert math.degrees(tr2.yaw_rate(0.4)) == pytest.approx(-40.0, abs=0.5)
    assert math.degrees(tr2.yaw_rate(0.9)) == pytest.approx(0.0, abs=0.5)


def test_entry_crossing_without_a_pose_is_ignored():
    sa = StaticAvoid(AvoidParams())
    sa.activate((0.0, 0.0, 0.0))
    island = sa.island
    sa.on_entry_crossing(None)
    assert sa.island == island and sa.island_source == 'estimate'


def test_odometer_at_an_image_stamp():
    """A frame processed late is placed where the car was when it was taken."""
    tr = PoseTracker()
    tr.imu(0.0, 0.0)
    for k in range(51):
        tr.odom(0.02 * k, 0.5, (0.01 * k, 0.0))
    assert tr.distance == pytest.approx(0.5)
    assert tr.distance_at(0.6) == pytest.approx(0.3, abs=1e-6)
    assert tr.distance_at(5.0) == pytest.approx(0.5)            # after the history: the latest


def test_a_line_before_the_corner_does_not_anchor_the_section():
    """The crosswalk before the 90 deg corner, 1.2 m back: started in the turn, the section frame
    must not take that line's heading (simulator --corner: 90 deg off, stuck)."""
    a = StaticAvoid(AvoidParams())
    a.activate((0.0, 0.0, math.radians(-60.0)), entry_back=1.2,
               entry_pose=(0.5, 1.0, math.radians(-90.0 + 90.0)))
    assert a.island_source == 'estimate' and a.frame.yaw0 == pytest.approx(math.radians(-60.0))
    b = StaticAvoid(AvoidParams())
    b.activate((0.0, 0.0, math.radians(-10.0)), entry_back=0.4, entry_pose=(-0.4, 0.0, 0.0))
    assert b.island_source == 'entry line' and b.frame.yaw0 == pytest.approx(0.0)


def test_beside_the_island_hand_over_waits_for_lane_1_or_3():
    a = _active_at_origin()
    a.on_lane_fit(SectionFrame(0.0, 0.0, 0.0))
    a.direction, a.plan = 0, None
    f = a.island[0]
    assert a.control((f + 0.01, 0.5, 0.0), Inhibit()) is not None      # at the front: drives on
    assert a.control((f + 0.65, 0.5, math.radians(25.0)), Inhibit()) is not None   # not straight yet
    assert a.state == ACTIVE
    assert a.control((f + 0.65, 0.5, math.radians(10.0)), Inhibit()) is None
    assert a.locked and a.done and 'lane 1' in a.event and 'sign lane 1' in a.event


def test_beside_the_island_hand_over_at_the_latest_overshoot():
    a = _active_at_origin()
    a.on_lane_fit(SectionFrame(0.0, 0.0, 0.0))
    a.plan = None
    f = a.island[0] + a.p.handover_past + a.p.handover_overshoot
    assert a.control((f + 0.01, -0.5, math.radians(30.0)), Inhibit()) is None
    assert 'heading 30 deg' in a.event and 'no sign' in a.event


def test_hits_beyond_the_yellow_lines_are_dropped_once_the_lanes_are_measured():
    a = _active_at_origin()
    beside = np.array([[2.0, 0.95]])                    # a cone 0.2 m outside the left yellow line
    on_road = np.array([[2.0, 0.60]])
    a.on_scan(np.vstack((beside, on_road)), np.zeros(2), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), None)
    assert a.costmap.count[a.costmap.cells(beside)[0], a.costmap.cells(beside)[1]].item() > 0  # no fit yet
    b = _active_at_origin()
    b.on_lane_fit(SectionFrame(0.0, 0.0, 0.0))
    b.on_scan(np.vstack((beside, on_road)), np.zeros(2), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), None)
    i, j, _ = b.costmap.cells(np.vstack((beside, on_road)))
    assert b.costmap.count[i[0], j[0]] == 0 and b.costmap.count[i[1], j[1]] > 0


def test_beams_to_far_points_clear_what_they_pass():
    """1112 auto 05, 24 s: cells marked from further back (the unseen body behind a hit) stayed for
    20 s, the beams through them ended on walls beyond scan_range_max and were dropped whole."""
    p = AvoidParams()
    cm = Costmap(p)
    origin = np.array([0.0, 0.0])
    cell = np.array([[1.0, 0.4]])
    for _ in range(5):
        cm.update(cell, origin)
    i, j, _ = cm.cells(cell)
    assert cm.count[i[0], j[0]] >= p.occupied_count
    far = np.array([[4.0, 1.6]])                          # same bearing, 4.4 m: beyond the range
    for _ in range(10):
        cm.update(far, origin)
    assert cm.count[i[0], j[0]] == 0
    assert not cm.occupied_at(far).any()                  # the far point itself is not marked


def test_no_new_start_after_a_hand_over_beside_the_island():
    """1113 auto 02 / 05: started again at the cone course's white line and cones past the fork."""
    a = _active_at_origin()
    a.on_lane_fit(SectionFrame(0.0, 0.0, 0.0))
    a.plan = None
    assert a.control((a.island[0] + 0.65, 0.5, 0.0), Inhibit()) is None
    assert a.finished
    limo = np.array([[1.0, 0.0]])
    started = [a.on_limos(20 + 0.1 * k, limo, Inhibit(), 0.3, (12.0, 0.0, 0.0), entry_back=0.5,
                          entry_pose=(11.5, 0.0, 0.0)) for k in range(6)]
    assert not any(started)
    b = _active_at_origin(AvoidParams(one_section_per_run=False))
    b.on_lane_fit(SectionFrame(0.0, 0.0, 0.0))
    b.plan = None
    b.control((b.island[0] + 0.65, 0.5, 0.0), Inhibit())
    started = [b.on_limos(20 + 0.1 * k, limo, Inhibit(), 0.3, (12.0, 0.0, 0.0), entry_back=0.5,
                          entry_pose=(11.5, 0.0, 0.0)) for k in range(6)]
    assert any(started)                                   # the retry as before


def test_escape_path_when_every_path_is_blocked():
    """Stuck behind a lane 2 LIMO 0.55 m ahead with the lane 1 LIMO 0.35 m further (1113 auto 05
    like): the escape search finds the quick swerve the lane-centre S-curves do not have."""
    p = AvoidParams(escape=True)                          # off by default for now
    a = StaticAvoid(p)
    a.activate((0.0, 0.0, 0.0), entry_back=0.0, entry_pose=(0.0, 0.0, 0.0))
    a.direction = 1
    for _ in range(6):
        a.costmap.update(np.array([[0.75 + 0.02 * k, y] for k in range(10) for y in (-0.08, 0.0, 0.08)]
                                  + [[1.10 + 0.02 * k, y] for k in range(10) for y in (0.42, 0.5, 0.58)]),
                         np.zeros(2))
    a.plan = None
    assert a.control((0.0, 0.0, 0.0), Inhibit(), 0.0) == (0.0, 0.0)          # waits stuck_time
    v, w = a.control((0.0, 0.0, 0.0), Inhibit(), 0.6)
    assert v > 0.0 and w < 0.0 and a.event.startswith('stuck: escape path')  # right, to lane 3
    esc = a.escape_plan
    assert esc.path[-1, 1] < -0.2
    off = StaticAvoid(replace(p, escape=False))
    off.activate((0.0, 0.0, 0.0), entry_back=0.0, entry_pose=(0.0, 0.0, 0.0))
    off.costmap = a.costmap
    off.plan = None
    assert off.control((0.0, 0.0, 0.0), Inhibit(), 0.6) == (0.0, 0.0)


def test_a_hand_over_at_an_estimated_island_does_not_end_the_run():
    """A false start before the section (no entry line, island estimated): reaching that island
    must not lock out the real section after it (session A, 11-13)."""
    a = _active_at_origin()                               # island_source 'estimate'
    a.plan = None
    s = a.island[0] + a.p.island_past_estimate + a.p.handover_past + a.p.handover_overshoot + 0.01
    assert a.control((s, 0.0, 0.0), Inhibit()) is None    # overshoot: handed over, in lane 2
    assert not a.finished and a.locked
    limo = np.array([[1.0, 0.0]])
    started = [a.on_limos(20 + 0.1 * k, limo, Inhibit(), 0.3, (12.0, 0.0, 0.0), entry_back=0.5,
                          entry_pose=(11.5, 0.0, 0.0)) for k in range(6)]
    assert any(started)                                   # the real section starts


def test_a_wrong_lock_does_not_delay_the_real_start():
    """1114 car: locked 4.4 m after the slow zone's lines; the real start line was 0.6 m ahead with
    the first LIMO 1.4 m ahead: start there, not after the base crossed the line."""
    a = finished_section()
    limo = np.array([[1.4, 0.0]])
    started = [a.on_limos(20 + 0.1 * k, limo, Inhibit(), 0.3, (12.0, 0.0, 0.0), entry_back=-0.6,
                          entry_pose=(12.6, 0.0, 0.0)) for k in range(4)]
    assert any(started) and not a.locked



def test_pose_of_a_line_still_ahead():
    tr = PoseTracker()
    tr.imu(0.0, 0.3)
    for k in range(21):
        tr.odom(0.05 * k, 0.4, (0.02 * k * math.cos(0.3), 0.02 * k * math.sin(0.3)))
    x, y, yaw = tr.line_pose(tr.distance + 0.5)
    assert x == pytest.approx(0.9 * math.cos(0.3), abs=1e-6) and yaw == pytest.approx(0.3)
    assert tr.line_pose(tr.distance + 2.0) is None
    assert tr.line_pose(0.2) == pytest.approx(tr.at_distance(0.2))


def _course_tracker(yaw0=0.7, placed_deg=0.0, wobble_deg=0.0,
                    legs=((0.0, 900), (-math.pi / 2, 470), (math.pi, 200))):
    """Drive the 1114 course start: 9 m along +x, a right turn, 4.7 m down, a right turn, along -x;
    in a world frame turned by yaw0 (the IMU's own zero). placed_deg: the car set down that much
    off the track axis at the start (its IMU zero is then off too). wobble_deg: lane-following sway."""
    from clothoid_lidar.static_avoid import CourseMap
    tr = PoseTracker()
    course = CourseMap(AvoidParams())
    tr.course = course
    pts, heads = [], []
    x, y = 0.0, 0.0
    k = 0
    for h, n in legs:
        for _ in range(n):
            k += 1
            hh = h + math.radians(wobble_deg) * math.sin(k / 40.0)
            x += 0.01 * math.cos(hh); y += 0.01 * math.sin(hh)
            pts.append((x, y)); heads.append(hh)
    off = math.radians(placed_deg)
    c, s = math.cos(yaw0), math.sin(yaw0)
    t = 0.0
    tr.imu(t, yaw0 + off)
    tr.odom(t, 0.3, (0.0, 0.0))
    for (px, py), h in zip(pts, heads):
        t += 0.02
        tr.imu(t, yaw0 + h)
        tr.odom(t, 0.3, (c * px - s * py, s * px + c * py))
    return tr, course


def test_course_map_pose_from_the_start():
    tr, course = _course_tracker()
    assert course.x == pytest.approx(7.0, abs=0.05) and course.y == pytest.approx(-4.7, abs=0.05)
    assert abs(math.degrees(course.h)) == pytest.approx(180.0, abs=1.0)


def test_course_map_snaps_a_crooked_start_to_the_track_axes():
    """Set down 4 deg off: unsnapped the line would be ~1 m off at 14 m."""
    tr, course = _course_tracker(placed_deg=4.0, wobble_deg=1.5)
    assert course.snaps and abs(course.snaps[0][1]) == pytest.approx(4.0, abs=1.0)
    assert course.x == pytest.approx(7.0, abs=0.15) and course.y == pytest.approx(-4.7, abs=0.15)


def test_course_map_start_line():
    from clothoid_lidar.section_cues import Crossing
    from clothoid_lidar.static_avoid import course_start_line
    tr, course = _course_tracker()
    check = course_start_line(course, AvoidParams())
    d0 = tr.distance                                       # at (7.0, -4.7)
    real = Crossing(odometer=d0 - 1.25, heading=0.0, t=0.0)          # (8.25, -4.7), heading 180
    side = Crossing(odometer=9.0 + 4.0, heading=0.0, t=0.0)          # (9.0, -4.0), heading -90
    slow = Crossing(odometer=2.4, heading=0.0, t=0.0)                # top straight
    assert check(real) is True and 'from the start line' in real.where
    assert check(side) is False                            # 0.99 m away but heading down
    assert check(slow) is False


def test_course_map_falls_back_after_passing_the_point_without_a_line():
    from clothoid_lidar.section_cues import EntryLineTracker
    tr, course = _course_tracker()
    entry = EntryLineTracker()
    assert course.should_fall_back(14.0, entry) is None   # at (7.0, -4.7): 1.25 m from the point
    course.min_far = 0.4                                  # it had come close
    course.x = 6.0                                        # and is 2.25 m past now
    why = course.should_fall_back(15.5, entry)
    assert why and 'passed the start line point' in why and course.should_fall_back(16, entry) is None
    far = _course_tracker()[1]
    far.x, far.y = 3.0, 3.0                               # a wrong start pose: never near it
    assert 'never within' in far.should_fall_back(17.5, entry)


def test_course_map_decides_without_the_lane_line_pair():
    from clothoid_lidar.section_cues import Crossing, EntryLineTracker, check_start_line
    tr = EntryLineTracker()
    tr.start_by_map = lambda c: c.odometer > 10.0
    a, b = Crossing(odometer=2.4, heading=0.0, t=0.0), Crossing(odometer=14.2, heading=0.0, t=0.0)
    tr.crossings += [a, b]
    decided = check_start_line(tr, None, None, None, 14.3, 0.5)     # no image needed
    assert decided == [a, b] and a.start is False and b.start is True


def test_course_origin_race_start_waits_for_the_line():
    """The 1114 course from the race start: after the right turn at x 9 still 'pending' (the start
    line makes it 'race'); the -30 deg lane-following sway is no turn."""
    tr, course = _course_tracker(wobble_deg=30.0, legs=((0.0, 900), (-math.pi / 2, 300)))
    assert course.origin == 'pending' and course.corner1[1] > 7.5      # the sway shortens x
    course.set_origin('race', 'line')
    course.set_origin('elsewhere', 'later')                  # judged once
    assert course.origin == 'race'


def test_course_origin_avoid_only_test_turns_at_once():
    """1113 avoid-only runs: a right turn after 1.2 m of straight -> not the race start."""
    tr, course = _course_tracker(legs=((0.0, 120), (-math.pi / 2, 100)))
    assert course.origin == 'elsewhere' and 'after 1.2 m of straight' in course.origin_why


@pytest.mark.parametrize('legs, why', [
    (((0.0, 1150),), 'no turn in the first'),                           # straight on past 11 m
    (((0.0, 900), (math.pi / 2, 100)), 'went left'),                    # a left turn at x 9
    (((0.0, 900), (-math.pi / 2, 300), (0.0, 100)), 'after the first right turn'),
])
def test_course_origin_not_the_race_course(legs, why):
    tr, course = _course_tracker(legs=legs)
    assert course.origin == 'elsewhere' and why in course.origin_why


def test_line_start_then_approach_speed_until_a_blocker():
    """Race start: the start line starts the mission (no blocker needed), approach_speed until a LIMO
    in our lane; a blocker alone starts nothing while not allowed."""
    p = AvoidParams()
    avoid = StaticAvoid(p)
    limo = np.array([[1.0, 0.0]])
    for k in range(5):                                       # blocked, but not allowed
        assert not avoid.on_limos(0.1 * k, limo, Inhibit(), 0.5, (0.0, 0.0, 0.0), None, None, False)
    line = (0.5, 0.0, 0.0)
    avoid.request_line_start(line)
    assert not avoid.on_limos(1.0, np.zeros((0, 2)), Inhibit(state_command=True), 0.5,
                              (0.6, 0.0, 0.0), 0.1, line, False)      # a mission stop: waits
    assert avoid.on_limos(1.1, np.zeros((0, 2)), Inhibit(), 0.5, (0.6, 0.0, 0.0), 0.1, line, False)
    assert avoid.state == ACTIVE and avoid.approach and avoid.island_source == 'entry line'
    avoid.on_limos(1.2, limo, Inhibit(), 0.5, (0.7, 0.0, 0.0), 0.2, line, False)
    assert not avoid.approach


def test_line_start_dropped_for_another_line_or_far_past():
    avoid = StaticAvoid(AvoidParams())
    avoid.request_line_start((0.5, 0.0, 0.0))
    assert not avoid.on_limos(1.0, np.zeros((0, 2)), Inhibit(), 0.5, (4.0, 0.0, 0.0), 3.5,
                              (0.5, 0.0, 0.0), False)
    assert avoid.line_start is None and avoid.state != ACTIVE
    other = StaticAvoid(AvoidParams())
    other.request_line_start((0.5, 0.0, 0.0))
    assert not other.on_limos(1.0, np.zeros((0, 2)), Inhibit(), 0.5, (0.6, 0.0, 0.0), 0.1,
                              (2.0, 0.0, 0.0), False)
    assert other.line_start is None and 'another line' in other.event


def test_line_start_waits_for_a_line_not_placed_yet_and_starts_on_one_ahead():
    """1114 23:39: the start line confirmed on a corner was placed > 1 m ahead of the base, the lookup
    within 1 m found nothing and the start was dropped (no blocker start at the race start: the car
    drove into the LIMOs). Unknown: wait; ahead (the node passes the requested line itself): start."""
    avoid = StaticAvoid(AvoidParams())
    line = (15.65, -5.0, math.pi)
    avoid.request_line_start(line)
    for k in range(5):
        assert not avoid.on_limos(0.1 * k, np.zeros((0, 2)), Inhibit(), 0.5, (14.3, -5.0, math.pi),
                                  None, None, False)
    assert avoid.line_start == line
    assert avoid.on_limos(1.0, np.zeros((0, 2)), Inhibit(), 0.5, (14.4, -5.0, math.pi), -1.25, line,
                          False)
    assert avoid.state == ACTIVE
