#!/usr/bin/env python3
"""Run static_avoid.py on a recorded bag, offline (no ROS nodes), and compare its local paths with the
path the car really drove (in the 11-05 bags a person drove by RC, seeing the whole section: the
reference path, give or take late / early turns).

The bag's own driving moves the car, so this checks the trigger, the costmap and the path choice,
not the closed loop. Cone-course flag = cone_sorter's start/continue logic on the same scans.

    python3 tools/static_avoid/replay_bag.py <bag> [--from 40 --to 70] [--direction auto|0|1|none]
        [--ignore-cone] [--plot out.png] [--snaps 6]
Prints every start / end, and how far the planned path is from the driven one 0.5 / 1.0 m ahead.
"""
import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'ros2_ws', 'src', 'clothoid_lidar'))
from clothoid_lidar.cone_sorter import ConeMission, ConeSorter, SorterParams  # noqa: E402
from clothoid_lidar.obstacle_extractor import (ExtractorParams, extract_obstacles,  # noqa: E402
                                               transform_points)
from clothoid_lidar.section_cues import (CameraModel, ColourParams, EntryLineTracker,  # noqa: E402
                                         WhiteParams, find_entry_line)
from clothoid_lidar.static_avoid import (ACTIVE, LETHAL, AvoidParams, Inhibit, LaneFitter,  # noqa: E402
                                         PoseTracker, StaticAvoid, base_to_section, lane_of,
                                         yaw_from_quaternion)

LIDAR_X = 0.103
LIMO = ExtractorParams(cluster_gap=0.08, min_points=3, max_width=0.45, min_width=0.12,
                       cone_radius=0.1)
CONE = ExtractorParams(cluster_gap=0.06)


def messages(bag, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    cls = {}
    while reader.has_next():
        topic, data, t = reader.read_next()
        c = cls.setdefault(topic, get_message(types[topic]))
        yield topic, t * 1e-9, deserialize_message(data, c)


def image_bgr(msg):
    arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
    return arr[:, :, ::-1] if msg.encoding == 'rgb8' else arr


def replay(bag, t_from, t_to, direction, ignore_cone, lane_width, cam=None):
    """Same pieces as static_avoid_node: LIMO trigger, costmap, plans, entry line (camera every
    3rd frame), island from the entry line, hand-over."""
    cam = cam or CameraModel()          # limo2 = car 7, the 11-05 bags
    white = WhiteParams()
    entry = EntryLineTracker()
    frames = 0
    p = AvoidParams(lane_width=lane_width)
    avoid = StaticAvoid(p)
    tracker = PoseTracker()
    fitter = LaneFitter(p, cam, ColourParams())
    cone = ConeMission(ConeSorter(SorterParams()))
    rec = dict(events=[], plans=[], trail=[], limos=[], cost_max=None, p=p, avoid=avoid,
               crossings=[])
    t0 = None
    on_since, on_total = None, 0.0
    s = 0.0
    for topic, t, msg in messages(bag, ['/scan', '/odom', '/imu', '/camera/color/image_raw']):
        t0 = t if t0 is None else t0
        s = t - t0
        if topic == '/imu':
            q = msg.orientation
            tracker.imu(s, yaw_from_quaternion(q.x, q.y, q.z, q.w))
            continue
        if topic == '/camera/color/image_raw':
            frames += 1
            if (frames - 1) % 3 or not tracker.ready or not (t_from - 8.0 <= s <= t_to):
                continue
            bgr = image_bgr(msg)
            c = entry.update(s, tracker.distance, find_entry_line(bgr, cam, white))
            if c is not None:
                rec['crossings'].append((s, c))
            had = avoid.lane_frame
            fitter.on_frame(bgr, avoid, tracker, entry)
            if avoid.lane_frame is not None and had is None:
                rec['events'].append((s, avoid.event))
            continue
        if topic == '/odom':
            tracker.odom(s, msg.twist.twist.linear.x)
            if avoid.state == ACTIVE and avoid.island_source == 'estimate':
                c = entry.recent(tracker.distance, 0.3, 0.0)     # same as the node
                if c is not None:
                    avoid.on_entry_crossing(tracker.at_distance(c.odometer))
                    if avoid.island_source == 'entry line':
                        rec['events'].append((s, 'entry line crossed after the start'))
            if avoid.frame is not None and tracker.ready:
                rec['trail'].append((s, *avoid.section_pose(tracker.latest())[:2],
                                     avoid.state == ACTIVE))
            continue
        if s < t_from or s > t_to or not tracker.ready:
            continue
        r = np.asarray(msg.ranges, float)
        ang = msg.angle_min + msg.angle_increment * np.arange(r.size)
        cone.update(transform_points(extract_obstacles(r, msg.angle_min, msg.angle_increment, CONE),
                                     LIDAR_X, 0.0, 0.0))
        limos = transform_points(extract_obstacles(r, msg.angle_min, msg.angle_increment, LIMO),
                                 LIDAR_X, 0.0, 0.0)
        inh = Inhibit(cone_active=cone.running and not ignore_cone)
        pose = tracker.at(s + 0.05)
        c = entry.recent(tracker.distance, p.island_from_entry_max + 1.0)
        entry_back = tracker.distance - c.odometer if c is not None else None
        entry_pose = tracker.at_distance(c.odometer) if c is not None else None
        if avoid.on_limos(s, limos, inh, tracker.speed, pose, entry_back, entry_pose):
            rec['events'].append((s, avoid.event.upper() + f' (odometer {tracker.distance:.2f} m)'))
            on_since = s
        if avoid.state != ACTIVE:
            continue
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        hits = np.column_stack((LIDAR_X + r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])))
        free = np.column_stack((np.cos(ang[~ok]), np.sin(ang[~ok])))
        avoid.on_scan(hits, np.array([LIDAR_X, 0.0]), pose, tracker.latest(), direction,
                      keep_candidates=True, free_dirs_base=free)
        sec = avoid.lane_pose(tracker.latest())
        if len(limos):
            rec['limos'].extend(base_to_section(limos, avoid.section_pose(pose)).tolist())
        cm = avoid.costmap.cost
        rec['cost_max'] = cm.copy() if rec['cost_max'] is None or rec['cost_max'].shape != cm.shape \
            else np.maximum(rec['cost_max'], cm)
        rec['plans'].append(dict(t=s, pose=sec, plan=avoid.plan, cost=cm.copy(),
                                 lane_rel=avoid.lane_in_costmap(),
                                 lane=avoid.lane(tracker.latest()),
                                 limos=base_to_section(limos, avoid.section_pose(pose))
                                 if len(limos) else limos))
        if avoid.control(tracker.latest(), inh) is None:
            rec['events'].append((s, 'END ' + avoid.event))
            sec = avoid.lane_pose(tracker.latest())
            rec['handover'] = dict(t=s, s=sec[0], d=sec[1], island=avoid.island,
                                   source=avoid.island_source)
            on_total += s - on_since
    if avoid.state == ACTIVE and on_since is not None:
        rec['events'].append((s, 'still active at the end of the window'))
        on_total += s - on_since
    rec['on_total'] = on_total
    rec['island'], rec['island_source'], rec['entry_s'] = (avoid.island, avoid.island_source,
                                                           avoid.entry_s)
    to_final_lane(rec, avoid)
    return rec


def to_final_lane(rec, avoid):
    """Everything in the final lane coordinates, costmaps resampled onto them: a plan made before
    the lane fit is drawn where it really was on the road."""
    p, final = rec['p'], avoid.lane_in_costmap()
    rec['lane_turn'] = None if final is None else math.degrees(final.yaw0)

    def sec2lane(q):
        return q if final is None or not len(q) else final.points_to(q)

    def lane2final(q, rel):
        return sec2lane(q if rel is None else rel.points_from(q))
    dmax = 1.5 * p.lane_width + 0.15
    ns = int(round((p.grid_s_max - p.grid_s_min) / p.resolution))
    nd = int(round(2 * dmax / p.resolution))
    ss, dd = np.meshgrid(p.grid_s_min + (np.arange(ns) + 0.5) * p.resolution,
                         -dmax + (np.arange(nd) + 0.5) * p.resolution, indexing='ij')
    grid = np.column_stack((ss.ravel(), dd.ravel()))
    if final is not None:
        grid = final.points_from(grid)
    full = avoid.costmap

    def resample(cost):
        keep, full.cost = full.cost, cost
        out = full.lookup(grid).reshape(ns, nd)
        full.cost = keep
        return out
    if rec['cost_max'] is not None:
        rec['cost_max'] = resample(rec['cost_max'])
    for pl in rec['plans']:
        rel = pl['lane_rel']
        pl['dmax'], pl['cost'] = dmax, resample(pl['cost'])
        s, d, psi = pl['pose']
        q = lane2final(np.array([[s, d]]), rel)[0]
        pl['pose'] = (float(q[0]), float(q[1]), psi)
        pl['plan'].path = lane2final(pl['plan'].path, rel)
        pl['plan'].candidates = [(lane2final(c, rel), v) for c, v in pl['plan'].candidates]
        if len(pl['limos']):
            pl['limos'] = sec2lane(pl['limos'])
    if rec['limos']:
        rec['limos'] = sec2lane(np.array(rec['limos'])).tolist()
    if rec['trail']:
        tr = sec2lane(np.array([(s, d) for _, s, d, _ in rec['trail']]))
        rec['trail'] = [(a[0], float(q[0]), float(q[1]), a[3]) for a, q in zip(rec['trail'], tr)]


def compare(rec, ahead):
    """|planned d - driven d| where the driven path reached s + ahead, per plan (m)."""
    tr = np.array([(s, d) for _, s, d, _ in rec['trail']]) if rec['trail'] else np.zeros((0, 2))
    out = []
    for pl in rec['plans']:
        if not pl['plan'].feasible:
            continue
        s0 = pl['pose'][0]
        later = tr[(tr[:, 0] >= s0 + ahead)]
        if not len(later):
            continue
        path = pl['plan'].path
        if path[-1, 0] < s0 + ahead:
            continue
        d_plan = np.interp(s0 + ahead, path[:, 0], path[:, 1])
        out.append(abs(d_plan - later[0, 1]))
    return np.array(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--from', dest='t_from', type=float, default=0.0)
    ap.add_argument('--to', dest='t_to', type=float, default=1e9)
    ap.add_argument('--direction', default='auto',
                    help='auto = the lane the driver ended in (1 -> 0, 3 -> 1), or 0 / 1 / none')
    ap.add_argument('--ignore-cone', action='store_true',
                    help='do not stop on the cone-course flag (the 11-05 obstacles were cones)')
    ap.add_argument('--lane-width', type=float, default=0.50)
    ap.add_argument('--plot', default=None)
    ap.add_argument('--truth', default='', help='tape-measured objects to draw, lane:distance from '
                    'the entry line to the rear [m], c prefix = cone, e.g. 2:3.00,3:2.50,c1:1.80')
    ap.add_argument('--snaps', type=int, default=6)
    a = ap.parse_args()

    direction = None if a.direction == 'none' else (None if a.direction == 'auto' else int(a.direction))
    rec = replay(a.bag, a.t_from, a.t_to, direction, a.ignore_cone, a.lane_width)
    if a.direction == 'auto' and rec['trail']:
        driven = [d for t, s, d, _ in rec['trail'] if s > 0.5 and t <= a.t_to]
        end_lane = lane_of(driven[-1], rec['p']) if driven else 2
        direction = {1: 0, 3: 1}.get(end_lane)
        rec = replay(a.bag, a.t_from, a.t_to, direction, a.ignore_cone, a.lane_width)
    print(f'{a.bag}  ({a.t_from:.0f}-{a.t_to:.0f} s, direction {direction}, '
          f'cone flag {"ignored" if a.ignore_cone else "used"})')
    for t_ev, what in rec['events']:
        print(f'  {t_ev:7.2f} s  {what}')
    for t_c, c in rec['crossings']:
        print(f'  {t_c:7.2f} s  entry line crossed at odometer {c.odometer:.2f} m, heading '
              f'{math.degrees(c.heading):+.1f} deg')
    if rec['entry_s'] is not None:
        print(f'  island {rec["island"][0]:.2f}~{rec["island"][1]:.2f} m from the start '
              f'({rec["island_source"]}, entry line at {rec["entry_s"]:.2f} m; fallback guess '
              f'{rec["p"].island_s_min:.1f})')
    if rec.get('handover'):
        h = rec['handover']
        print(f'  hand-over at s {h["s"]:.2f} m, {h["island"][0] - h["s"]:.2f} m before the island '
              f'({h["source"]}), lane {lane_of(h["d"], rec["p"])}')
    print('  lane fit: ' + ('none' if rec['lane_turn'] is None else f'{rec["lane_turn"]:+.1f} deg '
                            'against the section frame'))
    print(f'  active {rec["on_total"]:.1f} s, {len(rec["plans"])} plans, '
          f'{sum(not x["plan"].feasible for x in rec["plans"])} blocked')
    for ahead in (0.5, 1.0):
        e = compare(rec, ahead)
        if len(e):
            print(f'  planned vs driven, {ahead:.1f} m ahead: median {100 * np.median(e):.0f} cm, '
                  f'p90 {100 * np.percentile(e, 90):.0f} cm (n {len(e)})')
    rec['truth'] = [x.split(':') for x in a.truth.split(',') if x]
    if a.plot and rec['plans']:
        plot(a.plot, rec, a.bag, direction, a.snaps)


def lane_lines(ax, p):
    for c in (1.5, 0.5, -0.5, -1.5):
        ax.axvline(-c * p.lane_width, color='goldenrod' if abs(c) > 1 else '0.45',
                   lw=1.2 if abs(c) > 1 else 0.8, ls='-' if abs(c) > 1 else '--')


def draw_island(ax, rec, p):
    """True island (from the entry line, hatched) and the one the planner assumes (dashed)."""
    from matplotlib.patches import Polygon
    w2, tip = p.lane_width / 2.0, 0.25

    def hexagon(s0, s1):
        return [(0.0, s0), (-w2, s0 + tip), (-w2, s1 - tip), (0.0, s1), (w2, s1 - tip), (w2, s0 + tip)]
    if rec['entry_s'] is not None:
        e = rec['entry_s']
        ax.add_patch(Polygon(hexagon(*rec['island']), closed=True,
                             fc=(0.55, 0.55, 0.55, 0.25), ec='0.3', hatch='///', lw=1.0,
                             label='island (entry line + rule drawing)'))
        ax.axhline(e, color='0.2', lw=2.0, ls='-', alpha=0.6)
        ax.text(-1.5 * p.lane_width, e + 0.03, 'entry line', fontsize=7, color='0.2', clip_on=True)
    ax.add_patch(Polygon(hexagon(p.island_s_min, p.island_s_max), closed=True, fill=False,
                         ec='purple', ls='--', lw=1.0, label='island guess without the line'))
    if rec.get('handover'):
        h = rec['handover']
        ax.axhline(h['s'], color='tab:green', lw=1.2, ls=':')
        ax.plot(-h['d'], h['s'], '*', color='tab:green', ms=13, mec='k', label='hand-over (car)')
    if rec['entry_s'] is not None:
        from matplotlib.patches import Circle, Rectangle
        for k, (lane, dist) in enumerate(rec.get('truth', [])):
            cone = lane.startswith('c')
            d, s0 = p.lane_width * (2 - int(lane[-1])), rec['entry_s'] + float(dist)
            if cone:
                first = k == 0 or not rec['truth'][k - 1][0].startswith('c')
                ax.add_patch(Circle((-d, s0 + 0.04), 0.04, fill=False, ec='blue', lw=1.8,
                                    label='cone (tape)' if first else None))
            else:
                ax.add_patch(Rectangle((-d - 0.11, s0), 0.22, 0.32, fill=False, ec='blue', lw=1.8,
                                       label='LIMO (tape)' if k == 0 else None))


def cost_rgba(cost):
    """Light picture of a cost grid (rows s, columns d from -dmax): white = free, pale orange =
    near an obstacle, light red = lethal (the car centre may not be there), columns flipped so that
    left is left in the x = -d plots."""
    c = cost[:, ::-1]
    img = np.ones(c.shape + (4,))
    soft = np.clip(c / 100.0, 0.0, 1.0)
    img[..., 1] -= 0.35 * soft                      # white -> pale orange
    img[..., 2] -= 0.70 * soft
    lethal = c >= LETHAL
    img[lethal] = (1.0, 0.62, 0.62, 1.0)            # light red
    return img


def plot(path, rec, bag, direction, n_snaps):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import cm as mcm
    p = rec['p']
    plans = rec['plans']
    tr = np.array([(t, s, d) for t, s, d, _ in rec['trail']])
    idx = np.linspace(0, len(plans) - 1, n_snaps).round().astype(int) if n_snaps else []
    fig = plt.figure(figsize=(3.3 * (n_snaps + 2), 10.5))
    gs = fig.add_gridspec(1, n_snaps + 2)

    # overview: every 0.5 s plan from where the car was, over the max cost seen
    ax = fig.add_subplot(gs[0, :2])
    dmax = plans[0]['dmax']
    # the grid's columns run from d = -dmax (right) to +dmax (left); the plots use x = -d so that
    # left is left: flip the columns
    ax.imshow(cost_rgba(rec['cost_max']), origin='lower',
              extent=(-dmax, dmax, p.grid_s_min, p.grid_s_max), aspect='equal')
    lane_lines(ax, p)
    draw_island(ax, rec, p)
    t_first, t_last = plans[0]['t'], plans[-1]['t']
    step = max(1, int(round(0.5 / 0.1)))
    stops = []
    for k in range(0, len(plans), step):
        pl = plans[k]
        if not pl['plan'].feasible:       # every path blocked: the node sends a stop, no path
            stops.append(pl['pose'])
            continue
        colour = mcm.viridis((pl['t'] - t_first) / max(t_last - t_first, 1e-6))
        ax.plot(-pl['plan'].path[:, 1], pl['plan'].path[:, 0], '-', color=colour, lw=1.0, alpha=0.9)
    if stops:
        st = np.array(stops)
        ax.plot(-st[:, 1], st[:, 0], 'x', color='red', ms=7, mew=2,
                label='all paths blocked: stop (car pose)')
    m = tr[:, 0] <= t_last + 3.0
    ax.plot(-tr[m, 2], tr[m, 1], 'k-', lw=2.5, label='driven (RC)')
    ax.plot([], [], color=mcm.viridis(0.5), label='chosen path, one per 0.5 s (colour = time)')
    if rec['limos']:
        lm = np.array(rec['limos'])
        ax.plot(-lm[:, 1], lm[:, 0], '.', color='deepskyblue', ms=2, label='LIMO-sized clusters')
    ax.set_xlim(-dmax, dmax)
    smax = max([tr[m, 1].max()] + [pl['plan'].path[-1, 0] for pl in plans if pl['plan'].feasible])
    lo = min(-0.5, rec['entry_s'] - 0.3) if rec['entry_s'] is not None else -0.5
    ax.set_ylim(lo, min(p.grid_s_max, max(smax + 0.2, p.island_s_max + 0.3)))
    ax.set_xlabel('left  <-  -d [m]  ->  right')
    ax.set_ylabel('s along the section [m]')
    ax.legend(fontsize=6.5, loc='upper center', bbox_to_anchor=(0.5, -0.07), ncol=1, frameon=False)
    ax.set_title(f'{t_first:.1f}-{t_last:.1f} s, direction {direction}\n(background: highest cost '
                 f'seen at any time)', fontsize=9)

    for col, k in enumerate(idx):
        sh = plans[k]
        ax = fig.add_subplot(gs[0, col + 2])
        ax.imshow(cost_rgba(sh['cost']), origin='lower',
                  extent=(-dmax, dmax, p.grid_s_min, p.grid_s_max), aspect='equal')
        lane_lines(ax, p)
        draw_island(ax, rec, p)
        for cand, cst in sh['plan'].candidates:
            ax.plot(-cand[:, 1], cand[:, 0], color='0.75' if not np.isfinite(cst) else 'tab:blue',
                    lw=0.4, alpha=0.35)
        s0, d0, _ = sh['pose']
        if sh['plan'].feasible:
            ax.plot(-sh['plan'].path[:, 1], sh['plan'].path[:, 0], color='green', lw=2.2,
                    label='planned (chosen)')
        else:
            ax.plot(-d0, s0, 'x', color='red', ms=16, mew=3, label='all blocked: stop')
        later = tr[(tr[:, 0] >= sh['t']) & (tr[:, 0] <= sh['t'] + 8.0)]
        ax.plot(-later[:, 2], later[:, 1], 'k--', lw=1.5, label='driven next (RC)')
        ax.plot(-d0, s0, 'r^', ms=9)
        if len(sh['limos']):
            ax.plot(-sh['limos'][:, 1], sh['limos'][:, 0], 's', color='deepskyblue', mfc='none',
                    ms=8)
        ax.set_xlim(-dmax, dmax)
        ax.set_ylim(s0 - 0.6, s0 + 2.4)
        ax.set_title(f'{sh["t"]:.1f} s, lane {sh["lane"]}, '
                     f'{"ok" if sh["plan"].feasible else "STOP (all paths blocked)"}', fontsize=9)
        if col == 0:
            ax.legend(fontsize=6.5, loc='upper center', bbox_to_anchor=(0.5, -0.07),
                      frameon=False)
    fig.suptitle(f'static_avoid on {os.path.basename(bag.rstrip("/"))}. Costmap: light red = the car '
                 'centre may not go there (obstacle + car half width + 5 cm, + 0.3 m unseen body '
                 'behind each hit), orange = close to it. Left: the path chosen at every scan, one '
                 'per 0.5 s drawn (colour = time), over the RC path (black). Right: snapshots with '
                 'the candidates (blue, grey = blocked), the chosen path (green; red X = every path '
                 'blocked, the node stops) and where the RC car went next (black dashed)', fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=85)
    print('saved', path)


if __name__ == '__main__':
    main()
