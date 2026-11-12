"""Lane-sequence planner for the static obstacle section (ROS-free).

The car goes from lane centre to lane centre: 2 -> 1 -> 3, 2 -> 3, ... Every lane change is an S-curve
whose curvature never exceeds 1 / lc_radius (arc, straight at lc_max_heading, arc), so the car can
follow it, unlike a cubic squeezed into a short length. The road ahead is cut into steps of `step`
metres; for every lane and step we know whether staying there is free, and for every lane pair and
step whether a change started there is free (car footprint against the costmap's lethal cells, the
island). A shortest-path search over (lane, step) then finds the cheapest sequence that is in the
target lane by the deadline (the island front less a margin; if none, the front + the pointed tip).

Lane coordinates: s along the road, d left positive, lane centres +w (lane 1), 0, -w (lane 3).
"""
import heapq
import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

import numpy as np


def s_curve_length(delta: float, radius: float, max_heading: float) -> float:
    """Road length of an S-curve moving sideways by |delta| (arc, straight, arc)."""
    y = abs(delta)
    if y < 1e-9:
        return 0.0
    full = 2.0 * radius * (1.0 - math.cos(max_heading))
    if y <= full:
        return 2.0 * radius * math.sin(math.acos(1.0 - y / (2.0 * radius)))
    return 2.0 * radius * math.sin(max_heading) + (y - full) / math.tan(max_heading)


def s_curve(u: np.ndarray, delta: float, radius: float, max_heading: float
            ) -> Tuple[np.ndarray, np.ndarray]:
    """Sideways offset (0 .. delta) and heading (rad) at road distance u from the start of the
    change; before 0 it is 0, after its length delta."""
    y, sign = abs(delta), (1.0 if delta >= 0 else -1.0)
    u = np.asarray(u, float)
    if y < 1e-9:
        return np.zeros_like(u), np.zeros_like(u)
    full = 2.0 * radius * (1.0 - math.cos(max_heading))
    th = math.acos(1.0 - y / (2.0 * radius)) if y <= full else max_heading
    xa = radius * math.sin(th)                       # road length of one arc
    ya = radius * (1.0 - math.cos(th))
    xs = 0.0 if y <= full else (y - 2.0 * ya) / math.tan(th)
    length = 2.0 * xa + xs
    off = np.zeros_like(u)
    head = np.zeros_like(u)
    a = (u > 0) & (u <= xa)
    off[a] = radius - np.sqrt(radius ** 2 - u[a] ** 2)
    head[a] = np.arcsin(u[a] / radius)
    b = (u > xa) & (u <= xa + xs)
    off[b] = ya + (u[b] - xa) * math.tan(th)
    head[b] = th
    c = (u > xa + xs) & (u < length)
    r = length - u[c]
    off[c] = y - (radius - np.sqrt(radius ** 2 - r ** 2))
    head[c] = np.arcsin(r / radius)
    off[u >= length] = y
    return sign * off, sign * head


@dataclass
class Sequence:
    """Chosen plan: lane changes as (lane from, lane to, start s), and the path in lane coordinates."""
    changes: List[Tuple[int, int, float]]
    path: np.ndarray                  # (K, 2) s, d
    cost: float
    late: bool = False                # only the relaxed deadline (over the island tip) was reachable
    first_lane: int = 2
    lanes: List[int] = field(default_factory=list)


def plan_sequence(s0: float, d0: float, psi: float, target: Optional[int], deadline: float,
                  late_deadline: float, horizon_end: float, p,
                  blocked: Callable[[np.ndarray, np.ndarray, np.ndarray, bool], np.ndarray],
                  closeness: Callable[[np.ndarray, np.ndarray, np.ndarray], np.ndarray],
                  keep: Optional[Sequence] = None) -> Optional[Sequence]:
    """Cheapest lane sequence from the car (s0, d0, heading psi) into lane `target` (1 / 3; None = any
    lane) by `deadline`, else by `late_deadline` (late=True). blocked(s, d, heading, late) -> bool per
    sample; closeness(s, d, heading) -> cost per sample (0 .. 100). keep: the last chosen sequence,
    preferred while it is still free (no switching back and forth)."""
    w = p.lane_width
    centre = {1: w, 2: 0.0, 3: -w}
    step = p.seq_step
    n = int(math.ceil((horizon_end - s0) / step)) + 1
    grid = s0 + step * np.arange(n)
    R, th = p.lc_radius, math.radians(p.lc_max_heading_deg)

    # first leg: from the car into each lane, starting now; the present heading is blended out over
    # lc_blend metres (offset m0 (u - u^2/L + u^3/(3 L^2))) and the S-curve covers the rest
    m0 = math.tan(max(-1.0, min(1.0, psi)))
    Lb = p.lc_blend
    first = {}
    for lane, c in centre.items():
        shift = m0 * Lb / 3.0
        delta = c - d0 - shift
        L = max(s_curve_length(delta, R, th), Lb)
        k = int(math.ceil(L / step))
        u = np.arange(0.0, k * step + 1e-9, p.seq_sample)
        off, head = s_curve(u, delta, R, th)
        ub = np.minimum(u, Lb)
        blend = m0 * (ub - ub ** 2 / Lb + ub ** 3 / (3 * Lb ** 2))
        hb = np.where(u < Lb, np.arctan(m0 * (1 - u / Lb) ** 2), 0.0)
        d = d0 + blend + off
        h = head + hb
        first[lane] = (k, s0 + u, d, h)

    # the lane changes between lane centres, as offsets from their start
    change = {}
    for a in centre:
        for b in centre:
            if a == b:
                continue
            delta = centre[b] - centre[a]
            L = s_curve_length(delta, R, th)
            k = int(math.ceil(L / step))
            u = np.arange(0.0, k * step + 1e-9, p.seq_sample)
            off, head = s_curve(u, delta, R, th)
            change[(a, b)] = (k, u, off, head)

    us = np.arange(0.0, step + 1e-9, p.seq_sample)        # samples of one step in a lane

    def free(s, d, h, late):
        return not bool(blocked(s, d, h, late).any())

    def near(s, d, h):
        return float(closeness(s, d, h).mean())

    results = []
    for late, end in ((False, deadline), (True, late_deadline)):
        stay_ok = {}
        stay_cost = {}
        for lane, c in centre.items():
            s = grid[:, None] + us[None, :]
            d = np.full_like(s, c)
            h = np.zeros_like(s)
            bl = blocked(s.ravel(), d.ravel(), h.ravel(), late).reshape(s.shape).any(axis=1)
            cl = closeness(s.ravel(), d.ravel(), h.ravel()).reshape(s.shape).mean(axis=1)
            stay_ok[lane], stay_cost[lane] = ~bl, cl
        # Dijkstra over (lane, step index)
        INF = float('inf')
        best = {}
        prev = {}
        heap = []
        end_i = int(math.ceil((end - s0) / step))
        last_i = n - 1
        for lane, (k, s, d, h) in first.items():
            if k >= n or (lane == target and k > end_i) or not free(s, d, h, late):
                continue
            cost = (p.w_change * (abs(centre[lane] - d0) > 0.10) + p.w_lateral * abs(centre[lane] - d0)
                    + p.w_obstacle * near(s, d, h) * k * step)
            if keep is not None and keep.first_lane == lane:
                cost -= p.w_keep
            node = (lane, k)
            if cost < best.get(node, INF):
                best[node] = cost
                prev[node] = ('first', lane)
                heapq.heappush(heap, (cost, lane, k))
        goal = None
        while heap:
            cost, lane, i = heapq.heappop(heap)
            if cost > best.get((lane, i), INF):
                continue
            if (target is None or lane == target) and i >= end_i:
                # in the target lane by the deadline: it only has to stay free to the end
                if stay_ok[lane][i:last_i].all():
                    goal = (lane, i)
                    break
            if i >= last_i:
                continue
            # stay one step
            if stay_ok[lane][i]:
                nc = cost + p.w_obstacle * stay_cost[lane][i] * step
                if target is not None and lane != target:
                    nc += p.w_off_target * step
                if nc < best.get((lane, i + 1), INF):
                    best[(lane, i + 1)] = nc
                    prev[(lane, i + 1)] = ('stay', lane, i)
                    heapq.heappush(heap, (nc, lane, i + 1))
            # change lanes from here (not after the deadline into a wrong lane)
            for b in centre:
                if b == lane:
                    continue
                k, u, off, head = change[(lane, b)]
                j = i + k
                if j > last_i or (b == target and j > end_i):     # into the target lane too late
                    continue
                s = grid[i] + u
                d = centre[lane] + off
                if not free(s, d, head, late):
                    continue
                nc = (cost + p.w_change + p.w_lateral * abs(centre[b] - centre[lane])
                      + p.w_obstacle * near(s, d, head) * k * step)
                if target is not None and b != target:
                    nc += p.w_off_target * k * step
                if keep is not None and any(a == lane and bb == b and abs(st - grid[i]) <= 0.25
                                            for a, bb, st in keep.changes):
                    nc -= p.w_keep
                if nc < best.get((b, j), INF):
                    best[(b, j)] = nc
                    prev[(b, j)] = ('change', lane, i)
                    heapq.heappush(heap, (nc, b, j))
        if goal is None:
            continue
        # walk back
        node = goal
        legs = []
        changes = []
        while True:
            kind = prev[node]
            if kind[0] == 'first':
                legs.append(('first', kind[1]))
                break
            if kind[0] == 'stay':
                legs.append(('stay', kind[1], kind[2]))
                node = (kind[1], kind[2])
            else:
                legs.append(('change', kind[1], node[0], kind[2]))
                changes.append((kind[1], node[0], float(grid[kind[2]])))
                node = (kind[1], kind[2])
        legs.reverse()
        changes.reverse()
        pts = []
        lanes = []
        for leg in legs:
            if leg[0] == 'first':
                k, s, d, _ = first[leg[1]]
                pts.append(np.column_stack((s, d)))
                lanes.append(leg[1])
            elif leg[0] == 'stay':
                pts.append(np.column_stack((grid[leg[2]] + us, np.full_like(us, centre[leg[1]]))))
            else:
                k, u, off, _ = change[(leg[1], leg[2])]
                pts.append(np.column_stack((grid[leg[3]] + u, centre[leg[1]] + off)))
                lanes.append(leg[2])
        lane_end = goal[0]
        tail = np.arange(grid[goal[1]], horizon_end + 1e-9, p.seq_sample)
        pts.append(np.column_stack((tail, np.full_like(tail, centre[lane_end]))))
        path = np.vstack(pts)
        order = np.argsort(path[:, 0], kind='stable')
        path = path[order]
        keep_pts = np.concatenate(([True], np.diff(path[:, 0]) > 1e-6))
        results.append(Sequence(changes=changes, path=path[keep_pts], cost=best[goal], late=late,
                                first_lane=legs[0][1], lanes=lanes))
        break
    return results[0] if results else None
