"""Boundary tracing of the 2025 cone sorter (`cone_sorter_node.cpp`, ros1-2025), ROS-free.

From the first cone of a side it walks a graph of neighbouring cones depth first, keeps every chain
that cannot grow further and returns the one with the lowest weighted cost. Plain Python floats:
this runs on every scan, and numpy per point is much slower for arrays this small.

Two differences from 2025. `max_search_nodes` bounds the search, because the number of chains grows
quickly with dense cones and Python is slower than the original C++. The `cones on either side` cost is
left out, its 2025 weight was 0.
"""
import math
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]


def _wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _angle_between(ax: float, ay: float, bx: float, by: float) -> float:
    na, nb = math.hypot(ax, ay), math.hypot(bx, by)
    if na < 1e-9 or nb < 1e-9:
        return 0.0
    return math.acos(max(-1.0, min(1.0, (ax * bx + ay * by) / (na * nb))))


def _crosses_car(p: Point, q: Point, car_size: float) -> bool:
    """Does segment p-q cross the car, a segment on the x axis (from -size/2 to +size)?"""
    dx, dy = q[0] - p[0], q[1] - p[1]
    if abs(dy) < 1e-9:   # parallel to the car: only a collinear segment can touch it
        return abs(p[1]) < 1e-9 and max(p[0], q[0]) >= -car_size / 2 and min(p[0], q[0]) <= car_size
    t = -p[1] / dy
    if not -1e-9 <= t <= 1.0 + 1e-9:
        return False
    return -car_size / 2 - 1e-9 <= p[0] + t * dx <= car_size + 1e-9


def _neighbours(points: Sequence[Point], k: int, max_dist: float) -> List[List[int]]:
    """Undirected graph: j and i are linked if each is in the other's k nearest, within max_dist."""
    n = len(points)
    nearest = []
    for i in range(n):
        by_dist = sorted((math.dist(points[i], points[j]), j) for j in range(n) if j != i)
        nearest.append({j for d, j in by_dist[:k] if d <= max_dist})
    return [[j for j in range(n) if j != i and j in nearest[i] and i in nearest[j]]
            for i in range(n)]


def _reachable(graph: List[List[int]], start: int) -> int:
    seen, stack = {start}, [start]
    while stack:
        for j in graph[stack.pop()]:
            if j not in seen:
                seen.add(j)
                stack.append(j)
    return len(seen)


def _can_add(points, chain, depth, candidate, side, p) -> bool:
    if candidate in chain[:depth + 1]:
        return False
    cand = points[candidate]
    last = points[chain[depth]]
    if depth == 0:
        bearing = math.atan2(cand[1], cand[0])
        sign = (bearing > 0.0) - (bearing < 0.0)
        if sign != side and abs(bearing) >= math.radians(5.0):
            return False
    else:
        before = points[chain[depth - 1]]
        v1x, v1y = last[0] - before[0], last[1] - before[1]
        v2x, v2y = cand[0] - last[0], cand[1] - last[1]
        len1, len2 = math.hypot(v1x, v1y), math.hypot(v2x, v2y)
        if len1 < 1e-6 or len2 < 1e-6:
            return False
        # inside an ellipse ahead of the last segment
        a = p.ellipse_major_radius_factor * p.max_dist_between_cones
        b = p.ellipse_minor_radius_factor * p.max_dist_between_cones
        along = (v2x * v1x + v2y * v1y) / len1
        across = (v2y * v1x - v2x * v1y) / len1
        if (along * along) / (a * a) + (across * across) / (b * b) > 1.0:
            return False
        # turn limits
        angle1 = math.atan2(v1y, v1x)
        turn = _wrap(math.atan2(v2y, v2x) - angle1)
        if abs(turn) > math.radians(p.threshold_absolute_angle_deg):
            return False
        limit = math.radians(p.threshold_directional_angle_deg)
        toward_outside = turn < limit if side > 0 else turn > -limit
        if not (toward_outside or len2 < p.len_override_factor * p.max_dist_between_cones):
            return False
        # no sudden swap of the turning direction
        if depth >= 2:
            first = points[chain[depth - 2]]
            v0x, v0y = before[0] - first[0], before[1] - first[1]
            if math.hypot(v0x, v0y) > 1e-6:
                prev_turn = _wrap(angle1 - math.atan2(v0y, v0x))
                if turn * prev_turn < 0.0 and abs(turn - prev_turn) > 1.3:
                    return False
    return not _crosses_car(last, cand, p.car_size)


def _chains(points, graph, start, side, p) -> List[Tuple[int, ...]]:
    target = min(_reachable(graph, start), p.max_trace_length)
    if target < 2:
        return []
    chain = [-1] * target
    stack = [(start, 0)]
    found, nodes = set(), 0
    while stack and nodes < p.max_search_nodes:
        node, depth = stack.pop()
        nodes += 1
        chain[depth] = node
        for i in range(depth + 1, target):
            chain[i] = -1
        extended = False
        if depth < target - 1:
            for j in graph[node]:
                if _can_add(points, chain, depth, j, side, p):
                    extended = True
                    stack.append((j, depth + 1))
        if not extended:
            used = chain.index(-1) if -1 in chain else target
            if used >= 2:
                found.add(tuple(chain[:used]))
    return list(found)


def _skip_distance_cost(points, chain, side, p, graph, cache) -> float:
    """Soft cost of skipping nearer *eligible* neighbours, per branch decision.

    Prefixes share their nearest-distance result within one side of one scan.
    A farther continuation is still allowed to win on the other chain costs.
    """
    total = 0.0
    for depth, (node, chosen) in enumerate(zip(chain, chain[1:])):
        prefix = tuple(chain[:depth + 1])
        if prefix not in cache:
            distances = [math.dist(points[node], points[j]) for j in graph[node]
                         if _can_add(points, chain, depth, j, side, p)]
            cache[prefix] = min(distances) if distances else 0.0
        nearest = cache[prefix]
        if nearest > 1e-6:
            extra = max(0.0, math.dist(points[node], points[chosen]) / nearest - 1.0)
            total += extra * extra
    return total


def _cost(points, chain, side, p, graph=None, distance_cache=None) -> float:
    q = [points[i] for i in chain]
    n = len(q)
    segs = [(q[i + 1][0] - q[i][0], q[i + 1][1] - q[i][1]) for i in range(n - 1)]

    # sharp corners
    sharp_limit = math.radians(p.angle_cost_threshold_deg)
    total, count, sharp = 0.0, 0, 0
    for i in range(1, n - 1):
        inner = _angle_between(segs[i][0], segs[i][1], -segs[i - 1][0], -segs[i - 1][1])
        total += (math.pi - inner) / math.pi
        count += 1
        sharp += inner < sharp_limit
    angle = total / count * (sharp + 1) if count else 0.0

    # gaps longer than the threshold
    limit = p.residual_dist_threshold if p.residual_dist_threshold > 0.0 else p.max_dist_between_cones
    residual = sum(max(0.0, math.hypot(*s) - limit) ** 2 for s in segs) / len(segs)
    if graph is None:
        graph = _neighbours(points, p.max_n_neighbors, p.max_dist_between_cones)
    residual += _skip_distance_cost(points, chain, side, p, graph,
                                   {} if distance_cache is None else distance_cache)

    number = 1.0 / n   # longer is better
    initial = _angle_between(segs[0][0], segs[0][1], 1.0, 0.0)   # first step along the car

    headings = [math.atan2(s[1], s[0]) for s in segs]
    turns = [_wrap(headings[i] - headings[i - 1]) for i in range(1, len(headings))]
    # Direction-change penalty omitted for the host comparison.
    # turning away from the track (left boundary turning further left)
    wrong_limit = math.radians(p.wrong_dir_threshold_deg)
    wrong = sum(abs(t) for t in turns if t * side > 0.0 and abs(t) > wrong_limit)

    weight = (p.w_angle_cost + p.w_residual_dist_cost + p.w_num_cones_cost + p.w_initial_dir_cost
              + p.w_wrong_dir_cost)
    return (p.w_angle_cost * angle + p.w_residual_dist_cost * residual + p.w_num_cones_cost * number
            + p.w_initial_dir_cost * initial
            + p.w_wrong_dir_cost * wrong) / weight


def _point_segment_distance(point, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    length = dx*dx+dy*dy
    t = max(0., min(1., ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length)) if length > 1e-12 else 0.
    return math.hypot(point[0]-a[0]-t*dx, point[1]-a[1]-t*dy)


def _segments_cross(a, b, c, d):
    def cross(p, q, r):
        return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    return cross(a, b, c)*cross(a, b, d) < 0. and cross(c, d, a)*cross(c, d, b) < 0.


def _bridge_inner_gap(points, chain, other, graph, p, side):
    """Continue an observed inner corner across one gap with a confirmed tail.

    Keep the normal neighbour graph unchanged. Only a terminal inward turn can
    bridge up to two normal cone gaps, shorter than the opposite-wall distance.
    At least two new observations must form a normal-gap continuation. The tail
    uses the existing turn rules, starting a new turn history at the bridge.
    """
    if len(chain) < 2 or len(other) < 3 or len(chain) >= p.max_trace_length-1:
        return chain
    def heading_span(indices):
        headings = [math.atan2(points[b][1]-points[a][1], points[b][0]-points[a][0])
                    for a, b in zip(indices, indices[1:])]
        return max(abs(_wrap(h-headings[0])) for h in headings)
    outer_turn = heading_span(other)
    if outer_turn < math.radians(p.threshold_absolute_angle_deg) or heading_span(chain) >= outer_turn:
        # A curved opposite wall is the evidence that this is an inner endpoint.
        return chain
    end, previous = points[chain[-1]], points[chain[-2]]
    heading = math.atan2(end[1]-previous[1], end[0]-previous[0])
    wall = [points[i] for i in other]
    def wall_distance(point):
        return (min(_point_segment_distance(point, a, b) for a, b in zip(wall, wall[1:]))
                if len(wall) > 1 else math.dist(point, wall[0]))
    width = wall_distance(end)
    limit = min(2*p.max_dist_between_cones, width)
    used = set(chain) | set(other)
    prior_edges = list(zip(chain[:-2], chain[1:-1]))
    candidates = []
    for i, point in enumerate(points):
        if i in used:
            continue
        gap = math.dist(end, point)
        if not 1e-6 < gap < limit:
            continue
        turn = _wrap(math.atan2(point[1]-end[1], point[0]-end[0])-heading)
        turn_size = abs(turn)
        if turn*side >= 0 or not math.radians(p.threshold_absolute_angle_deg) < turn_size <= math.radians(
                p.threshold_absolute_angle_deg+p.threshold_directional_angle_deg):
            continue
        if wall_distance(point) <= gap:
            continue
        if _crosses_car(end, point, p.car_size):
            continue
        if any(_segments_cross(end, point, a, b) for a, b in zip(wall, wall[1:])):
            continue
        if any(_segments_cross(end, point, points[a], points[b]) for a, b in prior_edges):
            continue
        if gap <= p.max_dist_between_cones:
            # Do not skip a nearer observation to manufacture a longer gap.
            return chain
        tail = [chain[-1], i]
        blocked = used - {chain[-1]}
        while len(chain)+len(tail)-1 < p.max_trace_length:
            choices = [j for j in graph[tail[-1]] if j not in blocked
                       and wall_distance(points[j]) > gap
                       and _can_add(points, tail, len(tail)-1, j, side, p)
                       and not any(_segments_cross(points[tail[-1]], points[j], a, b)
                                   for a, b in zip(wall, wall[1:]))
                       and not any(_segments_cross(points[tail[-1]], points[j], points[a], points[b])
                                   for a, b in zip(chain[:-1], chain[1:]))]
            if not choices:
                break
            tail.append(min(choices, key=lambda j: (math.dist(points[tail[-1]], points[j]), j)))
        if len(tail) >= 3:
            candidates.append((gap, i, tail[1:]))
    if not candidates:
        return chain
    return list(chain)+min(candidates)[2]


def best_chains(points: Sequence[Point], first_left: Optional[int], first_right: Optional[int],
                params, left: int = 1, right: int = -1) -> Tuple[List[int], List[int]]:
    """Index chains of both boundaries; preserve two observed cones at a short wall end."""
    graph = _neighbours(points, params.max_n_neighbors, params.max_dist_between_cones)
    result = []
    for first, side in ((first_left, left), (first_right, right)):
        if first is None:
            result.append([])
            continue
        chains = _chains(points, graph, first, side, params)
        distance_cache = {}
        best = min(chains, key=lambda c: _cost(points, c, side, params, graph, distance_cache)) if chains else (first,)
        result.append(list(best))
    return result[0], result[1]


def extend_inner_gaps(points, left, right, params):
    """Extend published boundaries without changing the normal tracing seeds."""
    if not left or not right:
        return left, right
    graph = _neighbours(points, params.max_n_neighbors, params.max_dist_between_cones)
    extended_left = _bridge_inner_gap(points, left, right, graph, params, +1)
    extended_right = _bridge_inner_gap(points, right, extended_left, graph, params, -1)
    return extended_left, extended_right
