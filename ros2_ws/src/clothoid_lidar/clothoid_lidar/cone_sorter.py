"""Cone course start/end detection and left/right boundary sorting.

ROS-free. Ported from 2025 `cone_sorter_node.cpp`:
  * course start / continue zones and first-cone selection: same logic and parameters.
  * boundary tracing: `trace_method` picks one.
    `dfs2025` (default): the 2025 search over every chain with weighted costs, see cone_trace.py.
    `greedy`: a simpler chain from the first cone of each side to the nearest unused cone, limited by
    the cone gap and the turning angle.
All points are in base_link (x forward, y left).
"""
from dataclasses import dataclass, field
from typing import List, Optional
import time

import numpy as np

from clothoid_lidar.cone_trace import best_chains, extend_inner_gaps

LEFT = +1
RIGHT = -1


@dataclass
class SorterParams:
    trace_method: str = 'dfs2025'         # dfs2025 or greedy
    max_dist_between_cones: float = 0.4   # m, max gap inside one boundary
    max_trace_length: int = 20            # cones per boundary
    threshold_directional_angle_deg: float = 45.0  # max turn between two chain segments
    # first step of a chain, measured from the car's heading. The first cone is beside the car and the
    # corridor can already be at an angle to it, so this is wider than the limit between segments.
    first_step_angle_deg: float = 45.0
    # first cone of each side
    angle_from_left_first_cone: float = 45.0  # deg from +x, mirrored for the right side
    dist_from_left_first_cone: float = 0.7    # m
    dist_from_right_first_cone: float = 0.7   # m
    # course start zone (base_link box)
    start_x_min: float = 0.0
    start_x_max: float = 1.3
    start_y_min: float = -0.6
    start_y_max: float = 0.6
    start_min_count: int = 6
    # course continue zone
    ing_x_min: float = 0.0
    ing_x_max: float = 1.0
    ing_y_min: float = -1.0
    ing_y_max: float = 1.0
    ing_min_count: int = 4
    # dfs2025 only, 2025 launch values
    max_n_neighbors: int = 10
    threshold_absolute_angle_deg: float = 60.0
    ellipse_major_radius_factor: float = 2.0
    ellipse_minor_radius_factor: float = 1.0
    len_override_factor: float = 2.0
    car_size: float = 0.5                 # m, chains may not cross the car
    angle_cost_threshold_deg: float = 45.0
    residual_dist_threshold: float = 0.0  # m, 0 = max_dist_between_cones
    wrong_dir_threshold_deg: float = 45.0
    w_angle_cost: float = 1000.0
    w_residual_dist_cost: float = 200.0
    w_num_cones_cost: float = 5000.0
    w_initial_dir_cost: float = 1000.0
    w_change_dir_cost: float = 2000.0
    w_wrong_dir_cost: float = 1000.0
    max_search_nodes: int = 3000          # bounds the time of one search, not in 2025
    boundary_timeout: float = 0.25       # discard association after a scan gap, seconds


@dataclass
class SortResult:
    left: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    right: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))


def _count_in_box(cones: np.ndarray, x0, x1, y0, y1) -> int:
    if len(cones) == 0:
        return 0
    inside = ((cones[:, 0] > x0) & (cones[:, 0] < x1) & (cones[:, 1] > y0) & (cones[:, 1] < y1))
    return int(inside.sum())


class ConeSorter:
    def __init__(self, params: SorterParams):
        self.p = params

    # --- zones ------------------------------------------------------------------
    def start_zone_ok(self, cones: np.ndarray) -> bool:
        p = self.p
        return _count_in_box(cones, p.start_x_min, p.start_x_max,
                             p.start_y_min, p.start_y_max) >= p.start_min_count

    def continue_zone_ok(self, cones: np.ndarray) -> bool:
        p = self.p
        return _count_in_box(cones, p.ing_x_min, p.ing_x_max,
                             p.ing_y_min, p.ing_y_max) >= p.ing_min_count

    # --- first cone -------------------------------------------------------------
    def select_first_cone(self, cones: np.ndarray, side: int) -> Optional[int]:
        """Nearest cone on that side that is far enough off the heading and close enough."""
        p = self.p
        best, best_dist = None, np.inf
        for i, (x, y) in enumerate(cones):
            if side == LEFT and y <= 0.0 or side == RIGHT and y >= 0.0:
                continue
            angle = np.degrees(np.arctan2(y, x))
            if side == LEFT and not angle > p.angle_from_left_first_cone:
                continue
            if side == RIGHT and not angle < -p.angle_from_left_first_cone:
                continue
            dist = float(np.hypot(x, y))
            if dist < best_dist:
                best, best_dist = i, dist
        limit = p.dist_from_left_first_cone if side == LEFT else p.dist_from_right_first_cone
        if best is None or best_dist > limit:
            return None
        return best

    # --- boundaries -------------------------------------------------------------
    def extend_corners(self, cones: np.ndarray, result: SortResult) -> SortResult:
        """Add supported terminal observations only to the published boundaries.

        The mission retains the original trace for association on the next scan.
        A corner continuation must not silently become a new left/right seed.
        """
        if self.p.trace_method != 'dfs2025' or not len(result.left) or not len(result.right):
            return result
        points = [(float(x), float(y)) for x, y in cones]
        index = {point: i for i, point in enumerate(points)}
        left = [index[tuple(p)] for p in result.left]
        right = [index[tuple(p)] for p in result.right]
        left, right = extend_inner_gaps(points, left, right, self.p)
        return SortResult(left=cones[left], right=cones[right])

    def trace(self, cones: np.ndarray, start: int, used: set) -> List[int]:
        """Greedy chain: nearest unused cone within the gap, without a sharp turn."""
        p = self.p
        max_turn = np.radians(p.first_step_angle_deg)
        chain = [start]
        heading = np.array([1.0, 0.0])
        while len(chain) < p.max_trace_length:
            here = cones[chain[-1]]
            best, best_dist = None, np.inf
            for j, cone in enumerate(cones):
                if j in chain or j in used:
                    continue
                step = cone - here
                dist = float(np.hypot(*step))
                if dist < 1e-6 or dist > p.max_dist_between_cones or dist >= best_dist:
                    continue
                cos = float(np.clip(step @ heading / dist, -1.0, 1.0))
                if np.arccos(cos) > max_turn:
                    continue
                best, best_dist = j, dist
            if best is None:
                break
            step = cones[best] - here
            heading = step / np.hypot(*step)
            max_turn = np.radians(p.threshold_directional_angle_deg)
            chain.append(best)
        return chain

    def _tracked_seed(self, cones, previous, opposite):
        """Associate only current observations, within half a boundary cone gap.

        No old point is published or extrapolated. Ambiguous opposite-side matches
        are discarded; entry gates remain the fallback for reacquisition.
        """
        if not len(cones) or not len(previous):
            return None
        distance = np.linalg.norm(cones[:, None]-previous[None, :], axis=2).min(axis=1)
        valid = distance < self.p.max_dist_between_cones/2
        if len(opposite):
            other = np.linalg.norm(cones[:, None]-opposite[None, :], axis=2).min(axis=1)
            valid &= distance < other
        candidates = np.flatnonzero(valid)
        if not len(candidates):
            return None
        return int(candidates[np.argmin(np.linalg.norm(cones[candidates], axis=1))])

    def sort(self, cones: np.ndarray, previous: Optional[SortResult] = None) -> SortResult:
        cones = np.asarray(cones, float).reshape(-1, 2)
        first_left = self.select_first_cone(cones, LEFT)
        first_right = self.select_first_cone(cones, RIGHT)
        if previous is not None:
            tracked_left = self._tracked_seed(cones, previous.left, previous.right)
            tracked_right = self._tracked_seed(cones, previous.right, previous.left)
            if tracked_left is not None:
                first_left = tracked_left
            if tracked_right is not None:
                first_right = tracked_right
            # A strict reacquisition must not steal the other tracked boundary.
            if first_left is not None and first_left == first_right:
                if tracked_left is not None:
                    first_right = None
                else:
                    first_left = None
        if first_left is None and first_right is None:
            return SortResult()

        # fewer than 3 cones: only the first cones can be sent (2025)
        if len(cones) < 3:
            return SortResult(
                left=cones[[first_left]] if first_left is not None else np.zeros((0, 2)),
                right=cones[[first_right]] if first_right is not None else np.zeros((0, 2)))

        if self.p.trace_method == 'dfs2025':
            points = [(float(x), float(y)) for x, y in cones]
            left, right = best_chains(points, first_left, first_right, self.p, LEFT, RIGHT)
            return SortResult(left=cones[left] if left else np.zeros((0, 2)),
                              right=cones[right] if right else np.zeros((0, 2)))

        used = {i for i in (first_left, first_right) if i is not None}
        left = self.trace(cones, first_left, used - {first_left}) if first_left is not None else []
        used |= set(left)
        right = (self.trace(cones, first_right, used - {first_right})
                 if first_right is not None else [])
        return SortResult(left=cones[left] if left else np.zeros((0, 2)),
                          right=cones[right] if right else np.zeros((0, 2)))


class ConeMission:
    """IDLE -> RUNNING when the start zone is full and both first cones exist;
    back to IDLE when the continue zone empties."""

    def __init__(self, sorter: ConeSorter):
        self.sorter = sorter
        self.running = False
        self.previous = None
        self.last_stamp = None
        self.previous_stamp = None

    def update(self, cones: np.ndarray, stamp: Optional[float] = None) -> Optional[SortResult]:
        """Returns the sorted boundaries while the course is running, else None."""
        cones = np.asarray(cones, float).reshape(-1, 2)
        stamp = time.monotonic() if stamp is None else stamp
        if self.last_stamp is not None and not 0 <= stamp-self.last_stamp <= self.sorter.p.boundary_timeout:
            self.previous = None
        self.last_stamp = stamp
        if self.previous_stamp is not None and not 0 <= stamp-self.previous_stamp <= self.sorter.p.boundary_timeout:
            self.previous = None
        if not self.running:
            if (self.sorter.start_zone_ok(cones)
                    and self.sorter.select_first_cone(cones, LEFT) is not None
                    and self.sorter.select_first_cone(cones, RIGHT) is not None):
                self.running = True
                self.previous = self.sorter.sort(cones)
                self.previous_stamp = stamp
            return None
        if not self.sorter.continue_zone_ok(cones):
            self.running = False
            self.previous = None
            return None
        result = self.sorter.sort(cones, self.previous)
        self.previous = result
        self.previous_stamp = stamp
        # Host A/B comparison: publish the ordinary trace without bridging inner gaps.
        return result
