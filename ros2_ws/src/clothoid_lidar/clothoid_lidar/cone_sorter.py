"""Cone course start/end detection and left/right boundary sorting.

ROS-free. Ported from 2025 `cone_sorter_node.cpp`:
  * course start / continue zones and first-cone selection: same logic and parameters.
  * boundary tracing: SIMPLIFIED. 2025 searched every cone chain through an adjacency graph
    with DFS and scored them with seven weighted costs (angle, residual distance, number of
    cones, initial / changing direction, cones on either side, wrong direction). Here each
    side is a greedy nearest-neighbour chain from its first cone, limited by the cone gap and
    the turning angle. Good enough for a clean gate; upgrade it from the 2025 source
    (`git show ros1-2025:webot_ws/src/rubber_cone/src/cone_sorter_node.cpp`) if cones are
    dense or noisy.
All points are in base_link (x forward, y left).
"""
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

LEFT = +1
RIGHT = -1


@dataclass
class SorterParams:
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

    def sort(self, cones: np.ndarray) -> SortResult:
        cones = np.asarray(cones, float).reshape(-1, 2)
        first_left = self.select_first_cone(cones, LEFT)
        first_right = self.select_first_cone(cones, RIGHT)
        if first_left is None and first_right is None:
            return SortResult()

        # fewer than 3 cones: only the first cones can be sent (2025)
        if len(cones) < 3:
            return SortResult(
                left=cones[[first_left]] if first_left is not None else np.zeros((0, 2)),
                right=cones[[first_right]] if first_right is not None else np.zeros((0, 2)))

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

    def update(self, cones: np.ndarray) -> Optional[SortResult]:
        """Returns the sorted boundaries while the course is running, else None."""
        cones = np.asarray(cones, float).reshape(-1, 2)
        if not self.running:
            if (self.sorter.start_zone_ok(cones)
                    and self.sorter.select_first_cone(cones, LEFT) is not None
                    and self.sorter.select_first_cone(cones, RIGHT) is not None):
                self.running = True
            return None
        if not self.sorter.continue_zone_ok(cones):
            self.running = False
            return None
        return self.sorter.sort(cones)
